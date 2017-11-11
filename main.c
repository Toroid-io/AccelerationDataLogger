/* TODO
 * - Configu using SPI EEPROM
 * - Use UART for data transfer
 * - Acquisition program
 * - Read battery values
 */
#include "ch.h"
#include "hal.h"
#include "ChibiOS/os/hal/lib/streams/chprintf.h"
#include "MPU6050.h"
#include "ADXL345.h"

/*===========================================================================*/
/* Global variables - RTOS stuff                                             */
/*===========================================================================*/

enum state {IDLE,
	CALIBRATION,
	START_ACQUISITION,
	ACQUISITION,
	START_PRINT,
	PRINT} SystemState;
static MUTEX_DECL(SystemState_mutex);

struct configStructure {
	uint16_t magicNumber;
	uint16_t samplingSpeed;
	int16_t calibrationMPU[3];
	int16_t calibrationADXL[3];
	bool gyroActivated;
} systemConfig;

/* the mailboxes are used to send information into the spi thread */
#define NUM_BUFFERS 4
#define BUFFERS_SIZE 8
static char buffers[NUM_BUFFERS][BUFFERS_SIZE];
static msg_t free_buffers_queue[NUM_BUFFERS];
static mailbox_t free_buffers;
static msg_t filled_buffers_queue[NUM_BUFFERS];
static mailbox_t filled_buffers;

/* Mutex used to analyze the amount of memory available */
#define SPI_RAM_SIZE 80*1024
static MUTEX_DECL(memoryCounter_mutex);
volatile int32_t memoryCounter;

/* Various semaphores to synchronize stuff */
static BSEMAPHORE_DECL(writeSerial, 1);
static threads_queue_t threadQueue;

//static thread_reference_t mpu_thread = NULL;
//static thread_reference_t adxl_thread = NULL;

static int16_t calibrationADXL[3];
static int16_t calibrationMPU[3];

/*
 * SPI RX buffer
 */

static union Rxbuf {
	uint8_t c[32];
	int16_t i[16];
} rxbuf;

/*===========================================================================*/
/* Callbacks                                                                 */
/*===========================================================================*/

/* Triggered when the button is released.*/
static void extcb1(EXTDriver *extp, expchannel_t channel) {
  (void)extp;
  (void)channel;
  chSysLockFromISR();
  if (SystemState == IDLE)
	  SystemState = CALIBRATION;
  chSysUnlockFromISR();
}

static void extcb2(EXTDriver *extp, expchannel_t channel) {
  (void)extp;
  (void)channel;
  chSysLockFromISR();
  if (SystemState == IDLE)
	  SystemState = START_ACQUISITION;
  chSysUnlockFromISR();
}

static void gpt3cb(GPTDriver *gptp) {
	(void)gptp;
	static virtual_timer_t vt;
	//palClearPad(GPIOC, GPIOC_LED_2);
	chSysLockFromISR();
	chThdDequeueAllI(&threadQueue, 0);
	//chThdResumeI(&mpu_thread, (msg_t)NULL);
	//chThdResumeI(&adxl_thread, (msg_t)NULL);
	/* LED2 set to OFF after 1mS.*/
	//chVTResetI(&vt);
	//chVTSetI(&vt, MS2ST(1), led2off, NULL);
	chSysUnlockFromISR();
}

/*===========================================================================*/
/* Peripherals config structures                                             */
/*===========================================================================*/

#if (USE_BITBANG_I2C)
static const I2CConfig softI2C1 = {
	false,
	I2C1_SCL_LINE,
	I2C1_SDA_LINE,
	4000
};

static const I2CConfig softI2C2 = {
	false,
	I2C2_SCL_LINE,
	I2C2_SDA_LINE,
	4000
};
#else

static const I2CConfig i2ccfg1 = {
	0x20803C54, // Normal mode, 100kHz
	0,
	0
};

static const I2CConfig i2ccfg2 = {
	0x80201619, // Normal mode, 100kHz
	0,
	0
};

#endif

#if 0
static const SPIConfig eepromSPI = {
	NULL,
	GPIOA,
	GPIOA_SPI_CS_1,
	SPI_CR1_BR_2 | SPI_CR1_BR_1,
	0
};
#endif

static const SPIConfig ramSPI = {
	NULL,
	GPIOA,
	GPIOA_SPI_CS_2,
	SPI_CR1_BR_2 | SPI_CR1_BR_1,
	SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0
};

static const EXTConfig extcfg = {
  {
    {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOA, extcb1},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOC, extcb2},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL}
  }
};

static const GPTConfig gpt3cfg = {
  100000, // 100kHz timer clock.
  gpt3cb,
  0, 0
};

/*===========================================================================*/
/* Functions                                                                 */
/*===========================================================================*/
void modifyAxis(int16_t *ax, int16_t *ay, int16_t *az)
{
	*az= -*az;
	*ax = *ax;
	*ay = -*ay;
}

static void led2off(void *arg) {
  (void)arg;
  palSetPad(GPIOC, GPIOC_LED_2);
}

static void blinkAllLeds(void){
  for (int i = 0; i < 2; ++i) {
    palClearPad(GPIOC, GPIOC_LED_1);
    palClearPad(GPIOC, GPIOC_LED_2);
    palClearPad(GPIOB, GPIOB_LED_3);
    palClearPad(GPIOB, GPIOB_LED_4);
    osalThreadSleepMilliseconds(100);
    palSetPad(GPIOC, GPIOC_LED_1);
    palSetPad(GPIOC, GPIOC_LED_2);
    palSetPad(GPIOB, GPIOB_LED_3);
    palSetPad(GPIOB, GPIOB_LED_4);
    osalThreadSleepMilliseconds(100);
  }
}

static void I2CInit(void)
{
	/*
	 * Check if the I2C is clogged
	 */

	if (!palReadPad(GPIOB, GPIOB_I2C1_SDA)) {
		for (int i  = 0; i < 16; ++i) {
			palClearPad(GPIOB, GPIOB_I2C1_SCL);
			osalThreadSleepMilliseconds(1);
			palSetPad(GPIOB, GPIOB_I2C1_SCL);
			osalThreadSleepMilliseconds(1);
		}
	}
	if (!palReadPad(GPIOB, GPIOB_I2C2_SDA)) {
		for (int i  = 0; i < 16; ++i) {
			palClearPad(GPIOB, GPIOB_I2C2_SCL);
			osalThreadSleepMilliseconds(1);
			palSetPad(GPIOB, GPIOB_I2C2_SCL);
			osalThreadSleepMilliseconds(1);
		}
	}

#if USE_BITBANG_I2C
	i2cStart(&I2CD1, &softI2C1);
	i2cStart(&I2CD2, &softI2C2);
#else
	/*
	 * Activates the I2C driver in hardware mode
	 */
	palSetPadMode(GPIOB, GPIOB_I2C1_SCL, PAL_MODE_ALTERNATE(1) |
		      PAL_STM32_OSPEED_HIGHEST | PAL_STM32_OTYPE_OPENDRAIN | PAL_STM32_PUPDR_PULLUP);
	palSetPadMode(GPIOB, GPIOB_I2C1_SDA, PAL_MODE_ALTERNATE(1) |
		      PAL_STM32_OSPEED_HIGHEST | PAL_STM32_OTYPE_OPENDRAIN | PAL_STM32_PUPDR_PULLUP);
	palSetPadMode(GPIOB, GPIOB_I2C2_SCL, PAL_MODE_ALTERNATE(1) |
		      PAL_STM32_OSPEED_HIGHEST | PAL_STM32_OTYPE_OPENDRAIN | PAL_STM32_PUPDR_PULLUP);
	palSetPadMode(GPIOB, GPIOB_I2C2_SDA, PAL_MODE_ALTERNATE(1) |
		      PAL_STM32_OSPEED_HIGHEST | PAL_STM32_OTYPE_OPENDRAIN | PAL_STM32_PUPDR_PULLUP);
	i2cStart(&I2CD1, &i2ccfg1);
	i2cStart(&I2CD2, &i2ccfg2);
#endif
}

static void calibrateSensors(void)
{
  uint16_t i;
  int16_t ax, ay, az;
  int32_t calibrationAccADXL[3] = {0, 0, 0};
  int32_t calibrationAccMPU[3] = {0, 0, 0};

  /* Calibration loop */
  for (i = 0; i < 128; ++i) {
    palClearPad(GPIOC, GPIOC_LED_2);
    chThdSleepMilliseconds(10);
    palSetPad(GPIOC, GPIOC_LED_2);
    MPU6050_getAcceleration(&ax, &ay, &az);
    calibrationAccMPU[0] += ax;
    calibrationAccMPU[1] += ay;
    calibrationAccMPU[2] += az;
    ADXL345_getAcceleration(&ax, &ay, &az);
    modifyAxis(&ax, &ay, &az);
    calibrationAccADXL[0] += ax;
    calibrationAccADXL[1] += ay;
    calibrationAccADXL[2] += az;
    chThdSleepMilliseconds(10);
  }

  calibrationMPU[0] = calibrationAccMPU[0] / 128;
  calibrationMPU[1] = calibrationAccMPU[1] / 128;
  calibrationMPU[2] = calibrationAccMPU[2] / 128;

  calibrationADXL[0] = calibrationAccADXL[0] / 128;
  calibrationADXL[1] = calibrationAccADXL[1] / 128;
  calibrationADXL[2] = calibrationAccADXL[2] / 128;

  /* We finished the calibration here*/
  //chprintf((BaseSequentialStream *)&SD1,"Calibration finished\r\n");
}

void createMailboxes(void)
{
	chMBObjectInit(&filled_buffers, filled_buffers_queue, NUM_BUFFERS);
	chMBObjectInit(&free_buffers, free_buffers_queue, NUM_BUFFERS);
	for (uint8_t i = 0; i < NUM_BUFFERS; i++)
		(void)chMBPost(&free_buffers, (msg_t)&buffers[i], TIME_INFINITE);
}

static void resetAllMailboxes(void)
{
	chMBReset(&free_buffers);
	chMBReset(&filled_buffers);

	createMailboxes();
}

static void sensorStartup(void)
{
	/* Initialize MPU */
	MPU6050_reset();
	chThdSleepMilliseconds(100);
	MPU6050_resetGyroscopePath();
	MPU6050_resetAccelerometerPath();
	MPU6050_resetTemperaturePath();
	chThdSleepMilliseconds(100);
	MPU6050_initialize();
	MPU6050_setStandbyXGyroEnabled(true);
	MPU6050_setStandbyYGyroEnabled(true);
	MPU6050_setStandbyZGyroEnabled(true);
	chThdSleepMilliseconds(100);
	// verify connection
	while (!MPU6050_testConnection()) {
		chprintf((BaseSequentialStream *)&SD1,"MPU FAIL\r\n");
		chThdSleepMilliseconds(1000);
	}
	chprintf((BaseSequentialStream *)&SD1,"MPU OK\r\n");

	/* Initialize ADXL345 */
	ADXL345_initialize();
	chThdSleepMilliseconds(100);
	// verify connection
	while (!ADXL345_testConnection()) {
		chprintf((BaseSequentialStream *)&SD1,"ADXL FAIL\r\n");
		chThdSleepMilliseconds(1000);
	}
	chprintf((BaseSequentialStream *)&SD1,"ADXL OK\r\n");
}


/*===========================================================================*/
/* Threads                                                                   */
/*===========================================================================*/
/*
 * State machine thread
 */
static THD_WORKING_AREA(waThread0, 0x800);
static THD_FUNCTION(Thread0, arg) {
	(void)arg;
	while(true) {
		switch(SystemState) {
		case IDLE:
		default:
			break;
		case CALIBRATION:
			chprintf((BaseSequentialStream *)&SD1,"Calibration...\r\n");
			calibrateSensors();
			chprintf((BaseSequentialStream *)&SD1,"Calibration DONE\r\n");
			chMtxLock(&SystemState_mutex);
			SystemState = IDLE;
			chMtxUnlock(&SystemState_mutex);
			break;
		case START_ACQUISITION:
			chprintf((BaseSequentialStream *)&SD1,"Acquisition...\r\n");
			chMtxLock(&memoryCounter_mutex);
			memoryCounter = 0;
			chMtxUnlock(&memoryCounter_mutex);
			chMtxLock(&SystemState_mutex);
			SystemState = ACQUISITION;
			chMtxUnlock(&SystemState_mutex);

			/* Start the timer */
			gptStartContinuous(&GPTD3, 124); // Timer interrupt at 800Hz
			break;
		case ACQUISITION:
			/* Nothing to do here
			 * State changement will be done in the RAM write task
			 */
			break;
		case PRINT:
			chBSemSignal(&writeSerial);
			break;
		}
		chThdSleepMilliseconds(100);
	}
}
/*
 * MPU6050  polling thread.
 */
static THD_WORKING_AREA(waThread1, 0x800);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  void *pbuf;

  chRegSetThreadName("MPU6050_Poll");

  int16_t ax, ay, az;

  chprintf((BaseSequentialStream *)&SD1,"MPU Thread\r\n");

  while (true) {
    /* wait until the timer starts */
    palClearPad(GPIOB, GPIOB_LED_3);

    chSysLock();
    chThdEnqueueTimeoutS(&threadQueue, TIME_INFINITE);
    chSysUnlock();

    palSetPad(GPIOB, GPIOB_LED_3);
    MPU6050_getAcceleration(&ax, &ay, &az);

    ax -= calibrationMPU[0];
    ay -= calibrationMPU[1];
    az -= calibrationMPU[2];

    if (chMBFetch(&free_buffers, (msg_t *)&pbuf, TIME_INFINITE) == MSG_OK) {
      char *message = (char *)pbuf;
      message[0]  = 'M';
      message[1]  = (char)((ax) & 0xFF);
      message[2]  = (char)((ax >> 8) & 0xFF);
      message[3]  = (char)((ay) & 0xFF);
      message[4]  = (char)((ay >> 8) & 0xFF);
      message[5]  = (char)((az) & 0xFF);
      message[6]  = (char)((az >> 8) & 0xFF);
      (void)chMBPost(&filled_buffers, (msg_t)pbuf, TIME_INFINITE);
    }
  }
}


/*
 * ADXL345  polling thread.
 */
static THD_WORKING_AREA(waThread2, 0x800);
static THD_FUNCTION(Thread2, arg) {

  (void)arg;
  void *pbuf;

  chRegSetThreadName("ADXL345_Poll");

  int16_t ax, ay, az;

  chprintf((BaseSequentialStream *)&SD1,"ADXL Thread\r\n");

  while (true) {
    /* wait until the timer says */
    palClearPad(GPIOB, GPIOB_LED_4);
    chSysLock();
    chThdEnqueueTimeoutS(&threadQueue, TIME_INFINITE);
    chSysUnlock();

    palSetPad(GPIOB, GPIOB_LED_4);
    ADXL345_getAcceleration(&ax, &ay, &az);
    modifyAxis(&ax, &ay, &az);

    ax -= calibrationADXL[0];
    ay -= calibrationADXL[1];
    az -= calibrationADXL[2];

    if (chMBFetch(&free_buffers, (msg_t *)&pbuf, TIME_INFINITE) == MSG_OK) {
      char *message = (char *)pbuf;
      message[0]  = 'A';
      message[1]  = (char)((ax) & 0xFF);
      message[2]  = (char)((ax >> 8) & 0xFF);
      message[3]  = (char)((ay) & 0xFF);
      message[4]  = (char)((ay >> 8) & 0xFF);
      message[5]  = (char)((az) & 0xFF);
      message[6]  = (char)((az >> 8) & 0xFF);
      (void)chMBPost(&filled_buffers, (msg_t)pbuf, TIME_INFINITE);
    }
  }
}

/*
 * SPI bus thread
 */
static THD_WORKING_AREA(waThread5, 256);
static THD_FUNCTION(Thread5, arg) {

  (void)arg;

  chRegSetThreadName("SPI thread");

  char writeCommandAddress[4];
  writeCommandAddress[0] = 0x2;

  chMtxLock(&memoryCounter_mutex);
  memoryCounter = 0;
  chMtxUnlock(&memoryCounter_mutex);
  while (true) {

    void *pbuf;

    /* Waiting for a filled buffer.*/
    msg_t msg = chMBFetch(&filled_buffers, (msg_t *)&pbuf, TIME_INFINITE);
    /* Processing the event.*/
    if (msg == MSG_RESET) {
      /* The mailbox has been reset, this means network failure.*/
      // TODO
    }
    else {
      if ((memoryCounter + 7) >= SPI_RAM_SIZE) {
	      /* Do this as fast as possible */
	      gptStopTimer(&GPTD3);
	      chMtxLock(&SystemState_mutex);
	      SystemState = IDLE;
	      chMtxUnlock(&SystemState_mutex);
	      chprintf((BaseSequentialStream *)&SD1,"Acquisition DONE\r\n");
	      resetAllMailboxes();
	      continue;
      }
      char *message = (char *)pbuf;
      /* Write the address into the command buffer */
      writeCommandAddress[1] = (memoryCounter >> 16) & 0x1;
      writeCommandAddress[2] = (memoryCounter >>  8) & 0xFF;
      writeCommandAddress[3] = (memoryCounter)       & 0xFF;
      spiAcquireBus(&SPID1);              /* Acquire ownership of the bus.    */
      spiStart(&SPID1, &ramSPI);       /* Setup transfer parameters.       */
      spiSelect(&SPID1);                  /* Slave Select assertion.          */
      spiSend(&SPID1, 4, writeCommandAddress);          /* Atomic transfer operations.      */
      spiSend(&SPID1, 7, message);          /* Atomic transfer operations.      */
      spiUnselect(&SPID1);                /* Slave Select de-assertion.       */
      spiReleaseBus(&SPID1);              /* Ownership release.               */
      chMtxLock(&memoryCounter_mutex);
      memoryCounter += 7;
      chMtxUnlock(&memoryCounter_mutex);

      /* Returning the buffer to the free buffers pool.*/
      (void)chMBPost(&free_buffers, (msg_t)pbuf, TIME_INFINITE);

    }
  }

}

/*
 * Write to serial thread.
 * This is activated after the last reading.
 * Fetches the content from the SPI RAM and prints to the serial console
 */
static THD_WORKING_AREA(waThread6, 256);
static THD_FUNCTION(Thread6, arg) {
  (void)arg;
  chRegSetThreadName("UART printer");


  char readCommandAddress[5];
  readCommandAddress[0] = 0x3;
  readCommandAddress[1] = 0;
  readCommandAddress[2] = 0;
  readCommandAddress[3] = 0;
  readCommandAddress[4] = 0; // needed for the exchange

  uint8_t buffer;
  int32_t print_memoryCounter;

  while(true) {
	  sdAsynchronousRead(&SD1, &buffer, 1);
	  if (buffer != 'r') {
		  buffer = 0;
		  chThdSleepMilliseconds(100);
		  continue;
	  }
	  chMtxLock(&SystemState_mutex);
	  SystemState = PRINT;
	  chMtxUnlock(&SystemState_mutex);
	  print_memoryCounter = memoryCounter;
	  /* We will block the SPI device while printing */
	  spiAcquireBus(&SPID1);              /* Acquire ownership of the bus.    */
	  spiStart(&SPID1, &ramSPI);       /* Setup transfer parameters.       */
	  spiSelect(&SPID1);                  /* Slave Select assertion.          */
	  spiExchange(&SPID1, 5, readCommandAddress, rxbuf.c);
	  char id = rxbuf.c[4];
	  while (true) {
		  spiReceive(&SPID1, 6, rxbuf.c);          /* Atomic transfer operations.      */
		  print_memoryCounter -= 7;
		  if (id == 'A') {
			  chprintf((BaseSequentialStream *)&SD1,"%ld:A:%d:%d:%d;\r\n",
				   print_memoryCounter,
				   rxbuf.i[0],
				   rxbuf.i[1],
				   rxbuf.i[2]);
		  }
		  else if (id == 'M') {
			  chprintf((BaseSequentialStream *)&SD1,"%ld:M:%d:%d:%d;\r\n",
				   print_memoryCounter,
				   rxbuf.i[0],
				   rxbuf.i[1],
				   rxbuf.i[2]);
		  } else {
			  chprintf((BaseSequentialStream *)&SD1,"ERROR: Unknown device in memory\r\n");
		  }
		  if (print_memoryCounter <= 0)
			  break;
		  spiReceive(&SPID1, 1, &id);          /* Atomic transfer operations.      */
	  }
	  spiUnselect(&SPID1);                /* Slave Select de-assertion.       */
	  spiReleaseBus(&SPID1);              /* Ownership release.               */
	  chprintf((BaseSequentialStream *)&SD1,"Printing finished");
	  chMtxLock(&SystemState_mutex);
	  SystemState = IDLE;
	  chMtxUnlock(&SystemState_mutex);
  }
}

/*===========================================================================*/
/* Main                                                                      */
/*===========================================================================*/
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();


  /*
   * Activates the serial driver 1 using the driver default configuration.
   */
  sdStart(&SD1, NULL);

  chprintf((BaseSequentialStream *)&SD1,"Starting...\r\n");
  chprintf((BaseSequentialStream *)&SD1, "SYSCLK=%u\r\n", STM32_SYSCLK);

  /*
   * Blink all the LEDs
   */
  blinkAllLeds();

  I2CInit();

  sensorStartup();

  /* Initialize the timer */
  gptStart(&GPTD3, &gpt3cfg);

  /* Create and pre-fill the mailboxes.*/
  createMailboxes();

  /* Init thread queue (unlock accelerometer threads from interruption) */
  osalThreadQueueObjectInit(&threadQueue);

  SystemState = IDLE;

  /* We've finished the initialization */
  blinkAllLeds();

  /* NOTE
   * Differences between the RAM and the EEPROM
   *
   * RAM -> Command + 3 bytes (address)
   * EEPROM -> Command + 1 byte (address)
   *
   * EEPROM needs a WREN before writing (and to leave up CS)!!!
   */

  /*
   * Creates the threads
   */
  chThdCreateStatic(waThread0, sizeof(waThread0), NORMALPRIO+5, Thread0, NULL); // State machine thread
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO+2, Thread1, NULL); // MPU goes always first
  chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO+1, Thread2, NULL); // ADXL
  chThdCreateStatic(waThread5, sizeof(waThread5), NORMALPRIO,   Thread5, NULL); // SPI
  chThdCreateStatic(waThread6, sizeof(waThread6), NORMALPRIO-5, Thread6, NULL); // Serial printer

  /* Enable external (button) interrupts */
  extStart(&EXTD1, &extcfg);

  /*
   * Normal main() thread activity
   */
  while (true) {
    palClearPad(GPIOC, GPIOC_LED_1);
    osalThreadSleepMilliseconds(500);
    palSetPad(GPIOC, GPIOC_LED_1);
    osalThreadSleepMilliseconds(500);
  }
}
