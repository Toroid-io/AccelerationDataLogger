#include "ch.h"
#include "hal.h"
#include "ChibiOS/os/hal/lib/streams/chprintf.h"
#include "MPU6050.h"
#include "ADXL345.h"

/* We use these lines with leds */
#define LED_0 PAL_LINE(GPIOA, 3U)
#define LED_1 PAL_LINE(GPIOA, 2U)
#define LED_2 PAL_LINE(GPIOA, 1U)
#define LED_ONBOARD PAL_LINE(GPIOC, 13U)
/* We use this lines with I2C */
#define I2C1_SCL_PORT GPIOB
#define I2C1_SCL_PIN 6
#define I2C1_SDA_PORT GPIOB
#define I2C1_SDA_PIN 7
//#define I2C2_SCL_PORT GPIOB
//#define I2C2_SCL_PIN 10
//#define I2C2_SDA_PORT GPIOB
//#define I2C2_SDA_PIN 11
/* We use this lines with SPI */
#define GPIOA_SPI1NSS           4
#define SPI_CS_PORT   GPIOA
#define SPI_CS_PIN    4
#define SPI_SCK_PORT  GPIOA
#define SPI_SCK_PIN   5
#define SPI_MISO_PORT GPIOA
#define SPI_MISO_PIN  6
#define SPI_MOSI_PORT GPIOA
#define SPI_MOSI_PIN  7
#define SPI_HOLD_PORT GPIOB
#define SPI_HOLD_PIN  0

/* the mailboxes are used to send information into the spi thread */
#define NUM_BUFFERS 2
#define BUFFERS_SIZE 16
static char buffers[NUM_BUFFERS][BUFFERS_SIZE];
static msg_t free_buffers_queue[NUM_BUFFERS];
static mailbox_t free_buffers;
static msg_t filled_buffers_queue[NUM_BUFFERS];
static mailbox_t filled_buffers;

/* Mutex used to analyze the amount of memory available */
//#define SPI_RAM_SIZE 131072
#define SPI_RAM_SIZE 80*1024
static MUTEX_DECL(memoryCounter_mutex);
volatile int32_t memoryCounter;

/* Various semaphores to synchronize stuff */
static BSEMAPHORE_DECL(accel_blink, 1);
static BSEMAPHORE_DECL(i2c_mpu, 1);
static BSEMAPHORE_DECL(i2c_adxl, 1);
static BSEMAPHORE_DECL(writeSerial, 1);
static SEMAPHORE_DECL(i2c_sync, 0);

/*
 * I2C config
 *
 */
static const I2CConfig i2cfg1 = {
	OPMODE_I2C,
	200000,
	//100000,
	//FAST_DUTY_CYCLE_16_9,
	FAST_DUTY_CYCLE_2,
	//STD_DUTY_CYCLE,
};

static const I2CConfig i2cfg2 = {
	OPMODE_I2C,
	//100000,
	200000,
	//FAST_DUTY_CYCLE_16_9,
	FAST_DUTY_CYCLE_2,
	//STD_DUTY_CYCLE,
};

/*
 * Low speed SPI configuration (281.250kHz, CPHA=0, CPOL=0, MSb first).
 */
static const SPIConfig ls_spicfg = {
  NULL,
  GPIOA,
  GPIOA_SPI1NSS,
  SPI_CR1_BR_2 | SPI_CR1_BR_1
};

/*
 * SPI RX buffer
 */
static union Rxbuf {
	uint8_t c[32];
	int16_t i[16];
} rxbuf;

/*
 * Timer config
 */
static void gpt3cb(GPTDriver *gptp) {
	(void)gptp;
	chBSemSignal(&i2c_mpu);
	chBSemSignal(&i2c_adxl);
}

static const GPTConfig gpt3cfg = {
  1000000, // 1MHz timer clock.
  gpt3cb,
  0, 0
};

/*
 * MPU6050  polling thread.
 */
static THD_WORKING_AREA(waThread1, 256);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  void *pbuf;

  chRegSetThreadName("MPU6050_Poll");

  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  int16_t i;

 // MPU6050_reset();
  MPU6050_initialize();
  MPU6050_setSleepEnabled(false);
  chThdSleepMilliseconds(100);
  // verify connection
  while (MPU6050_testConnection() == false) {
    chprintf((BaseSequentialStream *)&SD1,"MPU FAIL\r\n");
    chThdSleepMilliseconds(1000);
  }
  chprintf((BaseSequentialStream *)&SD1,"MPU OK\r\n");

  /* Calibration loop */
  int32_t calibrationAccumulator[6] = {0, 0, 0, 0, 0, 0};
  for (i = 0; i < 128; ++i) {
    chThdSleepMilliseconds(10);
    MPU6050_getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    calibrationAccumulator[0] += ax;
    calibrationAccumulator[1] += ay;
    calibrationAccumulator[2] += az;
    calibrationAccumulator[3] += gx;
    calibrationAccumulator[4] += gy;
    calibrationAccumulator[5] += gz;
  }

  int16_t cax, cay, caz, cgx, cgy, cgz;
  cax = calibrationAccumulator[0] / 128;
  cay = calibrationAccumulator[1] / 128;
  caz = calibrationAccumulator[2] / 128;
  cgx = calibrationAccumulator[3] / 128;
  cgy = calibrationAccumulator[4] / 128;
  cgz = calibrationAccumulator[5] / 128;

  /* We finished the calibration here */
  chprintf((BaseSequentialStream *)&SD1,"MPU Calibrated\r\n");
  chSemSignal(&i2c_sync);

  while (true) {
    /* wait until the timer starts */
    chBSemWait(&i2c_mpu);
    MPU6050_getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    ax -= cax;
    ay -= cay;
    az -= caz;
    gx -= cgx;
    gy -= cgy;
    gz -= cgz;
    if (chMBFetch(&free_buffers, (msg_t *)&pbuf, TIME_INFINITE) == MSG_OK) {
      char *message = (char *)pbuf;
      message[0]  = 'M';
      message[1]  = (char)((ax) & 0xFF);
      message[2]  = (char)((ax >> 8) & 0xFF);
      message[3]  = (char)((ay) & 0xFF);
      message[4]  = (char)((ay >> 8) & 0xFF);
      message[5]  = (char)((az) & 0xFF);
      message[6]  = (char)((az >> 8) & 0xFF);
      message[7]  = (char)((gx) & 0xFF);
      message[8]  = (char)((gx >> 8) & 0xFF);
      message[9] = (char)((gy) & 0xFF);
      message[10]  = (char)((gy >> 8) & 0xFF);
      message[11] = (char)((gz) & 0xFF);
      message[12] = (char)((gz >> 8) & 0xFF);
      //chsnprintf((char *)pbuf, BUFFERS_SIZE, "A:%u:%d\r\n", counter++, ax);
      (void)chMBPost(&filled_buffers, (msg_t)pbuf, TIME_INFINITE);
    }
    if ((az > 16384) || (az < -16384)) {
      chBSemSignal(&accel_blink);
    }
  }
}

/*
 * ADXL345  polling thread.
 */
static THD_WORKING_AREA(waThread2, 128);
static THD_FUNCTION(Thread2, arg) {

  (void)arg;
  void *pbuf;

  chRegSetThreadName("ADXL345_Poll");

  int16_t ax, ay, az;
  int16_t cax, cay, caz;
  uint16_t i;

  ADXL345_initialize();
  // verify connection
  while (ADXL345_testConnection() == false) {
    chprintf((BaseSequentialStream *)&SD1,"ADXL FAIL\r\n");
    chThdSleepMilliseconds(1000);
  }
  chprintf((BaseSequentialStream *)&SD1,"ADXL OK\r\n");

  /* Calibration loop */
  int32_t calibrationAccumulator[3] = {0, 0, 0};
  for (i = 0; i < 128; ++i) {
    chThdSleepMilliseconds(2);
    ADXL345_getAcceleration(&ax, &ay, &az);
    calibrationAccumulator[0] += ax;
    calibrationAccumulator[1] += ay;
    calibrationAccumulator[2] += az;
  }

  cax = calibrationAccumulator[0] / 128;
  cay = calibrationAccumulator[1] / 128;
  caz = calibrationAccumulator[2] / 128;

  /* We finished the calibration here*/
  chprintf((BaseSequentialStream *)&SD1,"ADXL Calibrated\r\n");
  chSemSignal(&i2c_sync);

  // read raw accel measurements from device
  i = 0;
  while (true) {
    /* wait until the timer says */
    palClearLine(LED_1);
    chBSemWait(&i2c_adxl);
    palSetLine(LED_1);
    ADXL345_getAcceleration(&ax, &ay, &az);
    ax -= cax;
    ay -= cay;
    az -= caz;

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

    if ((az > 250) || (az < -250)) {
      chBSemSignal(&accel_blink);
    }
  }
}

/*
 * Accelerometer blink thread
 */
static THD_WORKING_AREA(waThread3, 128);
static THD_FUNCTION(Thread3, arg) {

  (void)arg;
  chRegSetThreadName("reblinker");
  palSetLineMode(LED_0, PAL_MODE_OUTPUT_OPENDRAIN);
  palSetLine(LED_0);

  while (true) {
    chBSemWait(&accel_blink);
    palClearLine(LED_0);
    chThdSleepMilliseconds(1);
    palSetLine(LED_0);
    chThdSleepMilliseconds(1);
  }
}

static THD_WORKING_AREA(waThread4, 128);
static THD_FUNCTION(Thread4, arg) {
  (void)arg;
  uint8_t i;

  chRegSetThreadName("Synchronization Thread");

  palSetLineMode(LED_1, PAL_MODE_OUTPUT_OPENDRAIN);
  palSetLine(LED_1);

  /* Wait until both accelerometers are configured and calibrated */
  chSemWait(&i2c_sync);
  chSemWait(&i2c_sync);

  for (i = 0; i < 4; ++i) {
	  palClearLine(LED_1);
	  chThdSleepMilliseconds(500);
	  palSetLine(LED_1);
	  chThdSleepMilliseconds(500);
  }

  chprintf((BaseSequentialStream *)&SD1,"Acquiring...\r\n");

  /* Start the timer */
  gptStart(&GPTD3, &gpt3cfg);
  gptStartContinuous(&GPTD3, 1250); // Timer interrupt at 800Hz
}


/*
 * SPI bus thread
 */
static THD_WORKING_AREA(waThread5, 256);
static THD_FUNCTION(Thread5, arg) {

  (void)arg;

  chRegSetThreadName("SPI thread");
  palSetLineMode(LED_2, PAL_MODE_OUTPUT_OPENDRAIN);

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
      char *message = (char *)pbuf;
      /* Write the address into the command buffer */
      writeCommandAddress[1] = (memoryCounter >> 16) & 0x1;
      writeCommandAddress[2] = (memoryCounter >>  8) & 0xFF;
      writeCommandAddress[3] = (memoryCounter)       & 0xFF;
      if (message[0] == 'A') {
        if ((memoryCounter + 7) >= SPI_RAM_SIZE)
		break;
	spiAcquireBus(&SPID1);              /* Acquire ownership of the bus.    */
	spiStart(&SPID1, &ls_spicfg);       /* Setup transfer parameters.       */
	spiSelect(&SPID1);                  /* Slave Select assertion.          */
        spiSend(&SPID1, 4, writeCommandAddress);          /* Atomic transfer operations.      */
        spiSend(&SPID1, 7, message);          /* Atomic transfer operations.      */
	spiUnselect(&SPID1);                /* Slave Select de-assertion.       */
	spiReleaseBus(&SPID1);              /* Ownership release.               */
        chMtxLock(&memoryCounter_mutex);
	memoryCounter += 7;
	chMtxUnlock(&memoryCounter_mutex);
      } else if (message[0] == 'M') {
        if ((memoryCounter + 13) >= SPI_RAM_SIZE)
		break;
	spiAcquireBus(&SPID1);              /* Acquire ownership of the bus.    */
	spiStart(&SPID1, &ls_spicfg);       /* Setup transfer parameters.       */
	spiSelect(&SPID1);                  /* Slave Select assertion.          */
        spiSend(&SPID1, 4, writeCommandAddress);          /* Atomic transfer operations.      */
        spiSend(&SPID1, 13, message);          /* Atomic transfer operations.      */
	spiUnselect(&SPID1);                /* Slave Select de-assertion.       */
	spiReleaseBus(&SPID1);              /* Ownership release.               */
        chMtxLock(&memoryCounter_mutex);
	memoryCounter += 13;
	chMtxUnlock(&memoryCounter_mutex);
      } else {
        chprintf((BaseSequentialStream *)&SD1,"ERROR: Unknown sensor\r\n");
      }

      /* Returning the buffer to the free buffers pool.*/
      (void)chMBPost(&free_buffers, (msg_t)pbuf, TIME_INFINITE);

    }
  }

  /* After finishing acquitision, go here
   * I2C threads will hang after a while because they will run out of mailboxes
   */
  palSetLine(LED_2);
  //chThdSleepMilliseconds(5000);
  chBSemSignal(&writeSerial);
  while (true) {
	  palSetLine(LED_2);
	  chThdSleepMilliseconds(500);
	  palClearLine(LED_2);
	  chThdSleepMilliseconds(500);
  }
}

static THD_WORKING_AREA(waThread6, 256);
static THD_FUNCTION(Thread6, arg) {
  (void)arg;
  chRegSetThreadName("UART printer");

  palSetLineMode(LED_0, PAL_MODE_OUTPUT_OPENDRAIN);
  chBSemWait(&writeSerial);

  char readCommandAddress[5];
  readCommandAddress[0] = 0x3;
  readCommandAddress[1] = 0;
  readCommandAddress[2] = 0;
  readCommandAddress[3] = 0;
  readCommandAddress[3] = 0; // Just in case

  /* We will block the SPI device while printing */
  spiAcquireBus(&SPID1);              /* Acquire ownership of the bus.    */
  spiStart(&SPID1, &ls_spicfg);       /* Setup transfer parameters.       */
  spiSelect(&SPID1);                  /* Slave Select assertion.          */
  spiExchange(&SPID1, 5, readCommandAddress, rxbuf.c);
  char id = rxbuf.c[4];
  while (true) {
      if (id == 'A') {
        spiReceive(&SPID1, 6, rxbuf.c);          /* Atomic transfer operations.      */
        chMtxLock(&memoryCounter_mutex);
	memoryCounter -= 7;
	chMtxUnlock(&memoryCounter_mutex);
	chprintf((BaseSequentialStream *)&SD1,"%ld:A:%d:%d:%d;\r\n",
		 memoryCounter,
		 rxbuf.i[0],
		 rxbuf.i[1],
		 rxbuf.i[2]);
      }
      else if (id == 'M') {
        spiReceive(&SPID1, 12, rxbuf.c);          /* Atomic transfer operations.      */
        chMtxLock(&memoryCounter_mutex);
	memoryCounter -= 13;
	chMtxUnlock(&memoryCounter_mutex);
	chprintf((BaseSequentialStream *)&SD1,"%ld:M:%d:%d:%d:%d:%d:%d;\r\n",
		 memoryCounter,
		 rxbuf.i[0],
		 rxbuf.i[1],
		 rxbuf.i[2],
		 rxbuf.i[3],
		 rxbuf.i[4],
		 rxbuf.i[5]);
      } else {
	chprintf((BaseSequentialStream *)&SD1,"ERROR: Unknown device in memory\r\n");
      }
      if (memoryCounter <= 0)
	      break;
      spiReceive(&SPID1, 1, &id);          /* Atomic transfer operations.      */
  }
  spiUnselect(&SPID1);                /* Slave Select de-assertion.       */
  spiReleaseBus(&SPID1);              /* Ownership release.               */

  chprintf((BaseSequentialStream *)&SD1,"We're all done\r\n");
  while (true) {
    palClearLine(LED_0);
    chThdSleepMilliseconds(500);
    palSetLine(LED_0);
    chThdSleepMilliseconds(500);
  }
}

/*
 * Write to serial thread.
 * This is activated after the last reading.
 * Fetches the content from the SPI RAM and prints to the serial console
 */

/*
 * LED blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waThread7, 128);
static THD_FUNCTION(Thread7, arg) {
  (void)arg;
  chRegSetThreadName("blinker");

  palSetLineMode(LED_ONBOARD, PAL_MODE_OUTPUT_OPENDRAIN);

  while (true) {
    palClearLine(LED_ONBOARD);
    chThdSleepMilliseconds(500);
    palSetLine(LED_ONBOARD);
    chThdSleepMilliseconds(500);
  }
}


/*
 * Application entry point.
 */
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
  palSetPadMode(GPIOA, 9, PAL_MODE_STM32_ALTERNATE_PUSHPULL);   /* SCL */
  palSetPadMode(GPIOA, 10, PAL_MODE_STM32_ALTERNATE_PUSHPULL);   /* SCL */

  osalThreadSleepMilliseconds(500);

  /*
   * I2C I/O pins setup.
   */
  palSetPadMode(I2C1_SCL_PORT, I2C1_SCL_PIN, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);   /* SCL */
  palSetPadMode(I2C1_SDA_PORT, I2C1_SDA_PIN, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);   /* SDA */
  i2cStart(&I2CD1, &i2cfg1);
  //palSetPadMode(I2C2_SCL_PORT, I2C2_SCL_PIN, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);   /* SCL */
  //palSetPadMode(I2C2_SDA_PORT, I2C2_SDA_PIN, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);   /* SDA */
  //i2cStart(&I2CD2, &i2cfg2);

  /*
   * SPI1 I/O pins setup.
   */
  palSetPadMode(IOPORT1, 5, PAL_MODE_STM32_ALTERNATE_PUSHPULL);     /* SCK. */
  palSetPadMode(IOPORT1, 6, PAL_MODE_STM32_ALTERNATE_PUSHPULL);     /* MISO.*/
  palSetPadMode(IOPORT1, 7, PAL_MODE_STM32_ALTERNATE_PUSHPULL);     /* MOSI.*/
  palSetPadMode(IOPORT1, GPIOA_SPI1NSS, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPad(IOPORT1, GPIOA_SPI1NSS);

  /* Create and pre-fill the mailboxes.*/
  chMBObjectInit(&filled_buffers, filled_buffers_queue, NUM_BUFFERS);
  chMBObjectInit(&free_buffers, free_buffers_queue, NUM_BUFFERS);
  uint8_t i;
  for (i = 0; i < NUM_BUFFERS; i++)
    (void)chMBPost(&free_buffers, (msg_t)&buffers[i], TIME_INFINITE);

  osalThreadSleepMilliseconds(5000);
  /*
   * Creates the threads
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO+1, Thread1, NULL); // MPU
  chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO+1, Thread2, NULL); // ADXL
  chThdCreateStatic(waThread3, sizeof(waThread3), NORMALPRIO-1, Thread3, NULL); // Accel blink
  chThdCreateStatic(waThread4, sizeof(waThread4), NORMALPRIO+2, Thread4, NULL); // Synchronization Thread
  chThdCreateStatic(waThread5, sizeof(waThread5), NORMALPRIO,   Thread5, NULL); // SPI
  chThdCreateStatic(waThread6, sizeof(waThread6), NORMALPRIO-5, Thread6, NULL); // Serial printer
  chThdCreateStatic(waThread7, sizeof(waThread7), NORMALPRIO-10,   Thread7, NULL); // Blink

  /*
   * Normal main() thread activity
   */
  while (true) {
    osalThreadSleepMilliseconds(500);
  }
}
