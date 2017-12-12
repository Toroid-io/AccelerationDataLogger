/* TODO
 * - Config variable sensors range (EEPROM config)
 * - Fix UART unhandled exception when system is not compiled with debug options
 */
#include "ch.h"
#include "hal.h"
#include "ChibiOS/os/hal/lib/streams/chprintf.h"
#include <string.h>
#include "MEMS/MPU6050.h"
#include "MEMS/ADXL345.h"
#include "EEPROM/StaticSystemConfig.h"


/*===========================================================================*/
/* Global variables - RTOS stuff                                             */
/*===========================================================================*/

volatile enum state {IDLE,
	CALIBRATION,
	START_ACQUISITION,
	ACQUISITION,
	DOWNLOAD_CONFIG,
	SAVE_CONFIG,
	PRINT_DATA,
	PRINT_CONFIG} systemState;
static MUTEX_DECL(systemState_mutex);

struct configStructure sysConf;

/* the mailboxes are used to send information into the spi thread */
#define NUM_BUFFERS 4
#define BUFFERS_SIZE 8
static char buffers[NUM_BUFFERS][BUFFERS_SIZE];
static msg_t free_buffers_queue[NUM_BUFFERS];
static mailbox_t free_buffers;
static msg_t filled_buffers_queue[NUM_BUFFERS];
static mailbox_t filled_buffers;

/* recvBuffer is used to store data coming from the UART (configs) */
uint8_t recvBuffer[32];

#define SPI_RAM_SIZE 128*1024
/* Mutex used to analyze the amount of memory available */
static MUTEX_DECL(memoryCounter_mutex);
volatile int32_t memoryCounter;

/* Various semaphores to synchronize stuff */
static BSEMAPHORE_DECL(printMutex, 1);
static threads_queue_t threadQueue;

/* ADC variables used to test USB, Battery voltage, VRefInt, Temp */
#define ADC_GRP_NUM_CHANNELS   2
#define ADC_GRP_BUF_DEPTH      128
static adcsample_t samples[ADC_GRP_NUM_CHANNELS * ADC_GRP_BUF_DEPTH];

/*===========================================================================*/
/* Callbacks                                                                 */
/*===========================================================================*/

/* Triggered when the calibration button is released.*/
static void extcb1(EXTDriver *extp, expchannel_t channel) {
  (void)extp;
  (void)channel;
  chSysLockFromISR();
  if (systemState == IDLE)
	  systemState = CALIBRATION;
  chSysUnlockFromISR();
}

/* Triggered when the acquisition button is released.*/
static void extcb2(EXTDriver *extp, expchannel_t channel) {
  (void)extp;
  (void)channel;
  chSysLockFromISR();
  if (systemState == IDLE)
	  systemState = START_ACQUISITION;
  chSysUnlockFromISR();
}

static void led2off(void *arg);
static void gpt3cb(GPTDriver *gptp) {
	(void)gptp;
	static virtual_timer_t vt;
	palClearPad(GPIOC, GPIOC_LED_2);
	chSysLockFromISR();
	chThdDequeueAllI(&threadQueue, 0);
	/* LED2 set to OFF after 800uS.*/
	chVTResetI(&vt);
	chVTSetI(&vt, US2ST(800), led2off, NULL);
	chSysUnlockFromISR();
}

/*
 * This callback is invoked when a transmission buffer has been completely
 * read by the driver.
 */
static void txend1(UARTDriver *uartp) {
  (void)uartp;
  //palSetPad(GPIOC, GPIOC_LED4);
}

/*
 * This callback is invoked when a transmission has physically completed.
 */
static void led3off(void *arg);
static void txend2(UARTDriver *uartp) {
  (void)uartp;
  static virtual_timer_t vt3;
  palClearPad(GPIOB, GPIOB_LED_3);
  chSysLockFromISR();
  chVTResetI(&vt3);
  chVTSetI(&vt3, US2ST(200), led3off, NULL);
  chSysUnlockFromISR();
}

/*
 * This callback is invoked on a receive error, the errors mask is passed
 * as parameter.
 */
static void rxerr(UARTDriver *uartp, uartflags_t e) {
  (void)uartp;
  (void)e;
}

/*
 * This callback is invoked when a character is received but the application
 * was not ready to receive it, the character is passed as parameter.
 */
static void rxchar(UARTDriver *uartp, uint16_t c) {
  (void)uartp;
  (void)c;
  if (systemState != IDLE)
	  return;
  switch (c) {
  case 'r':
  case 'R':
	  systemState = PRINT_DATA;
	  chSysLockFromISR();
	  chBSemSignalI(&printMutex);
	  chSysUnlockFromISR();
	  break;
  case 'g':
  case 'G':
	  systemState = PRINT_CONFIG;
	  chSysLockFromISR();
	  chBSemSignalI(&printMutex);
	  chSysUnlockFromISR();
	  break;
  case 'c':
  case 'C':
	  systemState = DOWNLOAD_CONFIG;
	  chSysLockFromISR();
	  uartStartSendI(&UARTD1, 15, "--Send Config--");
	  uartStartReceiveI(&UARTD1,
			    sizeof(struct configStructure),
			    (void *)recvBuffer);
	  chSysUnlockFromISR();
	  break;
  default:
	  break;

  }
}

/*
 * This callback is invoked when a receive buffer has been completely written.
 */
static void rxend(UARTDriver *uartp) {
  (void)uartp;
  if (systemState != DOWNLOAD_CONFIG)
	  osalSysHalt("UART rx was not expected");
  if(checkSystemConfig((struct configStructure *)recvBuffer)) {
	  chSysLockFromISR();
	  uartStartSendI(&UARTD1, 16, "--Config ERROR--");
	  chSysUnlockFromISR();
	  systemState = IDLE;
	  return;
  }
  chSysLockFromISR();
  uartStartSendI(&UARTD1, 13, "--Config OK--");
  chSysUnlockFromISR();
  memcpy(&sysConf, recvBuffer, sizeof(struct configStructure));
  systemState = SAVE_CONFIG;
}

/*
 * This callback is invoked when configured timeout reached.
 */
static void rxtimeout(UARTDriver *uartp) {
  (void)uartp;
  if (systemState != DOWNLOAD_CONFIG)
	  /* We can get here because commands triggered this interrupt */
	  return;
  chSysLockFromISR();
  uartStopReceiveI(&UARTD1);
  uartStartSendI(&UARTD1, 18, "--Config TIMEOUT--");
  chSysUnlockFromISR();
  systemState = IDLE;
}

/*===========================================================================*/
/* Peripherals config structures                                             */
/*===========================================================================*/

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
static const SPIConfig eepromSPI = {
	NULL,
	GPIOA,
	GPIOA_SPI_CS_1,
	SPI_CR1_BR_2 | SPI_CR1_BR_1,
	SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0
};

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

static const ADCConversionGroup adcgrpcfg = {
  TRUE,
  ADC_GRP_NUM_CHANNELS,
  NULL,
  NULL,
  ADC_CFGR1_CONT | ADC_CFGR1_RES_12BIT,             /* CFGR1 */
  ADC_TR(0, 0),                                     /* TR */
  ADC_SMPR_SMP_239P5,                               /* SMPR */
  ADC_CHSELR_CHSEL1 | ADC_CHSELR_CHSEL9
};

static UARTConfig uart_cfg_1 = {
  txend1,
  txend2,
  rxend,
  rxchar,
  rxerr,
  rxtimeout,
  115200*3,
  115200,
  USART_CR1_RTOIE,
  USART_CR2_LINEN | USART_CR2_RTOEN,
  0
};

/*===========================================================================*/
/* Functions                                                                 */
/*===========================================================================*/
void modifyAxis(int16_t *ax, int16_t *ay, int16_t *az)
{
	uint16_t tmp;
	*az= -*az;
	tmp = *ax;
	*ax = -*ay;
	*ay = -tmp;
}

void saturateInt16(int32_t *atmp, int16_t *acc)
{
	for (uint8_t i = 0; i < 3; ++i) {
		if (atmp[i] > INT16_MAX)
			acc[i] = INT16_MAX;
		else if (atmp[i] < INT16_MIN)
			acc[i] = INT16_MIN;
		else
			acc[i] = (int16_t)atmp[i];
	}
}

void exponentialFilter(int32_t *atmp, int32_t *g, int16_t *acc)
{
	/* Numerator and denominator values of the exponential filter */
	/* The calculation of the numerator gives a filter time-constant of 75 ms */
	int32_t num, den;
	den = 1000;
	num = (7500 * den) / (7500 + 100000 / sysConf.samplingSpeed);
	for (uint8_t i = 0; i < 3; ++i) {
	    g[i] = (g[i] * num + (den - num) * acc[i]) / den;
	    atmp[i] = acc[i] - g[i];
	}
	saturateInt16(atmp, acc);
}

static void led2off(void *arg) {
  (void)arg;
  palSetPad(GPIOC, GPIOC_LED_2);
}

static void led3off(void *arg) {
  (void)arg;
  palSetPad(GPIOB, GPIOB_LED_3);
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
	 * Check if the I2C is clogged and send some SCK pulses to release it
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

  sysConf.calibrationMPU[0] = calibrationAccMPU[0] / 128;
  sysConf.calibrationMPU[1] = calibrationAccMPU[1] / 128;
  sysConf.calibrationMPU[2] = calibrationAccMPU[2] / 128;

  sysConf.calibrationADXL[0] = calibrationAccADXL[0] / 128;
  sysConf.calibrationADXL[1] = calibrationAccADXL[1] / 128;
  sysConf.calibrationADXL[2] = calibrationAccADXL[2] / 128;

  saveSystemConfigEEPROM(&eepromSPI, &sysConf);

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
		chprintf((BaseSequentialStream *)&SD2,"MPU FAIL\r\n");
		chThdSleepMilliseconds(1000);
	}
	chprintf((BaseSequentialStream *)&SD2,"MPU OK\r\n");

	/* Initialize ADXL345 */
	ADXL345_initialize();
	chThdSleepMilliseconds(100);
	// verify connection
	while (!ADXL345_testConnection()) {
		chprintf((BaseSequentialStream *)&SD2,"ADXL FAIL\r\n");
		chThdSleepMilliseconds(1000);
	}
	chprintf((BaseSequentialStream *)&SD2,"ADXL OK\r\n");
}

void getVoltages(uint16_t *vusb, uint16_t *vbat)
{
	uint32_t usbtmp = 0;
	uint32_t battmp = 0;
	/* Samples are alternating: vbat, vusb, vbat... */
	for(uint8_t i = 0; i < ADC_GRP_BUF_DEPTH; ++i) {
		battmp += samples[ADC_GRP_NUM_CHANNELS*i];
		usbtmp += samples[ADC_GRP_NUM_CHANNELS*i + 1];
	}
	battmp /= ADC_GRP_BUF_DEPTH;
	usbtmp /= ADC_GRP_BUF_DEPTH;

	/* Reference voltage is corrected to handle resistor offset
	 * (instead of 3300). Voltages are in mV.
	 */
	battmp = (battmp * 3394 * 57) / (4096 * 10);
	usbtmp = (usbtmp * 3327 * 86) / (4096 * 47);

	*vusb = (uint16_t)usbtmp;
	*vbat = (uint16_t)battmp;

	////chprintf((BaseSequentialStream *)&SD2,"USB: %u - BAT: %u\r\n",
	//	 *vusb, *vbat);
	return;
}

uint8_t checkVoltageLevel(void)
{
	uint16_t vusb, vbat;
	getVoltages(&vusb, &vbat);
	/* When running with USB voltage should not be a problem */
	if (vusb > 4000)
		return 0;
	  /* When running on batteries, 6.8V seems a logic voltage threshold */
	if (vbat < 6800) {
		return 1;
	}
	return 0;
}


/*===========================================================================*/
/* Threads                                                                   */
/*===========================================================================*/
/*
 * State machine thread
 */
static THD_WORKING_AREA(waThread0, 0x200);
static THD_FUNCTION(Thread0, arg) {
	(void)arg;
	uint32_t timerPeriod;
	while (true) {
		switch(systemState) {
		case IDLE:
		default:
			break;
		case CALIBRATION:
			chprintf((BaseSequentialStream *)&SD2,"Calibration...\r\n");
			for (uint8_t i = 0; i < 10*sysConf.calibrationDelay; i++) {
				palClearPad(GPIOC, GPIOC_LED_2);
				chThdSleepMilliseconds(50);
				palSetPad(GPIOC, GPIOC_LED_2);
				chThdSleepMilliseconds(50);

			}
			calibrateSensors();
			chprintf((BaseSequentialStream *)&SD2,"Calibration DONE\r\n");
			chMtxLock(&systemState_mutex);
			systemState = IDLE;
			chMtxUnlock(&systemState_mutex);
			break;
		case START_ACQUISITION:
			chprintf((BaseSequentialStream *)&SD2,"Acquisition...\r\n");
			for (uint8_t i = 0; i < 10*sysConf.acquisitionDelay; i++) {
				palClearPad(GPIOC, GPIOC_LED_2);
				chThdSleepMilliseconds(50);
				palSetPad(GPIOC, GPIOC_LED_2);
				chThdSleepMilliseconds(50);

			}
			chMtxLock(&memoryCounter_mutex);
			memoryCounter = 0;
			chMtxUnlock(&memoryCounter_mutex);
			chMtxLock(&systemState_mutex);
			systemState = ACQUISITION;
			chMtxUnlock(&systemState_mutex);
			/* Calculate the new timer period */
			timerPeriod = 100000/sysConf.samplingSpeed - 1;
			chprintf((BaseSequentialStream *)&SD2,"Timer period %u\r\n", (uint16_t)timerPeriod);
			/* Start the timer */
			gptStartContinuous(&GPTD3, (uint16_t) timerPeriod);
			break;
		case ACQUISITION:
			/* Nothing to do here
			 * State changement will be done in the RAM write task
			 */
			break;
		case DOWNLOAD_CONFIG:
			/* Nothing to do here
			 * State changement will be done in the UART callbacks
			 */
			break;
		case SAVE_CONFIG:
			/* We have a new config, save it in the EEPROM */
			saveSystemConfigEEPROM(&eepromSPI, &sysConf);
			/* Discard all measurements */
			chMtxLock(&memoryCounter_mutex);
			memoryCounter = 0;
			chMtxUnlock(&memoryCounter_mutex);
			chMtxLock(&systemState_mutex);
			systemState = IDLE;
			chMtxUnlock(&systemState_mutex);
			break;
		case PRINT_DATA:
			/* Nothing to do here
			 * Mutex will be released in ISR callback
			 * State will change in UART write task
			 */
			break;
		case PRINT_CONFIG:
			/* Nothing to do here
			 * Mutex will be released in ISR callback
			 * State will change in UART write task
			 */
			break;
		}
		chThdSleepMilliseconds(100);
	}
}
/*
 * MPU6050  polling thread.
 */
static THD_WORKING_AREA(waThread1, 0x200);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  void *pbuf;

  chRegSetThreadName("MPU6050_Poll");

  int16_t acc[3];
  int32_t atmp[3];
  int32_t g[3] = {0, 0, 0};

  chprintf((BaseSequentialStream *)&SD2,"MPU Thread\r\n");

  while (true) {
    /* wait until the timer starts */
    chSysLock();
    chThdEnqueueTimeoutS(&threadQueue, TIME_INFINITE);
    chSysUnlock();

    MPU6050_getAcceleration(acc, acc + 1, acc + 2);

    switch(sysConf.filterType) {
    case  0:
	    /* Do nothing, send the accelerations as they are sampled */
	    break;
    case 1:
	    atmp[0] = acc[0] - sysConf.calibrationMPU[0];
	    atmp[1] = acc[1] - sysConf.calibrationMPU[1];
	    atmp[2] = acc[2] - sysConf.calibrationMPU[2];

	    saturateInt16(atmp, acc);
	    break;
    case 2:
	    exponentialFilter(atmp, g, acc);
	    break;
    default:
	    osalSysHalt("Invalid filter type in MPU Thread");
	    break;
    }

    if (chMBFetch(&free_buffers, (msg_t *)&pbuf, TIME_INFINITE) == MSG_OK) {
      char *message = (char *)pbuf;
      message[0]  = 'M';
      message[1]  = (char)((acc[0]) & 0xFF);
      message[2]  = (char)((acc[0] >> 8) & 0xFF);
      message[3]  = (char)((acc[1]) & 0xFF);
      message[4]  = (char)((acc[1] >> 8) & 0xFF);
      message[5]  = (char)((acc[2]) & 0xFF);
      message[6]  = (char)((acc[2] >> 8) & 0xFF);
      (void)chMBPost(&filled_buffers, (msg_t)pbuf, TIME_INFINITE);
    }
  }
}


/*
 * ADXL345  polling thread.
 */
static THD_WORKING_AREA(waThread2, 0x200);
static THD_FUNCTION(Thread2, arg) {

  (void)arg;
  void *pbuf;

  chRegSetThreadName("ADXL345_Poll");

  int16_t acc[3];
  int32_t atmp[3];
  int32_t g[3] = {0, 0, 0};

  chprintf((BaseSequentialStream *)&SD2,"ADXL Thread\r\n");

  while (true) {
    /* wait until the timer says */
    chSysLock();
    chThdEnqueueTimeoutS(&threadQueue, TIME_INFINITE);
    chSysUnlock();

    ADXL345_getAcceleration(acc, acc + 1, acc + 2);
    modifyAxis(acc, acc + 1, acc + 2);

    switch(sysConf.filterType) {
    case 0:
	    /* Do nothing, send the accelerations as they are sampled */
	    break;
    case 1:
	    atmp[0]  = acc[0] - sysConf.calibrationADXL[0];
	    atmp[1]  = acc[1] - sysConf.calibrationADXL[1];
	    atmp[2]  = acc[2] - sysConf.calibrationADXL[2];

	    saturateInt16(atmp , acc);
	    break;
    case 2:
	    exponentialFilter(atmp, g, acc);
	    break;
    default:
	    osalSysHalt("Invalid filter type in ADXL Thread");
	    break;
    }

    if (chMBFetch(&free_buffers, (msg_t *)&pbuf, TIME_INFINITE) == MSG_OK) {
      char *message = (char *)pbuf;
      message[0]  = 'A';
      message[1]  = (char)((acc[0]) & 0xFF);
      message[2]  = (char)((acc[0] >> 8) & 0xFF);
      message[3]  = (char)((acc[1]) & 0xFF);
      message[4]  = (char)((acc[1] >> 8) & 0xFF);
      message[5]  = (char)((acc[2]) & 0xFF);
      message[6]  = (char)((acc[2] >> 8) & 0xFF);
      (void)chMBPost(&filled_buffers, (msg_t)pbuf, TIME_INFINITE);
    }
  }
}

/*
 * SPI bus thread
 */
static THD_WORKING_AREA(waThread3, 0x200);
static THD_FUNCTION(Thread3, arg) {

  (void)arg;

  chRegSetThreadName("SPI thread");

  char writeCommandAddress[4];
  writeCommandAddress[0] = 0x2;
  writeCommandAddress[1] = 0;
  writeCommandAddress[2] = 0;
  writeCommandAddress[3] = 0;

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
	      chMtxLock(&systemState_mutex);
	      systemState = IDLE;
	      chMtxUnlock(&systemState_mutex);
	      chprintf((BaseSequentialStream *)&SD2,"Acquisition DONE\r\n");
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
 * Write to UART thread.
 * This is activated after the last reading.
 * Fetches the content from the SPI RAM and prints to the serial console
 */
static THD_WORKING_AREA(waThread4, 0x200);
static THD_FUNCTION(Thread4, arg) {
  (void)arg;
  chRegSetThreadName("UART data printer");

  uint8_t rxbuf[8];

  char readCommandAddress[5];
  readCommandAddress[0] = 0x3;
  readCommandAddress[1] = 0;
  readCommandAddress[2] = 0;
  readCommandAddress[3] = 0;

  int32_t print_memoryCounter;
  //systime_t oldt;
  while (true) {

	chBSemWait(&printMutex);

	if (systemState == PRINT_CONFIG) {
		uartAcquireBus(&UARTD1);
		size_t structSize = sizeof(struct configStructure);
		uartSendTimeout(&UARTD1, &structSize, &sysConf, TIME_INFINITE);
		uartReleaseBus(&UARTD1);
	        chMtxLock(&systemState_mutex);
	        systemState = IDLE;
	        chMtxUnlock(&systemState_mutex);
	} else if (systemState == PRINT_DATA) {
	        /* measure execution time */
	        //oldt = chVTGetSystemTimeX();

	        print_memoryCounter = memoryCounter;
		if (print_memoryCounter == 0) {
			char * noDataMsg = "--NO DATA--";
			uartAcquireBus(&UARTD1);
			uartStartSend(&UARTD1, 11, noDataMsg);
			uartReleaseBus(&UARTD1);
		}
	        /* We will block the SPI and UART devices while printing */
		uartAcquireBus(&UARTD1);
	        spiAcquireBus(&SPID1);
	        spiStart(&SPID1, &ramSPI);
	        spiSelect(&SPID1);
	        spiSend(&SPID1, 4, readCommandAddress);
	        while (print_memoryCounter > 0) {
			spiReceive(&SPID1, 7, rxbuf);
			size_t msgSize = 7;
			uartSendTimeout(&UARTD1, &msgSize, rxbuf, TIME_INFINITE);
			print_memoryCounter -= 7;
	        }
	        spiUnselect(&SPID1);
		spiReleaseBus(&SPID1);
		uartReleaseBus(&UARTD1);
	        //chprintf((BaseSequentialStream *)&SD2,
	        //"\r\nPrinting finished in %lu ms\r\n",
	        //(chVTGetSystemTimeX()-oldt)/10);k
	        chMtxLock(&systemState_mutex);
	        systemState = IDLE;
	        chMtxUnlock(&systemState_mutex);

	} else {
	        osalSysHalt("Invalid state");

	}
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

   /* Activates the UART and SERIAL (debug) drivers */
  SYSCFG->CFGR1 |= SYSCFG_CFGR1_USART1TX_DMA_RMP;
  SYSCFG->CFGR1 |= SYSCFG_CFGR1_USART1RX_DMA_RMP;
  uartStart(&UARTD1, &uart_cfg_1);
  sdStart(&SD2, NULL);

  uartAcquireBus(&UARTD1);
  uartStartSend(&UARTD1, 13, "Starting...\r\n");
  uartReleaseBus(&UARTD1);

  chprintf((BaseSequentialStream *)&SD2,"Starting...\r\n");
  chprintf((BaseSequentialStream *)&SD2, "SYSCLK=%u\r\n", STM32_SYSCLK);

  /*
   * Blink all the LEDs
   */
  blinkAllLeds();

  /* Start ADC, wait for some samples and check for battery level*/
  adcStart(&ADCD1, NULL);
  adcStartConversion(&ADCD1, &adcgrpcfg, samples, ADC_GRP_BUF_DEPTH);
  chThdSleepMilliseconds(100);
  if (checkVoltageLevel())
	  goto low_voltage_handler;

  /* Get and verify system config from EEPROM */
  while (restoreSystemConfigEEPROM(&eepromSPI, &sysConf)) {
	  chprintf((BaseSequentialStream *)&SD2,"Invalid config in EEPROM\r\n");
	  chprintf((BaseSequentialStream *)&SD2,"Writing a new one...\r\n");
	  saveDefaultConfigEEPROM(&eepromSPI);
  }
  chprintf((BaseSequentialStream *)&SD2,"Good system config in EEPROM\r\n");

  printSystemConfig(&sysConf, (BaseSequentialStream *)&SD2);

  I2CInit();

  sensorStartup();

  /* Initialize the timer */
  gptStart(&GPTD3, &gpt3cfg);

  /* Create and pre-fill the mailboxes.*/
  createMailboxes();

  /* Init thread queue (unlock accelerometer threads from interruption) */
  osalThreadQueueObjectInit(&threadQueue);

  systemState = IDLE;

  /* We've finished the initialization */
  blinkAllLeds();

  /* Creates the threads */
  chThdCreateStatic(waThread0, sizeof(waThread0), NORMALPRIO+5, Thread0, NULL); // State machine thread
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO+3, Thread1, NULL); // MPU goes always first
  chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO+2, Thread2, NULL); // ADXL
  chThdCreateStatic(waThread3, sizeof(waThread3), NORMALPRIO,   Thread3, NULL); // SPI
  chThdCreateStatic(waThread4, sizeof(waThread4), NORMALPRIO-5, Thread4, NULL); // Serial printer

  /* Enable external (button) interrupts */
  extStart(&EXTD1, &extcfg);

  /* Normal main() thread activity.
   * Blink leds and handle all catastrophic errors.
   */
  while (true) {
	  palClearPad(GPIOC, GPIOC_LED_1);
	  osalThreadSleepMilliseconds(500);
	  palSetPad(GPIOC, GPIOC_LED_1);
	  osalThreadSleepMilliseconds(500);

	  /* Check if voltage levels are good */
	  if (checkVoltageLevel()) {
low_voltage_handler:
		  /* Give a huge priority and run an infinite loop.
		   * Nobody will preempt us */
		  chThdSetPriority(NORMALPRIO+10);
		  adcStopConversion(&ADCD1);
		  adcStop(&ADCD1);
		  while (true) {
			  for (uint32_t i = 0; i < 400000; ++i);
			  palSetPad(GPIOB, GPIOB_LED_4);
			  for (uint32_t i = 0; i < 400000; ++i);
			  palClearPad(GPIOB, GPIOB_LED_4);

		  }
	  }
  }
}
