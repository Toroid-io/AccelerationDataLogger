#include "hal.h"
#include "ch.h"
#include "MPU6050.h"
#include "ADXL345.h"

/* We use these lines with leds */
#define LINE_B12 PAL_LINE(GPIOB, 12U)
#define LINE_B14 PAL_LINE(GPIOB, 14U)
#define LINE_A8 PAL_LINE(GPIOA, 8U)
#define LINE_A10 PAL_LINE(GPIOA, 10U)
#define LINE_C13 PAL_LINE(GPIOC, 13U)
#define LED_0 LINE_B12
#define LED_1 LINE_B14
#define LED_2 LINE_A8
#define LED_ONBOARD LINE_C13
#define LED_WARNING LINE_A10
/* We use this lines with I2C */
#define I2C_SCL_PORT GPIOB
#define I2C_SCL_PIN 6
#define I2C_SDA_PORT GPIOB
#define I2C_SDA_PIN 7
/* We use these lines to indicate task start and stop */
#define LINE_B3 PAL_LINE(GPIOB, 3U)
#define LINE_B4 PAL_LINE(GPIOB, 4U)
#define LINE_B5 PAL_LINE(GPIOB, 5U)
#define LOGIC_0 LINE_B5
#define LOGIC_1 LINE_B4
#define LOGIC_2 LINE_B3

/* Using this mutex is not strictly required (because the I2C driver already has
 * mutual exclusion, but it is handy for debugging */
static MUTEX_DECL(i2c_device_mutex);
/* Various binary semaphores to synchronize stuff */
static BSEMAPHORE_DECL(mpu_blink, 1);
static BSEMAPHORE_DECL(adxl_blink, 1);
static BSEMAPHORE_DECL(i2c_mpu, 1);
static BSEMAPHORE_DECL(i2c_adxl, 1);

/*
 * I2C config
 */
static const I2CConfig i2cfg1 = {
	OPMODE_I2C,
	250000,
	//FAST_DUTY_CYCLE_16_9,
	FAST_DUTY_CYCLE_2,
	//STD_DUTY_CYCLE,
};


/* 
 * Timer config
 */
static void gpt3cb(GPTDriver *gptp) {
	(void)gptp;
	/* Tell the i2c thread that it can run! */
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
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("MPU6050_Poll");
  palSetLineMode(LOGIC_0, PAL_MODE_OUTPUT_PUSHPULL);
  palSetLine(LOGIC_0);

  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  chMtxLock(&i2c_device_mutex);
  MPU6050_initialize();

  // verify connection
  /*
  if (MPU6050_testConnection())
	  chprintf(chp, "MPU 6050 start OK\r\n");
  else
	  chprintf(chp, "MPU 6050 start error\r\n");
  */

  chMtxUnlock(&i2c_device_mutex);

  // use the code below to change accel/gyro offset values
  /*
     Serial.println("Updating internal sensor offsets...");
  // -76	-2359	1688	0	0	0
  Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
  Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
  Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
  Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
  Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
  Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
  Serial.print("\n");
  accelgyro.setXGyroOffset(220);
  accelgyro.setYGyroOffset(76);
  accelgyro.setZGyroOffset(-85);
  Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
  Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
  Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
  Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
  Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
  Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
  Serial.print("\n");
  */

  while (true) {
    /* wait until the timer says */
    chBSemWait(&i2c_mpu);
    chMtxLock(&i2c_device_mutex);
    palClearLine(LOGIC_0);
    MPU6050_getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    palSetLine(LOGIC_0);
    chMtxUnlock(&i2c_device_mutex);

    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);
    if ((az > 20000) || (az < -20000)) {
      chBSemSignal(&mpu_blink);
    }

#if 0
    #ifdef OUTPUT_READABLE_ACCELGYRO
        // display tab-separated accel/gyro x/y/z values
        Serial.print("a/g:\t");
        Serial.print(ax); Serial.print("\t");
        Serial.print(ay); Serial.print("\t");
        Serial.print(az); Serial.print("\t");
        Serial.print(gx); Serial.print("\t");
        Serial.print(gy); Serial.print("\t");
        Serial.println(gz);
    #endif

    #ifdef OUTPUT_BINARY_ACCELGYRO
        Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
        Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
        Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
        Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
        Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
        Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
    #endif
#endif
  }
}

/*
 * ADXL345  polling thread.
 * TODO: Revisar polling speed!
 */
static THD_WORKING_AREA(waThread2, 128);
static THD_FUNCTION(Thread2, arg) {

  (void)arg;
  chRegSetThreadName("ADXL345_Poll");
  palSetLineMode(LOGIC_1, PAL_MODE_OUTPUT_PUSHPULL);
  palSetLine(LOGIC_1);

  //void *pbuf;
  int16_t ax, ay, az;
  //uint16_t counter = 0;

  chMtxLock(&i2c_device_mutex);
  ADXL345_initialize();
  chMtxUnlock(&i2c_device_mutex);

  /* This is a little bit dirty, but this is the second started tread
   * FIXME
   */
  gptStart(&GPTD3, &gpt3cfg);
  gptStartContinuous(&GPTD3, 1250); // Timer interrupt at 800Hz

  // read raw accel measurements from device
  while (true) {
    /* wait until the timer says */
    chBSemWait(&i2c_adxl);
    chMtxLock(&i2c_device_mutex);
    palClearLine(LOGIC_1);
    ADXL345_getAcceleration(&ax, &ay, &az);
    palSetLine(LOGIC_1);
    chMtxUnlock(&i2c_device_mutex);

    if ((ax > 75) || (ax < -750)) {
      chBSemSignal(&adxl_blink);
    }
    /*
    // palSetLine(LOGIC_1);
    if (chMBFetch(&free_buffers, (msg_t *)&pbuf, TIME_INFINITE) == MSG_OK) {
      chsnprintf((char *)pbuf, BUFFERS_SIZE, "A:%u:%d\r\n", counter++, ax);
      (void)chMBPost(&filled_buffers, (msg_t)pbuf, TIME_INFINITE);
    }
    // palClearLine(LOGIC_1);
    */
  }
}

/*
 * MPU blinker thread
 */
static THD_WORKING_AREA(waThread3, 128);
static THD_FUNCTION(Thread3, arg) {

  (void)arg;
  chRegSetThreadName("reblinker");
  palSetLineMode(LED_0, PAL_MODE_OUTPUT_OPENDRAIN);

  while (true) {
    chBSemWait(&mpu_blink);
    palClearLine(LED_0);
    chThdSleepMilliseconds(1);
    palSetLine(LED_0);
    chThdSleepMilliseconds(1);
  }
}

/*
 * ADXL blinker thread
 */
static THD_WORKING_AREA(waThread4, 128);
static THD_FUNCTION(Thread4, arg) {

  (void)arg;
  chRegSetThreadName("reblinker");
  palSetLineMode(LED_1, PAL_MODE_OUTPUT_OPENDRAIN);

  while (true) {
    chBSemWait(&adxl_blink);
    palClearLine(LED_1);
    chThdSleepMilliseconds(1);
    palSetLine(LED_1);
    chThdSleepMilliseconds(1);
  }
}

/*
 * LED blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waThread6, 128);
static THD_FUNCTION(Thread6, arg) {
  (void)arg;
  chRegSetThreadName("blinker");

  palSetLineMode(LED_ONBOARD, PAL_MODE_OUTPUT_OPENDRAIN);
  palSetLineMode(LED_WARNING, PAL_MODE_OUTPUT_OPENDRAIN);
  palSetLineMode(LED_2, PAL_MODE_OUTPUT_OPENDRAIN);

  while (true) {
    palClearLine(LED_ONBOARD);
    palClearLine(LED_2);
    palClearLine(LED_WARNING);
    chThdSleepMilliseconds(500);
    palSetLine(LED_ONBOARD);
    palSetLine(LED_2);
    palSetLine(LED_WARNING);
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

  palSetPadMode(GPIOB, 6, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);   /* SCL */
  palSetPadMode(GPIOB, 7, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);   /* SDA */

  i2cStart(&I2CD1, &i2cfg1);

  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO+1, Thread1, NULL); // MPU
  chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO+1, Thread2, NULL); // ADXL
  chThdCreateStatic(waThread3, sizeof(waThread3), NORMALPRIO-1, Thread3, NULL); // MPU blink
  chThdCreateStatic(waThread4, sizeof(waThread4), NORMALPRIO-1, Thread4, NULL); // ADXL blink
  //chThdCreateStatic(waThread5, sizeof(waThread5), NORMALPRIO, Thread5, NULL); // Serial
  chThdCreateStatic(waThread6, sizeof(waThread6), NORMALPRIO-5, Thread6, NULL); // blinker

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state.
   */
  //int16_t ax, ay, az;
  //int16_t gx, gy, gz;
  while (true) {
    //osalThreadSleepMilliseconds(5000);
    //MPU6050_initialize();
    osalThreadSleepMilliseconds(50);
    //MPU6050_getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  }
}


#if 0
#include "ch.h"
#include "hal.h"
#include "math.h"
#include "string.h"
#include "chprintf.h"

/* the mailboxes are used to send information into the uart */
#define NUM_BUFFERS 4
#define BUFFERS_SIZE 16
static char buffers[NUM_BUFFERS][BUFFERS_SIZE];
static msg_t free_buffers_queue[NUM_BUFFERS];
static mailbox_t free_buffers;
static msg_t filled_buffers_queue[NUM_BUFFERS];
static mailbox_t filled_buffers;

BaseSequentialStream* chp = (BaseSequentialStream*) &SD1;

/*
 * Serial thread
 */
static THD_WORKING_AREA(waThread5, 128);
static THD_FUNCTION(Thread5, arg) {
  palSetLineMode(LOGIC_2, PAL_MODE_OUTPUT_PUSHPULL);
  palClearLine(LOGIC_2);

  (void)arg;
  chRegSetThreadName("reblinker");

  while (true) {
    void *pbuf;

    /* Waiting for a filled buffer.*/
    palClearLine(LOGIC_2);
    msg_t msg = chMBFetch(&filled_buffers, (msg_t *)&pbuf, TIME_INFINITE);
    palSetLine(LOGIC_2);

    /* Processing the event.*/
    if (msg == MSG_RESET) {
      /* The mailbox has been reset, this means network failure.*/
      // TODO
    }
    else {
      /* Processing incoming frame, print it out!*/
       chprintf(chp, pbuf);

      /* Returning the buffer to the free buffers pool.*/
      (void)chMBPost(&free_buffers, (msg_t)pbuf, TIME_INFINITE);
    }
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


 /* Create and pre-fill the mailboxes.*/
  chMBObjectInit(&filled_buffers, filled_buffers_queue, NUM_BUFFERS);
  chMBObjectInit(&free_buffers, free_buffers_queue, NUM_BUFFERS);
  uint8_t i;
  for (i = 0; i < NUM_BUFFERS; i++)
    (void)chMBPost(&free_buffers, (msg_t)&buffers[i], TIME_INFINITE);

  /*
   * Activates the serial driver 1 using the driver default configuration.
   */
  sdStart(&SD1, NULL);


  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state.
   */
  palSetLineMode(LOGIC_3, PAL_MODE_OUTPUT_PUSHPULL);
  palClearLine(LOGIC_3);
  while (true) {
	  palClearLine(LOGIC_3);
	  chThdSleepMilliseconds(250);
	  palSetLine(LOGIC_3);
	  chThdSleepMilliseconds(250);
  }
  return 0;
}
#endif
