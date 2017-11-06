/*
    ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/*
 * This file has been automatically generated using ChibiStudio board
 * generator plugin. Do not edit manually.
 */

#ifndef BOARD_H
#define BOARD_H

/*
 * Board identifier.
 */
#define BOARD_ST_NUCLEO_F072RB
#define BOARD_NAME                  "TOROID Acceleration Data Logger"

/*
 * Board oscillators-related settings.
 * NOTE: LSE not fitted.
 * NOTE: HSE fitted with 8Mhz Quartz
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                0U
#endif

#define STM32_LSEDRV                (3U << 3U)

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                8000000U
#endif

/*
 * MCU type as defined in the ST header.
 */
#define STM32F072xB

/*
 * IO pins assignments.
 */
#define GPIOA_USR_BTN_1             0U
#define GPIOA_VBAT_ADC              1U
#define GPIOA_DEBUG_TX              2U
#define GPIOA_DEBUG_RX              3U
#define GPIOA_SPI_CS_1              4U
#define GPIOA_SPI_SCK               5U
#define GPIOA_SPI_MISO              6U
#define GPIOA_SPI_MOSI              7U
#define GPIOA_SPI_CS_2              8U
#define GPIOA_USART_TX              9U
#define GPIOA_USART_RX              10U
#define GPIOA_USART_CTS             11U
#define GPIOA_USART_RTS             12U
#define GPIOA_SWDIO                 13U
#define GPIOA_SWCLK                 14U
#define GPIOA_PIN15_NOT_USED        15U

#define GPIOB_SPI_HOLD              0U
#define GPIOB_VUSB_ADC              1U
#define GPIOB_PIN2_NOT_USED         2U
#define GPIOB_INT1_ADXL             3U
#define GPIOB_INT2_ADXL             4U
#define GPIOB_INT_MPU               5U
#define GPIOB_I2C1_SCL              6U
#define GPIOB_I2C1_SDA              7U
#define GPIOB_PIN8_NOT_USED         8U
#define GPIOB_PIN9_NOT_USED         9U
#define GPIOB_I2C2_SCL              10U
#define GPIOB_I2C2_SDA              11U
#define GPIOB_PIN12_NOT_USED        12U
#define GPIOB_PIN13_NOT_USED        13U
#define GPIOB_LED_4                 14U
#define GPIOB_LED_3                 15U

#define GPIOC_PIN0_NOT_USED         0U
#define GPIOC_PIN1_NOT_USED         1U
#define GPIOC_PIN2_NOT_USED         2U
#define GPIOC_PIN3_NOT_USED         3U
#define GPIOC_PIN4_NOT_USED         4U
#define GPIOC_PIN5_NOT_USED         5U
#define GPIOC_PIN6_NOT_USED         6U
#define GPIOC_PIN7_NOT_USED         7U
#define GPIOC_PIN8_NOT_USED         8U
#define GPIOC_PIN9_NOT_USED         9U
#define GPIOC_PIN10_NOT_USED        10U
#define GPIOC_PIN11_NOT_USED        11U
#define GPIOC_PIN12_NOT_USED        12U
#define GPIOC_USR_BTN_2             13U
#define GPIOC_LED_1                 14U
#define GPIOC_LED_2                 15U

#define GPIOD_PIN0_NOT_USED         0U
#define GPIOD_PIN1_NOT_USED         1U
#define GPIOD_PIN2_NOT_USED         2U
#define GPIOD_PIN3_NOT_USED         3U
#define GPIOD_PIN4_NOT_USED         4U
#define GPIOD_PIN5_NOT_USED         5U
#define GPIOD_PIN6_NOT_USED         6U
#define GPIOD_PIN7_NOT_USED         7U
#define GPIOD_PIN8_NOT_USED         8U
#define GPIOD_PIN9_NOT_USED         9U
#define GPIOD_PIN10_NOT_USED        10U
#define GPIOD_PIN11_NOT_USED        11U
#define GPIOD_PIN12_NOT_USED        12U
#define GPIOD_PIN13_NOT_USED        13U
#define GPIOD_PIN14_NOT_USED        14U
#define GPIOD_PIN15_NOT_USED        15U

#define GPIOE_PIN0_NOT_USED         0U
#define GPIOE_PIN1_NOT_USED         1U
#define GPIOE_PIN2_NOT_USED         2U
#define GPIOE_PIN3_NOT_USED         3U
#define GPIOE_PIN4_NOT_USED         4U
#define GPIOE_PIN5_NOT_USED         5U
#define GPIOE_PIN6_NOT_USED         6U
#define GPIOE_PIN7_NOT_USED         7U
#define GPIOE_PIN8_NOT_USED         8U
#define GPIOE_PIN9_NOT_USED         9U
#define GPIOE_PIN10_NOT_USED        10U
#define GPIOE_PIN11_NOT_USED        11U
#define GPIOE_PIN12_NOT_USED        12U
#define GPIOE_PIN13_NOT_USED        13U
#define GPIOE_PIN14_NOT_USED        14U
#define GPIOE_PIN15_NOT_USED        15U

#define GPIOF_PIN0_NOT_USED         0U
#define GPIOF_PIN1_NOT_USED         1U
#define GPIOF_PIN2_NOT_USED         2U
#define GPIOF_PIN3_NOT_USED         3U
#define GPIOF_PIN4_NOT_USED         4U
#define GPIOF_PIN5_NOT_USED         5U
#define GPIOF_PIN6_NOT_USED         6U
#define GPIOF_PIN7_NOT_USED         7U
#define GPIOF_PIN8_NOT_USED         8U
#define GPIOF_PIN9_NOT_USED         9U
#define GPIOF_PIN10_NOT_USED        10U
#define GPIOF_PIN11_NOT_USED        11U
#define GPIOF_PIN12_NOT_USED        12U
#define GPIOF_PIN13_NOT_USED        13U
#define GPIOF_PIN14_NOT_USED        14U
#define GPIOF_PIN15_NOT_USED        15U

/*
 * IO lines assignments.
 */
#define LED_1                 PAL_LINE(GPIOC, 14U)
#define LINE_SWDIO                  PAL_LINE(GPIOA, 13U)
#define LINE_SWCLK                  PAL_LINE(GPIOA, 14U)

#define I2C1_SDA_LINE	PAL_LINE(GPIOB, GPIOB_I2C1_SDA)
#define I2C1_SCL_LINE	PAL_LINE(GPIOB, GPIOB_I2C1_SCL)
#define I2C2_SDA_LINE	PAL_LINE(GPIOB, GPIOB_I2C2_SDA)
#define I2C2_SCL_LINE	PAL_LINE(GPIOB, GPIOB_I2C2_SCL)

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
/* MODE */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2U))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2U))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2U))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2U))
/* ODR - Initial value */
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
/* OTYPE - Output type */
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
/* OSPEED - Output speed */
#define PIN_OSPEED_VERYLOW(n)       (0U << ((n) * 2U))
#define PIN_OSPEED_LOW(n)           (1U << ((n) * 2U))
#define PIN_OSPEED_MEDIUM(n)        (2U << ((n) * 2U))
#define PIN_OSPEED_HIGH(n)          (3U << ((n) * 2U))
/* IO Pull Up/Down resistor */
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2U))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2U))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2U))
/* Alternate function */
#define PIN_AFIO_AF(n, v)           ((v) << (((n) % 8U) * 4U))

/*
 * GPIOA setup:
 *
 * PA0  - USR_BTN_1                 (input floating).
 * PA1  - VBAT_ADC                  (?).
 * PA2  - DEBUG_TX                  (alternate 1).
 * PA3  - DEBUG_RX                  (alternate 1).
 * PA4  - SPI_CS_1                  (output pushpull high).
 * PA5  - SPI_SCK                   (alternate 0).
 * PA6  - SPI_MISO                  (alternate 0).
 * PA7  - SPI_MOSI                  (alternate 0).
 * PA8  - SPI_CS_2                  (output pushpull high).
 * PA9  - TX                        (alternate 1).
 * PA10 - RX                        (alternate 1).
 * PA11 - CTS                       (alternate 1).
 * PA12 - RTS                       (alternate 1).
 * PA13 - SWDIO                     (alternate 0).
 * PA14 - SWCLK                     (alternate 0).
 * PA15 - NOT_USED                  (input pullup).
 */
#define VAL_GPIOA_MODER             (PIN_MODE_INPUT(GPIOA_USR_BTN_1) |      \
                                     PIN_MODE_INPUT(GPIOA_VBAT_ADC) |       \
                                     PIN_MODE_ALTERNATE(GPIOA_DEBUG_TX) |   \
                                     PIN_MODE_ALTERNATE(GPIOA_DEBUG_RX) |   \
                                     PIN_MODE_OUTPUT(GPIOA_SPI_CS_1) |      \
                                     PIN_MODE_ALTERNATE(GPIOA_SPI_SCK) |        \
                                     PIN_MODE_ALTERNATE(GPIOA_SPI_MISO) |       \
                                     PIN_MODE_ALTERNATE(GPIOA_SPI_MOSI) |       \
                                     PIN_MODE_OUTPUT(GPIOA_SPI_CS_2) |      \
                                     PIN_MODE_ALTERNATE(GPIOA_USART_TX) |       \
                                     PIN_MODE_ALTERNATE(GPIOA_USART_RX) |       \
                                     PIN_MODE_ALTERNATE(GPIOA_USART_CTS) |      \
                                     PIN_MODE_ALTERNATE(GPIOA_USART_RTS) |      \
                                     PIN_MODE_ALTERNATE(GPIOA_SWDIO) |      \
                                     PIN_MODE_ALTERNATE(GPIOA_SWCLK) |      \
                                     PIN_MODE_INPUT(GPIOA_PIN15_NOT_USED))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_USR_BTN_1) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_VBAT_ADC) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOA_DEBUG_TX) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOA_DEBUG_RX) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI_CS_1) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI_SCK)  |   \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI_MISO) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI_MOSI) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI_CS_2) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USART_TX) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USART_RX) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USART_CTS) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USART_RTS) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWDIO) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWCLK) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN15_NOT_USED))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_HIGH(GPIOA_USR_BTN_1) |     \
                                     PIN_OSPEED_HIGH(GPIOA_VBAT_ADC) |      \
                                     PIN_OSPEED_LOW(GPIOA_DEBUG_TX) |       \
                                     PIN_OSPEED_LOW(GPIOA_DEBUG_RX) |       \
                                     PIN_OSPEED_HIGH(GPIOA_SPI_CS_1) |      \
                                     PIN_OSPEED_HIGH(GPIOA_SPI_SCK) |       \
                                     PIN_OSPEED_HIGH(GPIOA_SPI_MISO) |      \
                                     PIN_OSPEED_HIGH(GPIOA_SPI_MOSI) |      \
                                     PIN_OSPEED_HIGH(GPIOA_SPI_CS_2) |      \
                                     PIN_OSPEED_LOW(GPIOA_USART_TX) |       \
                                     PIN_OSPEED_LOW(GPIOA_USART_RX) |       \
                                     PIN_OSPEED_LOW(GPIOA_USART_CTS) |      \
                                     PIN_OSPEED_LOW(GPIOA_USART_RTS) |      \
                                     PIN_OSPEED_HIGH(GPIOA_SWDIO) |         \
                                     PIN_OSPEED_HIGH(GPIOA_SWCLK) |         \
                                     PIN_OSPEED_HIGH(GPIOA_PIN15_NOT_USED))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_FLOATING(GPIOA_USR_BTN_1) |    \
                                     PIN_PUPDR_PULLUP(GPIOA_VBAT_ADC) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_DEBUG_TX) |   \
                                     PIN_PUPDR_FLOATING(GPIOA_DEBUG_RX) |   \
                                     PIN_PUPDR_FLOATING(GPIOA_SPI_CS_1) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_SPI_SCK) |      \
                                     PIN_PUPDR_FLOATING(GPIOA_SPI_MISO) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_SPI_MOSI) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_SPI_CS_2) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_USART_TX) |   \
                                     PIN_PUPDR_FLOATING(GPIOA_USART_RX) |   \
                                     PIN_PUPDR_FLOATING(GPIOA_USART_CTS) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_USART_RTS) |  \
                                     PIN_PUPDR_PULLUP(GPIOA_SWDIO) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOA_SWCLK) |      \
                                     PIN_PUPDR_PULLUP(GPIOA_PIN15_NOT_USED))
#define VAL_GPIOA_ODR               (PIN_ODR_HIGH(GPIOA_USR_BTN_1) |             \
                                     PIN_ODR_HIGH(GPIOA_VBAT_ADC) |             \
                                     PIN_ODR_HIGH(GPIOA_DEBUG_TX) |         \
                                     PIN_ODR_HIGH(GPIOA_DEBUG_RX) |         \
                                     PIN_ODR_HIGH(GPIOA_SPI_CS_1) |             \
                                     PIN_ODR_HIGH(GPIOA_SPI_SCK) |         \
                                     PIN_ODR_HIGH(GPIOA_SPI_MISO) |             \
                                     PIN_ODR_HIGH(GPIOA_SPI_MOSI) |             \
                                     PIN_ODR_HIGH(GPIOA_SPI_CS_2) |             \
                                     PIN_ODR_HIGH(GPIOA_USART_TX) |             \
                                     PIN_ODR_HIGH(GPIOA_USART_RX) |            \
                                     PIN_ODR_HIGH(GPIOA_USART_CTS) |            \
                                     PIN_ODR_HIGH(GPIOA_USART_RTS) |            \
                                     PIN_ODR_HIGH(GPIOA_SWDIO) |            \
                                     PIN_ODR_HIGH(GPIOA_SWCLK) |            \
                                     PIN_ODR_HIGH(GPIOA_PIN15_NOT_USED))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_USR_BTN_1, 0U) |          \
                                     PIN_AFIO_AF(GPIOA_VBAT_ADC, 0U) |          \
                                     PIN_AFIO_AF(GPIOA_DEBUG_TX, 1U) |      \
                                     PIN_AFIO_AF(GPIOA_DEBUG_RX, 1U) |      \
                                     PIN_AFIO_AF(GPIOA_SPI_CS_1, 0U) |          \
                                     PIN_AFIO_AF(GPIOA_SPI_SCK, 0U) |     \
                                     PIN_AFIO_AF(GPIOA_SPI_MISO, 0U) |          \
                                     PIN_AFIO_AF(GPIOA_SPI_MOSI, 0U))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_SPI_CS_2, 0U) |          \
                                     PIN_AFIO_AF(GPIOA_USART_TX, 1U) |          \
                                     PIN_AFIO_AF(GPIOA_USART_RX, 1U) |         \
                                     PIN_AFIO_AF(GPIOA_USART_CTS, 1U) |         \
                                     PIN_AFIO_AF(GPIOA_USART_RTS, 1U) |         \
                                     PIN_AFIO_AF(GPIOA_SWDIO, 0U) |         \
                                     PIN_AFIO_AF(GPIOA_SWCLK, 0U) |         \
                                     PIN_AFIO_AF(GPIOA_PIN15_NOT_USED, 0U))

/*
 * GPIOB setup:
 *
 * PB0  - SPI_HOLD                  (output pushpull high).
 * PB1  - VUSB_ADC                  (?).
 * PB2  - PIN2_NOT_USED             (input pullup).
 * PB3  - INT1_ADXL                 (input pulldown).
 * PB4  - INT2_ADXL                 (input pulldown).
 * PB5  - INT_MPU                   (input pulldown).
 * PB6  - I2C1_SCL                  (output pullup - alternate 1).
 * PB7  - I2C1_SDA                  (input floating - alternate 1).
 * PB8  - PIN8_NOT_USED             (input pullup).
 * PB9  - PIN9_NOT_USED             (input pullup).
 * PB10 - I2C2_SCL                  (output pullup - alternate 1).
 * PB11 - I2C2_SDA                  (input floating - alternate 1).
 * PB12 - PIN12_NOT_USED            (input pullup).
 * PB13 - PIN13_NOT_USED            (input pullup).
 * PB14 - LED_4                     (output opendrain pullup).
 * PB15 - LED_3                     (output opendrain pullup).
 */
#define VAL_GPIOB_MODER             (PIN_MODE_OUTPUT(GPIOB_SPI_HOLD) |           \
                                     PIN_MODE_INPUT(GPIOB_VUSB_ADC) |           \
                                     PIN_MODE_INPUT(GPIOB_PIN2_NOT_USED) |           \
                                     PIN_MODE_INPUT(GPIOB_INT1_ADXL) |        \
                                     PIN_MODE_INPUT(GPIOB_INT2_ADXL) |           \
                                     PIN_MODE_INPUT(GPIOB_INT_MPU) |           \
                                     PIN_MODE_OUTPUT(GPIOB_I2C1_SCL) |           \
                                     PIN_MODE_INPUT(GPIOB_I2C1_SDA) |           \
                                     PIN_MODE_INPUT(GPIOB_PIN8_NOT_USED) |           \
                                     PIN_MODE_INPUT(GPIOB_PIN9_NOT_USED) |           \
                                     PIN_MODE_OUTPUT(GPIOB_I2C2_SCL) |          \
                                     PIN_MODE_INPUT(GPIOB_I2C2_SDA) |          \
                                     PIN_MODE_INPUT(GPIOB_PIN12_NOT_USED) |          \
                                     PIN_MODE_INPUT(GPIOB_PIN13_NOT_USED) |          \
                                     PIN_MODE_OUTPUT(GPIOB_LED_4) |          \
                                     PIN_MODE_OUTPUT(GPIOB_LED_3))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOB_SPI_HOLD) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_VUSB_ADC) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN2_NOT_USED) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_INT1_ADXL) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOB_INT2_ADXL) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_INT_MPU) |       \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_I2C1_SCL) |       \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_I2C1_SDA) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN8_NOT_USED) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN9_NOT_USED) |       \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_I2C2_SCL) |      \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_I2C2_SDA) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN12_NOT_USED) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN13_NOT_USED) |      \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_LED_4) |      \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_LED_3))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_HIGH(GPIOB_SPI_HOLD) |          \
                                     PIN_OSPEED_HIGH(GPIOB_VUSB_ADC) |          \
                                     PIN_OSPEED_HIGH(GPIOB_PIN2_NOT_USED) |          \
                                     PIN_OSPEED_HIGH(GPIOB_INT1_ADXL) |           \
                                     PIN_OSPEED_HIGH(GPIOB_INT2_ADXL) |          \
                                     PIN_OSPEED_HIGH(GPIOB_INT_MPU) |          \
                                     PIN_OSPEED_HIGH(GPIOB_I2C1_SCL) |          \
                                     PIN_OSPEED_HIGH(GPIOB_I2C1_SDA) |          \
                                     PIN_OSPEED_HIGH(GPIOB_PIN8_NOT_USED) |          \
                                     PIN_OSPEED_HIGH(GPIOB_PIN9_NOT_USED) |          \
                                     PIN_OSPEED_HIGH(GPIOB_I2C2_SCL) |         \
                                     PIN_OSPEED_HIGH(GPIOB_I2C2_SDA) |         \
                                     PIN_OSPEED_HIGH(GPIOB_PIN12_NOT_USED) |         \
                                     PIN_OSPEED_HIGH(GPIOB_PIN13_NOT_USED) |         \
                                     PIN_OSPEED_HIGH(GPIOB_LED_4) |         \
                                     PIN_OSPEED_HIGH(GPIOB_LED_3))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_PULLUP(GPIOB_SPI_HOLD) |         \
                                     PIN_PUPDR_PULLUP(GPIOB_VUSB_ADC) |         \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN2_NOT_USED) |         \
                                     PIN_PUPDR_PULLUP(GPIOB_INT1_ADXL) |          \
                                     PIN_PUPDR_PULLUP(GPIOB_INT2_ADXL) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOB_INT_MPU) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOB_I2C1_SCL) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOB_I2C1_SDA) |        \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN8_NOT_USED) |         \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN9_NOT_USED) |         \
                                     PIN_PUPDR_PULLUP(GPIOB_I2C2_SCL) |        \
                                     PIN_PUPDR_PULLUP(GPIOB_I2C2_SDA) |        \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN12_NOT_USED) |        \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN13_NOT_USED) |        \
                                     PIN_PUPDR_PULLUP(GPIOB_LED_4) |        \
                                     PIN_PUPDR_PULLUP(GPIOB_LED_3))
#define VAL_GPIOB_ODR               (PIN_ODR_HIGH(GPIOB_SPI_HOLD) |             \
                                     PIN_ODR_HIGH(GPIOB_VUSB_ADC) |             \
                                     PIN_ODR_HIGH(GPIOB_PIN2_NOT_USED) |             \
                                     PIN_ODR_HIGH(GPIOB_INT1_ADXL) |              \
                                     PIN_ODR_HIGH(GPIOB_INT2_ADXL) |             \
                                     PIN_ODR_HIGH(GPIOB_INT_MPU) |             \
                                     PIN_ODR_HIGH(GPIOB_I2C1_SCL) |             \
                                     PIN_ODR_HIGH(GPIOB_I2C1_SDA) |             \
                                     PIN_ODR_HIGH(GPIOB_PIN8_NOT_USED) |             \
                                     PIN_ODR_HIGH(GPIOB_PIN9_NOT_USED) |             \
                                     PIN_ODR_HIGH(GPIOB_I2C2_SCL) |            \
                                     PIN_ODR_HIGH(GPIOB_I2C2_SDA) |            \
                                     PIN_ODR_HIGH(GPIOB_PIN12_NOT_USED) |            \
                                     PIN_ODR_HIGH(GPIOB_PIN13_NOT_USED) |            \
                                     PIN_ODR_HIGH(GPIOB_LED_4) |            \
                                     PIN_ODR_HIGH(GPIOB_LED_3))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_SPI_HOLD, 0U) |          \
                                     PIN_AFIO_AF(GPIOB_VUSB_ADC, 0U) |          \
                                     PIN_AFIO_AF(GPIOB_PIN2_NOT_USED, 0U) |          \
                                     PIN_AFIO_AF(GPIOB_INT1_ADXL, 0U) |           \
                                     PIN_AFIO_AF(GPIOB_INT2_ADXL, 0U) |          \
                                     PIN_AFIO_AF(GPIOB_INT_MPU, 0U) |          \
                                     PIN_AFIO_AF(GPIOB_I2C1_SCL, 0U) |          \
                                     PIN_AFIO_AF(GPIOB_I2C1_SDA, 0U))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_PIN8_NOT_USED, 0U) |          \
                                     PIN_AFIO_AF(GPIOB_PIN9_NOT_USED, 0U) |          \
                                     PIN_AFIO_AF(GPIOB_I2C2_SCL, 0U) |         \
                                     PIN_AFIO_AF(GPIOB_I2C2_SDA, 0U) |         \
                                     PIN_AFIO_AF(GPIOB_PIN12_NOT_USED, 0U) |         \
                                     PIN_AFIO_AF(GPIOB_PIN13_NOT_USED, 0U) |         \
                                     PIN_AFIO_AF(GPIOB_LED_4, 0U) |         \
                                     PIN_AFIO_AF(GPIOB_LED_3, 0U))

/*
 * GPIOC setup:
 *
 * PC0  - PIN0_NOT_USED             (input pullup).
 * PC1  - PIN1_NOT_USED             (input pullup).
 * PC2  - PIN2_NOT_USED             (input pullup).
 * PC3  - PIN3_NOT_USED             (input pullup).
 * PC4  - PIN4_NOT_USED             (input pullup).
 * PC5  - PIN5_NOT_USED             (input pullup).
 * PC6  - PIN6_NOT_USED             (input pullup).
 * PC7  - PIN7_NOT_USED             (input pullup).
 * PC8  - PIN8_NOT_USED             (input pullup).
 * PC9  - PIN9_NOT_USED             (input pullup).
 * PC10 - PIN10_NOT_USED            (input pullup).
 * PC11 - PIN11_NOT_USED            (input pullup).
 * PC12 - PIN12_NOT_USED            (input pullup).
 * PC13 - USR_BTN_2                 (input floating).
 * PC14 - GPIOC_LED_1               (output opendrain pullup).
 * PC15 - GPIOC_LED_2               (output opendrain pullup).
 */
#define VAL_GPIOC_MODER             (PIN_MODE_INPUT(GPIOC_PIN0_NOT_USED) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN1_NOT_USED) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN2_NOT_USED) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN3_NOT_USED) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN4_NOT_USED) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN5_NOT_USED) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN6_NOT_USED) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN7_NOT_USED) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN8_NOT_USED) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN9_NOT_USED) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN10_NOT_USED) |          \
                                     PIN_MODE_INPUT(GPIOC_PIN11_NOT_USED) |          \
                                     PIN_MODE_INPUT(GPIOC_PIN12_NOT_USED) |          \
                                     PIN_MODE_INPUT(GPIOC_USR_BTN_2) |         \
                                     PIN_MODE_OUTPUT(GPIOC_LED_1) |          \
                                     PIN_MODE_OUTPUT(GPIOC_LED_2))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOC_PIN0_NOT_USED) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN1_NOT_USED) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN2_NOT_USED) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN3_NOT_USED) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN4_NOT_USED) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN5_NOT_USED) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN6_NOT_USED) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN7_NOT_USED) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN8_NOT_USED) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN9_NOT_USED) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN10_NOT_USED) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN11_NOT_USED) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN12_NOT_USED) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_USR_BTN_2) |     \
                                     PIN_OTYPE_OPENDRAIN(GPIOC_LED_1) |      \
                                     PIN_OTYPE_OPENDRAIN(GPIOC_LED_2))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_HIGH(GPIOC_PIN0_NOT_USED) |          \
                                     PIN_OSPEED_HIGH(GPIOC_PIN1_NOT_USED) |          \
                                     PIN_OSPEED_HIGH(GPIOC_PIN2_NOT_USED) |          \
                                     PIN_OSPEED_HIGH(GPIOC_PIN3_NOT_USED) |          \
                                     PIN_OSPEED_HIGH(GPIOC_PIN4_NOT_USED) |          \
                                     PIN_OSPEED_HIGH(GPIOC_PIN5_NOT_USED) |          \
                                     PIN_OSPEED_HIGH(GPIOC_PIN6_NOT_USED) |          \
                                     PIN_OSPEED_HIGH(GPIOC_PIN7_NOT_USED) |          \
                                     PIN_OSPEED_HIGH(GPIOC_PIN8_NOT_USED) |          \
                                     PIN_OSPEED_HIGH(GPIOC_PIN9_NOT_USED) |          \
                                     PIN_OSPEED_HIGH(GPIOC_PIN10_NOT_USED) |         \
                                     PIN_OSPEED_HIGH(GPIOC_PIN11_NOT_USED) |         \
                                     PIN_OSPEED_HIGH(GPIOC_PIN12_NOT_USED) |         \
                                     PIN_OSPEED_HIGH(GPIOC_USR_BTN_2) |        \
                                     PIN_OSPEED_HIGH(GPIOC_LED_1) |         \
                                     PIN_OSPEED_HIGH(GPIOC_LED_2))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_PULLUP(GPIOC_PIN0_NOT_USED) |         \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN1_NOT_USED) |         \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN2_NOT_USED) |         \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN3_NOT_USED) |         \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN4_NOT_USED) |         \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN5_NOT_USED) |         \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN6_NOT_USED) |         \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN7_NOT_USED) |         \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN8_NOT_USED) |         \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN9_NOT_USED) |         \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN10_NOT_USED) |        \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN11_NOT_USED) |        \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN12_NOT_USED) |        \
                                     PIN_PUPDR_FLOATING(GPIOC_USR_BTN_2) |     \
                                     PIN_PUPDR_PULLUP(GPIOC_LED_1) |        \
                                     PIN_PUPDR_PULLUP(GPIOC_LED_2))
#define VAL_GPIOC_ODR               (PIN_ODR_HIGH(GPIOC_PIN0_NOT_USED) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN1_NOT_USED) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN2_NOT_USED) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN3_NOT_USED) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN4_NOT_USED) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN5_NOT_USED) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN6_NOT_USED) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN7_NOT_USED) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN8_NOT_USED) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN9_NOT_USED) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN10_NOT_USED) |            \
                                     PIN_ODR_HIGH(GPIOC_PIN11_NOT_USED) |            \
                                     PIN_ODR_HIGH(GPIOC_PIN12_NOT_USED) |            \
                                     PIN_ODR_HIGH(GPIOC_USR_BTN_2) |           \
                                     PIN_ODR_HIGH(GPIOC_LED_1) |            \
                                     PIN_ODR_HIGH(GPIOC_LED_2))
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(GPIOC_PIN0_NOT_USED, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_PIN1_NOT_USED, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_PIN2_NOT_USED, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_PIN3_NOT_USED, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_PIN4_NOT_USED, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_PIN5_NOT_USED, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_PIN6_NOT_USED, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_PIN7_NOT_USED, 0U))
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(GPIOC_PIN8_NOT_USED, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_PIN9_NOT_USED, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_PIN10_NOT_USED, 0U) |         \
                                     PIN_AFIO_AF(GPIOC_PIN11_NOT_USED, 0U) |         \
                                     PIN_AFIO_AF(GPIOC_PIN12_NOT_USED, 0U) |         \
                                     PIN_AFIO_AF(GPIOC_USR_BTN_2, 0U) |        \
                                     PIN_AFIO_AF(GPIOC_LED_1, 0U) |         \
                                     PIN_AFIO_AF(GPIOC_LED_2, 0U))

/*
 * GPIOD setup:
 *
 * PD0  - PIN0                      (input pullup).
 * PD1  - PIN1                      (input pullup).
 * PD2  - PIN2                      (input pullup).
 * PD3  - PIN3                      (input pullup).
 * PD4  - PIN4                      (input pullup).
 * PD5  - PIN5                      (input pullup).
 * PD6  - PIN6                      (input pullup).
 * PD7  - PIN7                      (input pullup).
 * PD8  - PIN8                      (input pullup).
 * PD9  - PIN9                      (input pullup).
 * PD10 - PIN10                     (input pullup).
 * PD11 - PIN11                     (input pullup).
 * PD12 - PIN12                     (input pullup).
 * PD13 - PIN13                     (input pullup).
 * PD14 - PIN14                     (input pullup).
 * PD15 - PIN15                     (input pullup).
 */
#define VAL_GPIOD_MODER             (PIN_MODE_INPUT(GPIOD_PIN0_NOT_USED) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN1_NOT_USED) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN2_NOT_USED) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN3_NOT_USED) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN4_NOT_USED) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN5_NOT_USED) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN6_NOT_USED) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN7_NOT_USED) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN8_NOT_USED) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN9_NOT_USED) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN10_NOT_USED) |          \
                                     PIN_MODE_INPUT(GPIOD_PIN11_NOT_USED) |          \
                                     PIN_MODE_INPUT(GPIOD_PIN12_NOT_USED) |          \
                                     PIN_MODE_INPUT(GPIOD_PIN13_NOT_USED) |          \
                                     PIN_MODE_INPUT(GPIOD_PIN14_NOT_USED) |          \
                                     PIN_MODE_INPUT(GPIOD_PIN15_NOT_USED))
#define VAL_GPIOD_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOD_PIN0_NOT_USED) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN1_NOT_USED) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN2_NOT_USED) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN3_NOT_USED) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN4_NOT_USED) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN5_NOT_USED) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN6_NOT_USED) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN7_NOT_USED) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN8_NOT_USED) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN9_NOT_USED) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN10_NOT_USED) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN11_NOT_USED) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN12_NOT_USED) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN13_NOT_USED) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN14_NOT_USED) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN15_NOT_USED))
#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_HIGH(GPIOD_PIN0_NOT_USED) |          \
                                     PIN_OSPEED_HIGH(GPIOD_PIN1_NOT_USED) |          \
                                     PIN_OSPEED_HIGH(GPIOD_PIN2_NOT_USED) |          \
                                     PIN_OSPEED_HIGH(GPIOD_PIN3_NOT_USED) |          \
                                     PIN_OSPEED_HIGH(GPIOD_PIN4_NOT_USED) |          \
                                     PIN_OSPEED_HIGH(GPIOD_PIN5_NOT_USED) |          \
                                     PIN_OSPEED_HIGH(GPIOD_PIN6_NOT_USED) |          \
                                     PIN_OSPEED_HIGH(GPIOD_PIN7_NOT_USED) |          \
                                     PIN_OSPEED_HIGH(GPIOD_PIN8_NOT_USED) |          \
                                     PIN_OSPEED_HIGH(GPIOD_PIN9_NOT_USED) |          \
                                     PIN_OSPEED_HIGH(GPIOD_PIN10_NOT_USED) |         \
                                     PIN_OSPEED_HIGH(GPIOD_PIN11_NOT_USED) |         \
                                     PIN_OSPEED_HIGH(GPIOD_PIN12_NOT_USED) |         \
                                     PIN_OSPEED_HIGH(GPIOD_PIN13_NOT_USED) |         \
                                     PIN_OSPEED_HIGH(GPIOD_PIN14_NOT_USED) |         \
                                     PIN_OSPEED_HIGH(GPIOD_PIN15_NOT_USED))
#define VAL_GPIOD_PUPDR             (PIN_PUPDR_PULLUP(GPIOD_PIN0_NOT_USED) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN1_NOT_USED) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN2_NOT_USED) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN3_NOT_USED) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN4_NOT_USED) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN5_NOT_USED) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN6_NOT_USED) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN7_NOT_USED) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN8_NOT_USED) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN9_NOT_USED) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN10_NOT_USED) |        \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN11_NOT_USED) |        \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN12_NOT_USED) |        \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN13_NOT_USED) |        \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN14_NOT_USED) |        \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN15_NOT_USED))
#define VAL_GPIOD_ODR               (PIN_ODR_HIGH(GPIOD_PIN0_NOT_USED) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN1_NOT_USED) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN2_NOT_USED) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN3_NOT_USED) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN4_NOT_USED) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN5_NOT_USED) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN6_NOT_USED) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN7_NOT_USED) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN8_NOT_USED) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN9_NOT_USED) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN10_NOT_USED) |            \
                                     PIN_ODR_HIGH(GPIOD_PIN11_NOT_USED) |            \
                                     PIN_ODR_HIGH(GPIOD_PIN12_NOT_USED) |            \
                                     PIN_ODR_HIGH(GPIOD_PIN13_NOT_USED) |            \
                                     PIN_ODR_HIGH(GPIOD_PIN14_NOT_USED) |            \
                                     PIN_ODR_HIGH(GPIOD_PIN15_NOT_USED))
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(GPIOD_PIN0_NOT_USED, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN1_NOT_USED, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN2_NOT_USED, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN3_NOT_USED, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN4_NOT_USED, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN5_NOT_USED, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN6_NOT_USED, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN7_NOT_USED, 0U))
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(GPIOD_PIN8_NOT_USED, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN9_NOT_USED, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN10_NOT_USED, 0U) |         \
                                     PIN_AFIO_AF(GPIOD_PIN11_NOT_USED, 0U) |         \
                                     PIN_AFIO_AF(GPIOD_PIN12_NOT_USED, 0U) |         \
                                     PIN_AFIO_AF(GPIOD_PIN13_NOT_USED, 0U) |         \
                                     PIN_AFIO_AF(GPIOD_PIN14_NOT_USED, 0U) |         \
                                     PIN_AFIO_AF(GPIOD_PIN15_NOT_USED, 0U))

/*
 * GPIOE setup:
 *
 * PE0  - PIN0                      (input pullup).
 * PE1  - PIN1                      (input pullup).
 * PE2  - PIN2                      (input pullup).
 * PE3  - PIN3                      (input pullup).
 * PE4  - PIN4                      (input pullup).
 * PE5  - PIN5                      (input pullup).
 * PE6  - PIN6                      (input pullup).
 * PE7  - PIN7                      (input pullup).
 * PE8  - PIN8                      (input pullup).
 * PE9  - PIN9                      (input pullup).
 * PE10 - PIN10                     (input pullup).
 * PE11 - PIN11                     (input pullup).
 * PE12 - PIN12                     (input pullup).
 * PE13 - PIN13                     (input pullup).
 * PE14 - PIN14                     (input pullup).
 * PE15 - PIN15                     (input pullup).
 */
#define VAL_GPIOE_MODER             (PIN_MODE_INPUT(GPIOE_PIN0_NOT_USED) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN1_NOT_USED) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN2_NOT_USED) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN3_NOT_USED) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN4_NOT_USED) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN5_NOT_USED) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN6_NOT_USED) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN7_NOT_USED) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN8_NOT_USED) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN9_NOT_USED) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN10_NOT_USED) |          \
                                     PIN_MODE_INPUT(GPIOE_PIN11_NOT_USED) |          \
                                     PIN_MODE_INPUT(GPIOE_PIN12_NOT_USED) |          \
                                     PIN_MODE_INPUT(GPIOE_PIN13_NOT_USED) |          \
                                     PIN_MODE_INPUT(GPIOE_PIN14_NOT_USED) |          \
                                     PIN_MODE_INPUT(GPIOE_PIN15_NOT_USED))
#define VAL_GPIOE_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOE_PIN0_NOT_USED) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN1_NOT_USED) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN2_NOT_USED) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN3_NOT_USED) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN4_NOT_USED) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN5_NOT_USED) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN6_NOT_USED) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN7_NOT_USED) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN8_NOT_USED) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN9_NOT_USED) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN10_NOT_USED) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN11_NOT_USED) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN12_NOT_USED) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN13_NOT_USED) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN14_NOT_USED) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN15_NOT_USED))
#define VAL_GPIOE_OSPEEDR           (PIN_OSPEED_HIGH(GPIOE_PIN0_NOT_USED) |          \
                                     PIN_OSPEED_HIGH(GPIOE_PIN1_NOT_USED) |          \
                                     PIN_OSPEED_HIGH(GPIOE_PIN2_NOT_USED) |          \
                                     PIN_OSPEED_HIGH(GPIOE_PIN3_NOT_USED) |          \
                                     PIN_OSPEED_HIGH(GPIOE_PIN4_NOT_USED) |          \
                                     PIN_OSPEED_HIGH(GPIOE_PIN5_NOT_USED) |          \
                                     PIN_OSPEED_HIGH(GPIOE_PIN6_NOT_USED) |          \
                                     PIN_OSPEED_HIGH(GPIOE_PIN7_NOT_USED) |          \
                                     PIN_OSPEED_HIGH(GPIOE_PIN8_NOT_USED) |          \
                                     PIN_OSPEED_HIGH(GPIOE_PIN9_NOT_USED) |          \
                                     PIN_OSPEED_HIGH(GPIOE_PIN10_NOT_USED) |         \
                                     PIN_OSPEED_HIGH(GPIOE_PIN11_NOT_USED) |         \
                                     PIN_OSPEED_HIGH(GPIOE_PIN12_NOT_USED) |         \
                                     PIN_OSPEED_HIGH(GPIOE_PIN13_NOT_USED) |         \
                                     PIN_OSPEED_HIGH(GPIOE_PIN14_NOT_USED) |         \
                                     PIN_OSPEED_HIGH(GPIOE_PIN15_NOT_USED))
#define VAL_GPIOE_PUPDR             (PIN_PUPDR_PULLUP(GPIOE_PIN0_NOT_USED) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN1_NOT_USED) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN2_NOT_USED) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN3_NOT_USED) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN4_NOT_USED) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN5_NOT_USED) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN6_NOT_USED) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN7_NOT_USED) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN8_NOT_USED) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN9_NOT_USED) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN10_NOT_USED) |        \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN11_NOT_USED) |        \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN12_NOT_USED) |        \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN13_NOT_USED) |        \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN14_NOT_USED) |        \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN15_NOT_USED))
#define VAL_GPIOE_ODR               (PIN_ODR_HIGH(GPIOE_PIN0_NOT_USED) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN1_NOT_USED) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN2_NOT_USED) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN3_NOT_USED) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN4_NOT_USED) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN5_NOT_USED) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN6_NOT_USED) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN7_NOT_USED) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN8_NOT_USED) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN9_NOT_USED) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN10_NOT_USED) |            \
                                     PIN_ODR_HIGH(GPIOE_PIN11_NOT_USED) |            \
                                     PIN_ODR_HIGH(GPIOE_PIN12_NOT_USED) |            \
                                     PIN_ODR_HIGH(GPIOE_PIN13_NOT_USED) |            \
                                     PIN_ODR_HIGH(GPIOE_PIN14_NOT_USED) |            \
                                     PIN_ODR_HIGH(GPIOE_PIN15_NOT_USED))
#define VAL_GPIOE_AFRL              (PIN_AFIO_AF(GPIOE_PIN0_NOT_USED, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN1_NOT_USED, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN2_NOT_USED, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN3_NOT_USED, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN4_NOT_USED, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN5_NOT_USED, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN6_NOT_USED, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN7_NOT_USED, 0U))
#define VAL_GPIOE_AFRH              (PIN_AFIO_AF(GPIOE_PIN8_NOT_USED, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN9_NOT_USED, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN10_NOT_USED, 0U) |         \
                                     PIN_AFIO_AF(GPIOE_PIN11_NOT_USED, 0U) |         \
                                     PIN_AFIO_AF(GPIOE_PIN12_NOT_USED, 0U) |         \
                                     PIN_AFIO_AF(GPIOE_PIN13_NOT_USED, 0U) |         \
                                     PIN_AFIO_AF(GPIOE_PIN14_NOT_USED, 0U) |         \
                                     PIN_AFIO_AF(GPIOE_PIN15_NOT_USED, 0U))

/*
 * GPIOF setup:
 *
 * PF0  - PIN0                      (input floating).
 * PF1  - PIN1                      (input floating).
 * PF2  - PIN2                      (input pullup).
 * PF3  - PIN3                      (input pullup).
 * PF4  - PIN4                      (input pullup).
 * PF5  - PIN5                      (input pullup).
 * PF6  - PIN6                      (input pullup).
 * PF7  - PIN7                      (input pullup).
 * PF8  - PIN8                      (input pullup).
 * PF9  - PIN9                      (input pullup).
 * PF10 - PIN10                     (input pullup).
 * PF11 - PIN11                     (input pullup).
 * PF12 - PIN12                     (input pullup).
 * PF13 - PIN13                     (input pullup).
 * PF14 - PIN14                     (input pullup).
 * PF15 - PIN15                     (input pullup).
 */
#define VAL_GPIOF_MODER             (PIN_MODE_INPUT(GPIOF_PIN0_NOT_USED) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN1_NOT_USED) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN2_NOT_USED) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN3_NOT_USED) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN4_NOT_USED) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN5_NOT_USED) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN6_NOT_USED) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN7_NOT_USED) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN8_NOT_USED) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN9_NOT_USED) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN10_NOT_USED) |          \
                                     PIN_MODE_INPUT(GPIOF_PIN11_NOT_USED) |          \
                                     PIN_MODE_INPUT(GPIOF_PIN12_NOT_USED) |          \
                                     PIN_MODE_INPUT(GPIOF_PIN13_NOT_USED) |          \
                                     PIN_MODE_INPUT(GPIOF_PIN14_NOT_USED) |          \
                                     PIN_MODE_INPUT(GPIOF_PIN15_NOT_USED))
#define VAL_GPIOF_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOF_PIN0_NOT_USED) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN1_NOT_USED) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN2_NOT_USED) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN3_NOT_USED) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN4_NOT_USED) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN5_NOT_USED) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN6_NOT_USED) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN7_NOT_USED) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN8_NOT_USED) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN9_NOT_USED) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN10_NOT_USED) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN11_NOT_USED) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN12_NOT_USED) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN13_NOT_USED) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN14_NOT_USED) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN15_NOT_USED))
#define VAL_GPIOF_OSPEEDR           (PIN_OSPEED_HIGH(GPIOF_PIN0_NOT_USED) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN1_NOT_USED) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN2_NOT_USED) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN3_NOT_USED) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN4_NOT_USED) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN5_NOT_USED) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN6_NOT_USED) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN7_NOT_USED) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN8_NOT_USED) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN9_NOT_USED) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN10_NOT_USED) |         \
                                     PIN_OSPEED_HIGH(GPIOF_PIN11_NOT_USED) |         \
                                     PIN_OSPEED_HIGH(GPIOF_PIN12_NOT_USED) |         \
                                     PIN_OSPEED_HIGH(GPIOF_PIN13_NOT_USED) |         \
                                     PIN_OSPEED_HIGH(GPIOF_PIN14_NOT_USED) |         \
                                     PIN_OSPEED_HIGH(GPIOF_PIN15_NOT_USED))
#define VAL_GPIOF_PUPDR             (PIN_PUPDR_FLOATING(GPIOF_PIN0_NOT_USED) |         \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN1_NOT_USED) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN2_NOT_USED) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN3_NOT_USED) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN4_NOT_USED) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN5_NOT_USED) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN6_NOT_USED) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN7_NOT_USED) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN8_NOT_USED) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN9_NOT_USED) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN10_NOT_USED) |        \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN11_NOT_USED) |        \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN12_NOT_USED) |        \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN13_NOT_USED) |        \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN14_NOT_USED) |        \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN15_NOT_USED))
#define VAL_GPIOF_ODR               (PIN_ODR_HIGH(GPIOF_PIN0_NOT_USED) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN1_NOT_USED) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN2_NOT_USED) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN3_NOT_USED) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN4_NOT_USED) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN5_NOT_USED) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN6_NOT_USED) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN7_NOT_USED) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN8_NOT_USED) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN9_NOT_USED) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN10_NOT_USED) |            \
                                     PIN_ODR_HIGH(GPIOF_PIN11_NOT_USED) |            \
                                     PIN_ODR_HIGH(GPIOF_PIN12_NOT_USED) |            \
                                     PIN_ODR_HIGH(GPIOF_PIN13_NOT_USED) |            \
                                     PIN_ODR_HIGH(GPIOF_PIN14_NOT_USED) |            \
                                     PIN_ODR_HIGH(GPIOF_PIN15_NOT_USED))
#define VAL_GPIOF_AFRL              (PIN_AFIO_AF(GPIOF_PIN0_NOT_USED, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN1_NOT_USED, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN2_NOT_USED, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN3_NOT_USED, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN4_NOT_USED, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN5_NOT_USED, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN6_NOT_USED, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN7_NOT_USED, 0U))
#define VAL_GPIOF_AFRH              (PIN_AFIO_AF(GPIOF_PIN8_NOT_USED, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN9_NOT_USED, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN10_NOT_USED, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_PIN11_NOT_USED, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_PIN12_NOT_USED, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_PIN13_NOT_USED, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_PIN14_NOT_USED, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_PIN15_NOT_USED, 0U))


#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* BOARD_H */
