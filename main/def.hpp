/*
 * def.h
 *
 *  Created on: Nov 13, 2018
 *      Author: juri
 */

#ifndef MAIN_DEF_H_
#define MAIN_DEF_H_

#include "gitversion.hpp"

// #define FW_VERSION 13
#define FW_VERSION_NAME "0.02.001"

/**
 * BUILD TARGET
 */

#define TARGET_ESP32 0
#define TARGET_PI3 3

#define BUILD_TARGET TARGET_PI3

/**
 * BUILD VERSION
 */
#define VERSION_CC 0  // companion computer
#define VERSION_SC 1  // safety computer
#define VERSION_GS 2  // groundstation

#define BUILD_VERSION VERSION_CC

// address of this device (for diavenCom) should match build version
// since this addresses just a device type, not a instance
#define THIS_SEND_ADDR BUILD_VERSION

#if BUILD_TARGET == TARGET_ESP32

#include "driver/gpio.h"
/**
 * I2C 0,1
 */
#define I2C_USER_COM_SLAVE_ADRESS 0x28
// i2c 0 : master
#define PIN_NUM_I2C0_SCL GPIO_NUM_22  // orange (old:yellow)
#define PIN_NUM_I2C0_SDA GPIO_NUM_21  // yellow (old:green)

// i2c 1 : slave
// #define PIN_NUM_I2C1_SCL GPIO_NUM_27  // orange (old:yellow)
// #define PIN_NUM_I2C1_SDA GPIO_NUM_34  // yellow (old:green)
#define PIN_NUM_I2C1_SCL GPIO_NUM_19  // orange (old:yellow)
#define PIN_NUM_I2C1_SDA GPIO_NUM_23  // yellow (old:green)

#define PIN_NUM_I2C1_ADD0 GPIO_NUM_36  // SESOR_VP
#define PIN_NUM_I2C1_ADD1 GPIO_NUM_39  // SESOR_VN

/**
 * weather statoin
 */
#define PIN_NUM_WEATHER_RX GPIO_NUM_19
// has to be ADC1!
#define ADC_NUM_WEATHER_VOLTAGE ADC1_CHANNEL_7  // this means GPIO 35

/**
 * SPI 0,1
 */
#define PIN_NUM_MISO GPIO_NUM_19
#define PIN_NUM_MOSI GPIO_NUM_23
#define PIN_NUM_CLK GPIO_NUM_18
#define PIN_NUM_CS GPIO_NUM_5
#define PIN_SPI_READY GPIO_NUM_4
#define PIN_NUM_CS_1 GPIO_NUM_32
#define PIN_SPI_READY_1 GPIO_NUM_35

/**
 * UART0
 */
#define UART0_TX_PIN GPIO_NUM_1
#define UART0_RX_PIN GPIO_NUM_3
// pin5 EN
// pin6 GPIO_NUM_0
#define UART0_BAUD_RATE 115200
#define UART0_RTS UART_PIN_NO_CHANGE
#define UART0_CTS UART_PIN_NO_CHANGE

/**
 * UART1
 */
#define UART1_TX_PIN GPIO_NUM_26
#define UART1_RX_PIN GPIO_NUM_25
#define UART1_RTS UART_PIN_NO_CHANGE
#define UART1_CTS UART_PIN_NO_CHANGE
// interrupt pin 5
#define GSM_ON_OFF_OUT_GPIO GPIO_NUM_33

/**
 * UART2
 */
#define UART2_TX_PIN GPIO_NUM_17
#define UART2_RX_PIN GPIO_NUM_16
#define UART2_RTS UART_PIN_NO_CHANGE
#define UART2_CTS UART_PIN_NO_CHANGE

/**
 * peripheral
 */
#define LED2_GPIO -1
// #define LED_GPIO GPIO_NUM_2
// #define BUTTON_GPIO GPIO_NUM_0

/**
 * safety trigger
 */
#define ENG_OFF_TRIGGER_GPIO GPIO_NUM_32
#define PARASHUTE_TRIGGER_GPIO GPIO_NUM_33

#endif

#if BUILD_TARGET == TARGET_PI3

/**
 * I2C 0,1
 */
#define I2C_USER_COM_SLAVE_ADRESS 0x28
// i2c 0 : master
#define PIN_NUM_I2C0_SCL 0  // orange (old:yellow)
#define PIN_NUM_I2C0_SDA 0  // yellow (old:green)

// i2c 1 : slave
// #define PIN_NUM_I2C1_SCL GPIO_NUM_27  // orange (old:yellow)
// #define PIN_NUM_I2C1_SDA GPIO_NUM_34  // yellow (old:green)
#define PIN_NUM_I2C1_SCL 0  // orange (old:yellow)
#define PIN_NUM_I2C1_SDA 0  // yellow (old:green)

#define PIN_NUM_I2C1_ADD0 0  // SESOR_VP
#define PIN_NUM_I2C1_ADD1 0  // SESOR_VN

/**
 * weather statoin
 */
#define PIN_NUM_WEATHER_RX 0
// has to be ADC1!
#define ADC_NUM_WEATHER_VOLTAGE 0  // this means GPIO 35

/**
 * SPI 0,1
 */
#define PIN_NUM_MISO 0
#define PIN_NUM_MOSI 0
#define PIN_NUM_CLK 0
#define PIN_NUM_CS 0
#define PIN_SPI_READY 21
#define PIN_NUM_CS_1 0
#define PIN_SPI_READY_1 22

/**
 * UART0
 */
#define UART0_TX_PIN 0
#define UART0_RX_PIN 0
// pin5 EN
// pin6 GPIO_NUM_0
#define UART0_BAUD_RATE 115200
#define UART0_RTS 0
#define UART0_CTS 0

/**
 * UART1
 */

// onboard uart: /dev/ttyS0
// gps via usb: /dev/ttyACM0
// ft232: /dev/ttyUSB0

// #define UART1_TX_PIN "/dev/ttyUSB0" // USB to Serial (FT232)
#define UART1_TX_PIN "/dev/ttyACM0"  //"/dev/ttyACM0" // GPS via USB
#define UART1_RX_PIN 0
#define UART1_RTS 0
#define UART1_CTS 0
// interrupt pin 5
#define GSM_ON_OFF_OUT_GPIO 0

/**
 * UART2
 */
#define UART2_TX_PIN "/dev/ttyS0"  // onboard UART
#define UART2_RX_PIN 0
#define UART2_RTS 0
#define UART2_CTS 0

/**
 * peripheral
 */
#define LED2_GPIO 25
#define XSHUT0 \
  22  // XSHUT for sensor 1 (vl53l0 - forward to USB in Pi) = WPi GPIO 22
#define XSHUT1 \
  21  // XSHUT for sensor 2 (vl53l1 - forward to HDMI in Pi) = WPi GPIO 21
#define XSHUT2 \
  0  // XSHUT for sensor 3 (vl53l2 - backward from USB in Pi) = WPi GPIO 0
#define XSHUT3 \
  6  // XSHUT for sensor 4 (vl53l3 - backward from HDMI in Pi) = WPi GPIO 6
// #define LED_GPIO 24
// #define BUTTON_GPIO 23

/**
 * safety trigger
 */
#define ENG_OFF_TRIGGER_GPIO 0
#define PARASHUTE_TRIGGER_GPIO 0

#endif

#endif /* MAIN_DEF_H_ */
