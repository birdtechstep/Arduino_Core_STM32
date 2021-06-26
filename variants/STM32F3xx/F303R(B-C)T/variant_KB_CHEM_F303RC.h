/*
 *******************************************************************************
 * Copyright (c) 2020-2021, STMicroelectronics
 * All rights reserved.
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 *******************************************************************************
 */
#pragma once

/*----------------------------------------------------------------------------
 *        STM32 pins number
 *----------------------------------------------------------------------------*/
#define PA3  0
#define PA2  1
#define PA10 2
#define PB3  3
#define PB5  4
#define PB4  5
#define PB10 6
#define PA8  7
#define PA9  8
#define PC7  9
#define PB6  10
#define PA7  A6
#define PA6  A7
#define PA5  13
#define PB9  14
#define PB8  15
// ST Morpho
// CN7 Left Side
#define PC10 16
#define PC12 17
#define PF11 18
#define PA13 19
#define PA14 20
#define PA15 21
#define PB7  22
#define PC13 23
#define PC14 24
#define PC15 25    // DISC_PIN
#define PF0  26
#define PF1  27
#define PC2  A8
#define PC3  A9
// CN7 Right Side
#define PC11 30
#define PD2  31	   // LED_BUILTIN
// CN10 Left Side
#define PC9  32
// CN10 Right side
#define PC8  33
#define PC6  34
#define PC5  A10
#define PA12 36
#define PA11 37
#define PB12 38
#define PB11 A11
#define PB2  A12
#define PB1  41
#define PB15 42
#define PB14 43
#define PB13 44
#define PC4  A13
#define PA0  A0
#define PA1  A1     // BOOT0
#define PA4  A2
#define PB0  A3
#define PC1  A4
#define PC0  A5

// Alternate pins number
#define PA1_ALT1                (PA1  | ALT1)
#define PA2_ALT1                (PA2  | ALT1)
#define PA3_ALT1                (PA3  | ALT1)
#define PA4_ALT1                (PA4  | ALT1)
#define PA6_ALT1                (PA6  | ALT1)
#define PA7_ALT1                (PA7  | ALT1)
#define PA7_ALT2                (PA7  | ALT2)
#define PA7_ALT3                (PA7  | ALT3)
#define PA9_ALT1                (PA9  | ALT1)
#define PA10_ALT1               (PA10 | ALT1)
#define PA11_ALT1               (PA11 | ALT1)
#define PA11_ALT2               (PA11 | ALT2)
#define PA12_ALT1               (PA12 | ALT1)
#define PA12_ALT2               (PA12 | ALT2)
#define PA13_ALT1               (PA13 | ALT1)
#define PA15_ALT1               (PA15 | ALT1)
#define PB0_ALT1                (PB0  | ALT1)
#define PB0_ALT2                (PB0  | ALT2)
#define PB1_ALT1                (PB1  | ALT1)
#define PB1_ALT2                (PB1  | ALT2)
#define PB3_ALT1                (PB3  | ALT1)
#define PB4_ALT1                (PB4  | ALT1)
#define PB4_ALT2                (PB4  | ALT2)
#define PB5_ALT1                (PB5  | ALT1)
#define PB5_ALT2                (PB5  | ALT2)
#define PB6_ALT1                (PB6  | ALT1)
#define PB6_ALT2                (PB6  | ALT2)
#define PB7_ALT1                (PB7  | ALT1)
#define PB7_ALT2                (PB7  | ALT2)
#define PB8_ALT1                (PB8  | ALT1)
#define PB8_ALT2                (PB8  | ALT2)
#define PB9_ALT1                (PB9  | ALT1)
#define PB9_ALT2                (PB9  | ALT2)
#define PB14_ALT1               (PB14 | ALT1)
#define PB15_ALT1               (PB15 | ALT1)
#define PB15_ALT2               (PB15 | ALT2)
#define PC0_ALT1                (PC0  | ALT1)
#define PC1_ALT1                (PC1  | ALT1)
#define PC2_ALT1                (PC2  | ALT1)
#define PC3_ALT1                (PC3  | ALT1)
#define PC6_ALT1                (PC6  | ALT1)
#define PC7_ALT1                (PC7  | ALT1)
#define PC8_ALT1                (PC8  | ALT1)
#define PC9_ALT1                (PC9  | ALT1)
#define PC10_ALT1               (PC10 | ALT1)
#define PC11_ALT1               (PC11 | ALT1)

// Alias GPIO1 extention
#define EXT_GPIO1               PC15
#define EXT_GPIO2               PB9
#define EXT_GPIO3               PD2
#define EXT_GPIO4               PC10
#define EXT_GPIO5               PB0
#define EXT_GPIO6               PB1
#define EXT_GPIO7               PB10
#define EXT_GPIO8               PB11
#define EXT_GPIO9               PB12
#define EXT_GPIO10              PB13
#define EXT_GPIO11              PB14
#define EXT_GPIO12              PB15
#define EXT_GPIO13              PC6
#define EXT_GPIO14              PC7
#define EXT_GPIO15              PC8

// Alias UEXT
#define UEXTPWR                 PF4
#define UEXT3                   PB10
#define UEXT4                   PB11
#define UEXT5                   PA9
#define UEXT6                   PA10
#define UEXT7                   PA6
#define UEXT8                   PA7
#define UEXT9                   PA5
#define UEXT10                  PB5

#define NUM_DIGITAL_PINS        52
#define NUM_ANALOG_INPUTS       14

// On-board LED pin number
#ifndef LED_BUILTIN
  #define LED_BUILTIN           PD2
#endif

// SPI Definitions
// SPI1 on default Arduino UNO R3 connector (10-13)

// SPI definitions
#define PIN_SPI_SS              PA2
#define PIN_SPI_SS1             PA2
#define PIN_SPI_SS2             PA3
#define PIN_SPI_SS3             PB13
#define PIN_SPI_MOSI            PA7
#define PIN_SPI_MISO            PA6
#define PIN_SPI_SCK             PA5

// SD card SPI2 definitions
// These definitions are used by SD library but could not be used
// as SPI_2 is not instantiated
// Could be reviewed when SPI_INTERFACES_COUNT will be supported
// See https://github.com/stm32duino/Arduino_Core_STM32/issues/705
// #define SDCARD_SPI              SPI_2
// #define SDCARD_SS_PIN           PD2           // D254
// #define SDCARD_SCK_PIN          PIN_SPI2_SCK  // D32
// #define SDCARD_MISO_PIN         PIN_SPI2_MISO // D33
// #define SDCARD_MOSI_PIN         PIN_SPI2_MOSI // D34

// I2C Definitions
#ifndef PIN_WIRE_SDA
  #define PIN_WIRE_SDA          PB9
  #define SDA_PIN               PB9
#endif
#ifndef PIN_WIRE_SCL
  #define PIN_WIRE_SCL          PB8
  #define SCL_PIN               PB8
#endif

// Timer Definitions
// Use TIM6 when possible as servo and tone don't need GPIO output pin
#ifndef TIMER_TONE
  #define TIMER_TONE            TIM6
#endif
#ifndef TIMER_SERVO
  #define TIMER_SERVO           TIM2
#endif

// UART Definitions
#ifndef SERIAL_UART_INSTANCE
  #define SERIAL_UART_INSTANCE  2
#endif

// Default pin used for generic 'Serial' instance
// Mandatory for Firmata
#ifndef PIN_SERIAL_RX
  #define PIN_SERIAL_RX         PB11
#endif
#ifndef PIN_SERIAL_TX
  #define PIN_SERIAL_TX         PB10
#endif

// USB, pull this pin low to enable the USB attach pullup
#define USBD_ATTACH_PIN         PC15
#define USBD_ATTACH_LEVEL       LOW

/* Extra HAL modules */
#if !defined(HAL_DAC_MODULE_DISABLED)
  #define HAL_DAC_MODULE_ENABLED
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus
  // These serial port names are intended to allow libraries and architecture-neutral
  // sketches to automatically default to the correct port name for a particular type
  // of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
  // the first hardware serial port whose RX/TX pins are not dedicated to another use.
  //
  // SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
  //
  // SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
  //
  // SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
  //
  // SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
  //
  // SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
  //                            pins are NOT connected to anything by default.
  #ifndef SERIAL_PORT_MONITOR
    #define SERIAL_PORT_MONITOR     Serial
  #endif
  #ifndef SERIAL_PORT_HARDWARE
    #define SERIAL_PORT_HARDWARE    Serial1
  #endif
  #ifndef SERIAL_PORT_USBVIRTUAL
    #define SERIAL_PORT_USBVIRTUAL  SerialUSB
  #endif
#endif
