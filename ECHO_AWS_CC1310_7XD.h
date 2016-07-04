/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/** ============================================================================
 *  @file       CC1310_LAUNCHXL.h
 *
 *  @brief      CC1310 LaunchPad Board Specific header file.
 *
 *  NB! This is the board file for CC1310 LaunchPad PCB version 1.0
 *
 *  ============================================================================
 */
#ifndef __CC1310_LAUNCHXL_BOARD_H__
#define __CC1310_LAUNCHXL_BOARD_H__

#ifdef __cplusplus
extern "C" {
#endif

/** ============================================================================
 *  Includes
 *  ==========================================================================*/
#include <ti/drivers/PIN.h>
#include <driverlib/ioc.h>
#include <ti/drivers/SPI.h>
#include <driverlib/ioc.h>

/** ============================================================================
 *  Externs
 *  ==========================================================================*/
extern const PIN_Config BoardGpioInitTable[];


/** ============================================================================
 *  Defines
 *  ==========================================================================*/

/* Same RF Configuration as 7x7 EM */
#define CC1310EM_7ID

/* GPIO Pins  */
/* Input GPIO Pins  */
#define ECHO_AWS_BUTTON                   IOID_18
#define ECHO_AWS_WIFI_INTERRUPT           IOID_11
#define ECHO_AWS_USB_INTERRUPT            IOID_22
#define ECHO_AWS_SPI1_MISO                IOID_15
#define ECHO_AWS_SENSOR_DATA              IOID_8

/* Output GPIO Pins */
#define ECHO_AWS_RED_LED                  IOID_6
#define ECHO_AWS_32K_CLOCK                IOID_9
/* SPI 0 */
#define ECHO_AWS_SR_OUTPUT_ENABLE         IOID_1
#define ECHO_AWS_SR_RESET                 IOID_2
#define ECHO_AWS_SR_LOAD_OUTPUT_REGISTER  IOID_3
//#define Board_SR_SHIFT_CLOCK           IOID_4
//#define Board_SR_SHIFT_DATA            IOID_5
#define ECHO_AWS_SPI0_MISO                IOID_29
#define ECHO_AWS_SPI0_CSN                 PIN_UNASSIGNED
#define ECHO_AWS_SPI0_MOSI                IOID_5
#define ECHO_AWS_SPI0_CLK                 IOID_4

#define ECHO_AWS_SENSOR_CLK               IOID_7
/* WiFi Interface Signals */
#define ECHO_AWS_WIFI_HIBERNATE           IOID_10
#define ECHO_AWS_WIFI_CHIP_SELECT         IOID_12
/* SPI 1 */
#define ECHO_AWS_SPI1_CLK            IOID_13
#define ECHO_AWS_SPI1_MOSI           IOID_14

#define ECHO_AWS_SPI1_CSN            PIN_UNASSIGNED
#define ECHO_AWS_USB_CHIP_SELECT          IOID_19
/* USB Interface Signals */
#define Board_USB_RESET                IOID_20
#define Board_USB_SUSPEND              IOID_21
/* LED PWM Intensity Signals */
#define Board_LED_INTENSITY_0          IOID_23
#define Board_LED_INTENSITY_1          IOID_24
#define Board_LED_INTENSITY_2          IOID_25




/** ============================================================================
 *  Instance identifiers
 *  ==========================================================================*/
/* Generic SPI instance identifiers */
#define Board_SPI0                  ECHO_AWS_SPI0
#define Board_SPI1                  ECHO_AWS_SPI1
/* Generic UART instance identifiers */
#define Board_UART                  CC1310_LAUNCHXL_UART0


/** ============================================================================
 *  Number of peripherals and their names
 *  ==========================================================================*/

/*!
 *  @def    CC1310_LAUNCHXL_I2CName
 *  @brief  Enum of I2C names on the CC2650 dev board
 */
typedef enum CC1310_LAUNCHXL_I2CName {
    CC1310_LAUNCHXL_I2C0 = 0,

    CC1310_LAUNCHXL_I2CCOUNT
} CC1310_LAUNCHXL_I2CName;

/*!
 *  @def    CC1310_LAUNCHXL_CryptoName
 *  @brief  Enum of Crypto names on the CC2650 dev board
 */
typedef enum CC1310_LAUNCHXL_CryptoName {
    CC1310_LAUNCHXL_CRYPTO0 = 0,

    CC1310_LAUNCHXL_CRYPTOCOUNT
} CC1310_LAUNCHXL_CryptoName;


/*!
 *  @def    ECHO_AWS_SPIName
 *  @brief  Enum of SPI names on the ECHO_AWS board
 */
typedef enum ECHO_AWS_SPIName {
    ECHO_AWS_SPI0 = 0,
    ECHO_AWS_SPI1,

    ECHO_AWS_SPICOUNT
} ECHO_AWS_SPIName;

/*!
 *  @def    CC1310_LAUNCHXL_UARTName
 *  @brief  Enum of UARTs on the CC2650 dev board
 */
typedef enum CC1310_LAUNCHXL_UARTName {
    CC1310_LAUNCHXL_UART0 = 0,

    CC1310_LAUNCHXL_UARTCOUNT
} CC1310_LAUNCHXL_UARTName;

/*!
 *  @def    CC1310_LAUNCHXL_UdmaName
 *  @brief  Enum of DMA buffers
 */
typedef enum CC1310_LAUNCHXL_UdmaName {
    CC1310_LAUNCHXL_UDMA0 = 0,

    CC1310_LAUNCHXL_UDMACOUNT
} CC1310_LAUNCHXL_UdmaName;

#ifdef __cplusplus
}
#endif

#endif /* __CC1310_LAUNCHXL_BOARD_H__ */
