/**
 * \file
 *
 * \brief Board configuration.
 *
 * Copyright (c) 2014-2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#include "pio.h"

#ifndef CONF_BOARD_H_INCLUDED
#define CONF_BOARD_H_INCLUDED

//#ifndef CONF_BOARD_UART_CONSOLE
//#define CONF_BOARD_UART_CONSOLE
//#endif

#ifndef CONF_BOARD_USART0
#define CONF_BOARD_USART0
#endif

#ifndef BOARD_FLEXCOM_SPI
/** FLEXCOM base address for SPI mode*/
#define BOARD_FLEXCOM_SPI    FLEXCOM5
#endif

//#ifndef BOARD_FLEXCOM_USART
/** FLEXCOM base address for USART mode*/
//#define BOARD_FLEXCOM_USART  FLEXCOM6
//#endif

/* I2C defines - I2C_FLAG */

/** Configure TWI4 pins */
#define CONF_BOARD_TWI4
#define CONF_BOARD_TWI1
#define CONF_BOARD_TWI3

/** TWI Bus Clock 400kHz */
#define TWI_CLK					400000 //sheena change I2C clk=100KHZ must be 100 for MAX7358!!!

/** Flexcom application to use */
#define BOARD_FLEXCOM_TWI4		FLEXCOM4
#define BOARD_FLEXCOM_TWI1		FLEXCOM1
#define BOARD_FLEXCOM_TWI3		FLEXCOM3

/** TWI ID for simulated EEPROM application to use */
#define BOARD_ID_TWI			ID_TWI4

/** TWI Base for simulated TWI EEPROM application to use */
#define BOARD_BASE_TWI4			TWI4
#define BOARD_BASE_TWI1			TWI1
#define BOARD_BASE_TWI3			TWI3

/** SPI MACRO definition */
#define CONF_BOARD_SPI //sheena 7-2-2018 for I2C, this must be commented

/** SPI slave select MACRO definition */
#define CONF_BOARD_SPI_NPCS0

/** Spi Hw ID . */
#define SPI_ID				ID_SPI5

/** SPI base address for SPI master mode*/
#define SPI_MASTER_BASE      SPI5
/** SPI base address for SPI slave mode, (on different board) */
#define SPI_SLAVE_BASE       SPI5
/** FLEXCOM base address for SPI mode*/
#define BOARD_FLEXCOM_SPI    FLEXCOM5

/*External Interrupt setup for PA30 Motion Sensor INT*/
#define PIN_EXT_MotionINT_MASK  PIO_PA30

/*External Interrupt setup for CHIRP_INT_0/1/2/3 INT*/
#define PIN_EXT_ChirpINT0_MASK  PIO_PA21
#define PIN_EXT_ChirpINT1_MASK  PIO_PA22
#define PIN_EXT_ChirpINT2_MASK  PIO_PA23
#define PIN_EXT_ChirpINT3_MASK  PIO_PA29

#define PIN_EXT_INTERRUPT_PIO   PIOA
#define PIN_EXT_INTERRUPT_ID    ID_PIOA
#define PIN_EXT_INTERRUPT_TYPE  PIO_INPUT
#define PIN_EXT_INTERRUPT_ATTR  (PIO_DEFAULT | PIO_IT_RISE_EDGE)
#define PIN_EXT_INTERRUPT_IRQn  PIOA_IRQn


/*for chirp chip control*/
#define PIN_EXT_MotionINT		   IOPORT_CREATE_PIN(PIOA, 30) //PIO_PA30
#define CHIRP_RST				   IOPORT_CREATE_PIN(PIOA, 15)
#define CHIRP_PROG_0			   IOPORT_CREATE_PIN(PIOA, 16)
#define CHIRP_PROG_1			   IOPORT_CREATE_PIN(PIOA, 18)
#define CHIRP_PROG_2			   IOPORT_CREATE_PIN(PIOA, 19)
#define CHIRP_PROG_3			   IOPORT_CREATE_PIN(PIOA, 31)
#define CHIRP_INT_0				   IOPORT_CREATE_PIN(PIOA, 21)
#define CHIRP_INT_1				   IOPORT_CREATE_PIN(PIOA, 22)
#define CHIRP_INT_2				   IOPORT_CREATE_PIN(PIOA, 23)
#define CHIRP_INT_3				   IOPORT_CREATE_PIN(PIOA, 29)
#define CHIRP_OK_0				   IOPORT_CREATE_PIN(PIOA, 0)
#define CHIRP_OK_1				   IOPORT_CREATE_PIN(PIOA, 1)
#define CHIRP_OK_2				   IOPORT_CREATE_PIN(PIOA, 5)
#define CHIRP_OK_3				   IOPORT_CREATE_PIN(PIOB, 15)

#endif /* CONF_BOARD_H_INCLUDED */
