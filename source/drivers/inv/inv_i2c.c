/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2020 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively “Software”) is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * ________________________________________________________________________________________________________
 */
 #include <asf.h>
 #include "twi.h"
 #include "inv_i2c.h"
 #include "inv_uart.h"
 //#include "flexcom.h"
 //#include "conf_board.h"

 /* On Atmel platform, Slave Address should be set to 0x68 as pin SA0 is logic low */
 #define ICM_I2C_ADDR     0x68 // 0x69 /* I2C slave address for ICM20603 */
 #define DATA_ACCURACY_MASK  ((uint32_t)0x7)

 twi_options_t opt;
 twi_packet_t packet_tx, packet_rx;

void i2c_master_initialize1(void)
{
	/* Insert application code here, after the board has been initialized. */
	/* Enable the peripheral and set TWI mode. */
	flexcom_enable(BOARD_FLEXCOM_TWI1);
	flexcom_set_opmode(BOARD_FLEXCOM_TWI1, FLEXCOM_TWI);
	
	/* Configure the options of TWI driver */
	opt.master_clk = sysclk_get_peripheral_hz();
	opt.speed = TWI_CLK;

	//const char str1[] = "-E-\tTWI master initialization failed.\r\n";

	//twi_master_init(BOARD_BASE_TWI, &opt);
	if (twi_master_init(BOARD_BASE_TWI1, &opt) != TWI_SUCCESS) {
		//usart_serial_write_packet(CONF_UART, (const uint8_t*)str1, sizeof(str1) - 1);
	}
	NVIC_DisableIRQ(FLEXCOM1_IRQn);
	NVIC_ClearPendingIRQ(FLEXCOM1_IRQn);
	NVIC_SetPriority(FLEXCOM1_IRQn, 1);
	NVIC_EnableIRQ(FLEXCOM1_IRQn);
} 
 
void i2c_master_initialize3(void)
{
	/* Insert application code here, after the board has been initialized. */
	/* Enable the peripheral and set TWI mode. */
	flexcom_enable(BOARD_FLEXCOM_TWI3);
	flexcom_set_opmode(BOARD_FLEXCOM_TWI3, FLEXCOM_TWI);
	
	/* Configure the options of TWI driver */
	opt.master_clk = sysclk_get_peripheral_hz();
	opt.speed = TWI_CLK;

	//const char str1[] = "-E-\tTWI master initialization failed.\r\n";

	//twi_master_init(BOARD_BASE_TWI, &opt);
	if (twi_master_init(BOARD_BASE_TWI3, &opt) != TWI_SUCCESS) {
		//usart_serial_write_packet(CONF_UART, (const uint8_t*)str1, sizeof(str1) - 1);
	}
	NVIC_DisableIRQ(FLEXCOM3_IRQn);
	NVIC_ClearPendingIRQ(FLEXCOM3_IRQn);
	NVIC_SetPriority(FLEXCOM3_IRQn, 1);
	NVIC_EnableIRQ(FLEXCOM3_IRQn);
}

void i2c_master_init(void)
{
	i2c_master_initialize1();
	i2c_master_initialize3();

	pio_pull_down(PIOA, PIO_PA3 | PIO_PA4, 0);
	pio_pull_up(PIOA, PIO_PA3 | PIO_PA4, 0);

	pio_pull_down(PIOB, PIO_PB2 | PIO_PB3, 0);
	pio_pull_up(PIOB, PIO_PB2 | PIO_PB3, 0);
}

 void i2c_master_deinit1(void)
 {
	 /* Disable the peripheral and TWI mode. */
	 flexcom_disable(BOARD_FLEXCOM_TWI1);
	 twi_disable_master_mode(BOARD_BASE_TWI1);
 }
 
 void i2c_master_deinit3(void)
  {
	  /* Disable the peripheral and TWI mode. */
	  flexcom_disable(BOARD_FLEXCOM_TWI3);
	  twi_disable_master_mode(BOARD_BASE_TWI3);
  }

unsigned long i2c_master_read_register1(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, unsigned char *RegisterValue)
 {
	 twi_packet_t packet_read;
	 
	 packet_read.chip = Address;
	 packet_read.addr[0] = RegisterAddr;
	 packet_read.addr_length = 1;
	 packet_read.buffer = RegisterValue;
	 packet_read.length = RegisterLen;
	 
	 if(twi_master_read((Twi*)BOARD_BASE_TWI1, &packet_read) == TWI_SUCCESS){
		 return TWI_SUCCESS;
	 }
	 return TWI_BUSY;
 }
 unsigned long i2c_master_read_register1_raw(unsigned char Address, unsigned short len, unsigned char *data)
 {
	 twi_packet_t packet_read;
	 
	 packet_read.chip = Address; //address of I2C device to be accessed
	 //packet_read.addr[0] = RegisterAddr;
	 packet_read.addr_length = 0; //no internal address
	 packet_read.buffer = data;
	 packet_read.length = len;
	 
	 if(twi_master_read((Twi*)BOARD_BASE_TWI1, &packet_read) == TWI_SUCCESS){
		 return TWI_SUCCESS;
	 }
	 return TWI_BUSY;
 }
  
unsigned long i2c_master_read_register3(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, unsigned char *RegisterValue)
{
	twi_packet_t packet_read;
	
	packet_read.chip = Address;
	packet_read.addr[0] = RegisterAddr;
	packet_read.addr_length = 1;
	packet_read.buffer = RegisterValue;
	packet_read.length = RegisterLen;
	
	if(twi_master_read((Twi*)BOARD_BASE_TWI3, &packet_read) == TWI_SUCCESS){
		return TWI_SUCCESS;
	}
	return TWI_BUSY;
} 
unsigned long i2c_master_read_register3_raw(unsigned char Address, unsigned short len, unsigned char *data)
{
	twi_packet_t packet_read;
	 
	packet_read.chip = Address; //address of I2C device to be accessed
	//packet_read.addr[0] = RegisterAddr;
	packet_read.addr_length = 0; //no internal address
	packet_read.buffer = data;
	packet_read.length = len;
	 
	if(twi_master_read((Twi*)BOARD_BASE_TWI3, &packet_read) == TWI_SUCCESS){
		return TWI_SUCCESS;
	}
	return TWI_BUSY;
}

 unsigned long i2c_master_write_register1(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, unsigned char *RegisterValue)
 {
	 twi_packet_t packet_write;

	 packet_write.chip = Address;
	 packet_write.addr[0] = RegisterAddr;
	 packet_write.addr_length = 1;
	 packet_write.buffer = RegisterValue;
	 packet_write.length = RegisterLen;

	 return twi_master_write((Twi*)BOARD_BASE_TWI1, &packet_write);
 }
 unsigned long i2c_master_write_register1_raw(unsigned char Address, unsigned short len, unsigned char *data)
 {
	 twi_packet_t packet_write;

	 packet_write.chip = Address; //address of I2C device to be accessed
	 //packet_write.addr[0] = RegisterAddr;
	 packet_write.addr_length = 0; //no internal address
	 packet_write.buffer = data;
	 packet_write.length = len;

	 return twi_master_write((Twi*)BOARD_BASE_TWI1, &packet_write);
 }
 unsigned long i2c_master_write_register3(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, unsigned char *RegisterValue)
 {
	 twi_packet_t packet_write;

	 packet_write.chip = Address;
	 packet_write.addr[0] = RegisterAddr;
	 packet_write.addr_length = 1;
	 packet_write.buffer = RegisterValue;
	 packet_write.length = RegisterLen;

	 return twi_master_write((Twi*)BOARD_BASE_TWI3, &packet_write);
 }
 unsigned long i2c_master_write_register3_raw(unsigned char Address, unsigned short len, unsigned char *data)
 {
	 twi_packet_t packet_write;

	 packet_write.chip = Address; //address of I2C device to be accessed
	 //packet_write.addr[0] = RegisterAddr;
	 packet_write.addr_length = 0; //no internal address
	 packet_write.buffer = data;
	 packet_write.length = len;


	 return twi_master_write((Twi*)BOARD_BASE_TWI3, &packet_write);
 }