/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2019 InvenSense Inc. All rights reserved.
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


#ifndef INV_SPI_H
#define INV_SPI_H

#define SPI_Handler				FLEXCOM5_Handler
#define SPI_IRQn				FLEXCOM5_IRQn
#define SPI_CHIP_SEL0			0 /* PA11_SPI5_NPCS0 */
#define SPI_CHIP_SEL1			1 /* PA05_SPI5_NPCS1 */

#define SPI_CLK_POLARITY		1
#define SPI_CLK_PHASE			0
#define SPI_DLYBS				0x40 /* Delay before SPCK. */
#define SPI_DLYBCT				0x01 /* Delay between consecutive transfers. */
#define SPI_CLK_SPEED			6000000

#define READ_BIT_MASK			0x80
#define WRITE_BIT_MASK			0x7F

typedef enum spi_num {
	INV_SPI_CS0,
	INV_SPI_CS1
}spi_num_t;

int spi_master_init(unsigned spi_num);
void spi_master_deinit(unsigned spi_num);
int spi_master_write_register(unsigned spi_num, uint8_t register_addr, uint32_t len, const uint8_t * value);
int spi_master_read_register(unsigned spi_num, uint8_t register_addr, uint32_t len, uint8_t * value);


#endif /* INV_SPI_H */