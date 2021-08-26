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

#include <string.h>

#include <asf.h>
#include "inv_spi.h"

#define SPI_BUFFER_SIZE    1040

struct spi_mapping {
	uint32_t spi_chip_sel;
	ioport_pin_t pio_cs_idx;

	Pdc * p_pdc;
	uint8_t tx_buffer[SPI_BUFFER_SIZE];
	uint8_t rx_buffer[SPI_BUFFER_SIZE];
};

struct spi_mapping sm[2] = {
	{
	.spi_chip_sel = SPI_CHIP_SEL0,
	.pio_cs_idx = PIO_PA11_IDX,
	.p_pdc = 0,
	.tx_buffer = {0},
	.rx_buffer = {0}
	},

	{
	.spi_chip_sel = SPI_CHIP_SEL1,
	.pio_cs_idx = PIO_PA5_IDX,
	.p_pdc = 0,
	.tx_buffer = {0},
	.rx_buffer = {0}
	},
};

/* Function prototype declaration */
/**
 * \brief Initialize SPI as master.
 * PDC and Interrupts Initialized enabled
 */

int spi_master_init(unsigned spi_num)
{
	ioport_set_pin_mode(PIO_PA14_IDX, IOPORT_MODE_MUX_A);
	ioport_set_pin_mode(PIO_PA13_IDX, IOPORT_MODE_MUX_A);
	ioport_set_pin_mode(PIO_PA12_IDX, IOPORT_MODE_MUX_A);
	ioport_set_pin_mode(sm[spi_num].pio_cs_idx, IOPORT_MODE_MUX_A);
	ioport_disable_pin(PIO_PA14_IDX);
	ioport_disable_pin(PIO_PA13_IDX);
	ioport_disable_pin(PIO_PA12_IDX);
	ioport_disable_pin(sm[spi_num].pio_cs_idx);
	
	/* Get pointer to SPI master PDC register base */
	sm[spi_num].p_pdc = spi_get_pdc_base(SPI_MASTER_BASE);
	
	/* Enable the peripheral and set SPI mode. */
	
	flexcom_enable(BOARD_FLEXCOM_SPI);
	flexcom_set_opmode(BOARD_FLEXCOM_SPI, FLEXCOM_SPI);

	spi_disable(SPI_MASTER_BASE);
	spi_reset(SPI_MASTER_BASE);
	spi_set_lastxfer(SPI_MASTER_BASE);
	spi_set_master_mode(SPI_MASTER_BASE);
	spi_disable_mode_fault_detect(SPI_MASTER_BASE);
	
	spi_configure_cs_behavior(SPI_MASTER_BASE, sm[spi_num].spi_chip_sel, SPI_CS_RISE_NO_TX);
	
	spi_set_peripheral_chip_select_value(SPI_MASTER_BASE, sm[spi_num].spi_chip_sel);
	
	spi_set_clock_polarity(SPI_MASTER_BASE, sm[spi_num].spi_chip_sel, SPI_CLK_POLARITY);
	spi_set_clock_phase(SPI_MASTER_BASE, sm[spi_num].spi_chip_sel, SPI_CLK_PHASE);
	spi_set_bits_per_transfer(SPI_MASTER_BASE, sm[spi_num].spi_chip_sel, SPI_CSR_BITS_8_BIT);
	spi_set_baudrate_div(SPI_MASTER_BASE, sm[spi_num].spi_chip_sel, (sysclk_get_peripheral_hz() / SPI_CLK_SPEED));
	spi_set_transfer_delay(SPI_MASTER_BASE, sm[spi_num].spi_chip_sel, SPI_DLYBS, SPI_DLYBCT);

	spi_enable(SPI_MASTER_BASE);
	pdc_disable_transfer(sm[spi_num].p_pdc, PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS);

	return 0;
}

int spi_master_write_register(unsigned spi_num, uint8_t register_addr, uint32_t len, const uint8_t * value)
{
	register_addr &= WRITE_BIT_MASK;
	
	if(len+1 > SPI_BUFFER_SIZE)
		return -1;
	
	pdc_packet_t pdc_spi_packet;
	
	/* Desactivate Irq during buffer write*/
	cpu_irq_enter_critical();
	pdc_spi_packet.ul_addr = (uint32_t)&sm[spi_num].rx_buffer[0];
	pdc_spi_packet.ul_size = len + 1;
	pdc_rx_init(sm[spi_num].p_pdc, &pdc_spi_packet, NULL);

	sm[spi_num].tx_buffer[0] = (uint8_t) register_addr;
	memcpy(&sm[spi_num].tx_buffer[1], (uint8_t *)value, len);

	pdc_spi_packet.ul_addr = (uint32_t)&sm[spi_num].tx_buffer[0];
	pdc_spi_packet.ul_size = len + 1;
	pdc_tx_init(sm[spi_num].p_pdc, &pdc_spi_packet, NULL);
	
	/* Enable the RX and TX PDC transfer requests */
	pdc_enable_transfer(sm[spi_num].p_pdc, PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);
	/* Re activate Irq */
	cpu_irq_leave_critical();
	/* Waiting transfer done*/
	while((spi_read_status(SPI_MASTER_BASE) & SPI_SR_TXEMPTY) == 0);
		
	/* Disable the RX and TX PDC transfer requests */
	pdc_disable_transfer(sm[spi_num].p_pdc, PERIPH_PTCR_RXTDIS |
			PERIPH_PTCR_TXTDIS);
	
	return 0;
}

int spi_master_read_register(unsigned spi_num, uint8_t register_addr, uint32_t len, uint8_t * value)
{
	if(len+1 > SPI_BUFFER_SIZE)
		return -1;
	
	pdc_packet_t pdc_spi_packet;
	
	/* Desactivate Irq during buffer write*/
	cpu_irq_enter_critical();
	pdc_spi_packet.ul_addr = (uint32_t)&sm[spi_num].rx_buffer[0];
	pdc_spi_packet.ul_size = len + 1;
	pdc_rx_init(sm[spi_num].p_pdc, &pdc_spi_packet, NULL);
	
	sm[spi_num].tx_buffer[0] = (uint8_t) register_addr | READ_BIT_MASK;
	memset(&sm[spi_num].tx_buffer[1], 0x00, len);

	pdc_spi_packet.ul_addr = (uint32_t)&sm[spi_num].tx_buffer[0];
	pdc_spi_packet.ul_size = len + 1;
	pdc_tx_init(sm[spi_num].p_pdc, &pdc_spi_packet, NULL);
	/* Re activate Irq */
	cpu_irq_leave_critical();

	/* Enable the RX and TX PDC transfer requests */
	pdc_enable_transfer(sm[spi_num].p_pdc, PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);
	
	/* Waiting transfer done*/
	while((spi_read_status(SPI_MASTER_BASE) & SPI_SR_ENDRX) == 0);
	
	/* Disable the RX and TX PDC transfer requests */
	pdc_disable_transfer(sm[spi_num].p_pdc, PERIPH_PTCR_RXTDIS |
				PERIPH_PTCR_TXTDIS);
	
	memcpy(value, &sm[spi_num].rx_buffer[1], len);

	return 0;
}

void spi_master_deinit(unsigned spi_num)
{
	ioport_reset_pin_mode(PIO_PA14_IDX);
	ioport_reset_pin_mode(PIO_PA13_IDX);
	ioport_reset_pin_mode(PIO_PA12_IDX);
	ioport_reset_pin_mode(sm[spi_num].pio_cs_idx);

	flexcom_disable(BOARD_FLEXCOM_SPI);
	
	spi_disable(SPI_MASTER_BASE);
	spi_reset(SPI_MASTER_BASE);
	
	pdc_disable_transfer(sm[spi_num].p_pdc, PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS);

	return;
}

/**
 * \brief Interrupt handler for the SPI master.
 */
static void spi_master_irq_handler(void)
{
	/* Disable the RX and TX PDC transfer requests */
	pdc_disable_transfer(sm[INV_SPI_CS0].p_pdc, PERIPH_PTCR_RXTDIS |
			PERIPH_PTCR_TXTDIS);
	pdc_disable_transfer(sm[INV_SPI_CS1].p_pdc, PERIPH_PTCR_RXTDIS |
			PERIPH_PTCR_TXTDIS);
	
	NVIC_ClearPendingIRQ(FLEXCOM5_IRQn);

	if(SPI_MASTER_BASE->SPI_SR & SPI_SR_RXBUFF) {
		/* Disable SPI IRQ */
		spi_disable_interrupt(SPI_MASTER_BASE, SPI_IDR_RXBUFF);
	}
}

void FLEXCOM5_Handler(void)
{
	spi_master_irq_handler();
}
