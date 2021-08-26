/*
 Copyright © 2016-2019, Chirp Microsystems.  All rights reserved.

 Chirp Microsystems CONFIDENTIAL

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL CHIRP MICROSYSTEMS BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 You can contact the authors of this program by email at support@chirpmicro.com
 or by mail at 2560 Ninth Street, Suite 220, Berkeley, CA 94710.
 */

/**
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# Minimal main function that starts with a call to board_init()
 * -# Basic usage of on-board LED and button
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#include <asf.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include "chirp_smartsonic.h"
#include "soniclib.h"
#include "chirp_bsp.h"
#include "inv_uart.h"
#include "inv_i2c.h"
#include "pio_handler.h"
#include "ioport.h"
#include "pdc.h"
#include "time.h"

extern ch_group_t	*sensor_group_ptr;

i2c_trans_data_t	i2c_nb_transactions[CHBSP_NUM_I2C_BUSES];		// array of structures to track non-blocking I2C transactions

#define I2C_TIMEOUT_COUNT	(10000)		// loop counter to detect timeout in I2C handler

#ifdef ICM42688
extern void imu_data_ready_isr_callback(void);
#endif
void ext_MotionINT_handler(void);

static void ext_int_handler(uint32_t gpio_pin);
static void flexcom_handler(uint8_t port_index);

/**
 * \brief Parallel IO Controller A interrupt handler.
 * Redefined PIOA interrupt handler for NVIC interrupt table.
 *
 * Important note: on Smartsonic board, due to the level shifter on the INT line,
 *    the ISR must be as short as possible to ensure the INT level is set back to
 *    0 logical level before the CHx01 is ready for a new measurement. The maximum
 *    time for ISR is 8 us.
 */
void PIOA_Handler(void)
{
	/* Read the ISR and IMR registers to know which interrupt(s) is(are) pending */
	uint32_t status = pio_get_interrupt_status(PIOA);
	status &= pio_get_interrupt_mask(PIOA);

	if (status & PIN_EXT_MotionINT_MASK)
		ext_MotionINT_handler();
	if (status & PIN_EXT_ChirpINT0_MASK)
		ext_int_handler(0);
	if (status & PIN_EXT_ChirpINT1_MASK)
		ext_int_handler(1);
	if (status & PIN_EXT_ChirpINT2_MASK)
		ext_int_handler(2);
	if (status & PIN_EXT_ChirpINT3_MASK)
		ext_int_handler(3);
}

void ext_MotionINT_handler(void) {
#ifdef ICM42688
	imu_data_ready_isr_callback();
#endif // ICM42688
}

static void ext_int_handler(uint32_t sensor_id) {
	ch_io_int_callback_t func_ptr = sensor_group_ptr->io_int_callback;
	uint32_t gpio_pin = chirp_pin_io[sensor_id];

	/* Put the line in output to stabilize it to 0V until the next trig */
	ioport_set_pin_level(gpio_pin, IOPORT_PIN_LEVEL_LOW);	// set to low level
	ioport_set_pin_dir(gpio_pin, IOPORT_DIR_OUTPUT);		// set pin direction as output
	pio_disable_interrupt(PIN_EXT_INTERRUPT_PIO, chirp_pin_io_irq[sensor_id]); // disable interrupt

	if (func_ptr != NULL) {
		// Call application callback function - pass I/O index to identify interrupting device
		(*func_ptr)(sensor_group_ptr, sensor_id);
	}
}

void FLEXCOM1_Handler(void) {
	flexcom_handler(1);			// call local handler with index value
}
	
void FLEXCOM3_Handler(void) {
	flexcom_handler(3);			// call local handler with index value
}


void flexcom_handler(uint8_t port_index) {
	Twi *twi_ptr;
	Pdc *pdc_ptr;
	uint8_t *buf_ptr;
	uint32_t num_bytes;
	uint8_t	 bus_index = 0;
	uint32_t twi_status;
	uint32_t timeout_count = 0;
	uint8_t error = 0;

	if (port_index == 1) {
		twi_ptr = BOARD_BASE_TWI1;
		bus_index = 0;
	} else if (port_index == 3) {
		twi_ptr = BOARD_BASE_TWI3;
		bus_index = 1;
	} else {
		error = 1;		// bad port index
		return;
	}

	pdc_ptr   = twi_get_pdc_base(twi_ptr);
	buf_ptr   = i2c_nb_transactions[bus_index].buf_ptr;
	num_bytes = i2c_nb_transactions[bus_index].num_bytes;	

	twi_status = twi_get_interrupt_status(twi_ptr);
	twi_status &= twi_get_interrupt_mask(twi_ptr);


	if ((!error) && (twi_status & TWI_SR_ENDRX))  {
		/* Disable the RX PDC transfer requests */
		pdc_disable_transfer(pdc_ptr, PERIPH_PTCR_RXTDIS);

		/* Disable TWI interrupts */
		twi_disable_interrupt(twi_ptr, TWI_SR_ENDRX);

		/* Wait for next-to-last byte to be read */
		timeout_count = 0;
		while ((twi_ptr->TWI_SR & TWI_SR_RXRDY) == 0) {
			if (++timeout_count >= I2C_TIMEOUT_COUNT) {
				break;
			}
		}

		/* Set stop command */
		twi_ptr->TWI_CR = TWI_CR_STOP;

		buf_ptr[num_bytes-2] = twi_ptr->TWI_RHR;

		/* Wait for last byte to be read */
		timeout_count = 0;
		while ((twi_ptr->TWI_SR & TWI_SR_RXRDY) == 0) {
			if (++timeout_count >= I2C_TIMEOUT_COUNT) {
				break;
			}
		}

		buf_ptr[num_bytes-1] = twi_ptr->TWI_RHR;

		/* Wait for transfer to complete */
		timeout_count = 0;
		while ((twi_ptr->TWI_SR & TWI_SR_TXCOMP) == 0) {
			if (++timeout_count >= I2C_TIMEOUT_COUNT) {
				break;
			}
		}

		/* Notify sensor driver that this transaction is complete */
		ch_io_notify(sensor_group_ptr, bus_index);
	}

}


void RTT_Handler(void) {

	/* Get RTT status */
	rtt_get_status(RTT);
}

/* Interrupt handler for TC0/Channel0 peripheral */
void TC0_Handler(void)
{
	/* This handles the case of counter overflow on TC_CHANNEL_LSEPOCH and clears the status register */
	time_get_in_us();
}

/* Interrupt handler for TC0/Channel1 peripheral */
void TC1_Handler(void)
{
	uint32_t status = tc_get_status(TC0, TC_CHANNEL_US);
	uint32_t int_mask = tc_get_interrupt_mask(TC0, TC_CHANNEL_US);

	if (status & (int_mask & TC_IMR_CPCS)) {
		chbsp_periodic_timer_handler();
	}
}

void sensor_led_on(uint32_t chirp_led_pin) {
	ioport_set_pin_dir(chirp_led_pin, IOPORT_DIR_OUTPUT); 
	ioport_set_pin_level(chirp_led_pin, IOPORT_PIN_LEVEL_LOW); 
}	

void sensor_led_off(uint32_t chirp_led_pin) {
	ioport_set_pin_dir(chirp_led_pin, IOPORT_DIR_OUTPUT); 
	ioport_set_pin_level(chirp_led_pin, IOPORT_PIN_LEVEL_HIGH); 
}	

void sensor_led_toggle(uint32_t chirp_led_pin)
{
	ioport_toggle_pin_level(chirp_led_pin);
}

void indicate_alive(void) {
	ioport_set_pin_dir(CHIRP_OK_0, IOPORT_DIR_OUTPUT); //CHIRP_OK_0=output
	ioport_set_pin_dir(CHIRP_OK_1, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(CHIRP_OK_2, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(CHIRP_OK_3, IOPORT_DIR_OUTPUT);
	
	ioport_set_pin_level(CHIRP_OK_0, IOPORT_PIN_LEVEL_LOW); //CHIRP_OK_0=L LED=on
	ioport_set_pin_level(CHIRP_OK_1, IOPORT_PIN_LEVEL_LOW);
	ioport_set_pin_level(CHIRP_OK_2, IOPORT_PIN_LEVEL_LOW);
	ioport_set_pin_level(CHIRP_OK_3, IOPORT_PIN_LEVEL_LOW);
	
	delay_s(1);		//light up for 1s
	
	ioport_set_pin_level(CHIRP_OK_0, IOPORT_PIN_LEVEL_HIGH); //CHIRP_OK_0=H LED=off
	ioport_set_pin_level(CHIRP_OK_1, IOPORT_PIN_LEVEL_HIGH);
	ioport_set_pin_level(CHIRP_OK_2, IOPORT_PIN_LEVEL_HIGH);
	ioport_set_pin_level(CHIRP_OK_3, IOPORT_PIN_LEVEL_HIGH);
}



