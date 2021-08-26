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

#include <stdio.h>
#include <stdint.h>
#include <asf.h>

#include "chirp_smartsonic.h"		// header with board-specific defines
#include "soniclib.h"				// Chirp SonicLib API definitions
#include "chirp_bsp.h"
#include "time.h"

#include "board.h"					// board header in Atmel ASF
#include "ioport.h"
#include "inv_i2c.h"
#include "inv_uart.h"
#include "delay.h"
#include "pdc.h"
#include "adc2.h"
#include "pio_handler.h"
#include "rtt.h" 
#include "sleep.h"
#include "twi.h"

/** Reference voltage for ADC, in mV. */
#define VOLT_REF        (3277)
#define DEF_1KHZ		(1000)
#define MAX_CH			(8)
#define NUM_CH			(2)

/** The ADC maximal digital value */
#define MAX_DIGITAL_12_BIT (4095UL)
#define MAX_DIGITAL_13_BIT (8190UL)
#define MAX_DIGITAL_14_BIT (16381UL)
#define MAX_DIGITAL_15_BIT (32761UL)
#define MAX_DIGITAL_16_BIT (65521UL)

#define ADC_MODE_NO_AUTOTEST     0x00000000

static uint8_t chirp_i2c_addrs[] = CHIRP_I2C_ADDRS;
static uint8_t chirp_i2c_buses[] = CHIRP_I2C_BUSES;

/*
 * Here you set the pin masks for each of the prog pins
 */
uint32_t chirp_pin_prog[]   = CHIRP_PIN_PROG; 	
uint32_t chirp_pin_io[]     = CHIRP_PIN_IO;
uint32_t chirp_pin_io_irq[] = CHIRP_PIN_IO_IRQ;					
uint32_t chirp_led_pins[]   = CHIRP_PIN_LED;

/* Chirp sensor group pointer */
ch_group_t	*sensor_group_ptr;

/* Callback function pointers */
static ch_timer_callback_t  periodic_timer_callback_ptr = NULL;

static uint16_t periodic_timer_interval_ms;				//

static uint16_t ultrasound_timer_period_in_tick = 0xFFFF;
static uint16_t ultrasound_prev_period_end_in_tick;

/* Counter used to decimate call to ultrasound timer callback from TC0 ISR in case decimation
   factor is != 1 */
static uint8_t decimation_counter = 0;

/* Used in case the timer resolution and range (16 bits HW counter) overflows */
static uint8_t decimation_factor;

/** The maximal digital value */
static uint32_t g_max_digital;

static struct adc_config adc_cfg;
static Adc* adc_local = ADC;
static uint32_t* adc_reg = (uint32_t*) 0x40038094; //ADC_ACR

extern void ext_MotionINT_handler(void);

/* Forward declaration */
static void program_next_period(void);
static uint32_t get_period_in_tick(uint32_t interval_us);
static void Measure_power(void);
static void ADC0_init(void);
static void ext_int_init(void);
static void find_sensors(void);
static uint32_t Measure_Idd(unsigned int count);

void find_sensors(void) {
	uint8_t sig_bytes[2];
	
	ioport_set_pin_dir(CHIRP_RST, IOPORT_DIR_OUTPUT); //reset=output
	ioport_set_pin_level(CHIRP_RST, IOPORT_PIN_LEVEL_HIGH); //reset=H
	
	/* Drive PROG low on all sensor ports */
	ioport_set_pin_dir(CHIRP_PROG_0, IOPORT_DIR_OUTPUT); //PROG_0=output
	ioport_set_pin_dir(CHIRP_PROG_1, IOPORT_DIR_OUTPUT); //PROG_1=output
	ioport_set_pin_dir(CHIRP_PROG_2, IOPORT_DIR_OUTPUT); //PROG_2=output
	ioport_set_pin_dir(CHIRP_PROG_3, IOPORT_DIR_OUTPUT); //PROG_3=output
	ioport_set_pin_level(CHIRP_PROG_0, IOPORT_PIN_LEVEL_LOW); //PROG_0=L
	ioport_set_pin_level(CHIRP_PROG_1, IOPORT_PIN_LEVEL_LOW); //PROG_1=L
	ioport_set_pin_level(CHIRP_PROG_2, IOPORT_PIN_LEVEL_LOW); //PROG_2=L
	ioport_set_pin_level(CHIRP_PROG_3, IOPORT_PIN_LEVEL_LOW); //PROG_3=L
	
	/* check sensor 0 (on board chip or J6) */
	ioport_set_pin_level(CHIRP_PROG_0, IOPORT_PIN_LEVEL_HIGH);
	i2c_master_initialize1();
	sig_bytes[0] = 0;
	sig_bytes[1] = 0;
	i2c_master_read_register1(CH_I2C_ADDR_PROG, 0x00, 2, sig_bytes);
	printf("Chirp sensor 0 ");
	if ((sig_bytes[0] == CH_SIG_BYTE_0) && (sig_bytes[1] == CH_SIG_BYTE_1)) {
		printf("found\n");
		} else {
		printf("not found\n");
	}
	ioport_set_pin_level(CHIRP_PROG_0, IOPORT_PIN_LEVEL_LOW);

	/* check sensor 1 (J7) */
	ioport_set_pin_level(CHIRP_PROG_1, IOPORT_PIN_LEVEL_HIGH);
	i2c_master_initialize1();
	sig_bytes[0] = 0;
	sig_bytes[1] = 0;
	i2c_master_read_register1(CH_I2C_ADDR_PROG, 0x00, 2, sig_bytes);
	printf("Chirp sensor 1 ");
	if ((sig_bytes[0] == CH_SIG_BYTE_0) && (sig_bytes[1] == CH_SIG_BYTE_1)) {
		printf("found\n");
		} else {
		printf("not found\n");
	}
	ioport_set_pin_level(CHIRP_PROG_1, IOPORT_PIN_LEVEL_LOW);
	
	/* check sensor 2 (J8) */
	ioport_set_pin_level(CHIRP_PROG_2, IOPORT_PIN_LEVEL_HIGH);
	i2c_master_initialize3();
	sig_bytes[0] = 0;
	sig_bytes[1] = 0;
	i2c_master_read_register3(CH_I2C_ADDR_PROG, 0x00, 2, sig_bytes);
	printf("Chirp sensor 2 ");
	if ((sig_bytes[0] == CH_SIG_BYTE_0) && (sig_bytes[1] == CH_SIG_BYTE_1)) {
		printf("found\n");
		} else {
		printf("not found\n");
	}
	ioport_set_pin_level(CHIRP_PROG_2, IOPORT_PIN_LEVEL_LOW);
	
	/* check sensor 3 (J9) */
	ioport_set_pin_level(CHIRP_PROG_3, IOPORT_PIN_LEVEL_HIGH);
	i2c_master_initialize3();
	sig_bytes[0] = 0;
	sig_bytes[1] = 0;
	i2c_master_read_register3(CH_I2C_ADDR_PROG, 0x00, 2, sig_bytes);
	printf("Chirp sensor 3 ");
	if ((sig_bytes[0] == CH_SIG_BYTE_0) && (sig_bytes[1] == CH_SIG_BYTE_1)) {
		printf("found\n");
		} else {
		printf("not found\n");
	}
	ioport_set_pin_level(CHIRP_PROG_3, IOPORT_PIN_LEVEL_LOW);
}

void ADC0_init(void) {
	adc_enable();
	adc_select_clock_source_mck(ADC);
	adc_get_config_defaults(&adc_cfg);
	adc_init(ADC, &adc_cfg);
	adc_channel_enable(ADC, ADC_CHANNEL_0);
	adc_set_trigger(ADC, ADC_TRIG_FREERUN);

	g_max_digital = MAX_DIGITAL_12_BIT;
	adc_set_resolution(ADC, ADC_12_BITS);
	
	adc_set_writeprotect(ADC, 0);
	*adc_reg = ADC_MODE_NO_AUTOTEST;// Set ADC converter to normal operation
	adc_set_writeprotect(ADC, 1);
}

uint32_t Measure_Idd(unsigned int count) {
	uint32_t i;
	uint64_t Sum_Idd = 0;
	unsigned int Average_Idd, WaitTime, ODR = 100;
	
	for (i=0; i < count; ++i) {
		WaitTime = (1000000/ODR/10)-30; //10 samples per ODR cycle, it was for motion sensor
		delay_us(WaitTime);
		adc_local->ADC_CR = ADC_CR_START; //start ADC conversion
		delay_us(30); //wait for ADC conversion time
		Sum_Idd += adc_channel_get_value(ADC, ADC_CHANNEL_0);
	}

	Average_Idd = (Sum_Idd / count)* VOLT_REF / g_max_digital/0.6171; //5.1R sense, x100 and 1.21 Op-amp gain. mA to uA
	
	return Average_Idd;
}

void Measure_power(void) {
	uint32_t ul_vol;
	
	ul_vol = Measure_Idd(100);  //300 averaging
	printf("Chirp sensor Idd: %ld uA\n\n", ul_vol);
}

void ext_int_init(void)
{
	/* Enable the peripheral clock for the MAG extension board interrupt pin. */
	pmc_enable_periph_clk(PIN_EXT_INTERRUPT_ID);

	// Enable pull-downs on the INT pins
	pio_pull_up(PIN_EXT_INTERRUPT_PIO, 1 << CHIRP_INT_0 | 1 << CHIRP_INT_1 | 1 << CHIRP_INT_2 | 1 << CHIRP_INT_3, 0);
	pio_pull_down(PIN_EXT_INTERRUPT_PIO, 1 << CHIRP_INT_0 | 1 << CHIRP_INT_1 | 1 << CHIRP_INT_2 | 1 << CHIRP_INT_3, 1);

	/* Configure PIOs as input pins. */
	pio_configure(PIN_EXT_INTERRUPT_PIO, PIN_EXT_INTERRUPT_TYPE, PIN_EXT_MotionINT_MASK, PIN_EXT_INTERRUPT_ATTR);
	pio_configure(PIN_EXT_INTERRUPT_PIO, PIN_EXT_INTERRUPT_TYPE, PIN_EXT_ChirpINT0_MASK, PIN_EXT_INTERRUPT_ATTR);
	pio_configure(PIN_EXT_INTERRUPT_PIO, PIN_EXT_INTERRUPT_TYPE, PIN_EXT_ChirpINT1_MASK, PIN_EXT_INTERRUPT_ATTR);
	pio_configure(PIN_EXT_INTERRUPT_PIO, PIN_EXT_INTERRUPT_TYPE, PIN_EXT_ChirpINT2_MASK, PIN_EXT_INTERRUPT_ATTR);
	pio_configure(PIN_EXT_INTERRUPT_PIO, PIN_EXT_INTERRUPT_TYPE, PIN_EXT_ChirpINT3_MASK, PIN_EXT_INTERRUPT_ATTR);

	/* Initialize PIO interrupt handler, interrupt on rising edge. */
	pio_handler_set(PIN_EXT_INTERRUPT_PIO, PIN_EXT_INTERRUPT_ID, PIN_EXT_MotionINT_MASK,
		PIN_EXT_INTERRUPT_ATTR, NULL);
	pio_handler_set(PIN_EXT_INTERRUPT_PIO, PIN_EXT_INTERRUPT_ID, PIN_EXT_ChirpINT0_MASK,
		PIN_EXT_INTERRUPT_ATTR, NULL);
	pio_handler_set(PIN_EXT_INTERRUPT_PIO, PIN_EXT_INTERRUPT_ID, PIN_EXT_ChirpINT1_MASK,
		PIN_EXT_INTERRUPT_ATTR, NULL);
	pio_handler_set(PIN_EXT_INTERRUPT_PIO, PIN_EXT_INTERRUPT_ID, PIN_EXT_ChirpINT2_MASK,
		PIN_EXT_INTERRUPT_ATTR, NULL);
	pio_handler_set(PIN_EXT_INTERRUPT_PIO, PIN_EXT_INTERRUPT_ID, PIN_EXT_ChirpINT3_MASK,
		PIN_EXT_INTERRUPT_ATTR, NULL);

	/* Initialize and enable push button (PIO) interrupt. */
	pio_handler_set_priority(PIN_EXT_INTERRUPT_PIO, PIN_EXT_INTERRUPT_IRQn, 0);

	/* Disable Motion interrupt before IMU initialization */
	pio_disable_interrupt(PIN_EXT_INTERRUPT_PIO, PIN_EXT_MotionINT_MASK);
	/* Disable all CHx01 interrupts */
	pio_disable_interrupt(PIN_EXT_INTERRUPT_PIO, PIN_EXT_ChirpINT0_MASK);
	pio_disable_interrupt(PIN_EXT_INTERRUPT_PIO, PIN_EXT_ChirpINT1_MASK);
	pio_disable_interrupt(PIN_EXT_INTERRUPT_PIO, PIN_EXT_ChirpINT2_MASK);
	pio_disable_interrupt(PIN_EXT_INTERRUPT_PIO, PIN_EXT_ChirpINT3_MASK);
}

/*!
 * \brief Initialize board hardware
 *
 * \note This function performs all necessary initialization on the board.
 */
void chbsp_board_init(ch_group_t *grp_ptr) {

	/* Make local copy of group pointer */
	sensor_group_ptr = grp_ptr;

	/* Initialize group descriptor */
	grp_ptr->num_ports = CHBSP_MAX_DEVICES;
	grp_ptr->num_i2c_buses = CHBSP_NUM_I2C_BUSES;
	grp_ptr->rtc_cal_pulse_ms = CHBSP_RTC_CAL_PULSE_MS;
	
	/* Initialize the SAM system. */
	sysclk_init();
	board_init_I2C();

	configure_console();	
	
	ADC0_init();	
	ext_int_init();

	/* Probe I2C bus to find connected sensor(s) */
	find_sensors();
	Measure_power();

	indicate_alive();
}


/*!
 * \brief Assert the reset pin
 *
 * This function drives the sensor reset pin low.
 */
void chbsp_reset_assert(void) {

	ioport_set_pin_level(CHIRP_RST, IOPORT_PIN_LEVEL_LOW); //reset=L 
}

/*!
 * \brief Deassert the reset pin
 *
 * This function drives the sensor reset pin high.
 */
void chbsp_reset_release(void) {

	ioport_set_pin_level(CHIRP_RST, IOPORT_PIN_LEVEL_HIGH); //reset=H
}

/*!
 * \brief Assert the PROG pin
 *
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
 *
 * This function drives the sensor PROG pin high on the specified port.
 */
void chbsp_program_enable(ch_dev_t *dev_ptr) {
	uint8_t dev_num = ch_get_dev_num(dev_ptr);
	
	// select Chirp chip PROGRAM line on Atmel board according to chip number
	ioport_set_pin_level(chirp_pin_prog[dev_num], IOPORT_PIN_LEVEL_HIGH); //PROG_0=H
}

/*!
 * \brief Deassert the PROG pin
 *
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
 *
 * This function drives the sensor PROG pin low on the specified port.
 */
void chbsp_program_disable(ch_dev_t *dev_ptr) {
	uint8_t dev_num = ch_get_dev_num(dev_ptr);

	// select Chirp chip PROGRAM line on Atmel board according to chip number
	ioport_set_pin_level(chirp_pin_prog[dev_num], IOPORT_PIN_LEVEL_LOW); //PROG_0=L
}

/*!
 * \brief Configure the Chirp sensor INT pin as an output for one sensor.
 *
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
 *
 * This function configures the Chirp sensor INT pin as an output (from the perspective 
 * of the host system).
 */
void chbsp_set_io_dir_out(ch_dev_t *dev_ptr) {
	uint8_t dev_num = ch_get_dev_num(dev_ptr);

	ioport_set_pin_dir(chirp_pin_io[dev_num], IOPORT_DIR_OUTPUT);
}


/*!
 * \brief Configure the Chirp sensor INT pin as an input for one sensor.
 *
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
 *
 * This function configures the Chirp sensor INT pin as an input (from the perspective of 
 * the host system).
 */
void chbsp_set_io_dir_in(ch_dev_t *dev_ptr) {
	uint8_t dev_num = ch_get_dev_num(dev_ptr);

	ioport_set_pin_dir(chirp_pin_io[dev_num], IOPORT_DIR_INPUT);
}


/*!
 * \brief Configure the Chirp sensor INT pins as outputs for a group of sensors
 *
 * \param grp_ptr 		pointer to the ch_group_t config structure for a group of sensors
 *
 * This function configures each Chirp sensor's INT pin as an output (from the perspective 
 * of the host system).
 */
void chbsp_group_set_io_dir_out(ch_group_t *grp_ptr) {
	ioport_port_mask_t mask = 0;
	uint8_t dev_num;

	for (dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++)
	{
		ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

		if (ch_sensor_is_connected(dev_ptr))
			mask |= ioport_pin_to_mask(chirp_pin_io[dev_num]);
	}

	ioport_set_port_dir(IOPORT_PIOA, mask, IOPORT_DIR_OUTPUT);
}

/*!
 * \brief Configure the Chirp sensor INT pins as inputs for a group of sensors
 *
 * \param grp_ptr 		pointer to the ch_group_t config structure for a group of sensors
 * 
 * \note This function assumes a bidirectional level shifter is interfacing the ICs.
 */
void chbsp_group_set_io_dir_in(ch_group_t *grp_ptr) {
	ioport_port_mask_t mask = 0;
	uint8_t dev_num;

	for (dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++) {
		ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

		if (ch_sensor_is_connected(dev_ptr)) {
			mask |= ioport_pin_to_mask(chirp_pin_io[dev_num]);
		}
	}

	ioport_set_port_dir(IOPORT_PIOA, mask, IOPORT_DIR_INPUT);
}


/*!
 * \brief Initialize the I/O pins.
 *
 * \param grp_ptr 		pointer to the ch_group_t config structure for a group of sensors
 * 
 * Configure reset and program pins as outputs. Assert reset and program. Configure 
 * sensor INT pin as input.
 */
void chbsp_group_pin_init(ch_group_t *grp_ptr) {
	uint8_t dev_num;
	uint8_t port_num;

	ioport_set_pin_dir(CHIRP_PROG_0, IOPORT_DIR_OUTPUT); //PROG_0=output
	ioport_set_pin_dir(CHIRP_PROG_1, IOPORT_DIR_OUTPUT); //PROG_1=output
	ioport_set_pin_dir(CHIRP_PROG_2, IOPORT_DIR_OUTPUT); //PROG_2=output
	ioport_set_pin_dir(CHIRP_PROG_3, IOPORT_DIR_OUTPUT); //PROG_3=output

	ioport_set_pin_level(CHIRP_PROG_0, IOPORT_PIN_LEVEL_LOW); //PROG_0=L
	ioport_set_pin_level(CHIRP_PROG_1, IOPORT_PIN_LEVEL_LOW); //PROG_1=L
	ioport_set_pin_level(CHIRP_PROG_2, IOPORT_PIN_LEVEL_LOW); //PROG_2=L
	ioport_set_pin_level(CHIRP_PROG_3, IOPORT_PIN_LEVEL_LOW); //PROG_3=L	

	ioport_set_pin_dir(CHIRP_RST, IOPORT_DIR_OUTPUT); //reset=output
	chbsp_reset_assert();


	for (dev_num = 0; dev_num < grp_ptr->num_ports; dev_num++) {
		ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

		chbsp_program_enable(dev_ptr);
	}

	/* Initialize IO pins */
	chbsp_group_set_io_dir_in(grp_ptr);  

	/* Enable the peripheral clock for the MAG extension board interrupt pin. */
	pmc_enable_periph_clk(PIN_EXT_INTERRUPT_ID);
	
	/* Configure PIOs as input pins. */
	for(port_num = 0; port_num < grp_ptr->num_ports; port_num++ ) {
		pio_configure(PIN_EXT_INTERRUPT_PIO, PIN_EXT_INTERRUPT_TYPE, chirp_pin_io_irq[port_num], 
				      PIN_EXT_INTERRUPT_ATTR);
	}

	pio_configure(PIN_EXT_INTERRUPT_PIO, PIN_EXT_INTERRUPT_TYPE, PIN_EXT_MotionINT_MASK, 
			      PIN_EXT_INTERRUPT_ATTR);		//configure motionINT pin (although not used)
	
	/* Initialize PIO interrupt handler, interrupt on rising edge. */
	pio_handler_set(PIN_EXT_INTERRUPT_PIO, PIN_EXT_INTERRUPT_ID, chirp_pin_io_irq[0],
						PIN_EXT_INTERRUPT_ATTR, NULL);
								
	pio_handler_set(PIN_EXT_INTERRUPT_PIO, PIN_EXT_INTERRUPT_ID, PIN_EXT_MotionINT_MASK,
						PIN_EXT_INTERRUPT_ATTR, (void (*) (uint32_t, uint32_t))ext_MotionINT_handler);

	/* Initialize and enable push button (PIO) interrupt. */
	pio_handler_set_priority(PIN_EXT_INTERRUPT_PIO, PIN_EXT_INTERRUPT_IRQn, 0);		
	
}

/*!
 * \brief Set the INT pins low for a group of sensors.
 *
 * \param grp_ptr 		pointer to the ch_group_t config structure for a group of sensors
 * 
 * This function drives the INT line low for each sensor in the group.
 */
void chbsp_group_io_clear(ch_group_t *grp_ptr) {
	ioport_port_mask_t mask = 0;
	uint8_t dev_num;

	for (dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++) {
		ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

		if (ch_sensor_is_connected(dev_ptr)) {
			mask |= ioport_pin_to_mask(chirp_pin_io[dev_num]);
		}
	}

	ioport_set_port_level(IOPORT_PIOA, mask, IOPORT_PIN_LEVEL_LOW);
}

 /*!
 * \brief Set the INT pins high for a group of sensors.
 * 
 * \param grp_ptr 		pointer to the ch_group_t config structure for a group of sensors
 *
 * This function drives the INT line high for each sensor in the group.
 */
void chbsp_group_io_set(ch_group_t *grp_ptr) {
	uint8_t dev_num;
	ioport_port_mask_t mask = 0;

	for (dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++) {
		ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

		if (ch_sensor_is_connected(dev_ptr)) {
			mask |= ioport_pin_to_mask(chirp_pin_io[dev_num]);
		}
	}

	ioport_set_port_level(IOPORT_PIOA, mask, IOPORT_PIN_LEVEL_HIGH);
}


/*!
 * \brief Disable interrupts for a group of sensors
 * 
 * \param grp_ptr 	pointer to the ch_group_t config structure for a group of sensors
 *
 * For each sensor in the group, this function disables the host interrupt associated 
 * with the Chirp sensor device's INT line.
 */
void chbsp_group_io_interrupt_enable(ch_group_t *grp_ptr) {
	uint8_t dev_num;

	for (dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++) {
		ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

		chbsp_io_interrupt_enable(dev_ptr);
	}
}

/*!
 * \brief Enable the interrupt for one sensor
 *
 * \param dev_ptr	pointer to the ch_dev_t config structure for a sensor
 *
 * This function enables the host interrupt associated with the Chirp sensor device's 
 * INT line.
 */
void chbsp_io_interrupt_enable(ch_dev_t *dev_ptr) {
	uint8_t dev_num = ch_get_dev_num(dev_ptr);

	if (ch_sensor_is_connected(dev_ptr)) {
		pio_handler_clear_pending_IRQ(PIN_EXT_INTERRUPT_PIO, chirp_pin_io_irq[dev_num]);
		pio_enable_interrupt(PIN_EXT_INTERRUPT_PIO, chirp_pin_io_irq[dev_num]);
	}
}

/*!
 * \brief Disable interrupts for a group of sensors
 * 
 * \param grp_ptr 		pointer to the ch_group_t config structure for a group of sensors
 *
 * For each sensor in the group, this function disables the host interrupt associated 
 * with the Chirp sensor device's INT line.
 */
void chbsp_group_io_interrupt_disable(ch_group_t *grp_ptr) {
	uint8_t dev_num;

	for (dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++) {
		ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

		chbsp_io_interrupt_disable(dev_ptr);
	}
}

/*!
 * \brief Disable the interrupt for one sensor
 *
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
 *
 * This function disables the host interrupt associated with the Chirp sensor device's 
 * INT line.
 */
void chbsp_io_interrupt_disable(ch_dev_t *dev_ptr) {

	if (dev_ptr->sensor_connected) {
		pio_disable_interrupt(PIN_EXT_INTERRUPT_PIO, chirp_pin_io_irq[dev_ptr->io_index]);
	}
}

/*!
 * \brief Set the INT pins low for a one sensor.
 * 
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
 *
 * This function drives the INT line low for one sensor.
 */
void chbsp_io_clear(ch_dev_t *dev_ptr) {
	ioport_set_pin_level(chirp_pin_io[dev_ptr->io_index], IOPORT_PIN_LEVEL_LOW);
}

/*!
 * \brief Set the INT pins high for a one sensor.
 * 
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
 *
 * This function drives the INT line high for one sensor.
 */
void chbsp_io_set(ch_dev_t *dev_ptr) {
	ioport_set_pin_level(chirp_pin_io[dev_ptr->io_index], IOPORT_PIN_LEVEL_HIGH);
}

/*!
 * \brief Set callback routine for Chirp sensor I/O interrupt
 *
 * \param callback_func_ptr 	pointer to application function to be called when interrupt occurs
 *
 * This function sets up the specified callback routine to be called whenever the interrupt 
 * associated with the sensor's INT line occurs.  The callback routine address in stored in
 * a pointer variable that will later be accessed from within the interrupt handler to call 
 * the function.
 *
 * The callback function will be called at interrupt level from the interrupt 
 * service routine.
 */
void chbsp_io_callback_set(ch_io_int_callback_t callback_func_ptr) {

	io_int_callback_ptr = callback_func_ptr;
}


/*!
 * \brief Delay for specified number of microseconds
 * 
 * \param us  	number of microseconds to delay before returning
 *
 * This function waits for the specified number of microseconds before returning to 
 * the caller.
 */
void chbsp_delay_us(uint32_t us) {

	delay_us(us);
}

/*!
 * \brief Delay for specified number of milliseconds.
 *
 * \param ms 	number of milliseconds to delay before returning
 *
 * This function waits for the specified number of milliseconds before returning to 
 * the caller.
 */
void chbsp_delay_ms(uint32_t ms) {

	delay_us(ms*1000);
}

/*!
 * \brief Initialize the host's I2C hardware.
 *
 * \return 0 if successful, 1 on error
 *
 * This function performs general I2C initialization on the host system.
 */
int chbsp_i2c_init(void) {

	i2c_master_init(); 
	return 0;

}

/*!
 * \brief Return I2C information for a sensor port on the board.
 *
 * \param grp_ptr 		pointer to the ch_group_t config structure for a group of sensors
 * \param dev_num		device number within sensor group
 * \param info_ptr		pointer to structure to be filled with I2C config values
 *
 * \return 0 if successful, 1 if error
 *
 * This function returns I2C values in the ch_i2c_info_t structure specified by \a info_ptr.
 * The structure includes three fields.  
 *  - The \a address field contains the I2C address for the sensor.  
 *  - The \a bus_num field contains the I2C bus number (index).  
 *  - The \a drv_flags field contains various bit flags through which the BSP can inform 
 *  SonicLib driver functions to perform specific actions during I2C I/O operations.
 */
uint8_t chbsp_i2c_get_info(ch_group_t __attribute__((unused)) *grp_ptr, uint8_t io_index, ch_i2c_info_t *info_ptr) {
	uint8_t ret_val = 1;

	if (io_index <= CHBSP_MAX_DEVICES) {
		info_ptr->address = chirp_i2c_addrs[io_index];
		info_ptr->bus_num = chirp_i2c_buses[io_index];

		info_ptr->drv_flags = 0;	// no special I2C handling by SonicLib driver is needed

		ret_val = 0;
	}

	return ret_val;
}

/*!
 * \brief Write bytes to an I2C slave.
 * 
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
 * \param data 			data to be transmitted
 * \param num_bytes 	length of data to be transmitted
 *
 * \return 0 if successful, 1 on error or NACK
 *
 * This function writes one or more bytes of data to an I2C slave device.
 * The I2C interface must have already been initialized using \a chbsp_i2c_init().
 */
int chbsp_i2c_write(ch_dev_t *dev_ptr, uint8_t *data, uint16_t num_bytes) {
	int error = 0;

	if (dev_ptr->i2c_bus_index == 0) {
		error = i2c_master_write_register1_raw(dev_ptr->i2c_address, num_bytes, data); //I2C bus 0 (TWI1)
		
	} else if (dev_ptr->i2c_bus_index == 1) {
		error = i2c_master_write_register3_raw(dev_ptr->i2c_address, num_bytes, data); //I2C bus 1 (TWI3)
	}

	return error;
}

/*!
 * \brief Write bytes to an I2C slave using memory addressing.
 * 
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
 * \param mem_addr		internal memory or register address within device
 * \param data 			data to be transmitted
 * \param num_bytes 	length of data to be transmitted
 *
 * \return 0 if successful, 1 on error or NACK
 *
 * This function writes one or more bytes of data to an I2C slave device using an internal 
 * memory or register address.  The remote device will write \a num_bytes bytes of
 * data starting at internal memory/register address \a mem_addr.
 * The I2C interface must have already been initialized using \a chbsp_i2c_init().
 */
int chbsp_i2c_mem_write(ch_dev_t *dev_ptr, uint16_t mem_addr, uint8_t *data, uint16_t num_bytes) {
	int error=0;
	if (dev_ptr->i2c_bus_index == 0) {
		// I2C bus 0 (TWI1)
		error = i2c_master_write_register1(dev_ptr->i2c_address, mem_addr, num_bytes, data); 
		
		} else if (dev_ptr->i2c_bus_index == 1) {
		// I2C bus 1 (TWI3)
		error = i2c_master_write_register3(dev_ptr->i2c_address, mem_addr, num_bytes, data); 
	}
	return error;
}

/*!
 * \brief Write bytes to an I2C slave, non-blocking.
 *
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
 * \param data 			pointer to the start of data to be transmitted
 * \param num_bytes		length of data to be transmitted
 *
 * \return 0 if successful, 1 on error or NACK
 *
 * This function initiates a non-blocking write of the specified number of bytes to an 
 * I2C slave device. 
 *
 * The I2C interface must have already been initialized using \a chbsp_i2c_init().
 */
int chbsp_i2c_write_nb(ch_dev_t __attribute__((unused)) *dev_ptr, uint8_t __attribute__((unused)) *data, uint16_t __attribute__((unused)) num_bytes) {

	// XXX not implemented
	return 1;
}

/*!
 * \brief Write bytes to an I2C slave using memory addressing, non-blocking.
 *
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
 * \param mem_addr		internal memory or register address within device
 * \param data 			pointer to the start of data to be transmitted
 * \param num_bytes		length of data to be transmitted
 *
 * \return 0 if successful, 1 on error or NACK
 *
 * This function initiates a non-blocking write of the specified number of bytes to an 
 * I2C slave device, using an internal memory or register address.  The remote device will write 
 * \a num_bytes bytes of data starting at internal memory/register address \a mem_addr.
 *
 * The I2C interface must have already been initialized using \a chbsp_i2c_init().
 */
int chbsp_i2c_mem_write_nb(ch_dev_t __attribute__((unused)) *dev_ptr, uint16_t __attribute__((unused)) mem_addr, uint8_t __attribute__((unused)) *data, uint16_t __attribute__((unused)) num_bytes) {

	// XXX not implemented
	return 1;
}

/*!
 * \brief Read bytes from an I2C slave.
 * 
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
 * \param data 			pointer to receive data buffer
 * \param num_bytes 	number of bytes to read
 *
 * \return 0 if successful, 1 on error or NACK
 *
 * This function reads the specified number of bytes from an I2C slave device.
 * The I2C interface must have already been initialized using \a chbsp_i2c_init().
 */
int chbsp_i2c_read(ch_dev_t *dev_ptr, uint8_t *data, uint16_t num_bytes) {
	int error = 1;		// default is error return
	uint8_t i2c_addr = ch_get_i2c_address(dev_ptr);
	uint8_t bus_num  = ch_get_i2c_bus(dev_ptr);

	if (bus_num == 0) {
		// I2C bus 0 (TWI1)
		error = i2c_master_read_register1_raw(i2c_addr, num_bytes, data); 
		
	} else if (bus_num == 1) {
		// I2C bus 1 (TWI3)
		error = i2c_master_read_register3_raw(i2c_addr, num_bytes, data); 
	}
	return error;
}

/*!
 * \brief Read bytes from an I2C slave using memory addressing.
 * 
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
 * \param mem_addr		internal memory or register address within device
 * \param data 			pointer to receive data buffer
 * \param num_bytes 	number of bytes to read
 *
 * \return 0 if successful, 1 on error or NACK
 *
 * This function reads the specified number of bytes from an I2C slave device, using
 * an internal memory or register address.  The remote device will return \a num_bytes bytes
 * starting at internal memory/register address \a mem_addr.
 *
 * The I2C interface must have already been initialized using \a chbsp_i2c_init().
 */
int chbsp_i2c_mem_read(ch_dev_t *dev_ptr, uint16_t mem_addr, uint8_t *data, uint16_t num_bytes) {
	int error = 1;		// default is error return
	uint8_t i2c_addr = ch_get_i2c_address(dev_ptr);
	uint8_t bus_num  = ch_get_i2c_bus(dev_ptr);

	if (bus_num == 0) {
		// I2C bus 0 (TWI1)
		error = i2c_master_read_register1(i2c_addr, mem_addr, num_bytes, data); 
		
		} else if (bus_num == 1) {
		// I2C bus 1 (TWI3)
		error = i2c_master_read_register3(i2c_addr, mem_addr, num_bytes, data); 
	}
	return error;
}

/*!
 * \brief Read bytes from an I2C slave, non-blocking.
 *
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
 * \param data 			pointer to receive data buffer
 * \param num_bytes 	number of bytes to read
 *
 * \return 0 if successful, 1 on error or NACK
 *
 * This function initiates a non-blocking read of the specified number of bytes from 
 * an I2C slave.
 * 
 * The I2C interface must have already been initialized using \a chbsp_i2c_init().
 */
int chbsp_i2c_read_nb(ch_dev_t *dev_ptr, uint8_t *data, uint16_t num_bytes) {
	Twi *twi_ptr;		// pointer to Atmel TWI (I2C) control struct
	Pdc *pdc_ptr;		// pointer to Atmel peripheral DMA controller struct
	pdc_packet_t pdc_packet;
	uint8_t  i2c_addr = ch_get_i2c_address(dev_ptr);
	uint8_t  bus_num  = ch_get_i2c_bus(dev_ptr);
	int error = 0;

	if (bus_num == 0) {
		twi_ptr = BOARD_BASE_TWI1;
		pdc_ptr = twi_get_pdc_base(BOARD_BASE_TWI1);
	} else if (bus_num == 1) {
		twi_ptr = BOARD_BASE_TWI3;
		pdc_ptr = twi_get_pdc_base(BOARD_BASE_TWI3);
	} else {
		error = 1;
	}

	/* Save buffer pointer and transfer length - it will be needed when reading final 2 bytes 
	 * in DMA interrupt handler 
	 */
	if ((!error) && (data != NULL)) {
		i2c_nb_transactions[bus_num].buf_ptr = data;
		i2c_nb_transactions[bus_num].num_bytes = num_bytes;
	} else {
		error = 1;
	}

	if (!error) {
		/* Construct transfer packet */
	 	twi_packet_t twi_packet;

	 	twi_packet.chip = i2c_addr;		 	//address of I2C device to be accessed	
		twi_packet.addr[0] = 0;
		twi_packet.addr_length = 0;			// no internal mem/reg address
	 	twi_packet.buffer = data;
	 	twi_packet.length = num_bytes;
	 
		pdc_disable_transfer(pdc_ptr, PERIPH_PTCR_TXTDIS | PERIPH_PTCR_RXTDIS);

		pdc_packet.ul_addr = (uint32_t) data;
    	pdc_packet.ul_size = (num_bytes - 2);	

		pdc_rx_init(pdc_ptr, &pdc_packet, NULL);

		/* Set read mode, slave address, and internal address length */
		twi_ptr->TWI_MMR = 0;
		twi_ptr->TWI_MMR = TWI_MMR_MREAD | 
						   TWI_MMR_DADR(twi_packet.chip) |
						   ((twi_packet.addr_length << TWI_MMR_IADRSZ_Pos) & TWI_MMR_IADRSZ_Msk);

		/* No internal memory (register) address within remote device */
		twi_ptr->TWI_IADR = 0;
		
		/* Enable the RX PDC transfer requests */
		pdc_enable_transfer(pdc_ptr, PERIPH_PTCR_RXTEN);

		/* Start the transfer */
		twi_ptr->TWI_CR = TWI_CR_START;		
	
		/* Enable end-of-receive interrupt */
		twi_enable_interrupt(twi_ptr, TWI_IER_ENDRX);	
		
	}
	return error;
}

/*!
 * \brief Read bytes from an I2C slave using memory addressing, non-blocking.
 *
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
 * \param mem_addr		internal memory or register address within device
 * \param data 			pointer to receive data buffer
 * \param num_bytes 	number of bytes to read
 *
 * \return 0 if successful, 1 on error or NACK
 *
 * This function initiates a non-blocking read of the specified number of bytes from an I2C slave.
 *
 * The I2C interface must have already been initialized using \a chbsp_i2c_init().
 */
int chbsp_i2c_mem_read_nb(ch_dev_t *dev_ptr, uint16_t mem_addr, uint8_t *data, uint16_t num_bytes) {
	Twi *twi_ptr;		// pointer to Atmel TWI (I2C) control struct
	Pdc *pdc_ptr;		// pointer to Atmel peripheral DMA controller struct
	pdc_packet_t pdc_packet;
	uint8_t  i2c_addr = ch_get_i2c_address(dev_ptr);
	uint8_t  bus_num  = ch_get_i2c_bus(dev_ptr);
	int error=0;;

	if (bus_num == 0) {
		twi_ptr = BOARD_BASE_TWI1;
		pdc_ptr = twi_get_pdc_base(BOARD_BASE_TWI1);
	} else if (bus_num == 1) {
		twi_ptr = BOARD_BASE_TWI3;
		pdc_ptr = twi_get_pdc_base(BOARD_BASE_TWI3);
	} else {
		error = 1;
	}

	/* Save buffer pointer and transfer length - it will be needed when reading final 2 bytes 
	 * in DMA interrupt handler 
	 */
	if ((!error) && (data != NULL)) {
		i2c_nb_transactions[bus_num].buf_ptr = data;
		i2c_nb_transactions[bus_num].num_bytes = num_bytes;
	} else {
		error = 1;
	}

	if (!error) {
		/* Construct transfer packet */
	 	twi_packet_t twi_packet;
	 
	 	twi_packet.chip = i2c_addr; 			// address of I2C device to be accessed	
		twi_packet.addr[0] = mem_addr;			// internal mem address
		twi_packet.addr_length = 1;				// mem address is single byte
	 	twi_packet.buffer = data;
	 	twi_packet.length = num_bytes;
	 
		pdc_disable_transfer(pdc_ptr, PERIPH_PTCR_TXTDIS | PERIPH_PTCR_RXTDIS);

		pdc_packet.ul_addr = (uint32_t) data;
		pdc_packet.ul_size = (num_bytes - 2);

		pdc_rx_init(pdc_ptr, &pdc_packet, NULL);

		/* Set read mode, slave address, and internal address length */
		twi_ptr->TWI_MMR = 0;
		twi_ptr->TWI_MMR = TWI_MMR_MREAD | 
						   TWI_MMR_DADR(twi_packet.chip) |
						   ((twi_packet.addr_length << TWI_MMR_IADRSZ_Pos) & TWI_MMR_IADRSZ_Msk);

		/* Set internal memory (register) address within remote device */
		twi_ptr->TWI_IADR = 0;
		twi_ptr->TWI_IADR = twi_mk_addr(twi_packet.addr, twi_packet.addr_length);
		
		/* Enable the RX PDC transfer requests */
		pdc_enable_transfer(pdc_ptr, PERIPH_PTCR_RXTEN);

		/* Start the transfer */
		twi_ptr->TWI_CR = TWI_CR_START;		
	
		/* Enable end-of-receive interrupt */
		twi_enable_interrupt(twi_ptr, TWI_IER_ENDRX);	
		
	}
	return error;
}

/*!
 * \brief Reset I2C bus associated with device.
 *
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
 *
 * This function performs a reset of the I2C interface for the specified device.
 */
void chbsp_i2c_reset(ch_dev_t * dev_ptr) {
	uint8_t  bus_num  = ch_get_i2c_bus(dev_ptr);

	if (bus_num == 0) {					 // I2C bus 0 (TWI1)
		i2c_master_initialize1();
	} else if (bus_num == 1) {			 // I2C bus 1 (TWI3)
		i2c_master_initialize3();
	}
}

/*!
 * \brief Initialize periodic timer.
 *
 * \param interval_ms		timer interval, in milliseconds
 * \param callback_func_ptr	address of routine to be called every time the timer expires
 *
 * \return 0 if successful, 1 if error
 *
 * This function initializes a periodic timer on the board.  The timer is programmed 
 * to generate an interrupt after every \a interval_ms milliseconds.  
 *
 * The \a callback_func_ptr parameter specifies a callback routine that will be called when the 
 * timer expires (and interrupt occurs).  The \a chbsp_periodic_timer_handler function 
 * will call this function.
 */
uint8_t chbsp_periodic_timer_init(uint16_t interval_ms, ch_timer_callback_t callback_func_ptr) {
	static bool is_hw_init_done = false;

	/* Save timer interval and callback function */
	periodic_timer_interval_ms = interval_ms;
	periodic_timer_callback_ptr = callback_func_ptr;

	/* Initialize the HW only 1 time at startup. Skip the init on subsequent calls. */
	if (!is_hw_init_done) {
		/* Configure the PMC to enable the TC module and channels */
		sysclk_enable_peripheral_clock(ID_TC0);
		sysclk_enable_peripheral_clock(ID_TC1);
		/* Create on PCK3 a 499985 Hz clock from the PLLA clock. */
		pmc_disable_pck(PMC_PCK_3);
		pmc_switch_pck_to_pllack(PMC_PCK_3, PMC_PCK_PRES(240 - 1));
		pmc_enable_pck(PMC_PCK_3);

		/* Reset all TC0 counters */
		TC0->TC_BCR = TC_BCR_SYNC;

		/* Enable TC0 - Channel 0 interrupt */
		NVIC_DisableIRQ(TC0_IRQn);
		NVIC_ClearPendingIRQ(TC0_IRQn);
		NVIC_SetPriority(TC0_IRQn, 1);
		NVIC_EnableIRQ(TC0_IRQn);

		/* Enable TC0 - Channel 1 interrupt */
		NVIC_DisableIRQ(TC1_IRQn);
		NVIC_ClearPendingIRQ(TC1_IRQn);
		NVIC_SetPriority(TC1_IRQn, 1);
		NVIC_EnableIRQ(TC1_IRQn);

		/* Create the lsepoch timer running on PCK3 and start it immediately */
		tc_init(TC0, TC_CHANNEL_LSEPOCH,
			TC_CMR_TCCLKS_TIMER_CLOCK5 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP);
		tc_enable_interrupt(TC0, TC_CHANNEL_LSEPOCH, TC_IER_COVFS);
		tc_start(TC0, TC_CHANNEL_LSEPOCH);

		/* Create the ultrasound periodic timer. */
		tc_init(TC0, TC_CHANNEL_US,
			TC_CMR_TCCLKS_TIMER_CLOCK5 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP);
	}

	/* Mark the HW init as done */
	is_hw_init_done = true;

	/* Convert the ODR in ms to ticks */
	ultrasound_timer_period_in_tick = get_period_in_tick(interval_ms * 1000);
	
	return 0;
}

void chbsp_periodic_timer_change_period(uint32_t new_interval_us)
{
    uint16_t prev_expiration = ultrasound_prev_period_end_in_tick - ultrasound_timer_period_in_tick;

    ultrasound_timer_period_in_tick = get_period_in_tick(new_interval_us);

    ultrasound_prev_period_end_in_tick = prev_expiration;
	
    program_next_period();
}

uint32_t get_period_in_tick(uint32_t interval_us) {
	uint64_t timer_period_in_tick = (uint64_t) ULTRASOUND_TIMER_FREQUENCY * interval_us / 1000000;

	/* If the ODR is too slow to be handled then program a faster interrupt and decimate it */
	if (timer_period_in_tick > UINT16_MAX)
		decimation_factor = timer_period_in_tick / UINT16_MAX + 1;
	else
		decimation_factor = 1;

	/* Calculate the final tick in case a decimation is needed */
	return (uint32_t) (timer_period_in_tick / decimation_factor);	
}

void program_next_period(void)
{
	uint32_t time = ultrasound_prev_period_end_in_tick + ultrasound_timer_period_in_tick;
	ultrasound_prev_period_end_in_tick = time;
	tc_write_rc(TC0, TC_CHANNEL_US, (uint16_t) (time & 0xFFFF));
}

/*!
 * \brief Enable periodic timer interrupt.
 *
 * This function enables the interrupt associated with the periodic timer initialized by 
 * \a chbsp_periodic_timer_init().
 */
void chbsp_periodic_timer_irq_enable(void) {

	/* Clear any pending CPCS before enabling it */
	tc_get_status(TC0, TC_CHANNEL_US);
	tc_enable_interrupt(TC0, TC_CHANNEL_US, TC_IER_CPCS);
}

/*!
 * \brief Disable periodic timer interrupt.
 *
 * This function enables the interrupt associated with the periodic timer initialized by 
 * \a chbsp_periodic_timer_init().
 */
void chbsp_periodic_timer_irq_disable(void) {

	tc_disable_interrupt(TC0, TC_CHANNEL_US, TC_IDR_CPCS);
}


/*!
 * \brief Start periodic timer.
 *
 * \return 0 if successful, 1 if error
 *
 * This function starts the periodic timer initialized by \a chbsp_periodic_timer_init().
 */
uint8_t chbsp_periodic_timer_start(void) {

	decimation_counter = 0;
	/* The timer start done at the very end is resetting the counter */
	ultrasound_prev_period_end_in_tick = 0;
	program_next_period();

	/* Start the HW counter (this resets the counter */
	tc_start(TC0, TC_CHANNEL_US);

	return 0;
}


/*!
 * \brief Stop periodic timer.
 *
 * \return 0 if successful, 1 if error
 *
 * This function stops the periodic timer initialized by \a chbsp_periodic_timer_init().
 */
uint8_t chbsp_periodic_timer_stop(void) {

	tc_stop(TC0, TC_CHANNEL_US);
	return 0;
}


/*!
 * \brief Periodic timer handler.
 *
 * \return 0 if successful, 1 if error
 *
 * This function handles the expiration of the periodic timer, re-arms it and any associated 
 * interrupts for the next interval, and calls the callback routine that was registered using 
 * \a chbsp_periodic_timer_init().
 */
void chbsp_periodic_timer_handler(void) {
	ch_timer_callback_t func_ptr = periodic_timer_callback_ptr;

	decimation_counter++;
	program_next_period();
	if (decimation_counter >= decimation_factor) {
		decimation_counter = 0;
		if (func_ptr != NULL) {
			(*func_ptr)();			// call application timer callback routine
		}
	}
}


/*!
 * \brief Put the processor into low-power sleep state.
 *
 * This function puts the host processor (MCU) into a low-power sleep mode, to conserve energy. 
 * The sleep state should be selected such that interrupts associated with the I2C, external 
 * GPIO pins, and the periodic timer (if used) are able to wake up the device.
 */
void chbsp_proc_sleep(void) {

	pmc_sleep(PROC_SLEEP_MODE);			// use sleep mode defined in chirp_smartsonic.h
}	

/*!
 * \brief Turn on an LED on the board.
 *
 * This function turns on an LED on the board. 
 *
 * The \a dev_num parameter contains the device number of a specific sensor.  This routine
 * will turn on the LED on the Chirp sensor daughterboard that is next to the specified
 * sensor.
 */
void chbsp_led_on(uint8_t led_num) {

	sensor_led_on(chirp_led_pins[led_num]);
}
	
/*!
 * \brief Turn off an LED on the board.
 *
 * This function turns off an LED on the board. 
 *
 * The \a dev_num parameter contains the device number of a specific sensor.  This routine
 * will turn off the LED on the Chirp sensor daughterboard that is next to the specified
 * sensor.
 */
void chbsp_led_off(uint8_t led_num) {

	sensor_led_off(chirp_led_pins[led_num]);
}

/*!
 * \brief Toggles an LED on the board.
 *
 * This function toggles an LED on the board. 
 *
 * The \a dev_num parameter contains the device number of a specific sensor.  This routine
 * will toggles the LED on the Chirp sensor daughterboard that is next to the specified
 * sensor.
 */
void chbsp_led_toggle(uint8_t led_num) {

	sensor_led_toggle(chirp_led_pins[led_num]);
}
	
/*!
 * \brief Output a text string via serial interface
 *
 * \param str	pointer to a string of characters to be output
 * 
 * This function prints debug information to the console.
 */
void chbsp_print_str(char *str) {
	printf(str);
}

/*!
 * \brief Return the current time in ms
 *
 * This function returns the system current time in ms.
 */
uint32_t chbsp_timestamp_ms(void) {
	uint32_t time = time_get_in_us();
	return (time / 1000);
}

/************* End of file chbsp_chirp_smartsonic.c  --   Copyright 2016-2019 Chirp Microsystems **********/
