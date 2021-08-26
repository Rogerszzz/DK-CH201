/*
Copyright © 2016-2019, Chirp Microsystems.  All rights reserved.
All rights reserved.

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
/** \file chirp_smartsonic.h */
#ifndef CHIRP_SMARTSONIC_H
#define CHIRP_SMARTSONIC_H

#include "soniclib.h"
#include "chirp_board_config.h"
#include "app_config.h"

/* Standard symbols used in board support package - use values from config header */
#define CHBSP_MAX_DEVICES 		CHIRP_MAX_NUM_SENSORS
#define CHBSP_NUM_I2C_BUSES 	CHIRP_NUM_I2C_BUSES

#if defined(CHIRP_RTC_CAL_PULSE_LEN_MS)
	/** Length of real-time clock calibration pulse, in milliseconds :
	 * length of pulse applied to sensor INT line during clock cal
	 */
	#define CHBSP_RTC_CAL_PULSE_MS CHIRP_RTC_CAL_PULSE_LEN_MS
#else
	/* Default value */
	#define CHBSP_RTC_CAL_PULSE_MS	(100)
#endif

/* I2C Address assignments for each possible device */
#define CHIRP_I2C_ADDRS		{45, 43, 44, 42 }
#define CHIRP_I2C_BUSES		{ 0,  0,  1,  1 }

/* IRQ assignments */
#define TWI1_IRQn           FLEXCOM1_IRQn
#define TWI3_IRQn           FLEXCOM3_IRQn

/* Processor sleep mode */
#define	PROC_SLEEP_MODE		SAM_PM_SMODE_SLEEP_WFI		/* wait for interrupt */

/* Structure to track non-blocking I2C transaction data */
typedef struct {
	uint8_t		*buf_ptr;		/* pointer to data buffer */
	uint16_t	num_bytes;		/* number of bytes to transfer */
} i2c_trans_data_t;

/* TC channel used for the ultrasound timer and lsepoch of the system */
#define TC_CHANNEL_LSEPOCH (0)
#define TC_CHANNEL_US      (1)

/* Define the HW frequency of the TC used for the ultrasound periodic timer */
#define ULTRASOUND_TIMER_FREQUENCY (499985) /* = 32768 * 3662 / 240 */
#define ULTRASOUND_DECIMATION_FACTOR (1)

extern uint32_t chirp_pin_prog[CHBSP_MAX_DEVICES];
extern uint32_t chirp_pin_io[CHBSP_MAX_DEVICES];
extern uint32_t chirp_pin_io_irq[CHBSP_MAX_DEVICES];
extern uint32_t chirp_led_pins[];

extern ch_group_t chirp_group;

extern i2c_trans_data_t	i2c_nb_transactions[CHBSP_NUM_I2C_BUSES];		// array of structures to track non-blocking I2C transactions

extern void sensor_led_on(uint32_t pin);
extern void sensor_led_off(uint32_t pin);
extern void sensor_led_toggle(uint32_t pin);
extern void indicate_alive(void);

extern ch_io_int_callback_t io_int_callback_ptr;	// pointer to sensor I/O interrupt callback function
extern void periodic_timer_callback(void);


#endif /* CHIRP_SMARTSONIC_H */