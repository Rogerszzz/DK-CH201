/*! \file ch_common.c
 *
 * \brief Chirp SonicLib API function common implementations
 * 
 * This file contains standard implementations of functions required to support the
 * SonicLib API.  The sensor firmware, in it's init routine, specifies which of these
 * common implementations should be used by initializing a set of function pointers.
 * These pointers, contained in the ch_api_funcs_t structure within the device descriptor,
 * can either direct the API calls to the functions in this file or to firmware-specific
 * equivalents that are supplied as part of the sensor firmware release.
 *
 */

/*
 Copyright © 2019-2020, Chirp Microsystems.  All rights reserved.

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


#include "soniclib.h"
#include "ch_common.h"
#include "chirp_bsp.h"
#include "ch_math_utils.h"

/* Local definitions */
#define CH_IQ_SAMPLES_PER_READ		64		// number of I/Q samples to read at a time

/* Forward references */
static uint8_t get_sample_data(ch_dev_t *dev_ptr, ch_iq_sample_t *buf_ptr, uint16_t start_sample, uint16_t num_samples,
							   ch_io_mode_t mode, uint8_t sample_size_in_byte);

/* Functions */

uint8_t ch_common_set_mode(ch_dev_t *dev_ptr, ch_mode_t mode) {
	uint8_t ret_val = 0;
	uint8_t	opmode_reg;
	uint8_t	period_reg;
	uint8_t	tick_interval_reg;

	if (dev_ptr->part_number == CH101_PART_NUMBER) {
		opmode_reg = CH101_COMMON_REG_OPMODE;
		period_reg = CH101_COMMON_REG_PERIOD;
		tick_interval_reg = CH101_COMMON_REG_TICK_INTERVAL;
	} else {
		opmode_reg = CH201_COMMON_REG_OPMODE;
		period_reg = CH201_COMMON_REG_PERIOD;
		tick_interval_reg = CH201_COMMON_REG_TICK_INTERVAL;
	}

	if (dev_ptr->sensor_connected) {
		switch (mode) {
			case CH_MODE_IDLE:
				chdrv_write_byte(dev_ptr, opmode_reg, CH_MODE_IDLE);
				chdrv_write_byte(dev_ptr, period_reg, 0);
				chdrv_write_word(dev_ptr, tick_interval_reg, 2048);		// XXX need define
				break;

			case CH_MODE_FREERUN:
				chdrv_write_byte(dev_ptr, opmode_reg, CH_MODE_FREERUN);
					// XXX need to set period / tick interval (?)
				break;

			case CH_MODE_TRIGGERED_TX_RX:
				chdrv_write_byte(dev_ptr, opmode_reg, CH_MODE_TRIGGERED_TX_RX);
				break;

			case CH_MODE_TRIGGERED_RX_ONLY:
				chdrv_write_byte(dev_ptr, opmode_reg, CH_MODE_TRIGGERED_RX_ONLY);
				break;

			default:
				ret_val = RET_ERR;				// return non-zero to indicate error
				break;
		}
	}

	return ret_val;
}

uint8_t ch_common_fw_load(ch_dev_t *dev_ptr) {
	uint8_t	ch_err = 0;
	uint16_t prog_mem_addr;
	uint16_t fw_size;

	if (dev_ptr->part_number == CH101_PART_NUMBER) {
		prog_mem_addr = CH101_PROG_MEM_ADDR;
		fw_size 	  = CH101_FW_SIZE;
	} else {
		prog_mem_addr = CH201_PROG_MEM_ADDR;
		fw_size 	  = CH201_FW_SIZE;
	}

	ch_err = chdrv_prog_mem_write(dev_ptr, prog_mem_addr, (uint8_t *) dev_ptr->firmware, fw_size);
	return ch_err;
}


uint8_t ch_common_set_sample_interval(ch_dev_t *dev_ptr, uint16_t interval_ms) {
	uint8_t	period_reg;
	uint8_t	tick_interval_reg;
	uint8_t ret_val = 0;

	if (dev_ptr->part_number == CH101_PART_NUMBER) {
		period_reg 		  = CH101_COMMON_REG_PERIOD;
		tick_interval_reg = CH101_COMMON_REG_TICK_INTERVAL;
	} else {
		period_reg 		  = CH201_COMMON_REG_PERIOD;
		tick_interval_reg = CH201_COMMON_REG_TICK_INTERVAL;
	}

	if (dev_ptr->sensor_connected) {
		uint32_t sample_interval = dev_ptr->rtc_cal_result * interval_ms / dev_ptr->group->rtc_cal_pulse_ms;
		uint32_t period;

		if (interval_ms != 0) {
			period = (sample_interval / 2048) + 1;				// XXX need define
			if (period > UINT8_MAX) {					/* check if result fits in register */
				ret_val = 1;
			}
		} else {
			period = 0;
		}

		if (ret_val == 0) {
			uint32_t tick_interval;

			if (period != 0) {
				tick_interval = sample_interval / period;
			} else {
				tick_interval = 0;
			}

#ifdef CHDRV_DEBUG
			char cbuf[80];
			snprintf(cbuf, sizeof(cbuf), "Set period=%lu, tick_interval=%lu\n", period, tick_interval);
			chbsp_print_str(cbuf);
#endif
			chdrv_write_byte(dev_ptr, period_reg, (uint8_t) period);
			chdrv_write_word(dev_ptr, tick_interval_reg, (uint16_t) tick_interval);
		}
	}

	return ret_val;
}

// Note: uses actual num_samples, even for CH201
uint8_t ch_common_set_num_samples(ch_dev_t *dev_ptr, uint16_t num_samples ) {
	uint8_t max_range_reg;
	uint8_t ret_val = 1;		// default is error (not connected or num_samples too big)

	if (dev_ptr->part_number == CH101_PART_NUMBER) {
		max_range_reg = CH101_COMMON_REG_MAX_RANGE;
	} else {
		max_range_reg = CH201_COMMON_REG_MAX_RANGE;
		num_samples /= 2;					// each internal count for CH201 represents 2 physical samples
	}

	if (dev_ptr->sensor_connected && (num_samples <= UINT8_MAX)) {
		ret_val = chdrv_write_byte(dev_ptr, max_range_reg, num_samples);
	}

	if (!ret_val) {
		if (dev_ptr->part_number == CH101_PART_NUMBER) {
			dev_ptr->num_rx_samples = num_samples;
		} else {
			dev_ptr->num_rx_samples = (num_samples * 2);	// store actual physical sample count
		}
	} else {
		dev_ptr->num_rx_samples = 0;
	}
	
	return ret_val;
}


uint8_t ch_common_set_max_range(ch_dev_t *dev_ptr, uint16_t max_range_mm) {
	uint8_t ret_val;
	uint32_t num_samples;

	ret_val = (!dev_ptr->sensor_connected);

	if (!ret_val) {
		num_samples = dev_ptr->api_funcs.mm_to_samples(dev_ptr, max_range_mm);

		if (num_samples > dev_ptr->max_samples) {
			num_samples = dev_ptr->max_samples;
			dev_ptr->max_range = ch_samples_to_mm(dev_ptr, num_samples);	// store reduced max range
		} else {
			dev_ptr->max_range = max_range_mm;							// store user-specified max range
		}


#ifdef CHDRV_DEBUG
		char cbuf[80];
		snprintf(cbuf, sizeof(cbuf), "num_samples=%lu\n", num_samples);
		chbsp_print_str(cbuf);
#endif
	}

	if (!ret_val) {
		ret_val = ch_set_num_samples(dev_ptr, num_samples);
	}

#ifdef CHDRV_DEBUG
	printf("Set samples: ret_val: %u  dev_ptr->num_rx_samples: %u\n", ret_val, dev_ptr->num_rx_samples);
#endif
	return ret_val;
}


uint16_t ch_common_mm_to_samples(ch_dev_t *dev_ptr, uint16_t num_mm) {
	uint8_t err;
	uint16_t scale_factor;
	uint32_t num_samples = 0;
	uint32_t divisor1;
	uint32_t divisor2 = (dev_ptr->group->rtc_cal_pulse_ms * CH_SPEEDOFSOUND_MPS);

	err = (!dev_ptr) || (!dev_ptr->sensor_connected);

	if (!err) {
		if (dev_ptr->part_number == CH101_PART_NUMBER) {
			divisor1 = 0x2000;			// (4*16*128)  XXX need define(s)
		} else {
			divisor1 = 0x4000;			// (4*16*128*2)  XXX need define(s)
		}

		if (dev_ptr->scale_factor == 0) {
			ch_common_store_scale_factor(dev_ptr);
		}

		scale_factor = dev_ptr->scale_factor;
	}

	if (!err) {
		// Two steps of division to avoid needing a type larger than 32 bits
		// Ceiling division to ensure result is at least enough samples to meet specified range
		// Oversample value is signed power of two for this firmware relative to standard f/8 sampling.

		num_samples = ((dev_ptr->rtc_cal_result * scale_factor) + (divisor1 - 1)) / divisor1;

		num_samples = (((num_samples * num_mm) << dev_ptr->oversample) + (divisor2 - 1)) / divisor2;

		err = (num_samples > UINT16_MAX);
	}

	if (!err) {
		if (dev_ptr->part_number == CH201_PART_NUMBER) {
			num_samples *= 2;			// each internal count for CH201 represents 2 physical samples
		}
	}

	if (err) {
		num_samples = 0;		// return zero if error
	}

	return (uint16_t) num_samples;
}


uint16_t ch_common_samples_to_mm(ch_dev_t *dev_ptr, uint16_t num_samples) {
	uint32_t	num_mm = 0;
	uint32_t	op_freq = dev_ptr->op_frequency;

	if (op_freq != 0) {
		num_mm = ((uint32_t) num_samples * CH_SPEEDOFSOUND_MPS * 8 * 1000) / (op_freq * 2);
	}

	/* Adjust for oversampling, if used */
	num_mm >>= dev_ptr->oversample;

	return (uint16_t) num_mm;
}



uint8_t ch_common_set_static_range(ch_dev_t *dev_ptr, uint16_t samples) {
	uint8_t ret_val = 1;  	// default is error return

	if (dev_ptr->part_number == CH101_PART_NUMBER) {			// CH101 only
		if (dev_ptr->sensor_connected) {
			ret_val = chdrv_write_byte(dev_ptr, CH101_COMMON_REG_STAT_RANGE, samples);

			if (!ret_val) {
				ret_val = chdrv_write_byte(dev_ptr, CH101_COMMON_REG_STAT_COEFF, 
						                   CH101_COMMON_STAT_COEFF_DEFAULT);
			}

			if (!ret_val) {
				dev_ptr->static_range = samples;
			}
		}
	}
	return ret_val;
}

uint32_t ch_common_get_range(ch_dev_t *dev_ptr, ch_range_t range_type) {
	uint8_t		tof_reg;
	uint32_t	range = CH_NO_TARGET;
	uint16_t 	time_of_flight;
	uint16_t 	scale_factor;
	int 		err;

	if (dev_ptr->sensor_connected) {

		if (dev_ptr->part_number == CH101_PART_NUMBER) {
			tof_reg = CH101_COMMON_REG_TOF;
		} else {
			tof_reg = CH201_COMMON_REG_TOF;
		}

		err = chdrv_read_word(dev_ptr, tof_reg, &time_of_flight);

		if (!err && (time_of_flight != UINT16_MAX)) { // If object detected

			if (dev_ptr->scale_factor == 0) {
				ch_common_store_scale_factor(dev_ptr);
			}
			scale_factor = dev_ptr->scale_factor;

			if (scale_factor != 0) {
				uint32_t num = (CH_SPEEDOFSOUND_MPS * dev_ptr->group->rtc_cal_pulse_ms * (uint32_t) time_of_flight);
				uint32_t den = ((uint32_t) dev_ptr->rtc_cal_result * (uint32_t) scale_factor) >> 11;		// XXX need define

				range = (num / den);

				if (dev_ptr->part_number == CH201_PART_NUMBER) {
					range *= 2;
				}

				if (range_type == CH_RANGE_ECHO_ONE_WAY) {
					range /= 2;
				}

				/* Adjust for oversampling, if used */
				range >>= dev_ptr->oversample;

				/* If rx-only node, adjust for pre-trigger time included in ToF */
				if (dev_ptr->mode == CH_MODE_TRIGGERED_RX_ONLY) {
					uint32_t pretrig_adj = (CH_SPEEDOFSOUND_MPS * dev_ptr->group->pretrig_delay_us * 32) / 1000;

					if (range > pretrig_adj) {
						range -= pretrig_adj;			// subtract adjustment from calculated range
					} else {
						range = CH_MIN_RANGE_VAL;		// underflow - range is very close to zero, use minimum value
					}
				}
			}
		}
	}
	return range;
}


uint16_t ch_common_get_amplitude(ch_dev_t *dev_ptr) {
	uint8_t  amplitude_reg;
	uint16_t amplitude = 0;

	if (dev_ptr->sensor_connected) {
		if (dev_ptr->part_number == CH101_PART_NUMBER) {
			amplitude_reg = CH101_COMMON_REG_AMPLITUDE;
		} else {
			amplitude_reg = CH201_COMMON_REG_AMPLITUDE;
		}

		chdrv_read_word(dev_ptr, amplitude_reg, &amplitude);
	}

	return amplitude;
}


uint8_t ch_common_get_locked_state(ch_dev_t *dev_ptr) {
	uint8_t ready_reg;
	uint8_t lock_mask = dev_ptr->freqLockValue;
	uint8_t ret_val = 0;

	if (dev_ptr->part_number == CH101_PART_NUMBER) {
		ready_reg = CH101_COMMON_REG_READY;
	} else {
		ready_reg = CH201_COMMON_REG_READY;
	}

	if (dev_ptr->sensor_connected) {
		uint8_t ready_value = 0;
		chdrv_read_byte(dev_ptr, ready_reg, &ready_value);
		if (ready_value & lock_mask) {
			ret_val = 1;
		}
	}
	return ret_val;
}

void ch_common_prepare_pulse_timer(ch_dev_t *dev_ptr) {
	uint8_t cal_trig_reg;

	if (dev_ptr->part_number == CH101_PART_NUMBER) {
		cal_trig_reg = CH101_COMMON_REG_CAL_TRIG;
	} else {
		cal_trig_reg = CH201_COMMON_REG_CAL_TRIG;
	}

	chdrv_write_byte(dev_ptr, cal_trig_reg, 0);
}

void ch_common_store_pt_result(ch_dev_t *dev_ptr) {
	uint8_t pt_result_reg;
	uint16_t rtc_cal_result;

	if (dev_ptr->part_number == CH101_PART_NUMBER) {
		pt_result_reg = CH101_COMMON_REG_CAL_RESULT;
	} else {
		pt_result_reg = CH201_COMMON_REG_CAL_RESULT;
	}

	chdrv_read_word(dev_ptr, pt_result_reg, &rtc_cal_result);
	dev_ptr->rtc_cal_result = rtc_cal_result;
}

void ch_common_store_op_freq(ch_dev_t *dev_ptr){
	uint8_t	 tof_sf_reg;
	uint16_t raw_freq;		// aka scale factor
	uint32_t freq_counter_cycles;
	uint32_t num;
	uint32_t den;
	uint32_t op_freq;

	if (dev_ptr->part_number == CH101_PART_NUMBER) {
		tof_sf_reg = CH101_COMMON_REG_TOF_SF;
	} else {
		tof_sf_reg = CH201_COMMON_REG_TOF_SF;
	}

	freq_counter_cycles = dev_ptr->freqCounterCycles;

	chdrv_read_word(dev_ptr, tof_sf_reg, &raw_freq);

	num = (uint32_t)(((dev_ptr->rtc_cal_result)*1000U) / (16U * freq_counter_cycles)) * (uint32_t)(raw_freq);
	den = (uint32_t)(dev_ptr->group->rtc_cal_pulse_ms);
	op_freq = (num/den);

	dev_ptr->op_frequency = op_freq;
}

void ch_common_store_bandwidth(ch_dev_t __attribute__((unused)) *dev_ptr) {
/*
 * Not supported in current GPR firmware
 */
}

void ch_common_store_scale_factor(ch_dev_t *dev_ptr) {
	uint8_t	err;
	uint8_t	tof_sf_reg;
	uint16_t scale_factor;

	if (dev_ptr->part_number == CH101_PART_NUMBER) {
		tof_sf_reg = CH101_COMMON_REG_TOF_SF;
	} else {
		tof_sf_reg = CH201_COMMON_REG_TOF_SF;
	}

	err = chdrv_read_word(dev_ptr, tof_sf_reg, &scale_factor);
	if (!err) {
		dev_ptr->scale_factor = scale_factor;
	} else {
		dev_ptr->scale_factor = 0;
	}
}


uint8_t ch_common_set_thresholds(ch_dev_t *dev_ptr, ch_thresholds_t *thresholds_ptr) {

	uint8_t	thresh_len_reg = 0;		// offset of register for this threshold's length
	uint8_t thresh_level_reg;	// threshold level reg (first in array)
	uint8_t max_num_thresholds;
	int ret_val = 1;		// default return = error
	uint8_t	thresh_num;
	uint8_t thresh_len;
	uint16_t thresh_level;
	uint16_t start_sample = 0;

	if (dev_ptr->sensor_connected) {
		
		if (dev_ptr->part_number == CH101_PART_NUMBER) {
			return ret_val;		// NOT SUPPORTED in CH101

		} else {
			thresh_level_reg = CH201_COMMON_REG_THRESHOLDS;
			max_num_thresholds = CH201_COMMON_NUM_THRESHOLDS;
		}

		for (thresh_num = 0; thresh_num < max_num_thresholds; thresh_num++) {

			if (thresh_num < (max_num_thresholds - 1)) {
				uint16_t next_start_sample = thresholds_ptr->threshold[thresh_num + 1].start_sample;

				thresh_len = (next_start_sample - start_sample);
				start_sample  = next_start_sample;
			} else {
				thresh_len = 0;
			}

			if (dev_ptr->part_number == CH201_PART_NUMBER) {
				if (thresh_num == 0) {
					thresh_len_reg = CH201_COMMON_REG_THRESH_LEN_0;
				} else if (thresh_num == 1) {
					thresh_len_reg = CH201_COMMON_REG_THRESH_LEN_1;
				} else if (thresh_num == 2) {
					thresh_len_reg = CH201_COMMON_REG_THRESH_LEN_2;
				} else if (thresh_num == 3) {
					thresh_len_reg = CH201_COMMON_REG_THRESH_LEN_3;
				} else if (thresh_num == 4) {
					thresh_len_reg = CH201_COMMON_REG_THRESH_LEN_4;
				} else if (thresh_num == 5) {
					thresh_len_reg = 0;			// last threshold does not have length field - assumed to extend to end of data
				}
			}

			if (thresh_len_reg != 0) {
				chdrv_write_byte(dev_ptr, thresh_len_reg, thresh_len); 	// set the length field (if any) for this threshold
			}
			// write level to this threshold's entry in register array
			thresh_level = thresholds_ptr->threshold[thresh_num].level;
			chdrv_write_word(dev_ptr, (thresh_level_reg + (thresh_num * sizeof(uint16_t))), thresh_level);
		}

		ret_val = 0;	// return OK
	}
	return ret_val;
}


uint8_t ch_common_get_thresholds(ch_dev_t *dev_ptr, ch_thresholds_t *thresholds_ptr) {
	uint8_t	thresh_len_reg = 0;		// offset of register for this threshold's length
	uint8_t thresh_level_reg;	// threshold level reg (first in array)
	uint8_t max_num_thresholds;
	uint8_t ret_val = 1;		// default = error return
	uint8_t thresh_num;
	uint8_t	thresh_len = 0;		// number of samples described by each threshold
	uint16_t	start_sample = 0;	// calculated start sample for each threshold

	if (dev_ptr->sensor_connected && (thresholds_ptr != NULL)) {
		
		if (dev_ptr->part_number == CH101_PART_NUMBER) {
			return ret_val;		// NOT SUPPORTED in CH101
			
		} else {
			thresh_level_reg = CH201_COMMON_REG_THRESHOLDS;
			max_num_thresholds = CH201_COMMON_NUM_THRESHOLDS;
		}

		for (thresh_num = 0; thresh_num < max_num_thresholds; thresh_num++) {

			if (dev_ptr->part_number == CH201_PART_NUMBER) {
				if (thresh_num == 0) {
					thresh_len_reg = CH201_COMMON_REG_THRESH_LEN_0;
				} else if (thresh_num == 1) {
					thresh_len_reg = CH201_COMMON_REG_THRESH_LEN_1;
				} else if (thresh_num == 2) {
					thresh_len_reg = CH201_COMMON_REG_THRESH_LEN_2;
				} else if (thresh_num == 3) {
					thresh_len_reg = CH201_COMMON_REG_THRESH_LEN_3;
				} else if (thresh_num == 4) {
					thresh_len_reg = CH201_COMMON_REG_THRESH_LEN_4;
				} else if (thresh_num == 5) {
					thresh_len_reg = 0;			// last threshold does not have length field - assumed to extend to end of data
				}
			}

			if (thresh_len_reg != 0) {
				// read the length field register for this threshold
				chdrv_read_byte(dev_ptr, thresh_len_reg, &thresh_len);
			} else {
				thresh_len = 0;
			}

			thresholds_ptr->threshold[thresh_num].start_sample = start_sample;
			start_sample += thresh_len;				// increment start sample for next threshold

			// get level from this threshold's entry in register array
			chdrv_read_word(dev_ptr, (thresh_level_reg + (thresh_num * sizeof(uint16_t))), 
						    &(thresholds_ptr->threshold[thresh_num].level));

		}
		ret_val = 0;	// return OK
	}
	return ret_val;
}


static uint8_t get_sample_data(ch_dev_t *dev_ptr, ch_iq_sample_t *buf_ptr, uint16_t start_sample, uint16_t num_samples,
							   ch_io_mode_t mode, uint8_t sample_size_in_bytes) {

	uint16_t   iq_data_addr;
	ch_group_t *grp_ptr = dev_ptr->group;
	int        error = 1;
	uint8_t	   use_prog_read = 0;		// default = do not use low-level programming interface

#ifndef USE_STD_I2C_FOR_IQ
	if (grp_ptr->num_connected[dev_ptr->i2c_bus_index] == 1) {		// if only one device on this bus
		use_prog_read = 1;											//   use low-level interface
	}
#endif

	if (dev_ptr->part_number == CH101_PART_NUMBER) {
		iq_data_addr = CH101_COMMON_REG_DATA;
	} else {
		iq_data_addr = CH201_COMMON_REG_DATA;
	}

	iq_data_addr += (start_sample * sample_size_in_bytes);

	if ((num_samples != 0) && ((start_sample + num_samples) <= dev_ptr->max_samples)) {
		uint16_t num_bytes = (num_samples * sample_size_in_bytes);

		if (mode == CH_IO_MODE_BLOCK) {
			/* blocking transfer */

			if (use_prog_read) {
				/* use low-level programming interface for speed */

				int num_transfers = (num_bytes + (CH_PROG_XFER_SIZE - 1)) / CH_PROG_XFER_SIZE;
    			int bytes_left = num_bytes;       // remaining bytes to read

				/* Convert register offsets to full memory addresses */
				if (dev_ptr->part_number == CH101_PART_NUMBER) {
					iq_data_addr += CH101_DATA_MEM_ADDR + CH101_COMMON_I2CREGS_OFFSET;
				} else {
					iq_data_addr += CH201_DATA_MEM_ADDR + CH201_COMMON_I2CREGS_OFFSET;
				}

				chbsp_program_enable(dev_ptr);					// assert PROG pin

    			for (int xfer = 0; xfer < num_transfers; xfer++) {
        			int bytes_to_read;
        			uint8_t message[] = { (0x80 | CH_PROG_REG_CTL), 0x09 };      // read burst command

        			if (bytes_left > CH_PROG_XFER_SIZE) {
                		bytes_to_read = CH_PROG_XFER_SIZE;
        			} else {
                		bytes_to_read = bytes_left;
        			}
        			chdrv_prog_write(dev_ptr, CH_PROG_REG_ADDR, (iq_data_addr + (xfer * CH_PROG_XFER_SIZE)));
        			chdrv_prog_write(dev_ptr, CH_PROG_REG_CNT, (bytes_to_read - 1));
        			error = chdrv_prog_i2c_write(dev_ptr, message, sizeof(message));
        			error |= chdrv_prog_i2c_read(dev_ptr, ((uint8_t *)buf_ptr + (xfer * CH_PROG_XFER_SIZE)), bytes_to_read);

        			bytes_left -= bytes_to_read;
    			}
    			chbsp_program_disable(dev_ptr);					// de-assert PROG pin

			} else {	/* if (use_prog_read) */
				/* use standard I2C interface */

				error = chdrv_burst_read(dev_ptr, iq_data_addr, (uint8_t *) buf_ptr, num_bytes);
			}

		} else {
			/* non-blocking transfer - queue a read transaction (must be started using ch_io_start_nb() ) */

			if (use_prog_read && (grp_ptr->i2c_drv_flags & I2C_DRV_FLAG_USE_PROG_NB)) {
				/* Use low-level programming interface to read data */

				/* Convert register offsets to full memory addresses */
				if (dev_ptr->part_number == CH101_PART_NUMBER) {
					iq_data_addr += (CH101_DATA_MEM_ADDR + CH101_COMMON_I2CREGS_OFFSET);
				} else {
					iq_data_addr += (CH201_DATA_MEM_ADDR + CH201_COMMON_I2CREGS_OFFSET);
				}

				error = chdrv_group_i2c_queue(grp_ptr, dev_ptr, 1, CHDRV_NB_TRANS_TYPE_PROG, iq_data_addr, num_bytes, 
					                      	(uint8_t *) buf_ptr);
			} else {
				/* Use regular I2C register interface to read data */
				error = chdrv_group_i2c_queue(grp_ptr, dev_ptr, 1, CHDRV_NB_TRANS_TYPE_STD, iq_data_addr, num_bytes, 
											  (uint8_t*) buf_ptr);
			}
		}
	}

	return error;
}

uint8_t	 ch_common_set_sample_window(ch_dev_t *dev_ptr, uint16_t start_sample, uint16_t num_samples) {
	uint8_t err = 1;
	uint16_t max_num_samples;


	if (dev_ptr->part_number == CH101_PART_NUMBER) {
		max_num_samples = CH101_MAX_NUM_SAMPLES;
	} else {
		max_num_samples = CH201_MAX_NUM_SAMPLES;
	}

	if ((start_sample + num_samples) <= max_num_samples) {
		dev_ptr->win_start_sample = start_sample;
		dev_ptr->num_win_samples = num_samples;

		err = 0;
	}

	return err;
}


uint16_t ch_common_get_amplitude_avg(ch_dev_t *dev_ptr) {
	ch_iq_sample_t window_buf[CH_IQ_SAMPLES_PER_READ];
	uint16_t start_sample = dev_ptr->win_start_sample;
	uint16_t num_samples = dev_ptr->num_win_samples;
	uint32_t total_amp = 0;
	uint32_t avg_amp = 0;
	uint8_t err = 0;

	if ((start_sample != 0) && (num_samples != 0)) {

		err = ch_get_iq_data(dev_ptr, window_buf, start_sample, num_samples, CH_IO_MODE_BLOCK);

		if (!err) {
			for (uint16_t idx = 0; idx < num_samples; idx++) {
				total_amp += ch_iq_to_amplitude(&(window_buf[idx]));		// add amplitude for this sample
			}

		avg_amp = (total_amp / num_samples);
		}
	}

	return (uint16_t) avg_amp;
}

uint8_t  ch_common_get_iq_data(ch_dev_t *dev_ptr, ch_iq_sample_t *buf_ptr, uint16_t start_sample, uint16_t num_samples,
							   ch_io_mode_t mode) {

	return get_sample_data(dev_ptr, buf_ptr, start_sample, num_samples, mode, sizeof(ch_iq_sample_t));
}


uint8_t ch_common_get_amplitude_data(ch_dev_t *dev_ptr, uint16_t *buf_ptr, uint16_t start_sample, uint16_t num_samples,
									 ch_io_mode_t mode) {

	ch_iq_sample_t	iq_buf[CH_IQ_SAMPLES_PER_READ];
	uint16_t		samples_in_chunk = 0;
	uint8_t			error = 0;
	uint16_t		sample_num   = start_sample;
	uint16_t		samples_left = num_samples;
	uint8_t 		chunks_left  = (num_samples + CH_IQ_SAMPLES_PER_READ - 1) / CH_IQ_SAMPLES_PER_READ;

	/* Validate mode (only blocking mode is supported) and sample count/offset */
	if ((mode != CH_IO_MODE_BLOCK) || (start_sample + num_samples > dev_ptr->max_samples)) {
		error = 1;
	}

	while (!error && (chunks_left-- > 0)) {

		/* Read I/Q data */
		if (samples_left > CH_IQ_SAMPLES_PER_READ) {
			samples_in_chunk = CH_IQ_SAMPLES_PER_READ;
		} else {
			samples_in_chunk = samples_left;
		}

		samples_left -= samples_in_chunk;			// adjust remaining sample count for next pass

		error = get_sample_data(dev_ptr, iq_buf, sample_num, samples_in_chunk, mode, sizeof(ch_iq_sample_t));
		if (error) {
			break;
		}

		/* Calculate amplitudes and store in user buffer */
		for (uint16_t idx = 0; idx < samples_in_chunk; idx++) {
			buf_ptr[sample_num++] = ch_iq_to_amplitude(&iq_buf[idx]);
		}

	}

	return error;
}


uint8_t ch_common_set_time_plan(ch_dev_t *dev_ptr, ch_time_plan_t time_plan) {
	uint8_t time_plan_reg;
	uint8_t ret_val = 1;		// default return is error

	if (dev_ptr->part_number == CH101_PART_NUMBER) {			// CH-101 only - SonicSync unsupported in CH-201
		time_plan_reg = CH101_COMMON_REG_TIME_PLAN;

		if (dev_ptr->sensor_connected) {
			chdrv_write_byte(dev_ptr, time_plan_reg, time_plan);
			ret_val = 0;
		}
	}

	return ret_val;			// error - SonicSync unsupported in CH-201
}


ch_time_plan_t ch_common_get_time_plan(ch_dev_t *dev_ptr) {
	uint8_t time_plan_reg;
	uint8_t time_plan = CH_TIME_PLAN_NONE;

	if (dev_ptr->part_number == CH101_PART_NUMBER) {			// CH-101 only - SonicSync unsupported in CH-201
		time_plan_reg = CH101_COMMON_REG_TIME_PLAN;

		if (dev_ptr->sensor_connected) {
			chdrv_read_byte(dev_ptr, time_plan_reg, &time_plan);
		}
	}

	return (ch_time_plan_t) time_plan;
}


uint8_t ch_common_set_rx_holdoff(ch_dev_t *dev_ptr, uint16_t num_samples) {
	uint8_t rx_holdoff_reg;
	uint16_t reg_value;
	uint8_t ret_val = RET_OK;

	if (dev_ptr->part_number == CH101_PART_NUMBER) {
		rx_holdoff_reg = CH101_COMMON_REG_RX_HOLDOFF;
		reg_value = num_samples;
	} else {
		rx_holdoff_reg = CH201_COMMON_REG_RX_HOLDOFF;
		reg_value = (num_samples / 2);			// CH201 value is 1/2 actual sample count
	}

	if (dev_ptr->sensor_connected) {
		ret_val |= chdrv_write_byte(dev_ptr, rx_holdoff_reg, (uint8_t) reg_value);
	}

	return ret_val;
}


uint16_t ch_common_get_rx_holdoff(ch_dev_t *dev_ptr) {
	uint8_t rx_holdoff_reg;
	uint8_t reg_val;
	uint16_t rx_holdoff = 0;

	if (dev_ptr->part_number == CH101_PART_NUMBER) {
		rx_holdoff_reg = CH101_COMMON_REG_RX_HOLDOFF;
	} else {
		rx_holdoff_reg = CH201_COMMON_REG_RX_HOLDOFF;
	}

	if (dev_ptr->sensor_connected) {
		chdrv_read_byte(dev_ptr, rx_holdoff_reg, &reg_val);
	}

	rx_holdoff = (uint16_t) reg_val;

	if (dev_ptr->part_number == CH201_PART_NUMBER) {
		rx_holdoff *= 2;			// CH201 reports 1/2 actual sample count
	}

	return rx_holdoff;	
}

