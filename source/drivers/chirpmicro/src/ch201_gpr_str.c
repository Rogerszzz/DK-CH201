/*! \file ch201_gpr_str.c
 *
 * \brief Chirp CH201 General Purpose Rangefinding / Static Target Rejection firmware interface
 * 
 * This file contains function definitions to interface a specific sensor firmware 
 * package to SonicLib, including the main initialization routine for the firmware.  
 * That routine initializes various fields within the \a ch_dev_t device descriptor 
 * and specifies the proper functions to implement SonicLib API calls.  Those may 
 * either be common implementations or firmware-specific routines located in this file.
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
#include "ch201_gpr_str.h"
#include "ch_common.h"
#include "chirp_bsp.h"			// board support package function definitions

void ch201_gpr_str_store_op_freq(ch_dev_t *dev_ptr);
void ch201_gpr_str_store_scale_factor(ch_dev_t *dev_ptr);
uint32_t ch201_gpr_str_get_range(ch_dev_t *dev_ptr, ch_range_t range_type);
uint16_t ch201_gpr_str_get_amplitude(ch_dev_t *dev_ptr);
uint8_t  ch201_gpr_str_get_iq_data(ch_dev_t *dev_ptr, ch_iq_sample_t *buf_ptr, uint16_t start_sample, uint16_t num_samples, ch_io_mode_t mode);
uint8_t ch201_gpr_str_set_target_interrupt(ch_dev_t *dev_ptr, uint8_t enable);
uint8_t ch201_gpr_str_get_target_interrupt(ch_dev_t *dev_ptr);
uint8_t ch201_gpr_str_set_num_samples(ch_dev_t *dev_ptr, uint16_t num_samples );
uint8_t ch201_gpr_str_set_max_range(ch_dev_t *dev_ptr, uint16_t max_range_mm);
uint8_t ch201_gpr_str_set_threshold(ch_dev_t *dev_ptr, uint8_t threshold_index, uint16_t amplitude);
uint16_t ch201_gpr_str_get_threshold(ch_dev_t *dev_ptr, uint8_t threshold_index);

uint8_t ch201_gpr_str_init(ch_dev_t *dev_ptr, ch_group_t *grp_ptr, uint8_t i2c_addr, uint8_t io_index, uint8_t i2c_bus_index) {
	
	dev_ptr->part_number = CH201_PART_NUMBER;
	dev_ptr->app_i2c_address = i2c_addr;
	dev_ptr->io_index = io_index;
	dev_ptr->i2c_bus_index = i2c_bus_index;

	dev_ptr->freqCounterCycles = CH201_COMMON_FREQCOUNTERCYCLES;
	dev_ptr->freqLockValue     = CH201_COMMON_READY_FREQ_LOCKED;

	/* Init firmware-specific function pointers */
	dev_ptr->firmware 					= ch201_gpr_str_fw;
	dev_ptr->fw_version_string			= ch201_gpr_str_version;
	dev_ptr->ram_init 					= get_ram_ch201_gpr_str_init_ptr();
	dev_ptr->get_fw_ram_init_size 		= get_ch201_gpr_str_fw_ram_init_size;
	dev_ptr->get_fw_ram_init_addr 		= get_ch201_gpr_str_fw_ram_init_addr;

	dev_ptr->prepare_pulse_timer 		= ch_common_prepare_pulse_timer;
	dev_ptr->store_pt_result 			= ch_common_store_pt_result;
	dev_ptr->store_op_freq 				= ch201_gpr_str_store_op_freq;
	dev_ptr->store_bandwidth 			= ch_common_store_bandwidth;
	dev_ptr->store_scalefactor 			= ch201_gpr_str_store_scale_factor;
	dev_ptr->get_locked_state 			= ch_common_get_locked_state;

	/* Init API function pointers */
	dev_ptr->api_funcs.fw_load          = ch_common_fw_load;
	dev_ptr->api_funcs.set_mode         = ch_common_set_mode;
	dev_ptr->api_funcs.set_sample_interval  = ch_common_set_sample_interval;
	dev_ptr->api_funcs.set_num_samples  = ch201_gpr_str_set_num_samples;
	dev_ptr->api_funcs.set_max_range    = ch201_gpr_str_set_max_range;
	dev_ptr->api_funcs.set_static_range = NULL;								// not supported
	dev_ptr->api_funcs.get_range        = ch201_gpr_str_get_range;
	dev_ptr->api_funcs.get_amplitude    = ch201_gpr_str_get_amplitude;
	dev_ptr->api_funcs.get_iq_data      = ch201_gpr_str_get_iq_data;
	dev_ptr->api_funcs.get_amplitude_data = NULL; // Not supported
	dev_ptr->api_funcs.samples_to_mm    = ch_common_samples_to_mm;
	dev_ptr->api_funcs.mm_to_samples    = ch_common_mm_to_samples;
	dev_ptr->api_funcs.set_threshold    = ch201_gpr_str_set_threshold;
	dev_ptr->api_funcs.get_threshold    = ch201_gpr_str_get_threshold;	
	dev_ptr->api_funcs.set_thresholds   = NULL;
	dev_ptr->api_funcs.get_thresholds   = NULL;
	dev_ptr->api_funcs.set_target_interrupt   = ch201_gpr_str_set_target_interrupt;
	dev_ptr->api_funcs.get_target_interrupt   = ch201_gpr_str_get_target_interrupt;	
	dev_ptr->api_funcs.set_sample_window = ch_common_set_sample_window;
	dev_ptr->api_funcs.get_amplitude_avg = ch_common_get_amplitude_avg;

	/* Init max sample count */
	dev_ptr->max_samples = CH201_GPR_STR_MAX_SAMPLES;

	/* This firmware does not use oversampling */
	dev_ptr->oversample = 0;

	/* Init device and group descriptor linkage */
	dev_ptr->group						= grp_ptr;			// set parent group pointer
	grp_ptr->device[io_index] 	   		= dev_ptr;			// add to parent group

	return 0;
}

void ch201_gpr_str_store_op_freq(ch_dev_t *dev_ptr){
	uint8_t	 tof_sf_reg;
	uint16_t raw_freq;		// aka scale factor
	uint32_t freq_counter_cycles;
	uint32_t num;
	uint32_t den;
	uint32_t op_freq;

	tof_sf_reg = CH201_GPR_STR_REG_TOF_SF;
	freq_counter_cycles = CH201_COMMON_FREQCOUNTERCYCLES;

	chdrv_read_word(dev_ptr, tof_sf_reg, &raw_freq);

	num = (uint32_t)(((dev_ptr->rtc_cal_result)*1000U) / (16U * freq_counter_cycles)) * (uint32_t)(raw_freq);
	den = (uint32_t)(dev_ptr->group->rtc_cal_pulse_ms);
	op_freq = (num/den);

	dev_ptr->op_frequency = op_freq;
}

void ch201_gpr_str_store_scale_factor(ch_dev_t *dev_ptr) {
	uint8_t	err;
	uint8_t	tof_sf_reg;
	uint16_t scale_factor;

	tof_sf_reg = CH201_GPR_STR_REG_TOF_SF;

	err = chdrv_read_word(dev_ptr, tof_sf_reg, &scale_factor);
	if (!err) {
		dev_ptr->scale_factor = scale_factor;
	} else {
		dev_ptr->scale_factor = 0;
	}
}

uint32_t ch201_gpr_str_get_range(ch_dev_t *dev_ptr, ch_range_t range_type) {
	uint8_t		tof_reg;
	uint32_t	range = CH_NO_TARGET;
	uint16_t 	time_of_flight;
	uint16_t 	scale_factor;
	int 		err;

	if (dev_ptr->sensor_connected) {
		tof_reg = CH201_GPR_STR_REG_TOF;

		err = chdrv_read_word(dev_ptr, tof_reg, &time_of_flight);

		if (!err && (time_of_flight != UINT16_MAX)) { // If object detected

			if (dev_ptr->scale_factor == 0) {
				ch201_gpr_str_store_scale_factor(dev_ptr);
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

			}
		}
	}
	return range;
}

uint16_t ch201_gpr_str_get_amplitude(ch_dev_t *dev_ptr) {
	uint8_t  amplitude_reg;
	uint16_t amplitude = 0;

	if (dev_ptr->sensor_connected) {
		amplitude_reg = CH201_GPR_STR_REG_AMPLITUDE;

		chdrv_read_word(dev_ptr, amplitude_reg, &amplitude);
	}

	return amplitude;
}

uint8_t  ch201_gpr_str_get_iq_data(ch_dev_t *dev_ptr, ch_iq_sample_t *buf_ptr, uint16_t start_sample, uint16_t num_samples,
ch_io_mode_t mode) {
	uint16_t   iq_data_addr;
	ch_group_t *grp_ptr = dev_ptr->group;
	int        error = 1;
	uint8_t	   use_prog_read = 0;		// default = do not use low-level programming interface

	#ifndef USE_STD_I2C_FOR_IQ
	if (grp_ptr->num_connected[dev_ptr->i2c_bus_index] == 1) {		// if only one device on this bus
		use_prog_read = 1;											//   use low-level interface
	}
	#endif

	iq_data_addr = CH201_GPR_STR_REG_DATA;
	
	iq_data_addr += (start_sample * sizeof(ch_iq_sample_t));

	if ((num_samples != 0) && ((start_sample + num_samples) <= dev_ptr->max_samples)) {
		uint16_t num_bytes = (num_samples * sizeof(ch_iq_sample_t));

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

uint8_t ch201_gpr_str_set_target_interrupt(ch_dev_t *dev_ptr, uint8_t enable) {
	uint8_t reg = CH201_GPR_STR_REG_INTCFG;
	uint8_t ret_val = RET_OK;

	if (dev_ptr->sensor_connected) {
		ret_val = chdrv_write_byte(dev_ptr, reg, enable);
	}
	return ret_val;
}

uint8_t ch201_gpr_str_get_target_interrupt(ch_dev_t *dev_ptr) {
	uint8_t reg = CH201_GPR_STR_REG_INTCFG;
	uint8_t enabled = 0;

	if (dev_ptr->sensor_connected) {
		chdrv_read_byte(dev_ptr, reg, &enabled);
	}
	return enabled;
}

uint8_t ch201_gpr_str_set_num_samples(ch_dev_t *dev_ptr, uint16_t num_samples ) {
	uint8_t max_range_reg;
	uint8_t ret_val = 1;		// default is error (not connected or num_samples too big)
	uint8_t low_gain_rxlen;

	max_range_reg = CH201_COMMON_REG_MAX_RANGE;
	num_samples /= 2;					// each internal count for CH201 represents 2 physical samples
		
	ret_val = chdrv_read_byte( dev_ptr, CH201_GPR_STR_REG_LOW_GAIN_RXLEN, &low_gain_rxlen);

	if (ret_val) return ret_val;
	// check if low gain Rx length is less than Rx length
	if (num_samples <= low_gain_rxlen ) {
		low_gain_rxlen = num_samples - 1;
		chdrv_write_byte(dev_ptr, CH201_GPR_STR_REG_LOW_GAIN_RXLEN, low_gain_rxlen);
	}

	if (dev_ptr->sensor_connected && (num_samples <= UINT8_MAX)) {
		ret_val = chdrv_write_byte(dev_ptr, max_range_reg, num_samples);
	}

	if (!ret_val) {
		dev_ptr->num_rx_samples = (num_samples * 2);	// store actual physical sample count
	} 
	else {
		dev_ptr->num_rx_samples = 0;
	}
	
	return ret_val;
}

uint8_t ch201_gpr_str_set_max_range(ch_dev_t *dev_ptr, uint16_t max_range_mm) {
	uint8_t ret_val;
	uint32_t num_samples;

	ret_val = (!dev_ptr->sensor_connected);

	if (!ret_val) {
		num_samples = ch_common_mm_to_samples(dev_ptr, max_range_mm);

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
		ret_val = ch201_gpr_str_set_num_samples(dev_ptr, num_samples);
	}

	#ifdef CHDRV_DEBUG
	printf("Set samples: ret_val: %u  dev_ptr->num_rx_samples: %u\n", ret_val, dev_ptr->num_rx_samples);
	#endif
	return ret_val;
}

uint8_t ch201_gpr_str_set_threshold(ch_dev_t *dev_ptr, uint8_t threshold_index, uint16_t amplitude) {
	uint8_t ret_val = RET_OK;
	uint8_t regs[] = {CH201_GPR_STR_REG_THRESH_0, CH201_GPR_STR_REG_THRESH_1};
	
	if (threshold_index >= CH201_GPR_STR_THRESHOLD_NUMBER)
		return RET_ERR;
	
	if (dev_ptr->sensor_connected) {
		ret_val = chdrv_write_word(dev_ptr, regs[threshold_index], amplitude);
	}

	return ret_val;
}

uint16_t ch201_gpr_str_get_threshold(ch_dev_t *dev_ptr, uint8_t threshold_index) {
	uint16_t amplitude = 0;
	uint8_t regs[] = {CH201_GPR_STR_REG_THRESH_0, CH201_GPR_STR_REG_THRESH_1};

	if ((threshold_index < CH201_GPR_STR_THRESHOLD_NUMBER) && dev_ptr->sensor_connected) {
		chdrv_read_word(dev_ptr, regs[threshold_index], &amplitude);
	}

	return amplitude;
}
