/*! \file ch101_gppc.c
 *
 * \brief Chirp CH101 General Purpose Pitch Catch firmware interface
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
#include "ch101_gppc.h"
#include "ch_common.h"
#include "chirp_bsp.h"
#include "ch_math_utils.h"

//#define DEBUG_DCO_SEARCH(X) X 

void ch101_gppc_store_bandwidth(ch_dev_t *dev_ptr);
uint8_t ch101_gppc_set_num_samples(ch_dev_t *dev_ptr, uint16_t num_samples);
uint32_t ch101_gppc_get_range(ch_dev_t *dev_ptr, ch_range_t range_type);
uint32_t ch101_gppc_get_tof_us(ch_dev_t *dev_ptr);
uint8_t ch101_gppc_set_static_coeff(ch_dev_t *dev_ptr, uint8_t static_coeff);
uint8_t ch101_gppc_get_static_coeff(ch_dev_t *dev_ptr);
uint8_t ch101_gppc_set_rx_holdoff(ch_dev_t *dev_ptr, uint16_t rx_holdoff);
uint16_t ch101_gppc_get_rx_holdoff(ch_dev_t *dev_ptr);
uint8_t ch101_gppc_set_tx_length(ch_dev_t *dev_ptr, uint8_t tx_length);
uint8_t ch101_gppc_get_tx_length(ch_dev_t *dev_ptr);
uint8_t ch101_gppc_get_rx_pulse_length(ch_dev_t *dev_ptr);
uint8_t ch101_gppc_set_threshold(ch_dev_t *dev_ptr, uint8_t threshold_index, uint16_t amplitude);
uint16_t ch101_gppc_get_threshold(ch_dev_t *dev_ptr, uint8_t threshold_index);
uint8_t  ch101_gppc_get_iq_data(ch_dev_t *dev_ptr, ch_iq_sample_t *buf_ptr, uint16_t start_sample, uint16_t num_samples,
							   ch_io_mode_t mode);

static uint8_t get_sample_data(ch_dev_t *dev_ptr, ch_iq_sample_t *buf_ptr, uint16_t start_sample, uint16_t num_samples,
							   ch_io_mode_t mode, uint8_t sample_size_in_byte);
							   							   
uint8_t ch101_gppc_init(ch_dev_t *dev_ptr, ch_group_t *grp_ptr, uint8_t i2c_addr, uint8_t io_index, uint8_t i2c_bus_index) {
	
	dev_ptr->part_number = CH101_PART_NUMBER;
	dev_ptr->app_i2c_address = i2c_addr;
	dev_ptr->io_index = io_index;
	dev_ptr->i2c_bus_index = i2c_bus_index;

	dev_ptr->freqCounterCycles = CH101_COMMON_FREQCOUNTERCYCLES;
	dev_ptr->freqLockValue     = CH101_GPPC_READY_FREQ_LOCKED;

	/* Init firmware-specific function pointers */
	dev_ptr->firmware 					= ch101_gppc_fw;
	dev_ptr->fw_version_string			= ch101_gppc_version;
	dev_ptr->ram_init 					= get_ram_ch101_gppc_init_ptr();
	dev_ptr->get_fw_ram_init_size 		= get_ch101_gppc_fw_ram_init_size;
	dev_ptr->get_fw_ram_init_addr 		= get_ch101_gppc_fw_ram_init_addr;

	dev_ptr->prepare_pulse_timer 		= ch_common_prepare_pulse_timer;
	dev_ptr->store_pt_result 			= ch_common_store_pt_result;
	dev_ptr->store_op_freq 				= ch_common_store_op_freq;
	dev_ptr->store_bandwidth 			= ch101_gppc_store_bandwidth;
	dev_ptr->store_scalefactor 			= ch_common_store_scale_factor;
	dev_ptr->get_locked_state 			= ch_common_get_locked_state;

	/* Init API function pointers */
	dev_ptr->api_funcs.fw_load          = ch_common_fw_load;
	dev_ptr->api_funcs.set_mode         = ch_common_set_mode;
	dev_ptr->api_funcs.set_sample_interval  = ch_common_set_sample_interval;
	dev_ptr->api_funcs.set_num_samples  = ch101_gppc_set_num_samples;
	dev_ptr->api_funcs.set_max_range    = ch_common_set_max_range;
	dev_ptr->api_funcs.set_static_range = NULL;
	dev_ptr->api_funcs.get_range        = ch101_gppc_get_range;
	dev_ptr->api_funcs.get_tof_us       = ch101_gppc_get_tof_us;
	dev_ptr->api_funcs.get_amplitude    = ch_common_get_amplitude;
	dev_ptr->api_funcs.get_iq_data      = ch101_gppc_get_iq_data;
	dev_ptr->api_funcs.get_amplitude_data = NULL; // Not supported
	dev_ptr->api_funcs.samples_to_mm    = ch_common_samples_to_mm;
	dev_ptr->api_funcs.mm_to_samples    = ch_common_mm_to_samples;
	dev_ptr->api_funcs.set_threshold    = ch101_gppc_set_threshold;
	dev_ptr->api_funcs.get_threshold    = ch101_gppc_get_threshold;
	dev_ptr->api_funcs.set_thresholds   = NULL; // Not supported
	dev_ptr->api_funcs.get_thresholds   = NULL;	// Not supported
	dev_ptr->api_funcs.set_static_coeff = ch101_gppc_set_static_coeff;
	dev_ptr->api_funcs.get_static_coeff = ch101_gppc_get_static_coeff;
	dev_ptr->api_funcs.set_rx_holdoff   = ch101_gppc_set_rx_holdoff;
	dev_ptr->api_funcs.get_rx_holdoff   = ch101_gppc_get_rx_holdoff;
	dev_ptr->api_funcs.set_tx_length    = ch101_gppc_set_tx_length;
	dev_ptr->api_funcs.get_tx_length    = ch101_gppc_get_tx_length;
	dev_ptr->api_funcs.get_rx_pulse_length = ch101_gppc_get_rx_pulse_length;
	dev_ptr->api_funcs.set_frequency    = ch101_gppc_set_frequency;
	
	/* Init max sample count */
	dev_ptr->max_samples = CH101_GPPC_MAX_SAMPLES;

	/* This firmware does not use oversampling */
	dev_ptr->oversample = 0;

	/* Init device and group descriptor linkage */
	dev_ptr->group						= grp_ptr;			// set parent group pointer
	grp_ptr->device[io_index] 	   		= dev_ptr;			// add to parent group

	return 0;
}

uint32_t ch101_gppc_get_range(ch_dev_t *dev_ptr, ch_range_t range_type) {
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

				range *= 2;

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

uint32_t ch101_gppc_get_tof_us(ch_dev_t *dev_ptr) {
	uint16_t time_of_flight;
	uint64_t time_of_flight_us = 0;
	int 	 err;
	
	if (dev_ptr->sensor_connected)
	{
		err = chdrv_read_word(dev_ptr, CH101_COMMON_REG_TOF, &time_of_flight);
		
		if (!err && (time_of_flight != UINT16_MAX)) { // If object detected
			time_of_flight_us = (uint64_t) time_of_flight*1000000/(dev_ptr->op_frequency*32);
		}
	}
	return (uint32_t)time_of_flight_us*2;
}


uint8_t ch101_gppc_set_num_samples(ch_dev_t *dev_ptr, uint16_t num_samples ) {
	uint8_t max_range_reg;
	uint8_t ret_val = 1;		// default is error (not connected or num_samples too big)

	max_range_reg = CH101_COMMON_REG_MAX_RANGE;
	num_samples /= 2;					// each internal count for CH201 represents 2 physical samples
		
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

uint32_t ch101_gppc_set_new_dco_code(ch_dev_t *dev_ptr, uint16_t dcocode){
	ch_common_set_mode(dev_ptr, CH_MODE_IDLE);
	chdrv_write_word(dev_ptr, CH101_GPPC_REG_DCO_SET, dcocode);
	chdrv_wait_for_lock(dev_ptr, CHDRV_FREQLOCK_TIMEOUT_MS);
	ch_common_set_mode(dev_ptr, CH_MODE_TRIGGERED_TX_RX);
	ch_common_store_op_freq(dev_ptr);
	return dev_ptr->op_frequency;
}

uint8_t ch101_gppc_set_frequency(ch_dev_t *dev_ptr, uint32_t target_freq_Hz) {
	uint32_t freq = 0;
	uint32_t dcoper[2];
	//initially, find two points on the DCO curve, which should be linear in period
	//increasing DCO code ~= increasing DCO period
	dcoper[0] = 1000000000U / ch101_gppc_set_new_dco_code(dev_ptr, CH_DCO_LOW);
	dcoper[1] = 1000000000U / ch101_gppc_set_new_dco_code(dev_ptr, CH_DCO_HIGH);
	uint32_t targetper = 1000000000U / target_freq_Hz; ///5617
	//Now interpolate to estimate the DCO code
	uint16_t dcoest = (int32_t)CH_DCO_LOW + ((int32_t) targetper - (int32_t) dcoper[0]) * (int32_t)(CH_DCO_HIGH - CH_DCO_LOW) /
	(int32_t)(dcoper[1] - dcoper[0]);
	freq = ch101_gppc_set_new_dco_code(dev_ptr, dcoest);

	DEBUG_DCO_SEARCH(printf("# Port %u, dco0=%lu, dco1=%lu, dcoest=%u, freq=%lu, targ= %lu\n",
	dev_ptr->io_index, dcoper[0], dcoper[1], dcoest, freq, target_freq_Hz);)
	int32_t minerr = abs(freq - target_freq_Hz);
	uint32_t minoff = 0;
	//if the error is too high, search around the estimate for the best code
	if (minerr > CH_DCO_SEARCH_THRESHOLD) {
		DEBUG_DCO_SEARCH(printf("# Frequency error above %dHz, searching for better match to %luHz\n",
		CH_DCO_SEARCH_THRESHOLD, target_freq_Hz);)
		int i;
		for (i = -5; i < 6; i++) { //+/-5 DCO codes should be about +/-1500Hz
			freq = ch101_gppc_set_new_dco_code(dev_ptr, dcoest + i);
			if (abs(freq - target_freq_Hz) < minerr) {
				minerr = abs(freq - target_freq_Hz);
				minoff = i;
				DEBUG_DCO_SEARCH(printf("# *");)
			}
			DEBUG_DCO_SEARCH(printf("# dcoest=%u, freq=%lu\n", dcoest + i, freq);)
		}
		dcoest = dcoest + minoff;
		freq = ch101_gppc_set_new_dco_code(dev_ptr, dcoest);
		DEBUG_DCO_SEARCH(printf("# Final setting dco=%u, freq=%lu\n", dcoest, freq);)
	}
	return 0;
}

uint8_t ch101_gppc_set_static_coeff(ch_dev_t *dev_ptr, uint8_t static_coeff) {
	uint8_t reg = CH101_GPPC_REG_ST_COEFF;
	uint8_t ret_val = RET_OK;

	if (dev_ptr->sensor_connected) {
		ret_val |= chdrv_write_byte(dev_ptr, reg, static_coeff);
	}

	return ret_val;		
}

uint8_t ch101_gppc_get_static_coeff(ch_dev_t *dev_ptr) {
	uint8_t reg = CH101_GPPC_REG_ST_COEFF;
	uint8_t static_coeff = 0;
	
	if (dev_ptr->sensor_connected) {
		chdrv_read_byte(dev_ptr, reg, &static_coeff);
	}

	return static_coeff;	
}

uint8_t ch101_gppc_set_rx_holdoff(ch_dev_t *dev_ptr, uint16_t num_samples) {
	uint8_t reg = CH101_GPPC_REG_RX_HOLDOFF;
	uint8_t ret_val = RET_OK;

	if (dev_ptr->sensor_connected) {
		ret_val |= chdrv_write_byte(dev_ptr, reg, num_samples);
	}

	return ret_val;
}

uint16_t ch101_gppc_get_rx_holdoff(ch_dev_t *dev_ptr) {
	uint8_t reg = CH101_GPPC_REG_RX_HOLDOFF;
	uint8_t num_samples = 0;

	if (dev_ptr->sensor_connected) {
		chdrv_read_byte(dev_ptr, reg, &num_samples);
	}

	return (uint16_t) num_samples;	
}

uint8_t ch101_gppc_set_tx_length(ch_dev_t *dev_ptr, uint8_t tx_length) {
	uint8_t reg = CH101_GPPC_REG_TX_LENGTH;
	uint8_t ret_val = RET_OK;

	if (dev_ptr->sensor_connected) {
		ret_val |= chdrv_write_byte(dev_ptr, reg, tx_length);
	}

	return ret_val;	
}

uint8_t ch101_gppc_get_tx_length(ch_dev_t *dev_ptr) {
	uint8_t reg = CH101_GPPC_REG_TX_LENGTH;
	uint8_t tx_length = 0;

	if (dev_ptr->sensor_connected) {
		chdrv_read_byte(dev_ptr, reg, &tx_length);
	}

	return tx_length;		
}

uint8_t ch101_gppc_get_rx_pulse_length(ch_dev_t *dev_ptr) {
	uint8_t reg = CH101_GPPC_REG_RX_PULSE_LENGTH;
	uint8_t rx_pulse_length = 0;

	if (dev_ptr->sensor_connected) {
		chdrv_read_byte(dev_ptr, reg, &rx_pulse_length);
	}

	return rx_pulse_length;			
}

static uint8_t get_sample_data(ch_dev_t *dev_ptr, ch_iq_sample_t *buf_ptr, uint16_t start_sample, uint16_t num_samples,
							   ch_io_mode_t mode, uint8_t sample_size_in_byte) {

	uint16_t   iq_data_addr;
	ch_group_t *grp_ptr = dev_ptr->group;
	int        error = 1;
	uint8_t	   use_prog_read = 0;		// default = do not use low-level programming interface

#ifndef USE_STD_I2C_FOR_IQ
	if (grp_ptr->num_connected[dev_ptr->i2c_bus_index] == 1) {		// if only one device on this bus
		use_prog_read = 1;											//   use low-level interface
	}
#endif

	iq_data_addr = CH101_GPPC_REG_DATA;

	iq_data_addr += (start_sample * sample_size_in_byte);

	if ((num_samples != 0) && ((start_sample + num_samples) <= dev_ptr->max_samples)) {
		uint16_t num_bytes = (num_samples * sample_size_in_byte);

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


uint8_t  ch101_gppc_get_iq_data(ch_dev_t *dev_ptr, ch_iq_sample_t *buf_ptr, uint16_t start_sample, uint16_t num_samples,
							   ch_io_mode_t mode) {

	return get_sample_data(dev_ptr, buf_ptr, start_sample, num_samples, mode, sizeof(ch_iq_sample_t));
}

void ch101_gppc_store_bandwidth(ch_dev_t *dev_ptr){
	uint32_t bandwidth = 0;
	ch_iq_sample_t QIData[(CH101_BANDWIDTH_INDEX_2 - CH101_BANDWIDTH_INDEX_1) + 1];	// buffer to read I/Q
	uint16_t start_sample = CH101_BANDWIDTH_INDEX_1;
	uint16_t num_samples = (CH101_BANDWIDTH_INDEX_2 - CH101_BANDWIDTH_INDEX_1) + 1;
	uint16_t ix = (CH101_BANDWIDTH_INDEX_2 - CH101_BANDWIDTH_INDEX_1);		// index to second sample for calc

	#if 0
	chdrv_burst_read(dev_ptr, CH101_GPPC_REG_DATA 	+ (CH101_BANDWIDTH_INDEX_1 * sizeof(ch_iq_sample_t)),(uint8_t *) QIData,
	(num_samples * sizeof(ch_iq_sample_t)));
	#else
	(void) ch_get_iq_data(dev_ptr, QIData, start_sample, num_samples, CH_IO_MODE_BLOCK);
	#endif

	uint32_t mag1sq = (uint32_t)((int32_t)QIData[0].i * (int32_t)QIData[0].i + (int32_t)QIData[0].q * (int32_t)QIData[0].q);

	uint32_t mag2sq = (uint32_t)((int32_t)QIData[ix].i * (int32_t)QIData[ix].i + (int32_t)QIData[ix].q * (int32_t)QIData[ix].q);

	//can perform below calculations using floating point for higher accuracy.
	bandwidth = FIXEDMUL(FP_log( FP_sqrt( FIXEDDIV(mag1sq,mag2sq))),
	(FIXEDDIV(INT2FIXED((uint64_t)dev_ptr->op_frequency),
	(FIXED_PI * ((CH101_BANDWIDTH_INDEX_2 - CH101_BANDWIDTH_INDEX_1) * 8 )))));

	dev_ptr->bandwidth = (uint16_t) FIXED2INT(bandwidth);
}

uint8_t ch101_gppc_set_threshold(ch_dev_t *dev_ptr, uint8_t threshold_index, uint16_t amplitude) {
	uint8_t ret_val = RET_OK;
	uint8_t reg = CH101_GPPC_REG_THRESHOLD;
	
	if (threshold_index >= CH101_GPPC_THRESHOLD_NUMBER)
		return RET_ERR;
		
	if (dev_ptr->sensor_connected) {
		ret_val = chdrv_write_word(dev_ptr, reg, amplitude);
	}

	return ret_val;
}

uint16_t ch101_gppc_get_threshold(ch_dev_t *dev_ptr, uint8_t threshold_index) {
	uint16_t amplitude = 0;
	uint8_t reg = CH101_GPPC_REG_THRESHOLD;
		
	if ((threshold_index < CH101_GPPC_THRESHOLD_NUMBER) && dev_ptr->sensor_connected) {
		chdrv_read_word(dev_ptr, reg, &amplitude);
	}

	return amplitude;
}
