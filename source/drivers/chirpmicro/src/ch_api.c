/*! \file ch_api.c
 \brief Chirp SonicLib public API functions for using the Chirp ultrasonic sensor.

 The user should not need to edit this file. This file relies on hardware interface
 functions declared in ch_bsp.h and supplied in the board support package (BSP) for
 the specific hardware platform being used.
 */

/*
 Copyright © 2019-2020 Chirp Microsystems.  All rights reserved.

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

#include "soniclib.h"
#include "ch_driver.h"
#include "chirp_bsp.h"
#include "ch_math_utils.h"

/*!
 * \brief Initialize a Chirp ultrasonic sensor descriptor structure
 *
 * \param dev_ptr 		a pointer to the ch_dev_t config structure for a sensor
 *
 * \return 0 (RET_OK) if successful, non-zero otherwise
 *
 */

uint8_t	ch_init(ch_dev_t *dev_ptr, ch_group_t *grp_ptr, uint8_t dev_num, ch_fw_init_func_t fw_init_func) {
	
	uint8_t	ret_val = RET_ERR;

	ch_i2c_info_t	i2c_info;

	if (fw_init_func != NULL) {
		/* Get I2C parameters from BSP */
		ret_val = chbsp_i2c_get_info(grp_ptr, dev_num, &i2c_info);
	
		if (ret_val == RET_OK) {
			/* Save special handling flags for Chirp driver */
			grp_ptr->i2c_drv_flags = i2c_info.drv_flags;

			/* Call asic f/w init function passed in as parameter */
			ret_val = (*fw_init_func)(dev_ptr, grp_ptr, i2c_info.address, dev_num, i2c_info.bus_num);
		}
	}

	return ret_val;
}


uint8_t	ch_get_config(ch_dev_t *dev_ptr, ch_config_t *config_ptr) {
	uint8_t ret_val = 0;

	config_ptr->mode         	= dev_ptr->mode;
	config_ptr->max_range    	= dev_ptr->max_range;
	config_ptr->static_range 	= dev_ptr->static_range;
	config_ptr->sample_interval	= dev_ptr->sample_interval;
	config_ptr->thresh_ptr   	= NULL;				// thresholds not returned here - use ch_get_thresholds()

	return ret_val;
}


uint8_t	ch_set_config(ch_dev_t *dev_ptr, ch_config_t *config_ptr) {
	uint8_t ret_val = 0;

	ret_val = ch_set_mode(dev_ptr, config_ptr->mode);						// set operating mode

	if (!ret_val) {
		dev_ptr->mode = config_ptr->mode;

		ret_val = ch_set_max_range(dev_ptr, config_ptr->max_range);			// set max range
	}

	if (!ret_val) {

		if (dev_ptr->part_number == CH101_PART_NUMBER) {					// static rejection only on CH101
			ret_val = ch_set_static_range(dev_ptr, config_ptr->static_range);	// set static target rejection range

			if (!ret_val) {
				dev_ptr->static_range = config_ptr->static_range;
			}
		}
	}

	if (!ret_val) {
		ret_val = ch_set_sample_interval(dev_ptr, config_ptr->sample_interval);		// set sample interval (free-run mode only)
	}

	if (!ret_val) {
		dev_ptr->sample_interval = config_ptr->sample_interval;

		if (dev_ptr->part_number == CH201_PART_NUMBER) {					// multi threshold only on CH201
			ret_val = ch_set_thresholds(dev_ptr, config_ptr->thresh_ptr);		// set multiple thresholds
		}
	}

	if (!ret_val) {
		ret_val = ch_set_target_interrupt(dev_ptr, config_ptr->enable_target_int);		// enable/disable target detection interrupt 
	}
	if (!ret_val) {
		ret_val = ch_set_time_plan(dev_ptr,config_ptr->time_plan);		// set time plan (sonicsync only)
	}

	return ret_val;
}




uint8_t	ch_group_start(ch_group_t *grp_ptr) {
	uint8_t ret_val;

	ret_val = chdrv_group_start(grp_ptr);

	return ret_val;
}

void ch_trigger(ch_dev_t *dev_ptr) {
	chdrv_hw_trigger(dev_ptr);
}

void ch_group_trigger(ch_group_t *grp_ptr) {
	chdrv_group_hw_trigger(grp_ptr);
}

void ch_reset(ch_dev_t *dev_ptr, ch_reset_t reset_type) {

	if (reset_type == CH_RESET_HARD) {
		chdrv_group_hard_reset(dev_ptr->group); 			// TODO need single device hard reset
	} else {
		chdrv_soft_reset(dev_ptr);
	}
}

void ch_group_reset(ch_group_t *grp_ptr, ch_reset_t reset_type) {
	if (reset_type == CH_RESET_HARD) {
		chdrv_group_hard_reset(grp_ptr);
	} else {
		chdrv_group_soft_reset(grp_ptr);
	}
}

uint8_t ch_sensor_is_connected(ch_dev_t *dev_ptr) {

	return dev_ptr->sensor_connected;
}


uint16_t ch_get_part_number(ch_dev_t *dev_ptr) {

	return dev_ptr->part_number;
}


uint8_t  ch_get_dev_num(ch_dev_t *dev_ptr) {

	return dev_ptr->io_index;
}


ch_dev_t *ch_get_dev_ptr(ch_group_t *grp_ptr, uint8_t dev_num) {

	return grp_ptr->device[dev_num];
}

uint8_t  ch_get_i2c_address(ch_dev_t *dev_ptr) {

	return dev_ptr->i2c_address;
}


uint8_t  ch_get_i2c_bus(ch_dev_t *dev_ptr) {

	return dev_ptr->i2c_bus_index;
}


uint8_t ch_get_num_ports(ch_group_t *grp_ptr) {

	return grp_ptr->num_ports;
}

const char *ch_get_fw_version_string(ch_dev_t *dev_ptr) {

	return dev_ptr->fw_version_string;
}

ch_mode_t ch_get_mode(ch_dev_t *dev_ptr) {

	return dev_ptr->mode;
}


uint8_t ch_set_mode(ch_dev_t *dev_ptr, ch_mode_t mode) {
	int	ret_val = RET_ERR;
	ch_set_mode_func_t func_ptr = dev_ptr->api_funcs.set_mode;

	if (func_ptr != NULL) {
		ret_val = (*func_ptr)(dev_ptr, mode);
	}

	if (ret_val == 0) {
		dev_ptr->mode = mode;
	}

	return ret_val;
}


uint16_t ch_get_sample_interval(ch_dev_t *dev_ptr) {
	uint16_t sample_interval = 0;

	if (dev_ptr->mode == CH_MODE_FREERUN) {
		sample_interval = dev_ptr->sample_interval;
	}

	return sample_interval;
}

uint8_t ch_set_sample_interval(ch_dev_t *dev_ptr, uint16_t sample_interval) {
	int	ret_val = RET_ERR;
	ch_set_sample_interval_func_t func_ptr = dev_ptr->api_funcs.set_sample_interval;

	if (func_ptr != NULL) {
		ret_val = (*func_ptr)(dev_ptr, sample_interval);
	}

	return ret_val;
}

uint16_t ch_get_num_samples(ch_dev_t *dev_ptr) {

	return dev_ptr->num_rx_samples;
}

uint8_t ch_set_num_samples(ch_dev_t *dev_ptr, uint16_t num_samples) {
	uint8_t	ret_val = RET_ERR;
	ch_set_num_samples_func_t func_ptr = dev_ptr->api_funcs.set_num_samples;

	if (func_ptr != NULL) {
		ret_val = (*func_ptr)(dev_ptr, num_samples);
	}

	dev_ptr->max_range = ch_samples_to_mm(dev_ptr, num_samples);	// store corresponding range in mm

	return ret_val;
}

uint16_t ch_get_max_range(ch_dev_t *dev_ptr) {

	return dev_ptr->max_range;
}


uint8_t ch_set_max_range(ch_dev_t *dev_ptr, uint16_t max_range) {
	uint8_t	ret_val = RET_ERR;
	ch_set_max_range_func_t func_ptr = dev_ptr->api_funcs.set_max_range;

	if (func_ptr != NULL) {
		ret_val = (*func_ptr)(dev_ptr, max_range);
	}

	return ret_val;
}


uint16_t ch_get_max_samples(ch_dev_t *dev_ptr) {

	return dev_ptr->max_samples;
}


uint8_t  ch_get_sample_window(ch_dev_t *dev_ptr, uint16_t *start_sample_ptr, uint16_t *num_samples_ptr) {
	uint8_t	ret_val = RET_ERR;

	if ((start_sample_ptr != NULL) && (num_samples_ptr != NULL)) {

		*start_sample_ptr = dev_ptr->win_start_sample;
		*num_samples_ptr  = dev_ptr->num_win_samples;
		ret_val = RET_OK;
	}
	return ret_val;
}




uint8_t	 ch_set_sample_window(ch_dev_t *dev_ptr, uint16_t start_sample, uint16_t num_samples) {
	uint8_t	ret_val = RET_ERR;
	ch_set_sample_window_func_t func_ptr = dev_ptr->api_funcs.set_sample_window;

	if (func_ptr != NULL) {
		ret_val = (*func_ptr)(dev_ptr, start_sample, num_samples);
	}

	return ret_val;
}


uint16_t ch_get_static_range(ch_dev_t *dev_ptr) {

	return dev_ptr->static_range;
}

uint8_t ch_set_static_range(ch_dev_t *dev_ptr, uint16_t num_samples) {
	uint8_t	ret_val = RET_OK;
	ch_set_static_range_func_t func_ptr = dev_ptr->api_funcs.set_static_range;

	if (func_ptr != NULL) {
		ret_val = (*func_ptr)(dev_ptr, num_samples);
	}

	return ret_val;
}

uint32_t ch_get_range(ch_dev_t *dev_ptr, ch_range_t range_type) {
	uint32_t	range = 0;
	ch_get_range_func_t func_ptr = dev_ptr->api_funcs.get_range;

	if (func_ptr != NULL) {
		range = (*func_ptr)(dev_ptr, range_type);
	}

	return range;
}

uint32_t ch_get_tof_us(ch_dev_t *dev_ptr) {
	uint32_t	tof_us = 0;
	ch_get_tof_us_func_t func_ptr = dev_ptr->api_funcs.get_tof_us;

	if (func_ptr != NULL) {
		tof_us = (*func_ptr)(dev_ptr);
	}

	return tof_us;
}

uint16_t ch_get_amplitude(ch_dev_t *dev_ptr) {
	int	amplitude = 0;
	ch_get_amplitude_func_t func_ptr = dev_ptr->api_funcs.get_amplitude;

	if (func_ptr != NULL) {
		amplitude = (*func_ptr)(dev_ptr);
	}

	return amplitude;
}

uint16_t ch_get_amplitude_avg(ch_dev_t *dev_ptr) {
	uint16_t	amplitude_avg = 0;
	ch_get_amplitude_avg_func_t func_ptr = dev_ptr->api_funcs.get_amplitude_avg;

	if (func_ptr != NULL) {
		amplitude_avg = (*func_ptr)(dev_ptr);
	}

	return amplitude_avg;
}


uint8_t ch_get_amplitude_data(ch_dev_t *dev_ptr, uint16_t *amp_buf_ptr, uint16_t start_sample, uint16_t num_samples, 
									 ch_io_mode_t mode) {
	uint16_t	error = 0;
	ch_get_amplitude_data_func_t func_ptr = dev_ptr->api_funcs.get_amplitude_data;

	if (func_ptr != NULL) {
		error = (*func_ptr)(dev_ptr, amp_buf_ptr, start_sample, num_samples, mode);
	}

	return error;
}


uint16_t ch_get_bandwidth(ch_dev_t *dev_ptr) {

	return dev_ptr->bandwidth;
}

uint8_t ch_set_frequency(ch_dev_t *dev_ptr, uint32_t target_freq_Hz) {
	int	ret_val = RET_OK;
	ch_set_frequency_func_t func_ptr = dev_ptr->api_funcs.set_frequency;

	if (func_ptr != NULL) {
		ret_val = (*func_ptr)(dev_ptr, target_freq_Hz);
	}

	return ret_val;	
}

uint32_t ch_get_frequency(ch_dev_t *dev_ptr) {

	return dev_ptr->op_frequency;
}

uint16_t ch_get_rtc_cal_pulselength(ch_dev_t *dev_ptr) {

	return dev_ptr->group->rtc_cal_pulse_ms;
}


uint16_t ch_get_rtc_cal_result(ch_dev_t *dev_ptr) {

	return dev_ptr->rtc_cal_result;
}


uint16_t ch_get_scale_factor(ch_dev_t *dev_ptr) {

	return dev_ptr->scale_factor;
}


uint8_t ch_get_iq_data(ch_dev_t *dev_ptr, ch_iq_sample_t *buf_ptr, uint16_t start_sample, uint16_t num_samples, ch_io_mode_t mode) {
	int	ret_val = 0;
	ch_get_iq_data_func_t func_ptr = dev_ptr->api_funcs.get_iq_data;

	if (func_ptr != NULL) {
		ret_val = (*func_ptr)(dev_ptr, buf_ptr, start_sample, num_samples, mode);
	}

	return ret_val;
}


uint16_t ch_samples_to_mm(ch_dev_t *dev_ptr, uint16_t num_samples) {
	int	num_mm = 0;
	ch_samples_to_mm_func_t func_ptr = dev_ptr->api_funcs.samples_to_mm;

	if (func_ptr != NULL) {
		num_mm = (*func_ptr)(dev_ptr, num_samples);
	}

	return num_mm;
}

uint16_t ch_mm_to_samples(ch_dev_t *dev_ptr, uint16_t num_mm) {
	int	num_samples = 0;
	ch_mm_to_samples_func_t func_ptr = dev_ptr->api_funcs.mm_to_samples;

	if (func_ptr != NULL) {
		num_samples = (*func_ptr)(dev_ptr, num_mm);
	}

	return num_samples;
}

uint8_t  ch_set_threshold(ch_dev_t *dev_ptr, uint8_t threshold_index, uint16_t amplitude) {
	int	ret_val = RET_OK;
	ch_set_threshold_func_t func_ptr = dev_ptr->api_funcs.set_threshold;

	if ((func_ptr != NULL)) {
		ret_val = (*func_ptr)(dev_ptr, threshold_index, amplitude);
	}

	return ret_val;	
}

uint16_t  ch_get_threshold(ch_dev_t *dev_ptr, uint8_t threshold_index) {
	uint16_t amplitude = 0;
	ch_get_threshold_func_t func_ptr = dev_ptr->api_funcs.get_threshold;

	if ((func_ptr != NULL)) {
		amplitude = (*func_ptr)(dev_ptr, threshold_index);
	}

	return amplitude;	
}

uint16_t ch_iq_to_amplitude(ch_iq_sample_t *iq_sample) {
	uint32_t amplitude;
	uint32_t i_sq = ((uint32_t) iq_sample->i * (uint32_t) iq_sample->i);
	uint32_t q_sq = ((uint32_t) iq_sample->q * (uint32_t) iq_sample->q);

	amplitude = sqrt_int32(i_sq + q_sq);

	return (uint16_t) amplitude;
}


uint8_t ch_set_thresholds(ch_dev_t *dev_ptr, ch_thresholds_t *thresh_ptr) {
	int	ret_val = RET_OK;
	ch_set_thresholds_func_t func_ptr = dev_ptr->api_funcs.set_thresholds;

	if ((func_ptr != NULL) && (thresh_ptr != NULL)) {
		ret_val = (*func_ptr)(dev_ptr, thresh_ptr);
		}

	return ret_val;
}

uint8_t ch_get_thresholds(ch_dev_t *dev_ptr, ch_thresholds_t *thresh_ptr) {
	int	ret_val = RET_ERR;
	ch_get_thresholds_func_t func_ptr = dev_ptr->api_funcs.get_thresholds;

	if ((func_ptr != NULL) && (thresh_ptr != NULL)) {
		ret_val = (*func_ptr)(dev_ptr, thresh_ptr);
	}

	return ret_val;
}

uint8_t ch_set_time_plan(ch_dev_t *dev_ptr, ch_time_plan_t time_plan) {
	uint8_t ret_val = RET_OK;
	ch_set_time_plan_func_t func_ptr = dev_ptr->api_funcs.set_time_plan;

	if (func_ptr != NULL) {
		ret_val = (*func_ptr)(dev_ptr, time_plan);
	}

	return ret_val;
}

ch_time_plan_t ch_get_time_plan(ch_dev_t *dev_ptr) {
	ch_time_plan_t time_plan = CH_TIME_PLAN_NONE;
	ch_get_time_plan_func_t func_ptr = dev_ptr->api_funcs.get_time_plan;

	if (func_ptr != NULL) {
		time_plan = (*func_ptr)(dev_ptr);
	}

	return time_plan;
}

/*!
 * \brief Start a non-blocking sensor readout
 *
 * \param grp_ptr 		pointer to the ch_group_t descriptor structure for a group of sensors
 *
 * This function starts a non-blocking I/O operation on the specified group of sensors.
 */
uint8_t ch_io_start_nb(ch_group_t *grp_ptr) {
	uint8_t ret_val = 1;

	if (grp_ptr->io_complete_callback != NULL) {		// only start I/O if there is a callback function

		chdrv_group_i2c_start_nb(grp_ptr);
		ret_val = 0;
	}

	return ret_val;
}

/*!
 * \brief Set callback function for Chirp sensor I/O interrupt
 *
 * \note
 */
void ch_io_int_callback_set(ch_group_t *grp_ptr, ch_io_int_callback_t callback_func_ptr) {

	grp_ptr->io_int_callback = callback_func_ptr;
}


/*!
 * \brief Set callback function for Chirp sensor I/O operation complete
 *
 * \note
 */
void ch_io_complete_callback_set(ch_group_t *grp_ptr, ch_io_complete_callback_t callback_func_ptr) {

	grp_ptr->io_complete_callback = callback_func_ptr;
}


/*!
 * \brief Continue a non-blocking readout
 *
 * \param grp_ptr 			pointer to the ch_group_t config structure for a group of sensors
 * \param i2c_bus_index		index value identifying I2C bus within group
 *
 * Call this function once from your I2C interrupt handler each time it completes an I/O operation.
 * It will call the function previously specified during \a ch_io_complete_callback_set() when all group
 * transactions are complete.
 */
void ch_io_notify(ch_group_t *grp_ptr, uint8_t i2c_bus_index) {
	
	chdrv_group_i2c_irq_handler(grp_ptr, i2c_bus_index);
}

uint8_t ch_set_target_interrupt(ch_dev_t *dev_ptr, uint8_t enable) {
	int	ret_val = RET_OK;
	ch_set_target_interrupt_func_t func_ptr = dev_ptr->api_funcs.set_target_interrupt;

	if (func_ptr != NULL) {
		ret_val = (*func_ptr)(dev_ptr, enable);
	}

	return ret_val;
}

uint8_t ch_get_target_interrupt(ch_dev_t *dev_ptr) {
	uint8_t	enabled = 0;
	ch_get_target_interrupt_func_t func_ptr = dev_ptr->api_funcs.get_target_interrupt;

	if (func_ptr != NULL) {
		enabled = (*func_ptr)(dev_ptr);
	}

	return enabled;
}

uint8_t ch_set_static_coeff(ch_dev_t *dev_ptr, uint8_t static_coeff) {
	int	ret_val = RET_OK;
	ch_set_static_coeff_func_t func_ptr = dev_ptr->api_funcs.set_static_coeff;

	if (func_ptr != NULL) {
		ret_val = (*func_ptr)(dev_ptr, static_coeff);
	}

	return ret_val;	
}

uint8_t ch_get_static_coeff(ch_dev_t *dev_ptr) {
	uint8_t	statc_coeff = 0;
	ch_get_static_coeff_func_t func_ptr = dev_ptr->api_funcs.get_static_coeff;

	if (func_ptr != NULL) {
		statc_coeff = (*func_ptr)(dev_ptr);
	}

	return statc_coeff;		
}

uint8_t ch_set_rx_holdoff(ch_dev_t *dev_ptr, uint16_t num_samples) {
	int	ret_val = RET_OK;
	ch_set_rx_holdoff_func_t func_ptr = dev_ptr->api_funcs.set_rx_holdoff;

	if (func_ptr != NULL) {
		ret_val = (*func_ptr)(dev_ptr, num_samples);
	}

	return ret_val;
}

uint16_t ch_get_rx_holdoff(ch_dev_t *dev_ptr) {
	uint16_t num_samples = 0;
	ch_get_rx_holdoff_func_t func_ptr = dev_ptr->api_funcs.get_rx_holdoff;

	if (func_ptr != NULL) {
		num_samples = (*func_ptr)(dev_ptr);
	}

	return num_samples;		
}

uint8_t ch_set_tx_length(ch_dev_t *dev_ptr, uint8_t tx_length) {
	int	ret_val = RET_OK;
	ch_set_tx_length_func_t func_ptr = dev_ptr->api_funcs.set_tx_length;

	if (func_ptr != NULL) {
		ret_val = (*func_ptr)(dev_ptr, tx_length);
	}

	return ret_val;	
}

uint8_t ch_get_tx_length(ch_dev_t *dev_ptr) {
	uint8_t tx_length = 0;
	ch_get_tx_length_func_t func_ptr = dev_ptr->api_funcs.get_tx_length;

	if (func_ptr != NULL) {
		tx_length = (*func_ptr)(dev_ptr);
	}

	return tx_length;		
}

uint8_t ch_get_rx_pulse_length(ch_dev_t *dev_ptr) {
	uint8_t rx_pulse_length = 0;
	ch_get_rx_pulse_length_func_t func_ptr = dev_ptr->api_funcs.get_rx_pulse_length;

	if (func_ptr != NULL) {
		rx_pulse_length = (*func_ptr)(dev_ptr);
	}

	return rx_pulse_length;		
}

void ch_set_rx_pretrigger(ch_group_t *grp_ptr, uint8_t enable) {

	if (enable) {
		chdrv_pretrigger_delay_set(grp_ptr, CHDRV_PRETRIGGER_DELAY_US);
	} else {
		chdrv_pretrigger_delay_set(grp_ptr, 0);
	}
}

uint8_t ch_get_rx_pretrigger(ch_group_t *grp_ptr) {
	uint8_t enabled = (grp_ptr->pretrig_delay_us != 0);

	return enabled;
}

