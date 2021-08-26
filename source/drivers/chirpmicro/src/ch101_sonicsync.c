#include "soniclib.h"
#include "ch101_sonicsync.h"
#include "ch_common.h"


uint8_t ch101_sonicsync_master_init(ch_dev_t *dev_ptr, ch_group_t *grp_ptr, uint8_t i2c_addr, uint8_t io_index, uint8_t i2c_bus_index) {
	
	dev_ptr->part_number = CH101_PART_NUMBER;
	dev_ptr->app_i2c_address = i2c_addr;
	dev_ptr->io_index = io_index;
	dev_ptr->i2c_bus_index = i2c_bus_index;

	/* Init firmware-specific function pointers */
	dev_ptr->firmware 					= ch101_sonicsync_master_fw;
	dev_ptr->fw_version_string			= ch101_sonicsync_master_version;
	dev_ptr->ram_init 					= get_ram_ch101_sonicsync_master_init_ptr();
	dev_ptr->get_fw_ram_init_size 		= get_ch101_sonicsync_master_fw_ram_init_size;
	dev_ptr->get_fw_ram_init_addr 		= get_ch101_sonicsync_master_fw_ram_init_addr;

	dev_ptr->prepare_pulse_timer 		= ch_common_prepare_pulse_timer;
	dev_ptr->store_pt_result 			= ch_common_store_pt_result;
	dev_ptr->store_op_freq 				= ch_common_store_op_freq;
	dev_ptr->store_bandwidth 			= ch_common_store_bandwidth;
	dev_ptr->store_scalefactor 			= ch_common_store_scale_factor;
	dev_ptr->get_locked_state 			= ch_sonicsync_get_locked_state;

	/* Init API function pointers */
	dev_ptr->api_funcs.fw_load          = ch_common_fw_load;
	dev_ptr->api_funcs.set_mode         = ch_common_set_mode;
	dev_ptr->api_funcs.set_sample_interval  = ch_common_set_sample_interval;
	dev_ptr->api_funcs.set_num_samples  = ch_common_set_num_samples;
	dev_ptr->api_funcs.set_max_range    = ch_common_set_max_range;
	dev_ptr->api_funcs.set_static_range = ch_common_set_static_range;
	dev_ptr->api_funcs.get_range        = ch_common_get_range;
	dev_ptr->api_funcs.get_amplitude    = ch_common_get_amplitude;
	dev_ptr->api_funcs.get_iq_data      = ch_common_get_iq_data;
	dev_ptr->api_funcs.samples_to_mm    = ch_common_samples_to_mm;
	dev_ptr->api_funcs.mm_to_samples    = ch_common_mm_to_samples;
	dev_ptr->api_funcs.set_thresholds   = NULL;								// not supported
	dev_ptr->api_funcs.get_thresholds   = NULL;								// not supported
	dev_ptr->api_funcs.set_time_plan    = ch_common_set_time_plan;
	dev_ptr->api_funcs.get_time_plan    = ch_common_get_time_plan;	

	/* Init max sample count */
	dev_ptr->max_samples = CH101_SONICSYNC_MAX_SAMPLES;

	/* This firmware does not use oversampling */
	dev_ptr->oversample = 0;

	/* Init device and group descriptor linkage */
	dev_ptr->group						= grp_ptr;			// set parent group pointer
	grp_ptr->device[io_index] 	   		= dev_ptr;			// add to parent group

	return 0;
}

uint8_t ch101_sonicsync_slave_init(ch_dev_t *dev_ptr, ch_group_t *grp_ptr, uint8_t i2c_addr, uint8_t io_index, uint8_t i2c_bus_index) {
	
	dev_ptr->part_number = CH101_PART_NUMBER;
	dev_ptr->app_i2c_address = i2c_addr;
	dev_ptr->io_index = io_index;
	dev_ptr->i2c_bus_index = i2c_bus_index;

	/* Init firmware-specific function pointers */
	dev_ptr->firmware 					= ch101_sonicsync_slave_fw;
	dev_ptr->fw_version_string			= ch101_sonicsync_slave_version;
	dev_ptr->ram_init 					= get_ram_ch101_sonicsync_slave_init_ptr();
	dev_ptr->get_fw_ram_init_size 		= get_ch101_sonicsync_slave_fw_ram_init_size;
	dev_ptr->get_fw_ram_init_addr 		= get_ch101_sonicsync_slave_fw_ram_init_addr;

	dev_ptr->prepare_pulse_timer 		= ch_common_prepare_pulse_timer;
	dev_ptr->store_pt_result 			= ch_common_store_pt_result;
	dev_ptr->store_op_freq 				= ch_common_store_op_freq;
	dev_ptr->store_bandwidth 			= ch_common_store_bandwidth;
	dev_ptr->store_scalefactor 			= ch_common_store_scale_factor;
	dev_ptr->get_locked_state 			= ch_sonicsync_get_locked_state;

	/* Init API function pointers */
	dev_ptr->api_funcs.fw_load          = ch_common_fw_load;
	dev_ptr->api_funcs.set_mode         = ch_common_set_mode;
	dev_ptr->api_funcs.set_sample_interval  = ch_common_set_sample_interval;
	dev_ptr->api_funcs.set_num_samples  = ch_common_set_num_samples;
	dev_ptr->api_funcs.set_max_range    = ch_common_set_max_range;
	dev_ptr->api_funcs.set_static_range = ch_common_set_static_range;
	dev_ptr->api_funcs.get_range        = ch_common_get_range;
	dev_ptr->api_funcs.get_amplitude    = ch_common_get_amplitude;
	dev_ptr->api_funcs.get_iq_data      = ch_common_get_iq_data;
	dev_ptr->api_funcs.samples_to_mm    = ch_common_samples_to_mm;
	dev_ptr->api_funcs.mm_to_samples    = ch_common_mm_to_samples;
	dev_ptr->api_funcs.set_thresholds   = NULL;								// not supported
	dev_ptr->api_funcs.get_thresholds   = NULL;								// not supported
	dev_ptr->api_funcs.set_time_plan    = ch_common_set_time_plan;
	dev_ptr->api_funcs.get_time_plan    = ch_common_get_time_plan;

	/* Init max sample count */
	dev_ptr->max_samples = CH101_GPR_OPEN_MAX_SAMPLES;

	/* This firmware does not use oversampling */
	dev_ptr->oversample = 0;

	/* Init device and group descriptor linkage */
	dev_ptr->group						= grp_ptr;			// set parent group pointer
	grp_ptr->device[io_index] 	   		= dev_ptr;			// add to parent group

	return 0;
}

uint8_t ch_sonicsync_get_locked_state(ch_dev_t *dev_ptr) {
	uint8_t ready_reg;
	uint8_t lock_mask;
	uint8_t ret_val = 0;

	if (dev_ptr->part_number == CH201_PART_NUMBER) {
		return ret_val;		// NOT SUPPORTED in CH201
	}
	if (dev_ptr->part_number == CH101_PART_NUMBER) {
		ready_reg = CH101_COMMON_REG_READY;
		lock_mask = CH101_SONICSYNC_READY_FREQ_LOCKED;
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