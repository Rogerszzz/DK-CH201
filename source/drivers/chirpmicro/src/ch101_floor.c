/*! \file ch101_floor.c
 *
 * \brief Chirp CH101 Floor Detection firmware interface
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
#include "ch101_floor.h"
#include "ch_common.h"
#include "ch_math_utils.h"


/* Forward references */
uint8_t	 ch101_floor_set_sample_window(ch_dev_t *dev_ptr, uint16_t start_sample, uint16_t end_sample);
uint16_t ch101_floor_get_amplitude_avg(ch_dev_t *dev_ptr);



uint8_t ch101_floor_init(ch_dev_t *dev_ptr, ch_group_t *grp_ptr, uint8_t i2c_addr, uint8_t io_index, uint8_t i2c_bus_index) {
	
	dev_ptr->part_number = CH101_PART_NUMBER;
	dev_ptr->app_i2c_address = i2c_addr;
	dev_ptr->io_index = io_index;
	dev_ptr->i2c_bus_index = i2c_bus_index;

	dev_ptr->freqCounterCycles = CH101_COMMON_FREQCOUNTERCYCLES;
	dev_ptr->freqLockValue     = CH101_COMMON_READY_FREQ_LOCKED;

	/* Init firmware-specific function pointers */
	dev_ptr->firmware 					= ch101_floor_fw;
	dev_ptr->fw_version_string			= ch101_floor_version;
	dev_ptr->ram_init 					= get_ram_ch101_floor_init_ptr();
	dev_ptr->get_fw_ram_init_size 		= get_ch101_floor_fw_ram_init_size;
	dev_ptr->get_fw_ram_init_addr 		= get_ch101_floor_fw_ram_init_addr;

	dev_ptr->prepare_pulse_timer 		= ch_common_prepare_pulse_timer;
	dev_ptr->store_pt_result 			= ch_common_store_pt_result;
	dev_ptr->store_op_freq 				= ch_common_store_op_freq;
	dev_ptr->store_bandwidth 			= ch101_floor_store_bandwidth;
	dev_ptr->store_scalefactor 			= ch_common_store_scale_factor;
	dev_ptr->get_locked_state 			= ch_common_get_locked_state;

	/* Init API function pointers */
	dev_ptr->api_funcs.fw_load          = ch_common_fw_load;
	dev_ptr->api_funcs.set_mode         = ch_common_set_mode;
	dev_ptr->api_funcs.set_sample_interval  = ch_common_set_sample_interval;
	dev_ptr->api_funcs.set_num_samples  = ch_common_set_num_samples;
	dev_ptr->api_funcs.set_max_range    = ch_common_set_max_range;
	dev_ptr->api_funcs.set_static_range = NULL;
	dev_ptr->api_funcs.get_range        = NULL;
	dev_ptr->api_funcs.get_amplitude    = NULL;
	dev_ptr->api_funcs.get_amplitude_avg    = ch101_floor_get_amplitude_avg;
	dev_ptr->api_funcs.set_sample_window    = ch101_floor_set_sample_window;
	dev_ptr->api_funcs.get_iq_data      = ch_common_get_iq_data;
	dev_ptr->api_funcs.get_amplitude_data = NULL; // Not supported
	dev_ptr->api_funcs.samples_to_mm    = ch_common_samples_to_mm;
	dev_ptr->api_funcs.mm_to_samples    = ch_common_mm_to_samples;
	dev_ptr->api_funcs.set_thresholds   = NULL;								// not supported
	dev_ptr->api_funcs.get_thresholds   = NULL;								// not supported

	/* Init max sample count */
	dev_ptr->max_samples = CH101_FLOOR_MAX_SAMPLES;

	/* This firmware uses oversampling */
	dev_ptr->oversample = 2;			// 4x oversampling (value is power of 2)


	/* Init device and group descriptor linkage */
	dev_ptr->group						= grp_ptr;			// set parent group pointer
	grp_ptr->device[io_index] 	   		= dev_ptr;			// add to parent group

	return 0;
}


uint8_t	 ch101_floor_set_sample_window(ch_dev_t *dev_ptr, uint16_t start_sample, uint16_t end_sample) {
	uint8_t	ret_val = 0;
	uint16_t num_win_samples = ((end_sample - start_sample) + 1);

	/* Check input parameters */
	if ((num_win_samples > CH101_FLOOR_MAX_SAMPLES) || 
		(start_sample >= CH101_FLOOR_MAX_SAMPLES) || 
		(end_sample > CH101_FLOOR_MAX_SAMPLES)) {

		ret_val = 1;	/* error */
	}

	/* Write window start and end registers */
	if (!ret_val) {
		chdrv_write_byte(dev_ptr, CH101_FLOOR_REG_RX_HOLDOFF, start_sample);

		chdrv_write_byte(dev_ptr, CH101_FLOOR_REG_RX_WIN_END, (end_sample + 1));

		dev_ptr->win_start_sample = start_sample;
		dev_ptr->num_win_samples = num_win_samples;
	}

	return ret_val;
}

#define	AMP_CORDIC_CORRECT_NUM	 (122)		/* numerator of Cordic amplitude correction (1.22) */
#define	AMP_CORDIC_CORRECT_DEN	 (100)		/* denominator of Cordic amplitude correction (1.22) */

uint16_t ch101_floor_get_amplitude_avg(ch_dev_t *dev_ptr) {
	uint32_t amp_total;
	uint16_t amp_lo;
	uint16_t amp_hi;
	uint16_t amp_avg = 0;
	uint8_t err = 0;

	/* Read total amplitude across window (two 16-bit halves) */

	err = chdrv_read_word(dev_ptr, CH101_FLOOR_REG_AMPLITUDE_LOW, &amp_lo);

	if (!err) {
		err = chdrv_read_word(dev_ptr, CH101_FLOOR_REG_AMPLITUDE_HIGH, &amp_hi);
	}


	/* Combine values and calculate average */
	if (!err) {
		amp_total = (amp_hi << 16) | amp_lo;

		amp_avg = (uint16_t) ((amp_total * AMP_CORDIC_CORRECT_NUM) / (dev_ptr->num_win_samples * AMP_CORDIC_CORRECT_DEN));
	}

	return amp_avg;
}

#define CH101_FLOOR_BANDWIDTH_INDEX_1	14		// XXX not final
#define CH101_FLOOR_BANDWIDTH_INDEX_2	18		// XXX not final

void ch101_floor_store_bandwidth(ch_dev_t *dev_ptr){
	uint32_t bandwidth = 0;
	ch_iq_sample_t QIData[(CH101_FLOOR_BANDWIDTH_INDEX_2 - CH101_FLOOR_BANDWIDTH_INDEX_1) + 1];	// buffer to read I/Q
	uint16_t start_sample = CH101_FLOOR_BANDWIDTH_INDEX_1;
	uint16_t num_samples = (CH101_FLOOR_BANDWIDTH_INDEX_2 - CH101_FLOOR_BANDWIDTH_INDEX_1) + 1;	
	uint16_t ix = (CH101_FLOOR_BANDWIDTH_INDEX_2 - CH101_FLOOR_BANDWIDTH_INDEX_1);		// index to second sample for calc

#if 0
	chdrv_burst_read(dev_ptr, CH101_FLOOR_REG_DATA 	+ (CH101_FLOOR_BANDWIDTH_INDEX_1 * sizeof(ch_iq_sample_t)),(uint8_t *) QIData, 
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

