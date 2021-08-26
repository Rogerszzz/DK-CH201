/*! \file ch201_presence.c
 *
 * \brief Chirp CH201 presence detection firmware interface
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
#include "ch201_presence.h"
#include "ch_common.h"

static uint16_t ch_presence_mm_to_samples(ch_dev_t *dev_ptr, uint16_t num_mm);

uint8_t ch201_presence_init(ch_dev_t *dev_ptr, ch_group_t *grp_ptr, uint8_t i2c_addr, uint8_t io_index, uint8_t i2c_bus_index) {

	dev_ptr->part_number = CH201_PART_NUMBER;
	dev_ptr->app_i2c_address = i2c_addr;
	dev_ptr->io_index = io_index;
	dev_ptr->i2c_bus_index = i2c_bus_index;

	dev_ptr->freqCounterCycles = CH201_PRESENCE_FREQCOUNTERCYCLES;
	dev_ptr->freqLockValue     = CH201_PRESENCE_READY_FREQ_LOCKED;

	/* Init firmware-specific function pointers */
	dev_ptr->firmware             = ch201_presence_fw;
	dev_ptr->fw_version_string    = ch201_presence_version;
	dev_ptr->ram_init             = get_ram_ch201_presence_init_ptr();
	dev_ptr->get_fw_ram_init_size = get_ch201_presence_fw_ram_init_size;
	dev_ptr->get_fw_ram_init_addr = get_ch201_presence_fw_ram_init_addr;

	dev_ptr->prepare_pulse_timer = ch_common_prepare_pulse_timer;
	dev_ptr->store_pt_result     = ch_common_store_pt_result;
	dev_ptr->store_op_freq       = ch_common_store_op_freq;
	dev_ptr->store_bandwidth     = ch_common_store_bandwidth;
	dev_ptr->store_scalefactor   = ch_common_store_scale_factor;
	dev_ptr->get_locked_state    = ch_common_get_locked_state;

	/* Init API function pointers */
	dev_ptr->api_funcs.fw_load          = ch_common_fw_load;
	dev_ptr->api_funcs.set_mode         = ch_common_set_mode;
	dev_ptr->api_funcs.set_sample_interval = ch_common_set_sample_interval;
	dev_ptr->api_funcs.set_num_samples  = ch_common_set_num_samples;
	dev_ptr->api_funcs.set_max_range    = ch_common_set_max_range;
	dev_ptr->api_funcs.set_static_range = NULL; // not supported
	dev_ptr->api_funcs.get_range        = ch_common_get_range;
	dev_ptr->api_funcs.get_amplitude    = ch_common_get_amplitude;
	dev_ptr->api_funcs.get_iq_data      = ch_common_get_iq_data;
	dev_ptr->api_funcs.get_amplitude_data = ch_common_get_amplitude_data;
	dev_ptr->api_funcs.samples_to_mm    = ch_common_samples_to_mm;
	dev_ptr->api_funcs.mm_to_samples    = ch_presence_mm_to_samples;
	dev_ptr->api_funcs.set_thresholds   = ch_common_set_thresholds;
	dev_ptr->api_funcs.get_thresholds   = ch_common_get_thresholds;

	/* Init max sample count */
	dev_ptr->max_samples = CH201_PRESENCE_MAX_SAMPLES;

	/* This firmware does not use oversampling */
	dev_ptr->oversample = 0;

	/* Init device and group descriptor linkage */
	dev_ptr->group            = grp_ptr; // set parent group pointer
	grp_ptr->device[io_index] = dev_ptr; // add to parent group

	return 0;
}

uint8_t ch201_presence_set_decimation(ch_dev_t *dev_ptr, uint8_t decimation)
{
	uint8_t ret = 1;
	uint8_t RegValue;

	if (dev_ptr->sensor_connected) {
		ret = chdrv_read_byte(dev_ptr, CH201_PRESENCE_DECIMATION, &RegValue);
		RegValue &= ~CH201_PRESENCE_DECIMATION_MASK;
		RegValue |= decimation;
		// Configure Decimation
		if (ret == 0)
			ret |= chdrv_write_byte(dev_ptr, CH201_PRESENCE_DECIMATION, RegValue);
	}

	return ret;
}

uint8_t ch201_presence_enable_cordic(ch_dev_t *dev_ptr, uint8_t enable)
{
	uint8_t ret = 1;
	uint8_t RegValue;

	if (dev_ptr->sensor_connected) {
		ret = chdrv_read_byte(dev_ptr, CH201_PRESENCE_DECIMATION, &RegValue);
		RegValue &= ~CH201_PRESENCE_CORDIC_MASK;
		RegValue |= enable;
		// Configure Decimation
		if (ret == 0)
			ret |= chdrv_write_byte(dev_ptr, CH201_PRESENCE_DECIMATION, RegValue);
	}

	return ret;
}

uint16_t ch_presence_mm_to_samples(ch_dev_t *dev_ptr, uint16_t num_mm) {
	uint32_t samples;
	//2-way range, sample rate is op_frequency/8
	#define DEN (8*343*1000)
	#define NUM (2*num_mm*dev_ptr->op_frequency)
	samples = (2*NUM + 3*DEN) / (2 * DEN); // NUM / DEM + 1.5 float-free
	#undef NUM
	#undef DEN

	samples = (samples > dev_ptr->max_samples) ? dev_ptr->max_samples : samples;

	return samples;
}