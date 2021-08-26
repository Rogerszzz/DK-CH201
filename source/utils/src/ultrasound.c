/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2020 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively �Software�) is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * ________________________________________________________________________________________________________
 */
#include "ultrasound.h"
#include "chirp_smartsonic.h"
#include "chirp_bsp.h"
#include "app_config.h"

static uint32_t odr_ms = MEASUREMENT_INTERVAL_MS;

/*
 * Define hooks to set and get the output data rate.
 */
uint8_t ultrasound_set_odr_ms(uint32_t odr)
{
#if defined(USE_FIXED_SENSOR_ODR)
	/* TODO TODO need to clean up this.
	   It seems we’ll need to refactor the redswallow layer to remove this ultrasound_set_odr_ms function.
	   The idea would be to move all protocol_cmd* function to a single file included in all app
	   using redswallow.
	   To have specific implementation of protocol_cmd* we could define them as weak so that
	   user can have its custom implementation. */
	odr = MEASUREMENT_INTERVAL_MS * chirp_group.sensor_count;
#endif

	chbsp_periodic_timer_init(odr / chirp_group.sensor_count, periodic_timer_callback);

#if defined(USE_FIXED_SENSOR_ODR)
	/* TODO TODO Once the redswallow layer refactor is done this can be removed as each
	   application will be independant. */
	odr_ms = MEASUREMENT_INTERVAL_MS;
#else
	odr_ms = odr;
#endif

	return 0;
}

uint32_t ultrasound_get_odr_ms(void)
{
	return odr_ms;
}