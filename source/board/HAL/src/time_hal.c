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
#include <stdbool.h>
#include <asf.h>
#include "time_hal.h"
#include "tc.h"
#include "chirp_smartsonic.h"

uint64_t time_hal_get_in_us(void)
{
	static uint64_t lsepoch_overflow_counter_in_tick = 0;

	cpu_irq_enter_critical();
	/* Check if a counter overflow occurs since last call and ensure no ovf occurs during the read */
	if (TC0->TC_CHANNEL[TC_CHANNEL_LSEPOCH].TC_SR & TC_SR_COVFS)
		lsepoch_overflow_counter_in_tick += 65536;

	uint16_t timer_counter = TC0->TC_CHANNEL[TC_CHANNEL_LSEPOCH].TC_CV;

	if (TC0->TC_CHANNEL[TC_CHANNEL_LSEPOCH].TC_SR & TC_SR_COVFS) {
		lsepoch_overflow_counter_in_tick += 65536;
		/* Overflow occurred during the reading */
		timer_counter = 0;
	}

	cpu_irq_leave_critical();

	/* Convert to us */
	uint64_t curr_time = ((lsepoch_overflow_counter_in_tick + timer_counter) * 1000000)
		/ ULTRASOUND_TIMER_FREQUENCY;

	return (curr_time);
}