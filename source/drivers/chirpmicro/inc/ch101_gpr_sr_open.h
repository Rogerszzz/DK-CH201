/*! \file ch101_gpr_sr_open.h
 *
 * \brief Internal definitions for the Chirp CH101 GPR Short-range Open sensor firmware.
 *
 * This file contains register offsets and other values for use with the CH101 GPR Short-rangeOpen
 * sensor firmware.  These values are subject to change without notice.
 *
 * You should not need to edit this file or call the driver functions directly.  Doing so
 * will reduce your ability to benefit from future enhancements and releases from Chirp.
 *
 */

/*
 * Copyright © 2016-2019, Chirp Microsystems.  All rights reserved.
 *
 * Chirp Microsystems CONFIDENTIAL
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CHIRP MICROSYSTEMS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * You can contact the authors of this program by email at support@chirpmicro.com
 * or by mail at 2560 Ninth Street, Suite 220, Berkeley, CA 94710.
 */

#ifndef CH101_GPR_SR_OPEN_H_
#define CH101_GPR_SR_OPEN_H_

#include "ch101.h"
#include "soniclib.h"
#include <stdint.h>

/* GPR firmware registers */
#define CH101_GPR_SR_OPEN_REG_OPMODE 			0x01
#define CH101_GPR_SR_OPEN_REG_TICK_INTERVAL 	0x02
#define CH101_GPR_SR_OPEN_REG_PERIOD 			0x05
#define CH101_GPR_SR_OPEN_REG_CAL_TRIG 			0x06
#define CH101_GPR_SR_OPEN_REG_CAL_TRIG 			0x06
#define CH101_GPR_SR_OPEN_REG_MAX_RANGE 		0x07
#define CH101_GPR_SR_OPEN_REG_CALC				0x08
#define CH101_GPR_SR_OPEN_REG_ST_RANGE 			0x12
#define CH101_GPR_SR_OPEN_REG_READY 			0x14
#define CH101_GPR_SR_OPEN_REG_TOF_SF 			0x16
#define CH101_GPR_SR_OPEN_REG_TOF 				0x18
#define CH101_GPR_SR_OPEN_REG_AMPLITUDE 		0x1A
#define CH101_GPR_SR_OPEN_REG_CAL_RESULT 		0x0A
#define CH101_GPR_SR_OPEN_REG_DATA 				0x1C

#define	CH101_GPR_SR_OPEN_CTR					(0x2B368)
#define CH101_GPR_SR_OPEN_MAX_SAMPLES			(150)

extern const char *ch101_gpr_sr_open_version;		// version string in fw .c file
extern const uint8_t ch101_gpr_sr_open_fw[CH101_FW_SIZE];

uint16_t get_ch101_gpr_sr_open_fw_ram_init_addr(void);
uint16_t get_ch101_gpr_sr_open_fw_ram_init_size(void);

const unsigned char * get_ram_ch101_gpr_sr_open_init_ptr(void);

uint8_t ch101_gpr_sr_open_init(ch_dev_t *dev_ptr, ch_group_t *grp_ptr, uint8_t i2c_addr, 
							   uint8_t dev_num, uint8_t i2c_bus_index);

void ch101_gpr_sr_open_store_pt_result(ch_dev_t *dev_ptr);



#endif
