/*! \file ch201_presence.h
 *
 * \brief Internal definitions for the Chirp CH201 presence detection sensor firmware.
 *
 * This file contains register offsets and other values for use with the CH201 presence 
 * sensor firmware.  These values are subject to change without notice.
 *
 * You should not need to edit this file or call the driver functions directly.  Doing so
 * will reduce your ability to benefit from future enhancements and releases from Chirp.
 *
 */

/*
 * Copyright © 2016-2020, Chirp Microsystems.  All rights reserved.
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

#ifndef CH201_PRESENCE_H_
#define CH201_PRESENCE_H_

#include <stdint.h>
#include "ch201.h"
#include "soniclib.h"

/* Presence detection firmware registers */
#define CH201_PRESENCE_REG_OPMODE        (0x01)
#define CH201_PRESENCE_REG_TICK_INTERVAL (0x02)
#define CH201_PRESENCE_REG_PERIOD        (0x05)
#define CH201_PRESENCE_REG_CAL_TRIG      (0x06)
#define CH201_PRESENCE_REG_MAX_RANGE     (0x07)
#define CH201_PRESENCE_REG_THRESH_LEN_0  (0x08)
#define CH201_PRESENCE_REG_THRESH_LEN_1  (0x09)
#define CH201_PRESENCE_REG_CAL_RESULT    (0x0A)
#define CH201_PRESENCE_REG_THRESH_LEN_2  (0x0C)
#define CH201_PRESENCE_REG_THRESH_LEN_3  (0x0D)
#define CH201_PRESENCE_PMUTRINGLENGTH    (0x12)
#define CH201_PRESENCE_DECIMATION        (0x13)
#define CH201_PRESENCE_REG_READY         (0x14)
#define CH201_PRESENCE_REG_THRESH_LEN_4  (0x15)
#define CH201_PRESENCE_REG_THRESHOLDS    (0x16) // start of array of six 2-byte threshold levels
#define CH201_PRESENCE_REG_TOF_SF        (0x22)
#define CH201_PRESENCE_REG_TOF           (0x24)
#define CH201_PRESENCE_REG_AMPLITUDE     (0x26)
#define CH201_PRESENCE_REG_DATA          (0x28)

#define CH201_PRESENCE_MAX_SAMPLES    (300) // max number of samples
#define CH201_PRESENCE_NUM_THRESHOLDS (6)   // total number of thresholds

// Enumerated values for various registers
#define CH201_PRESENCE_DECIMATION_MASK (0x03)
#define CH201_PRESENCE_DECIMATION_NO   (0x00)
#define CH201_PRESENCE_DECIMATION_2    (0x01)
#define CH201_PRESENCE_DECIMATION_3    (0x02)
#define CH201_PRESENCE_DECIMATION_4    (0x03)

#define CH201_PRESENCE_CORDIC_MASK     (0x80)
#define CH201_PRESENCE_CORDIC_EN       (0x80)
#define CH201_PRESENCE_CORDIC_DIS      (0x00)

#define CH201_PRESENCE_READY_FREQ_LOCKED (0x04)
#define CH201_PRESENCE_FREQCOUNTERCYCLES (1150)

extern const char *ch201_presence_version; // version string in fw .c file
extern const uint8_t ch201_presence_fw[CH201_FW_SIZE];

uint16_t get_ch201_presence_fw_ram_init_addr(void);
uint16_t get_ch201_presence_fw_ram_init_size(void);

const unsigned char * get_ram_ch201_presence_init_ptr(void);

uint8_t ch201_presence_init(ch_dev_t *dev_ptr, ch_group_t *grp_ptr, uint8_t i2c_addr, uint8_t dev_num, uint8_t i2c_bus_index);
uint8_t ch201_presence_set_decimation(ch_dev_t *dev_ptr, uint8_t decimation);
uint8_t ch201_presence_enable_cordic(ch_dev_t *dev_ptr, uint8_t enable);

#endif
