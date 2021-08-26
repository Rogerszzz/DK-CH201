/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2020 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively “Software”) is subject
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
/** \file ringbuf.h */
#ifndef RINGBUF_H
#define RINGBUF_H

#include <stdint.h>

typedef struct
{
	uint8_t* data;
	uint8_t* data_end;
	uint8_t* ptr_start;
	uint8_t* ptr_end;
	uint8_t unit_size;
	int data_size;
	int available_size;
}ringbuf_t;

void ringbuf_init(ringbuf_t* ringbuf, uint8_t* data, int data_size, int unit_size);
void ringbuf_clear(ringbuf_t* ringbuf);

int ringbuf_size(ringbuf_t* ringbuf);

int ringbuf_available(ringbuf_t* ringbuf);

int ringbuf_push(ringbuf_t* ringbuf, uint8_t* buf, int buf_size);

int ringbuf_pop(ringbuf_t* ringbuf, uint8_t* buf, int buf_size);

uint8_t* ringbuf_pop_resize(ringbuf_t* ringbuf, int buf_size);

uint8_t* ringbuf_push_resize(ringbuf_t* ringbuf, int buf_size);

#endif /* RINGBUF_H */