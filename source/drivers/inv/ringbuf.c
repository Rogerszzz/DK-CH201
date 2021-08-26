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
 #include "ringbuf.h"
 #include "string.h"

 void ringbuf_init(ringbuf_t* ringbuf, uint8_t* data, int data_size, int unit_size)
 {
	 ringbuf->data = data;
	 ringbuf->unit_size = unit_size;
	 ringbuf->data_end = ringbuf->data + data_size;//(data_size / ringbuf->unit_size) * ringbuf->unit_size;
	 ringbuf->ptr_start = ringbuf->data;
	 ringbuf->ptr_end = ringbuf->data;
	 ringbuf->data_size = ringbuf->data_end - ringbuf->data;
	 ringbuf->available_size = ringbuf->data_size;
 }

 void ringbuf_clear(ringbuf_t* ringbuf)
 {
	 ringbuf->ptr_start = ringbuf->data;
	 ringbuf->ptr_end = ringbuf->data;
	 ringbuf->available_size = ringbuf->data_size;
 }

 int ringbuf_size(ringbuf_t* ringbuf)
 {
	 return ringbuf->data_size - ringbuf->available_size;
 }

 int ringbuf_available(ringbuf_t* ringbuf)
 {
	 return ringbuf->available_size;
 }

 uint8_t* ringbuf_pop_resize(ringbuf_t* ringbuf, int buf_size)
 {
	 uint8_t* ptr = ringbuf->ptr_start;
	 int offset = 0;
	 
	 if (ringbuf->data_size - ringbuf->available_size < buf_size){
		return 0;
	 }
	 
	 ringbuf->available_size += buf_size;
	 offset = ringbuf->data_end - ringbuf->ptr_start;
	 
	 if (offset <= buf_size){
		ringbuf->ptr_start = ringbuf->data + (buf_size - offset);
	 }
	 else{
		ringbuf->ptr_start = ringbuf->ptr_start + buf_size;
	 }
	 
	 return ptr;
 }

 uint8_t* ringbuf_push_resize(ringbuf_t* ringbuf, int buf_size)
 {
	 uint8_t* ptr = ringbuf->ptr_end;
	 
	 ringbuf->available_size -= buf_size;
	 
	 if (ringbuf->available_size < 0)
	 {
		 ringbuf->available_size = 0;
		 
		 ringbuf->ptr_start += buf_size;
		 if(ringbuf->ptr_start >= ringbuf->data_end){
			ringbuf->ptr_start = ringbuf->data + (ringbuf->data_end - ringbuf->ptr_start);
		 }
	 }

	 ringbuf->ptr_end += buf_size;
	 if(ringbuf->ptr_end >= ringbuf->data_end)
	 {
		ringbuf->ptr_end = ringbuf->data + (ringbuf->data_end - ringbuf->ptr_end);
	 }
	 
	 return ptr;
 }

 int ringbuf_push(ringbuf_t* ringbuf, uint8_t* buf, int buf_size)
 {
	 uint8_t* ptr = ringbuf_push_resize(ringbuf, buf_size);
	 int offset = 0;
	 
	 if (ptr == 0)
	 return 0;

	 offset = ringbuf->data_end - ptr;
	 if (offset >= buf_size){
		memcpy(ptr, buf, buf_size);
	 }
	 else
	 {
		 memcpy(ptr, buf, offset);
		 memcpy(ringbuf->data, buf + offset, buf_size - offset);
	 }
	 return buf_size;
 }

 int ringbuf_pop(ringbuf_t* ringbuf, uint8_t* buf, int buf_size)
 {
	 uint8_t* ptr = ringbuf_pop_resize(ringbuf, buf_size);
	 int offset = 0;
	 
	 if (buf == 0){
		if (ptr == 0){
			return 0; 
		}
		else
			{return buf_size;
		}
	 }
	 
	 if (ptr == 0)
	 {
		 return 0;
	 }

	 offset = ringbuf->data_end - ptr;
	 if (offset >= buf_size){
		memcpy(buf, ptr, buf_size);
	 }
	 else
	 {
		 memcpy(buf, ptr, offset);
		 memcpy(buf + offset, ringbuf->data, buf_size - offset);
	 }
	 return buf_size;
 }
