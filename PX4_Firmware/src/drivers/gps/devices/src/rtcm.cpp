/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "rtcm.h"
#include <cstring>

RTCMParsing::RTCMParsing()
{
	reset();
}

RTCMParsing::~RTCMParsing()
{
	if (_buffer) {
		delete[](_buffer);
	}
}

void RTCMParsing::reset()
{
	if (!_buffer) {
		_buffer = new uint8_t[RTCM_INITIAL_BUFFER_LENGTH];
		_buffer_len = RTCM_INITIAL_BUFFER_LENGTH;
	}

	_pos = 0;
	_message_length = _buffer_len;
}

bool RTCMParsing::addByte(uint8_t b)
{
	_buffer[_pos++] = b;

	if (_pos == 3) {
		_message_length = (((uint16_t)_buffer[1] & 3) << 8) | (_buffer[2]);

		if (_message_length + 6 > _buffer_len) {
			uint16_t new_buffer_len = _message_length + 6;
			uint8_t *new_buffer = new uint8_t[new_buffer_len];
			memcpy(new_buffer, _buffer, 3);
			delete[](_buffer);
			_buffer = new_buffer;
			_buffer_len = new_buffer_len;
		}
	}

	return _message_length + 6 == _pos;
}
