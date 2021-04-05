/****************************************************************************
 *
 *   Copyright (C) 2013. All rights reserved.
 *   Author: Boriskin Aleksey <a.d.boriskin@gmail.com>
 *           Kistanov Alexander <akistanov@gramant.ru>
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

/** @file ASHTECH protocol definitions */

#pragma once

#include "gps_helper.h"
#include "base_station.h"
#include "../../definitions.h"
#include <math.h>

class RTCMParsing;

#define ASHTECH_RECV_BUFFER_SIZE 512

#define ASH_RESPONSE_TIMEOUT	200		// ms, timeout for waiting for a response

class GPSDriverAshtech : public GPSBaseStationSupport
{
public:
	/**
	 * @param heading_offset heading offset in radians [-pi, pi]. It is substracted from the measurement.
	 */
	GPSDriverAshtech(GPSCallbackPtr callback, void *callback_user, struct vehicle_gps_position_s *gps_position,
			 struct satellite_info_s *satellite_info, float heading_offset = 0.f);
	virtual ~GPSDriverAshtech();

	int receive(unsigned timeout) override;
	int configure(unsigned &baudrate, OutputMode output_mode) override;

private:
	enum class NMEACommand {
		Acked, // Command that returns a (N)Ack
		PRT,   // port config
		RID,   // board identification
		RECEIPT// board identification
	};

	enum class NMEACommandState {
		idle,
		waiting,
		nack,
		received
	};

	enum class NMEADecodeState {
		uninit,
		got_sync1,
		got_asteriks,
		got_first_cs_byte,
		decode_rtcm3
	};

	enum class AshtechBoard {
		trimble_mb_two,
		other
	};

	void decodeInit(void);
	int handleMessage(int len);
	int parseChar(uint8_t b);

	/**
	 * Write a command and wait for a (N)Ack
	 * @return 0 on success, <0 otherwise
	 */
	int writeAckedCommand(const void *buf, int buf_length, unsigned timeout);

	int waitForReply(NMEACommand command, const unsigned timeout);

	/**
	 * receive data for at least the specified amount of time
	 */
	void receiveWait(unsigned timeout_min);

	/**
	 * enable output of correction output
	 */
	void activateCorrectionOutput();

	void sendSurveyInStatusUpdate(bool active, bool valid, double latitude = (double)NAN, double longitude = (double)NAN,
				      float altitude = NAN);

	void activateRTCMOutput();

	struct satellite_info_s *_satellite_info {nullptr};
	struct vehicle_gps_position_s *_gps_position {nullptr};
	uint64_t _last_timestamp_time{0};

	NMEADecodeState _decode_state{NMEADecodeState::uninit};
	uint8_t _rx_buffer[ASHTECH_RECV_BUFFER_SIZE];
	uint16_t _rx_buffer_bytes{};
	bool _got_pashr_pos_message{false}; /**< If we got a PASHR,POS message, we will ignore GGA messages */

	NMEACommand _waiting_for_command;
	NMEACommandState _command_state{NMEACommandState::idle};
	char _port{'A'}; /**< port we are connected to (e.g. 'A') */
	AshtechBoard _board{AshtechBoard::other}; /**< board we are connected to */

	RTCMParsing	*_rtcm_parsing{nullptr};

	gps_abstime _survey_in_start{0};

	OutputMode _output_mode{OutputMode::GPS};
	bool _correction_output_activated{false};
	bool _configure_done{false};

	float _heading_offset;
};

