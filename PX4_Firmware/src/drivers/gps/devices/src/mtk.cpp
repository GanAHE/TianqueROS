/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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

/**
 * @file mtk.cpp
 *
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <julian@oes.ch>
 */

#include "mtk.h"

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <ctime>
#include <math.h>


GPSDriverMTK::GPSDriverMTK(GPSCallbackPtr callback, void *callback_user, struct vehicle_gps_position_s *gps_position) :
	GPSHelper(callback, callback_user),
	_gps_position(gps_position)
{
	decodeInit();
}

int
GPSDriverMTK::configure(unsigned &baudrate, OutputMode output_mode)
{
	if (output_mode != OutputMode::GPS) {
		GPS_WARN("MTK: Unsupported Output Mode %i", (int)output_mode);
		return -1;
	}

	if (baudrate > 0 && baudrate != MTK_BAUDRATE) {
		return -1;
	}

	/* set baudrate first */
	if (GPSHelper::setBaudrate(MTK_BAUDRATE) != 0) {
		return -1;
	}

	baudrate = MTK_BAUDRATE;

	/* Write config messages, don't wait for an answer */
	if (strlen(MTK_OUTPUT_5HZ) != write(MTK_OUTPUT_5HZ, strlen(MTK_OUTPUT_5HZ))) {
		goto errout;
	}

	gps_usleep(10000);

	if (strlen(MTK_SET_BINARY) != write(MTK_SET_BINARY, strlen(MTK_SET_BINARY))) {
		goto errout;
	}

	gps_usleep(10000);

	if (strlen(MTK_SBAS_ON) != write(MTK_SBAS_ON, strlen(MTK_SBAS_ON))) {
		goto errout;
	}

	gps_usleep(10000);

	if (strlen(MTK_WAAS_ON) != write(MTK_WAAS_ON, strlen(MTK_WAAS_ON))) {
		goto errout;
	}

	gps_usleep(10000);

	if (strlen(MTK_NAVTHRES_OFF) != write(MTK_NAVTHRES_OFF, strlen(MTK_NAVTHRES_OFF))) {
		goto errout;
	}

	return 0;

errout:
	GPS_WARN("mtk: config write failed");
	return -1;
}

int
GPSDriverMTK::receive(unsigned timeout)
{
	uint8_t buf[GPS_READ_BUFFER_SIZE];
	gps_mtk_packet_t packet{};

	/* timeout additional to poll */
	gps_abstime time_started = gps_absolute_time();

	int j = 0;

	while (true) {

		int ret = read(buf, sizeof(buf), timeout);

		if (ret > 0) {
			/* first read whatever is left */
			if (j < ret) {
				/* pass received bytes to the packet decoder */
				while (j < ret) {
					if (parseChar(buf[j], packet) > 0) {
						handleMessage(packet);
						return 1;
					}

					j++;
				}

				/* everything is read */
				j = 0;
			}

		} else {
			gps_usleep(20000);
		}

		/* in case we keep trying but only get crap from GPS */
		if (time_started + timeout * 1000 < gps_absolute_time()) {
			return -1;
		}
	}
}

void
GPSDriverMTK::decodeInit()
{
	_rx_ck_a = 0;
	_rx_ck_b = 0;
	_rx_count = 0;
	_decode_state = MTK_DECODE_UNINIT;
}
int
GPSDriverMTK::parseChar(uint8_t b, gps_mtk_packet_t &packet)
{
	int ret = 0;

	if (_decode_state == MTK_DECODE_UNINIT) {

		if (b == MTK_SYNC1_V16) {
			_decode_state = MTK_DECODE_GOT_CK_A;
			_mtk_revision = 16;

		} else if (b == MTK_SYNC1_V19) {
			_decode_state = MTK_DECODE_GOT_CK_A;
			_mtk_revision = 19;
		}

	} else if (_decode_state == MTK_DECODE_GOT_CK_A) {
		if (b == MTK_SYNC2) {
			_decode_state = MTK_DECODE_GOT_CK_B;

		} else {
			// Second start symbol was wrong, reset state machine
			decodeInit();
		}

	} else if (_decode_state == MTK_DECODE_GOT_CK_B) {
		// Add to checksum
		if (_rx_count < 33) {
			addByteToChecksum(b);
		}

		// Fill packet buffer
		((uint8_t *)(&packet))[_rx_count] = b;
		_rx_count++;

		/* Packet size minus checksum, XXX ? */
		if (_rx_count >= sizeof(packet)) {
			/* Compare checksum */
			if (_rx_ck_a == packet.ck_a && _rx_ck_b == packet.ck_b) {
				ret = 1;

			} else {
				ret = -1;
			}

			// Reset state machine to decode next packet
			decodeInit();
		}
	}

	return ret;
}

void
GPSDriverMTK::handleMessage(gps_mtk_packet_t &packet)
{
	if (_mtk_revision == 16) {
		_gps_position->lat = packet.latitude * 10; // from degrees*1e6 to degrees*1e7
		_gps_position->lon = packet.longitude * 10; // from degrees*1e6 to degrees*1e7

	} else if (_mtk_revision == 19) {
		_gps_position->lat = packet.latitude; // both degrees*1e7
		_gps_position->lon = packet.longitude; // both degrees*1e7

	} else {
		GPS_WARN("mtk: unknown revision");
		_gps_position->lat = 0;
		_gps_position->lon = 0;

		// Indicate this data is not usable and bail out
		_gps_position->eph = 1000.0f;
		_gps_position->epv = 1000.0f;
		_gps_position->fix_type = 0;
		return;
	}

	_gps_position->alt = (int32_t)(packet.msl_altitude * 10); // from cm to mm
	_gps_position->fix_type = packet.fix_type;
	_gps_position->eph = packet.hdop / 100.0f; // from cm to m
	_gps_position->epv = _gps_position->eph; // unknown in mtk custom mode, so we cheat with eph
	_gps_position->hdop = packet.hdop / 100.0f;
	_gps_position->vdop = _gps_position->hdop;
	_gps_position->vel_m_s = ((float)packet.ground_speed) / 100.0f; // from cm/s to m/s
	_gps_position->cog_rad = ((float)packet.heading) * M_DEG_TO_RAD_F * 1e-2f; //from deg *100 to rad
	_gps_position->satellites_used = packet.satellites;

	/* convert time and date information to unix timestamp */
	struct tm timeinfo = {};
	uint32_t timeinfo_conversion_temp;

	timeinfo.tm_mday = packet.date / 10000;
	timeinfo_conversion_temp = packet.date - timeinfo.tm_mday * 10000;
	timeinfo.tm_mon = (timeinfo_conversion_temp / 100) - 1;
	timeinfo.tm_year = (timeinfo_conversion_temp - (timeinfo.tm_mon + 1) * 100) + 100;

	timeinfo.tm_hour = (packet.utc_time / 10000000);
	timeinfo_conversion_temp = packet.utc_time - timeinfo.tm_hour * 10000000;
	timeinfo.tm_min = timeinfo_conversion_temp / 100000;
	timeinfo_conversion_temp -= timeinfo.tm_min * 100000;
	timeinfo.tm_sec = timeinfo_conversion_temp / 1000;
	timeinfo_conversion_temp -= timeinfo.tm_sec * 1000;

	timeinfo.tm_isdst = 0;

#ifndef NO_MKTIME

	time_t epoch = mktime(&timeinfo);

	if (epoch > GPS_EPOCH_SECS) {
		// FMUv2+ boards have a hardware RTC, but GPS helps us to configure it
		// and control its drift. Since we rely on the HRT for our monotonic
		// clock, updating it from time to time is safe.

		timespec ts{};
		ts.tv_sec = epoch;
		ts.tv_nsec = timeinfo_conversion_temp * 1000000ULL;

		setClock(ts);

		_gps_position->time_utc_usec = static_cast<uint64_t>(epoch) * 1000000ULL;
		_gps_position->time_utc_usec += timeinfo_conversion_temp * 1000ULL;

	} else {
		_gps_position->time_utc_usec = 0;
	}

#else
	_gps_position->time_utc_usec = 0;
#endif

	_gps_position->timestamp = gps_absolute_time();
	_gps_position->timestamp_time_relative = 0;

	// Position and velocity update always at the same time
	_rate_count_vel++;
	_rate_count_lat_lon++;
}

void
GPSDriverMTK::addByteToChecksum(uint8_t b)
{
	_rx_ck_a = _rx_ck_a + b;
	_rx_ck_b = _rx_ck_b + _rx_ck_a;
}
