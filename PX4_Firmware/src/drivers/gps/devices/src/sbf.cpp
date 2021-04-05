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

/**
 * @file sbf.cpp
 *
 * Septentrio protocol as defined in PPSDK SBF Reference Guide 4.1.8
 *
 * @author Matej Frančeškin <Matej.Franceskin@gmail.com>
 *
*/

#include "sbf.h"
#include <string.h>
#include <math.h>

#define SBF_CONFIG_TIMEOUT    500      // ms, timeout for waiting ACK
#define SBF_PACKET_TIMEOUT    2        // ms, if now data during this delay assume that full update received
#define DISABLE_MSG_INTERVAL  1000000  // us, try to disable message with this interval

/**** Trace macros, disable for production builds */
#define SBF_TRACE_PARSER(...)   {/*GPS_INFO(__VA_ARGS__);*/}    /* decoding progress in parse_char() */
#define SBF_TRACE_RXMSG(...)    {/*GPS_INFO(__VA_ARGS__);*/}    /* Rx msgs in payload_rx_done() */

/**** Warning macros, disable to save memory */
#define SBF_WARN(...)        {GPS_WARN(__VA_ARGS__);}
#define SBF_DEBUG(...)       {/*GPS_WARN(__VA_ARGS__);*/}

GPSDriverSBF::GPSDriverSBF(GPSCallbackPtr callback, void *callback_user,
			   struct vehicle_gps_position_s *gps_position,
			   struct satellite_info_s *satellite_info,
			   uint8_t dynamic_model)
	: GPSBaseStationSupport(callback, callback_user)
	, _gps_position(gps_position)
	, _satellite_info(satellite_info)
	, _dynamic_model(dynamic_model)
{
	decodeInit();
}

GPSDriverSBF::~GPSDriverSBF()
{
}

int
GPSDriverSBF::configure(unsigned &baudrate, OutputMode output_mode)
{
	_configured = false;

	setBaudrate(SBF_TX_CFG_PRT_BAUDRATE);
	baudrate = SBF_TX_CFG_PRT_BAUDRATE;

	_output_mode = output_mode;

	if (output_mode != OutputMode::RTCM) {
		sendMessage(SBF_CONFIG_FORCE_INPUT);
	}

	// Change the baudrate
	char msg[64];
	snprintf(msg, sizeof(msg), SBF_CONFIG_BAUDRATE, baudrate);

	if (!sendMessage(msg)) {
		return -1; // connection and/or baudrate detection failed
	}

	/* flush input and wait for at least 50 ms silence */
	decodeInit();
	receive(50);
	decodeInit();

	if (!sendMessageAndWaitForAck(SBF_CONFIG_RESET, SBF_CONFIG_TIMEOUT)) {
		return -1; // connection and/or baudrate detection failed
	}

	// at this point we have correct baudrate on both ends

	const char *config_cmds;

	if (_output_mode == OutputMode::RTCM) {
		config_cmds = SBF_CONFIG_RTCM;

	} else {

		if (_dynamic_model < 6) {
			snprintf(msg, sizeof(msg), SBF_CONFIG_RECEIVER_DYNAMICS, "low");

		} else if (_dynamic_model < 7) {
			snprintf(msg, sizeof(msg), SBF_CONFIG_RECEIVER_DYNAMICS, "moderate");

		} else if (_dynamic_model < 8) {
			snprintf(msg, sizeof(msg), SBF_CONFIG_RECEIVER_DYNAMICS, "high");

		} else {
			snprintf(msg, sizeof(msg), SBF_CONFIG_RECEIVER_DYNAMICS, "max");
		}

		sendMessageAndWaitForAck(msg, SBF_CONFIG_TIMEOUT);

		config_cmds = SBF_CONFIG;
	}

	uint8_t i = 0;
	msg[0] = 0;

	while (*config_cmds != 0) {
		msg[i] = *config_cmds;

		if (msg[i++] == '\n') {
			msg[i] = 0;

			sendMessageAndWaitForAck(msg, SBF_CONFIG_TIMEOUT);

			i = 0;
			msg[0] = 0;
		}

		config_cmds++;
	}

	if (_output_mode == OutputMode::RTCM) {
		if (_base_settings.type == BaseSettingsType::fixed_position) {
			snprintf(msg, sizeof(msg), SBF_CONFIG_RTCM_STATIC_COORDINATES,
				 _base_settings.settings.fixed_position.latitude,
				 _base_settings.settings.fixed_position.longitude,
				 static_cast<double>(_base_settings.settings.fixed_position.altitude));
			sendMessageAndWaitForAck(msg, SBF_CONFIG_TIMEOUT);
			snprintf(msg, sizeof(msg), SBF_CONFIG_RTCM_STATIC_OFFSET, 0.0, 0.0, 0.0);
			sendMessageAndWaitForAck(msg, SBF_CONFIG_TIMEOUT);
			sendMessageAndWaitForAck(SBF_CONFIG_RTCM_STATIC1, SBF_CONFIG_TIMEOUT);
			sendMessageAndWaitForAck(SBF_CONFIG_RTCM_STATIC2, SBF_CONFIG_TIMEOUT);
		}
	}

	_configured = true;
	return 0;
}

bool
GPSDriverSBF::sendMessage(const char *msg)
{
	SBF_DEBUG("Send MSG: %s", msg);

	// Send message
	int length = static_cast<int>(strlen(msg));

	return (write(msg, length) == length);
}

bool
GPSDriverSBF::sendMessageAndWaitForAck(const char *msg, const int timeout)
{
	SBF_DEBUG("Send MSG: %s", msg);

	// Send message
	int length = static_cast<int>(strlen(msg));

	if (write(msg, length) != length) {
		return false;
	}

	// Wait for acknowledge
	// For all valid set -, get - and exe -commands, the first line of the reply is an exact copy
	// of the command as entered by the user, preceded with "$R:"
	char buf[GPS_READ_BUFFER_SIZE];
	size_t offset = 1;
	gps_abstime time_started = gps_absolute_time();

	bool found_response = false;

	do {
		--offset; //overwrite the null-char
		int ret = read(reinterpret_cast<uint8_t *>(buf) + offset, sizeof(buf) - offset - 1, timeout);

		if (ret < 0) {
			// something went wrong when polling or reading
			SBF_WARN("sbf poll_or_read err");
			return false;
		}

		offset += ret;
		buf[offset++] = '\0';

		if (!found_response && strstr(buf, "$R: ") != nullptr) {
			SBF_DEBUG("READ %d: %s", (int)offset, buf);
			found_response = true;
		}

		if (offset >= sizeof(buf)) {
			offset = 1;
		}

	} while (time_started + 1000 * timeout > gps_absolute_time());

	return found_response;
}

int    // -1 = error, 0 = no message handled, 1 = message handled, 2 = sat info message handled
GPSDriverSBF::receive(unsigned timeout)
{
	// Do not receive messages until we're configured
	if (!_configured) {
		gps_usleep(timeout * 1000);
		return 0;
	}

	uint8_t buf[GPS_READ_BUFFER_SIZE];

	// timeout additional to poll
	gps_abstime time_started = gps_absolute_time();

	int handled = 0;

	while (true) {
		// Wait for only SBF_PACKET_TIMEOUT if something already received.
		int ret = read(buf, sizeof(buf), handled ? SBF_PACKET_TIMEOUT : timeout);

		if (ret < 0) {
			// something went wrong when polling or reading
			SBF_WARN("ubx poll_or_read err");
			return -1;

		} else {
			// SBF_DEBUG("Read %d bytes", ret);

			// pass received bytes to the packet decoder
			for (int i = 0; i < ret; i++) {
				handled |= parseChar(buf[i]);
				// SBF_DEBUG("parsed %d: 0x%x", i, buf[i]);
			}
		}

		if (handled > 0) {
			return handled;
		}

		// abort after timeout if no useful packets received
		if (time_started + timeout * 1000 < gps_absolute_time()) {
			SBF_DEBUG("timed out, returning");
			return -1;
		}
	}
}

int    // 0 = decoding, 1 = message handled, 2 = sat info message handled
GPSDriverSBF::parseChar(const uint8_t b)
{
	int ret = 0;

	switch (_decode_state) {

	// Expecting Sync1
	case SBF_DECODE_SYNC1:
		if (b == SBF_SYNC1) { // Sync1 found --> expecting Sync2
			SBF_TRACE_PARSER("A");
			ret = payloadRxAdd(b); // add a payload byte
			_decode_state = SBF_DECODE_SYNC2;

		} else if (b == RTCM3_PREAMBLE && _rtcm_parsing) {
			SBF_TRACE_PARSER("RTCM");
			_decode_state = SBF_DECODE_RTCM3;
			_rtcm_parsing->addByte(b);
		}

		break;

	// Expecting Sync2
	case SBF_DECODE_SYNC2:
		if (b == SBF_SYNC2) { // Sync2 found --> expecting CRC
			SBF_TRACE_PARSER("B");
			ret = payloadRxAdd(b); // add a payload byte
			_decode_state = SBF_DECODE_PAYLOAD;

		} else { // Sync1 not followed by Sync2: reset parser
			decodeInit();
		}

		break;

	// Expecting payload
	case SBF_DECODE_PAYLOAD:
		SBF_TRACE_PARSER(".");

		ret = payloadRxAdd(b); // add a payload byte

		if (ret < 0) {
			// payload not handled, discard message
			ret = 0;
			decodeInit();

		} else if (ret > 0) {
			ret = payloadRxDone(); // finish payload processing
			decodeInit();

		} else {
			// expecting more payload, stay in state SBF_DECODE_PAYLOAD
			ret = 0;

		}

		break;

	case SBF_DECODE_RTCM3:
		if (_rtcm_parsing->addByte(b)) {
			SBF_DEBUG("got RTCM message with length %i", (int)_rtcm_parsing->messageLength());
			gotRTCMMessage(_rtcm_parsing->message(), _rtcm_parsing->messageLength());
			decodeInit();
		}

		break;
	}

	return ret;
}

/**
 * Add payload rx byte
 */
int    // -1 = error, 0 = ok, 1 = payload completed
GPSDriverSBF::payloadRxAdd(const uint8_t b)
{
	int ret = 0;
	uint8_t *p_buf = reinterpret_cast<uint8_t *>(&_buf);

	p_buf[_rx_payload_index++] = b;

	if ((_rx_payload_index > 7 && _rx_payload_index >= _buf.length) || _rx_payload_index >= sizeof(_buf)) {
		ret = 1; // payload received completely
	}

	return ret;
}

/**
 * Calculate buffer CRC16
 */
uint16_t
crc16(const uint8_t *data_p, uint32_t length)
{
	uint8_t x;
	uint16_t crc = 0;

	while (length--) {
		x = crc >> 8 ^ *data_p++;
		x ^= x >> 4;
		crc = static_cast<uint16_t>((crc << 8) ^ (x << 12) ^ (x << 5) ^ x);
	}

	return crc;
}

/**
 * Finish payload rx
 */
int    // 0 = no message handled, 1 = message handled, 2 = sat info message handled
GPSDriverSBF::payloadRxDone()
{
	int ret = 0;
	struct tm timeinfo;
	time_t epoch;

	if (_buf.length <= 4 || _buf.crc16 != crc16(reinterpret_cast<uint8_t *>(&_buf) + 4, _buf.length - 4)) {
		return 1;
	}

	// handle message
	switch (_buf.msg_id) {
	case SBF_ID_PVTGeodetic:
		SBF_TRACE_RXMSG("Rx PVTGeodetic");
		_msg_status |= 1;

		if (_buf.payload_pvt_geodetic.mode_type < 1) {
			_gps_position->fix_type = 1;

		} else if (_buf.payload_pvt_geodetic.mode_type == 6) {
			_gps_position->fix_type = 4;

		} else if (_buf.payload_pvt_geodetic.mode_type == 5 || _buf.payload_pvt_geodetic.mode_type == 8) {
			_gps_position->fix_type = 5;

		} else if (_buf.payload_pvt_geodetic.mode_type == 4 || _buf.payload_pvt_geodetic.mode_type == 7) {
			_gps_position->fix_type = 6;

		} else {
			_gps_position->fix_type = 3;
		}

		// Check fix and error code
		_gps_position->vel_ned_valid = _gps_position->fix_type > 1 && _buf.payload_pvt_geodetic.error == 0;

		// Check boundaries and invalidate GPS velocities
		// We're not just checking for the do-not-use value (-2*10^10) but for any value beyond the specified max values
		if (fabsf(_buf.payload_pvt_geodetic.vn) > 600.0f || fabsf(_buf.payload_pvt_geodetic.ve) > 600.0f ||
		    fabsf(_buf.payload_pvt_geodetic.vu) > 600.0f) {
			_gps_position->vel_ned_valid = false;
		}

		// Check boundaries and invalidate position
		// We're not just checking for the do-not-use value (-2*10^10) but for any value beyond the specified max values
		if (fabs(_buf.payload_pvt_geodetic.latitude) > M_PI_2 || fabs(_buf.payload_pvt_geodetic.longitude) > M_PI_2 ||
		    fabs(_buf.payload_pvt_geodetic.height) > 100000.0 || fabs(_buf.payload_pvt_geodetic.undulation) > 100000.0) {
			_gps_position->fix_type = 0;
		}

		if (_buf.payload_pvt_geodetic.nr_sv < 255) {  // 255 = do not use value
			_gps_position->satellites_used = _buf.payload_pvt_geodetic.nr_sv;

			if (_satellite_info) {
				// Only fill in the satellite count for now (we could use the ChannelStatus message for the
				// other data, but it's really large: >800B)
				_satellite_info->timestamp = gps_absolute_time();
				_satellite_info->count = _gps_position->satellites_used;
				ret = 2;
			}

		} else {
			_gps_position->satellites_used = 0;
		}

		_gps_position->lat = static_cast<int>(round(_buf.payload_pvt_geodetic.latitude * M_RAD_TO_DEG * 1e7));
		_gps_position->lon = static_cast<int>(round(_buf.payload_pvt_geodetic.longitude * M_RAD_TO_DEG * 1e7));
		_gps_position->alt_ellipsoid = static_cast<int>(round(_buf.payload_pvt_geodetic.height * 1000));
		_gps_position->alt = static_cast<int>(round((_buf.payload_pvt_geodetic.height - static_cast<double>
						      (_buf.payload_pvt_geodetic.undulation)) * 1000));

		/* H and V accuracy are reported in 2DRMS, but based off the uBlox reporting we expect RMS.
		 * Devide by 100 from cm to m and in addition divide by 2 to get RMS. */
		_gps_position->eph = static_cast<float>(_buf.payload_pvt_geodetic.h_accuracy) / 200.0f;
		_gps_position->epv = static_cast<float>(_buf.payload_pvt_geodetic.v_accuracy) / 200.0f;

		_gps_position->vel_n_m_s = static_cast<float>(_buf.payload_pvt_geodetic.vn);
		_gps_position->vel_e_m_s = static_cast<float>(_buf.payload_pvt_geodetic.ve);
		_gps_position->vel_d_m_s = -1.0f * static_cast<float>(_buf.payload_pvt_geodetic.vu);
		_gps_position->vel_m_s = sqrtf(_gps_position->vel_n_m_s * _gps_position->vel_n_m_s + _gps_position->vel_e_m_s *
					       _gps_position->vel_e_m_s);

		_gps_position->cog_rad = static_cast<float>(_buf.payload_pvt_geodetic.cog) * M_DEG_TO_RAD_F;
		_gps_position->c_variance_rad = 1.0f * M_DEG_TO_RAD_F;

		// _buf.payload_pvt_geodetic.cog is set to -2*10^10 for velocities below 0.1m/s
		if (_buf.payload_pvt_geodetic.cog > 360.0f) {
			_buf.payload_pvt_geodetic.cog = NAN;
		}

		_gps_position->time_utc_usec = 0;
#ifndef NO_MKTIME
		/* convert to unix timestamp */
		memset(&timeinfo, 0, sizeof(timeinfo));

		timeinfo.tm_year = 1980 - 1900;
		timeinfo.tm_mon = 0;
		timeinfo.tm_mday = 6 + _buf.WNc * 7;
		timeinfo.tm_hour = 0;
		timeinfo.tm_min = 0;
		timeinfo.tm_sec = _buf.TOW / 1000;

		epoch = mktime(&timeinfo);

		if (epoch > GPS_EPOCH_SECS) {
			// FMUv2+ boards have a hardware RTC, but GPS helps us to configure it
			// and control its drift. Since we rely on the HRT for our monotonic
			// clock, updating it from time to time is safe.

			timespec ts;
			memset(&ts, 0, sizeof(ts));
			ts.tv_sec = epoch;
			ts.tv_nsec = (_buf.TOW % 1000) * 1000 * 1000;
			setClock(ts);

			_gps_position->time_utc_usec = static_cast<uint64_t>(epoch) * 1000000ULL;
			_gps_position->time_utc_usec += (_buf.TOW % 1000) * 1000;
		}

#endif
		_gps_position->timestamp = gps_absolute_time();
		_last_timestamp_time = _gps_position->timestamp;
		_rate_count_vel++;
		_rate_count_lat_lon++;
		ret |= (_msg_status == 7) ? 1 : 0;
		break;

	case SBF_ID_VelCovGeodetic:
		SBF_TRACE_RXMSG("Rx VelCovGeodetic");
		_msg_status |= 2;
		_gps_position->s_variance_m_s = _buf.payload_vel_col_geodetic.cov_ve_ve;

		if (_gps_position->s_variance_m_s < _buf.payload_vel_col_geodetic.cov_vn_vn) {
			_gps_position->s_variance_m_s = _buf.payload_vel_col_geodetic.cov_vn_vn;
		}

		if (_gps_position->s_variance_m_s < _buf.payload_vel_col_geodetic.cov_vu_vu) {
			_gps_position->s_variance_m_s = _buf.payload_vel_col_geodetic.cov_vu_vu;
		}

		break;

	case SBF_ID_DOP:
		SBF_TRACE_RXMSG("Rx DOP");
		_msg_status |= 4;
		_gps_position->hdop = _buf.payload_dop.hDOP * 0.01f;
		_gps_position->vdop = _buf.payload_dop.vDOP * 0.01f;
		break;

	default:
		break;
	}

	if (ret > 0) {
		_gps_position->timestamp_time_relative = static_cast<int32_t>(_last_timestamp_time - _gps_position->timestamp);
	}

	if (ret == 1) {
		_msg_status &= ~1;
	}

	return ret;
}

void
GPSDriverSBF::decodeInit()
{
	_decode_state = SBF_DECODE_SYNC1;
	_rx_payload_index = 0;

	if (_output_mode == OutputMode::RTCM) {
		if (!_rtcm_parsing) {
			_rtcm_parsing = new RTCMParsing();
		}

		_rtcm_parsing->reset();
	}
}

int
GPSDriverSBF::reset(GPSRestartType restart_type)
{
	bool res = false;

	switch (restart_type) {
	case GPSRestartType::Hot:
		res = sendMessageAndWaitForAck(SBF_CONFIG_RESET_HOT, SBF_CONFIG_TIMEOUT);
		break;

	case GPSRestartType::Warm:
		res = sendMessageAndWaitForAck(SBF_CONFIG_RESET_WARM, SBF_CONFIG_TIMEOUT);
		break;

	case GPSRestartType::Cold:
		res = sendMessageAndWaitForAck(SBF_CONFIG_RESET_COLD, SBF_CONFIG_TIMEOUT);
		break;

	default:
		break;
	}

	return (res) ? 0 : -2;
}
