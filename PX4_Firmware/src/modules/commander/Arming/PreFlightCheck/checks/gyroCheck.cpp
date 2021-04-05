/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include "../PreFlightCheck.hpp"

#include <drivers/drv_hrt.h>
#include <HealthFlags.h>
#include <px4_defines.h>
#include <systemlib/mavlink_log.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/subsystem_info.h>

using namespace time_literals;

bool PreFlightCheck::gyroCheck(orb_advert_t *mavlink_log_pub, vehicle_status_s &status, const uint8_t instance,
			       const bool optional, int32_t &device_id, const bool report_fail)
{
	const bool exists = (orb_exists(ORB_ID(sensor_gyro), instance) == PX4_OK);
	bool calibration_valid = false;
	bool gyro_valid = false;

	if (exists) {

		uORB::SubscriptionData<sensor_gyro_s> gyro{ORB_ID(sensor_gyro), instance};

		gyro_valid = (hrt_elapsed_time(&gyro.get().timestamp) < 1_s);

		if (!gyro_valid) {
			if (report_fail) {
				mavlink_log_critical(mavlink_log_pub, "Preflight Fail: no valid data from Gyro #%u", instance);
			}
		}

		device_id = gyro.get().device_id;

		calibration_valid = check_calibration("CAL_GYRO%u_ID", device_id);

		if (!calibration_valid) {
			if (report_fail) {
				mavlink_log_critical(mavlink_log_pub, "Preflight Fail: Gyro #%u uncalibrated", instance);
			}
		}

	} else {
		if (!optional && report_fail) {
			mavlink_log_critical(mavlink_log_pub, "Preflight Fail: Gyro Sensor #%u missing", instance);
		}
	}

	if (instance == 0) {
		set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_GYRO, exists, !optional, calibration_valid && gyro_valid, status);

	} else if (instance == 1) {
		set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_GYRO2, exists, !optional, calibration_valid && gyro_valid, status);
	}

	return calibration_valid && gyro_valid;
}
