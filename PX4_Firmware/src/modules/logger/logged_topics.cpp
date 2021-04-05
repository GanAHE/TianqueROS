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

#include "logged_topics.h"
#include "messages.h"

#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <uORB/topics/uORBTopics.hpp>

#include <string.h>

using namespace px4::logger;

void LoggedTopics::add_default_topics()
{
	add_topic("actuator_controls_0", 100);
	add_topic("actuator_controls_1", 100);
	add_topic("airspeed", 1000);
	add_topic("airspeed_validated", 1000);
	add_topic("camera_capture");
	add_topic("camera_trigger");
	add_topic("camera_trigger_secondary");
	add_topic("cellular_status", 200);
	add_topic("cpuload");
	add_topic("ekf_gps_drift");
	add_topic("esc_status", 250);
	add_topic("estimator_innovation_test_ratios", 200);
	add_topic("estimator_innovation_variances", 200);
	add_topic("estimator_innovations", 200);
	add_topic("estimator_sensor_bias", 1000);
	add_topic("estimator_status", 200);
	add_topic("home_position");
	add_topic("hover_thrust_estimate", 100);
	add_topic("input_rc", 200);
	add_topic("manual_control_setpoint", 200);
	add_topic("mission");
	add_topic("mission_result");
	add_topic("offboard_control_mode", 1000);
	add_topic("position_controller_status", 500);
	add_topic("position_setpoint_triplet", 200);
	add_topic("radio_status");
	add_topic("rate_ctrl_status", 200);
	add_topic("rpm", 500);
	add_topic("safety", 1000);
	add_topic("sensor_combined", 100);
	add_topic("sensor_correction", 1000);
	add_topic("sensor_preflight", 200);
	add_topic("sensor_selection");
	add_topic("system_power", 500);
	add_topic("tecs_status", 200);
	add_topic("trajectory_setpoint", 200);
	add_topic("vehicle_air_data", 200);
	add_topic("vehicle_angular_velocity", 20);
	add_topic("vehicle_attitude", 50);
	add_topic("vehicle_attitude_setpoint", 100);
	add_topic("vehicle_command");
	add_topic("vehicle_global_position", 200);
	add_topic("vehicle_land_detected");
	add_topic("vehicle_local_position", 100);
	add_topic("vehicle_local_position_setpoint", 100);
	add_topic("vehicle_magnetometer", 200);
	add_topic("vehicle_rates_setpoint", 20);
	add_topic("vehicle_roi", 1000);
	add_topic("vehicle_status", 200);
	add_topic("vehicle_status_flags");
	add_topic("vtol_vehicle_status", 200);
	add_topic("yaw_estimator_status", 200);

	// multi topics
	add_topic_multi("actuator_outputs", 100);
	add_topic_multi("logger_status");
	add_topic_multi("multirotor_motor_limits", 1000);
	add_topic_multi("telemetry_status", 1000);
	add_topic_multi("wind_estimate", 1000);

	// log all raw sensors at minimal rate (1 Hz)
	add_topic_multi("battery_status", 1000);
	add_topic_multi("differential_pressure", 1000);
	add_topic_multi("distance_sensor", 1000);
	add_topic_multi("optical_flow", 1000);
	add_topic_multi("sensor_accel", 1000);
	add_topic_multi("sensor_accel_status", 1000);
	add_topic_multi("sensor_baro", 1000);
	add_topic_multi("sensor_gyro", 1000);
	add_topic_multi("sensor_gyro_status", 1000);
	add_topic_multi("sensor_mag", 1000);
	add_topic_multi("vehicle_gps_position", 1000);

#ifdef CONFIG_ARCH_BOARD_PX4_SITL
	add_topic("actuator_controls_virtual_fw");
	add_topic("actuator_controls_virtual_mc");
	add_topic("fw_virtual_attitude_setpoint");
	add_topic("mc_virtual_attitude_setpoint");
	add_topic("time_offset");
	add_topic("vehicle_angular_acceleration", 10);
	add_topic("vehicle_angular_velocity", 10);
	add_topic("vehicle_attitude_groundtruth", 10);
	add_topic("vehicle_global_position_groundtruth", 100);
	add_topic("vehicle_local_position_groundtruth", 100);
#endif /* CONFIG_ARCH_BOARD_PX4_SITL */
}

void LoggedTopics::add_high_rate_topics()
{
	// maximum rate to analyze fast maneuvers (e.g. for racing)
	add_topic("actuator_controls_0");
	add_topic("actuator_outputs");
	add_topic("manual_control_setpoint");
	add_topic("rate_ctrl_status", 20);
	add_topic("sensor_combined");
	add_topic("vehicle_angular_acceleration");
	add_topic("vehicle_angular_velocity");
	add_topic("vehicle_attitude");
	add_topic("vehicle_attitude_setpoint");
	add_topic("vehicle_rates_setpoint");
}

void LoggedTopics::add_debug_topics()
{
	add_topic("debug_array");
	add_topic("debug_key_value");
	add_topic("debug_value");
	add_topic("debug_vect");
}

void LoggedTopics::add_estimator_replay_topics()
{
	// for estimator replay (need to be at full rate)
	add_topic("ekf2_timestamps");
	add_topic("ekf_gps_position");

	// current EKF2 subscriptions
	add_topic("airspeed");
	add_topic("optical_flow");
	add_topic("sensor_combined");
	add_topic("sensor_selection");
	add_topic("vehicle_air_data");
	add_topic("vehicle_land_detected");
	add_topic("vehicle_magnetometer");
	add_topic("vehicle_status");
	add_topic("vehicle_visual_odometry");
	add_topic("vehicle_visual_odometry_aligned");
	add_topic_multi("distance_sensor");
	add_topic_multi("vehicle_gps_position");
}

void LoggedTopics::add_thermal_calibration_topics()
{
	add_topic_multi("sensor_accel", 100);
	add_topic_multi("sensor_baro", 100);
	add_topic_multi("sensor_gyro", 100);
}

void LoggedTopics::add_sensor_comparison_topics()
{
	add_topic_multi("sensor_accel", 100);
	add_topic_multi("sensor_baro", 100);
	add_topic_multi("sensor_gyro", 100);
	add_topic_multi("sensor_mag", 100);
}

void LoggedTopics::add_vision_and_avoidance_topics()
{
	add_topic("collision_constraints");
	add_topic("obstacle_distance_fused");
	add_topic("onboard_computer_status", 200);
	add_topic("vehicle_mocap_odometry", 30);
	add_topic("vehicle_trajectory_waypoint", 200);
	add_topic("vehicle_trajectory_waypoint_desired", 200);
	add_topic("vehicle_visual_odometry", 30);
}

void LoggedTopics::add_system_identification_topics()
{
	// for system id need to log imu and controls at full rate
	add_topic("actuator_controls_0");
	add_topic("actuator_controls_1");
	add_topic("sensor_combined");
}

int LoggedTopics::add_topics_from_file(const char *fname)
{
	int ntopics = 0;

	/* open the topic list file */
	FILE *fp = fopen(fname, "r");

	if (fp == nullptr) {
		return -1;
	}

	/* call add_topic for each topic line in the file */
	for (;;) {
		/* get a line, bail on error/EOF */
		char line[80];
		line[0] = '\0';

		if (fgets(line, sizeof(line), fp) == nullptr) {
			break;
		}

		/* skip comment lines */
		if ((strlen(line) < 2) || (line[0] == '#')) {
			continue;
		}

		// read line with format: <topic_name>[ <interval>[ <instance>]]
		char topic_name[80];
		uint32_t interval_ms = 0;
		uint32_t instance = 0;
		int nfields = sscanf(line, "%s %u %u", topic_name, &interval_ms, &instance);

		if (nfields > 0) {
			int name_len = strlen(topic_name);

			if (name_len > 0 && topic_name[name_len - 1] == ',') {
				topic_name[name_len - 1] = '\0';
			}

			/* add topic with specified interval_ms */
			if ((nfields > 2 && add_topic(topic_name, interval_ms, instance))
			    || add_topic_multi(topic_name, interval_ms)) {
				ntopics++;

			} else {
				PX4_ERR("Failed to add topic %s", topic_name);
			}
		}
	}

	fclose(fp);
	return ntopics;
}

void LoggedTopics::initialize_mission_topics(MissionLogType mission_log_type)
{
	if (mission_log_type == MissionLogType::Complete) {
		add_mission_topic("camera_capture");
		add_mission_topic("mission_result");
		add_mission_topic("vehicle_global_position", 1000);
		add_mission_topic("vehicle_status", 1000);

	} else if (mission_log_type == MissionLogType::Geotagging) {
		add_mission_topic("camera_capture");
	}
}

void LoggedTopics::add_mission_topic(const char *name, uint16_t interval_ms)
{
	if (add_topic(name, interval_ms)) {
		++_num_mission_subs;
	}
}

bool LoggedTopics::add_topic(const orb_metadata *topic, uint16_t interval_ms, uint8_t instance)
{
	size_t fields_len = strlen(topic->o_fields) + strlen(topic->o_name) + 1; //1 for ':'

	if (fields_len > sizeof(ulog_message_format_s::format)) {
		PX4_WARN("skip topic %s, format string is too large: %zu (max is %zu)", topic->o_name, fields_len,
			 sizeof(ulog_message_format_s::format));

		return false;
	}

	if (_subscriptions.count >= MAX_TOPICS_NUM) {
		PX4_WARN("Too many subscriptions, failed to add: %s %d", topic->o_name, instance);
		return false;
	}

	RequestedSubscription &sub = _subscriptions.sub[_subscriptions.count++];
	sub.interval_ms = interval_ms;
	sub.instance = instance;
	sub.id = static_cast<ORB_ID>(topic->o_id);
	return true;
}

bool LoggedTopics::add_topic(const char *name, uint16_t interval_ms, uint8_t instance)
{
	const orb_metadata *const *topics = orb_get_topics();
	bool success = false;

	for (size_t i = 0; i < orb_topics_count(); i++) {
		if (strcmp(name, topics[i]->o_name) == 0) {
			bool already_added = false;

			// check if already added: if so, only update the interval
			for (int j = 0; j < _subscriptions.count; ++j) {
				if (_subscriptions.sub[j].id == static_cast<ORB_ID>(topics[i]->o_id) &&
				    _subscriptions.sub[j].instance == instance) {

					PX4_DEBUG("logging topic %s(%d), interval: %i, already added, only setting interval",
						  topics[i]->o_name, instance, interval_ms);

					_subscriptions.sub[j].interval_ms = interval_ms;
					success = true;
					already_added = true;
					break;
				}
			}

			if (!already_added) {
				success = add_topic(topics[i], interval_ms, instance);
				PX4_DEBUG("logging topic: %s(%d), interval: %i", topics[i]->o_name, instance, interval_ms);
				break;
			}
		}
	}

	return success;
}

bool LoggedTopics::add_topic_multi(const char *name, uint16_t interval_ms)
{
	// add all possible instances
	for (uint8_t instance = 0; instance < ORB_MULTI_MAX_INSTANCES; instance++) {
		add_topic(name, interval_ms, instance);
	}

	return true;
}


bool LoggedTopics::initialize_logged_topics(SDLogProfileMask profile)
{
	int ntopics = add_topics_from_file(PX4_STORAGEDIR "/etc/logging/logger_topics.txt");

	if (ntopics > 0) {
		PX4_INFO("logging %d topics from logger_topics.txt", ntopics);

	} else {
		initialize_configured_topics(profile);
	}

	return true;
}

void LoggedTopics::initialize_configured_topics(SDLogProfileMask profile)
{
	// load appropriate topics for profile
	// the order matters: if several profiles add the same topic, the logging rate of the last one will be used
	if (profile & SDLogProfileMask::DEFAULT) {
		add_default_topics();
	}

	if (profile & SDLogProfileMask::ESTIMATOR_REPLAY) {
		add_estimator_replay_topics();
	}

	if (profile & SDLogProfileMask::THERMAL_CALIBRATION) {
		add_thermal_calibration_topics();
	}

	if (profile & SDLogProfileMask::SYSTEM_IDENTIFICATION) {
		add_system_identification_topics();
	}

	if (profile & SDLogProfileMask::HIGH_RATE) {
		add_high_rate_topics();
	}

	if (profile & SDLogProfileMask::DEBUG_TOPICS) {
		add_debug_topics();
	}

	if (profile & SDLogProfileMask::SENSOR_COMPARISON) {
		add_sensor_comparison_topics();
	}

	if (profile & SDLogProfileMask::VISION_AND_AVOIDANCE) {
		add_vision_and_avoidance_topics();
	}
}
