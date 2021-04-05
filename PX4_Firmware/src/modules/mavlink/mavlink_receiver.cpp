/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file mavlink_receiver.cpp
 * MAVLink protocol message receive and dispatch
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Anton Babushkin <anton@px4.io>
 * @author Thomas Gubler <thomas@px4.io>
 */

#include <airspeed/airspeed.h>
#include <commander/px4_custom_mode.h>
#include <conversion/rotation.h>
#include <drivers/drv_rc_input.h>
#include <drivers/drv_tone_alarm.h>
#include <ecl/geo/geo.h>
#include <systemlib/px4_macros.h>

#include <math.h>
#include <poll.h>

#ifdef CONFIG_NET
#include <net/if.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#endif

#ifdef __PX4_DARWIN
#include <sys/param.h>
#include <sys/mount.h>
#else
#include <sys/statfs.h>
#endif

#ifndef __PX4_POSIX
#include <termios.h>
#endif

#include "mavlink_command_sender.h"
#include "mavlink_main.h"
#include "mavlink_receiver.h"

#ifdef CONFIG_NET
#define MAVLINK_RECEIVER_NET_ADDED_STACK 1360
#else
#define MAVLINK_RECEIVER_NET_ADDED_STACK 0
#endif

using matrix::wrap_2pi;

MavlinkReceiver::~MavlinkReceiver()
{
	delete _px4_accel;
	delete _px4_baro;
	delete _px4_gyro;
	delete _px4_mag;
}

MavlinkReceiver::MavlinkReceiver(Mavlink *parent) :
	ModuleParams(nullptr),
	_mavlink(parent),
	_mavlink_ftp(parent),
	_mavlink_log_handler(parent),
	_mission_manager(parent),
	_parameters_manager(parent),
	_mavlink_timesync(parent)
{
}

void
MavlinkReceiver::acknowledge(uint8_t sysid, uint8_t compid, uint16_t command, uint8_t result)
{
	vehicle_command_ack_s command_ack{};

	command_ack.timestamp = hrt_absolute_time();
	command_ack.command = command;
	command_ack.result = result;
	command_ack.target_system = sysid;
	command_ack.target_component = compid;

	_cmd_ack_pub.publish(command_ack);
}

void
MavlinkReceiver::handle_message(mavlink_message_t *msg)
{
	switch (msg->msgid) {
	case MAVLINK_MSG_ID_COMMAND_LONG:
		handle_message_command_long(msg);
		break;

	case MAVLINK_MSG_ID_COMMAND_INT:
		handle_message_command_int(msg);
		break;

	case MAVLINK_MSG_ID_COMMAND_ACK:
		handle_message_command_ack(msg);
		break;

	case MAVLINK_MSG_ID_OPTICAL_FLOW_RAD:
		handle_message_optical_flow_rad(msg);
		break;

	case MAVLINK_MSG_ID_PING:
		handle_message_ping(msg);
		break;

	case MAVLINK_MSG_ID_SET_MODE:
		handle_message_set_mode(msg);
		break;

	case MAVLINK_MSG_ID_ATT_POS_MOCAP:
		handle_message_att_pos_mocap(msg);
		break;

	case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
		handle_message_set_position_target_local_ned(msg);
		break;

	case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT:
		handle_message_set_position_target_global_int(msg);
		break;

	case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:
		handle_message_set_attitude_target(msg);
		break;

	case MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET:
		handle_message_set_actuator_control_target(msg);
		break;

	case MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE:
		handle_message_vision_position_estimate(msg);
		break;

	case MAVLINK_MSG_ID_ODOMETRY:
		handle_message_odometry(msg);
		break;

	case MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN:
		handle_message_gps_global_origin(msg);
		break;

	case MAVLINK_MSG_ID_RADIO_STATUS:
		handle_message_radio_status(msg);
		break;

	case MAVLINK_MSG_ID_MANUAL_CONTROL:
		handle_message_manual_control(msg);
		break;

	case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
		handle_message_rc_channels_override(msg);
		break;

	case MAVLINK_MSG_ID_HEARTBEAT:
		handle_message_heartbeat(msg);
		break;

	case MAVLINK_MSG_ID_DISTANCE_SENSOR:
		handle_message_distance_sensor(msg);
		break;

	case MAVLINK_MSG_ID_FOLLOW_TARGET:
		handle_message_follow_target(msg);
		break;

	case MAVLINK_MSG_ID_LANDING_TARGET:
		handle_message_landing_target(msg);
		break;

	case MAVLINK_MSG_ID_CELLULAR_STATUS:
		handle_message_cellular_status(msg);
		break;

	case MAVLINK_MSG_ID_ADSB_VEHICLE:
		handle_message_adsb_vehicle(msg);
		break;

	case MAVLINK_MSG_ID_UTM_GLOBAL_POSITION:
		handle_message_utm_global_position(msg);
		break;

	case MAVLINK_MSG_ID_COLLISION:
		handle_message_collision(msg);
		break;

	case MAVLINK_MSG_ID_GPS_RTCM_DATA:
		handle_message_gps_rtcm_data(msg);
		break;

	case MAVLINK_MSG_ID_BATTERY_STATUS:
		handle_message_battery_status(msg);
		break;

	case MAVLINK_MSG_ID_SERIAL_CONTROL:
		handle_message_serial_control(msg);
		break;

	case MAVLINK_MSG_ID_LOGGING_ACK:
		handle_message_logging_ack(msg);
		break;

	case MAVLINK_MSG_ID_PLAY_TUNE:
		handle_message_play_tune(msg);
		break;

	case MAVLINK_MSG_ID_OBSTACLE_DISTANCE:
		handle_message_obstacle_distance(msg);
		break;

	case MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_BEZIER:
		handle_message_trajectory_representation_bezier(msg);
		break;

	case MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS:
		handle_message_trajectory_representation_waypoints(msg);
		break;

	case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT:
		handle_message_named_value_float(msg);
		break;

	case MAVLINK_MSG_ID_DEBUG:
		handle_message_debug(msg);
		break;

	case MAVLINK_MSG_ID_DEBUG_VECT:
		handle_message_debug_vect(msg);
		break;

	case MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY:
		handle_message_debug_float_array(msg);
		break;

	case MAVLINK_MSG_ID_ONBOARD_COMPUTER_STATUS:
		handle_message_onboard_computer_status(msg);
		break;

	case MAVLINK_MSG_ID_STATUSTEXT:
		handle_message_statustext(msg);
		break;

	default:
		break;
	}

	/*
	 * Only decode hil messages in HIL mode.
	 *
	 * The HIL mode is enabled by the HIL bit flag
	 * in the system mode. Either send a set mode
	 * COMMAND_LONG message or a SET_MODE message
	 *
	 * Accept HIL GPS messages if use_hil_gps flag is true.
	 * This allows to provide fake gps measurements to the system.
	 */
	if (_mavlink->get_hil_enabled()) {
		switch (msg->msgid) {
		case MAVLINK_MSG_ID_HIL_SENSOR:
			handle_message_hil_sensor(msg);
			break;

		case MAVLINK_MSG_ID_HIL_STATE_QUATERNION:
			handle_message_hil_state_quaternion(msg);
			break;

		case MAVLINK_MSG_ID_HIL_OPTICAL_FLOW:
			handle_message_hil_optical_flow(msg);
			break;

		default:
			break;
		}
	}


	if (_mavlink->get_hil_enabled() || (_mavlink->get_use_hil_gps() && msg->sysid == mavlink_system.sysid)) {
		switch (msg->msgid) {
		case MAVLINK_MSG_ID_HIL_GPS:
			handle_message_hil_gps(msg);
			break;

		default:
			break;
		}

	}

	/* If we've received a valid message, mark the flag indicating so.
	   This is used in the '-w' command-line flag. */
	_mavlink->set_has_received_messages(true);
}

bool
MavlinkReceiver::evaluate_target_ok(int command, int target_system, int target_component)
{
	/* evaluate if this system should accept this command */
	bool target_ok = false;

	switch (command) {

	case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
	case MAV_CMD_REQUEST_PROTOCOL_VERSION:
		/* broadcast and ignore component */
		target_ok = (target_system == 0) || (target_system == mavlink_system.sysid);
		break;

	default:
		target_ok = (target_system == mavlink_system.sysid) && ((target_component == mavlink_system.compid)
				|| (target_component == MAV_COMP_ID_ALL));
		break;
	}

	return target_ok;
}

void
MavlinkReceiver::send_flight_information()
{
	mavlink_flight_information_t flight_info{};
	flight_info.flight_uuid = static_cast<uint64_t>(_param_com_flight_uuid.get());

	actuator_armed_s actuator_armed{};
	bool ret = _actuator_armed_sub.copy(&actuator_armed);

	if (ret && actuator_armed.timestamp != 0) {
		flight_info.arming_time_utc = flight_info.takeoff_time_utc = actuator_armed.armed_time_ms;
	}

	flight_info.time_boot_ms = hrt_absolute_time() / 1000;
	mavlink_msg_flight_information_send_struct(_mavlink->get_channel(), &flight_info);
}

void
MavlinkReceiver::send_storage_information(int storage_id)
{
	mavlink_storage_information_t storage_info{};
	const char *microsd_dir = PX4_STORAGEDIR;

	if (storage_id == 0 || storage_id == 1) { // request is for all or the first storage
		storage_info.storage_id = 1;

		struct statfs statfs_buf;
		uint64_t total_bytes = 0;
		uint64_t avail_bytes = 0;

		if (statfs(microsd_dir, &statfs_buf) == 0) {
			total_bytes = (uint64_t)statfs_buf.f_blocks * statfs_buf.f_bsize;
			avail_bytes = (uint64_t)statfs_buf.f_bavail * statfs_buf.f_bsize;
		}

		if (total_bytes == 0) { // on NuttX we get 0 total bytes if no SD card is inserted
			storage_info.storage_count = 0;
			storage_info.status = 0; // not available

		} else {
			storage_info.storage_count = 1;
			storage_info.status = 2; // available & formatted
			storage_info.total_capacity = total_bytes / 1024. / 1024.;
			storage_info.available_capacity = avail_bytes / 1024. / 1024.;
			storage_info.used_capacity = (total_bytes - avail_bytes) / 1024. / 1024.;
		}

	} else {
		// only one storage supported
		storage_info.storage_id = storage_id;
		storage_info.storage_count = 1;
	}

	storage_info.time_boot_ms = hrt_absolute_time() / 1000;
	mavlink_msg_storage_information_send_struct(_mavlink->get_channel(), &storage_info);
}

void
MavlinkReceiver::handle_message_command_long(mavlink_message_t *msg)
{
	/* command */
	mavlink_command_long_t cmd_mavlink;
	mavlink_msg_command_long_decode(msg, &cmd_mavlink);

	vehicle_command_s vcmd{};

	vcmd.timestamp = hrt_absolute_time();

	/* Copy the content of mavlink_command_long_t cmd_mavlink into command_t cmd */
	vcmd.param1 = cmd_mavlink.param1;
	vcmd.param2 = cmd_mavlink.param2;
	vcmd.param3 = cmd_mavlink.param3;
	vcmd.param4 = cmd_mavlink.param4;
	vcmd.param5 = (double)cmd_mavlink.param5;
	vcmd.param6 = (double)cmd_mavlink.param6;
	vcmd.param7 = cmd_mavlink.param7;
	vcmd.command = cmd_mavlink.command;
	vcmd.target_system = cmd_mavlink.target_system;
	vcmd.target_component = cmd_mavlink.target_component;
	vcmd.source_system = msg->sysid;
	vcmd.source_component = msg->compid;
	vcmd.confirmation = cmd_mavlink.confirmation;
	vcmd.from_external = true;

	handle_message_command_both(msg, cmd_mavlink, vcmd);
}

void
MavlinkReceiver::handle_message_command_int(mavlink_message_t *msg)
{
	/* command */
	mavlink_command_int_t cmd_mavlink;
	mavlink_msg_command_int_decode(msg, &cmd_mavlink);

	vehicle_command_s vcmd{};
	vcmd.timestamp = hrt_absolute_time();

	/* Copy the content of mavlink_command_int_t cmd_mavlink into command_t cmd */
	vcmd.param1 = cmd_mavlink.param1;
	vcmd.param2 = cmd_mavlink.param2;
	vcmd.param3 = cmd_mavlink.param3;
	vcmd.param4 = cmd_mavlink.param4;
	vcmd.param5 = ((double)cmd_mavlink.x) / 1e7;
	vcmd.param6 = ((double)cmd_mavlink.y) / 1e7;
	vcmd.param7 = cmd_mavlink.z;
	vcmd.command = cmd_mavlink.command;
	vcmd.target_system = cmd_mavlink.target_system;
	vcmd.target_component = cmd_mavlink.target_component;
	vcmd.source_system = msg->sysid;
	vcmd.source_component = msg->compid;
	vcmd.confirmation = false;
	vcmd.from_external = true;

	handle_message_command_both(msg, cmd_mavlink, vcmd);
}

template <class T>
void MavlinkReceiver::handle_message_command_both(mavlink_message_t *msg, const T &cmd_mavlink,
		const vehicle_command_s &vehicle_command)
{
	bool target_ok = evaluate_target_ok(cmd_mavlink.command, cmd_mavlink.target_system, cmd_mavlink.target_component);
	bool send_ack = true;
	uint8_t result = vehicle_command_ack_s::VEHICLE_RESULT_ACCEPTED;

	if (!target_ok) {
		acknowledge(msg->sysid, msg->compid, cmd_mavlink.command, vehicle_command_ack_s::VEHICLE_RESULT_FAILED);
		return;
	}

	if (cmd_mavlink.command == MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES) {
		/* send autopilot version message */
		_mavlink->send_autopilot_capabilites();

	} else if (cmd_mavlink.command == MAV_CMD_REQUEST_PROTOCOL_VERSION) {
		/* send protocol version message */
		_mavlink->send_protocol_version();

	} else if (cmd_mavlink.command == MAV_CMD_GET_HOME_POSITION) {
		_mavlink->configure_stream_threadsafe("HOME_POSITION", 0.5f);

	} else if (cmd_mavlink.command == MAV_CMD_SET_MESSAGE_INTERVAL) {
		if (set_message_interval((int)roundf(cmd_mavlink.param1), cmd_mavlink.param2, cmd_mavlink.param3)) {
			result = vehicle_command_ack_s::VEHICLE_RESULT_FAILED;
		}

	} else if (cmd_mavlink.command == MAV_CMD_GET_MESSAGE_INTERVAL) {
		get_message_interval((int)roundf(cmd_mavlink.param1));

	} else if (cmd_mavlink.command == MAV_CMD_REQUEST_FLIGHT_INFORMATION) {
		send_flight_information();

	} else if (cmd_mavlink.command == MAV_CMD_REQUEST_STORAGE_INFORMATION) {
		if ((int)roundf(cmd_mavlink.param1) == 1) {
			send_storage_information((int)roundf(cmd_mavlink.param1));
		}

	} else if (cmd_mavlink.command == MAV_CMD_REQUEST_MESSAGE) {
		uint16_t message_id = (uint16_t)roundf(cmd_mavlink.param1);
		bool stream_found = false;

		for (const auto &stream : _mavlink->get_streams()) {
			if (stream->get_id() == message_id) {
				stream->request_message(vehicle_command.param1, vehicle_command.param2, vehicle_command.param3,
							vehicle_command.param4, vehicle_command.param5, vehicle_command.param6, vehicle_command.param7);
				stream_found = true;
				break;
			}
		}

		// TODO: Handle the case where a message is requested which could be sent, but for which there is no stream.
		if (!stream_found) {
			result = vehicle_command_ack_s::VEHICLE_RESULT_DENIED;
		}

	} else if (cmd_mavlink.command == MAV_CMD_SET_CAMERA_ZOOM) {
		struct actuator_controls_s actuator_controls = {};
		actuator_controls.timestamp = hrt_absolute_time();

		for (size_t i = 0; i < 8; i++) {
			actuator_controls.control[i] = NAN;
		}

		switch ((int)(cmd_mavlink.param1 + 0.5f)) {
		case vehicle_command_s::VEHICLE_CAMERA_ZOOM_TYPE_RANGE:
			actuator_controls.control[actuator_controls_s::INDEX_CAMERA_ZOOM] = cmd_mavlink.param2 / 50.0f - 1.0f;
			break;

		case vehicle_command_s::VEHICLE_CAMERA_ZOOM_TYPE_STEP:
		case vehicle_command_s::VEHICLE_CAMERA_ZOOM_TYPE_CONTINUOUS:
		case vehicle_command_s::VEHICLE_CAMERA_ZOOM_TYPE_FOCAL_LENGTH:
		default:
			send_ack = false;
		}

		_actuator_controls_pubs[actuator_controls_s::GROUP_INDEX_GIMBAL].publish(actuator_controls);

	} else {

		send_ack = false;

		if (msg->sysid == mavlink_system.sysid && msg->compid == mavlink_system.compid) {
			PX4_WARN("ignoring CMD with same SYS/COMP (%d/%d) ID", mavlink_system.sysid, mavlink_system.compid);
			return;
		}

		if (cmd_mavlink.command == MAV_CMD_LOGGING_START) {
			// check that we have enough bandwidth available: this is given by the configured logger topics
			// and rates. The 5000 is somewhat arbitrary, but makes sure that we cannot enable log streaming
			// on a radio link
			if (_mavlink->get_data_rate() < 5000) {
				send_ack = true;
				result = vehicle_command_ack_s::VEHICLE_RESULT_DENIED;
				_mavlink->send_statustext_critical("Not enough bandwidth to enable log streaming");

			} else {
				// we already instanciate the streaming object, because at this point we know on which
				// mavlink channel streaming was requested. But in fact it's possible that the logger is
				// not even running. The main mavlink thread takes care of this by waiting for an ack
				// from the logger.
				_mavlink->try_start_ulog_streaming(msg->sysid, msg->compid);
			}

		} else if (cmd_mavlink.command == MAV_CMD_LOGGING_STOP) {
			_mavlink->request_stop_ulog_streaming();
		}

		if (!send_ack) {
			_cmd_pub.publish(vehicle_command);
		}
	}

	if (send_ack) {
		acknowledge(msg->sysid, msg->compid, cmd_mavlink.command, result);
	}
}

void
MavlinkReceiver::handle_message_command_ack(mavlink_message_t *msg)
{
	mavlink_command_ack_t ack;
	mavlink_msg_command_ack_decode(msg, &ack);

	MavlinkCommandSender::instance().handle_mavlink_command_ack(ack, msg->sysid, msg->compid, _mavlink->get_channel());

	vehicle_command_ack_s command_ack{};

	command_ack.timestamp = hrt_absolute_time();
	command_ack.result_param2 = ack.result_param2;
	command_ack.command = ack.command;
	command_ack.result = ack.result;
	command_ack.from_external = true;
	command_ack.result_param1 = ack.progress;
	command_ack.target_system = ack.target_system;
	command_ack.target_component = ack.target_component;

	_cmd_ack_pub.publish(command_ack);

	// TODO: move it to the same place that sent the command
	if (ack.result != MAV_RESULT_ACCEPTED && ack.result != MAV_RESULT_IN_PROGRESS) {
		if (msg->compid == MAV_COMP_ID_CAMERA) {
			PX4_WARN("Got unsuccessful result %d from camera", ack.result);
		}
	}
}

void
MavlinkReceiver::handle_message_optical_flow_rad(mavlink_message_t *msg)
{
	/* optical flow */
	mavlink_optical_flow_rad_t flow;
	mavlink_msg_optical_flow_rad_decode(msg, &flow);

	optical_flow_s f{};

	f.timestamp = hrt_absolute_time();
	f.time_since_last_sonar_update = flow.time_delta_distance_us;
	f.integration_timespan  = flow.integration_time_us;
	f.pixel_flow_x_integral = flow.integrated_x;
	f.pixel_flow_y_integral = flow.integrated_y;
	f.gyro_x_rate_integral  = flow.integrated_xgyro;
	f.gyro_y_rate_integral  = flow.integrated_ygyro;
	f.gyro_z_rate_integral  = flow.integrated_zgyro;
	f.gyro_temperature      = flow.temperature;
	f.ground_distance_m     = flow.distance;
	f.quality               = flow.quality;
	f.sensor_id             = flow.sensor_id;
	f.max_flow_rate         = _param_sens_flow_maxr.get();
	f.min_ground_distance   = _param_sens_flow_minhgt.get();
	f.max_ground_distance   = _param_sens_flow_maxhgt.get();

	/* read flow sensor parameters */
	const Rotation flow_rot = (Rotation)_param_sens_flow_rot.get();

	/* rotate measurements according to parameter */
	float zero_val = 0.0f;
	rotate_3f(flow_rot, f.pixel_flow_x_integral, f.pixel_flow_y_integral, zero_val);
	rotate_3f(flow_rot, f.gyro_x_rate_integral, f.gyro_y_rate_integral, f.gyro_z_rate_integral);

	_flow_pub.publish(f);

	/* Use distance value for distance sensor topic */
	if (flow.distance > 0.0f) { // negative values signal invalid data

		distance_sensor_s d{};

		d.timestamp = f.timestamp;
		d.min_distance = 0.3f;
		d.max_distance = 5.0f;
		d.current_distance = flow.distance; /* both are in m */
		d.type = 1;
		d.id = distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND;
		d.orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
		d.variance = 0.0;

		_flow_distance_sensor_pub.publish(d);
	}
}

void
MavlinkReceiver::handle_message_hil_optical_flow(mavlink_message_t *msg)
{
	/* optical flow */
	mavlink_hil_optical_flow_t flow;
	mavlink_msg_hil_optical_flow_decode(msg, &flow);

	optical_flow_s f{};

	f.timestamp = hrt_absolute_time(); // XXX we rely on the system time for now and not flow.time_usec;
	f.integration_timespan = flow.integration_time_us;
	f.pixel_flow_x_integral = flow.integrated_x;
	f.pixel_flow_y_integral = flow.integrated_y;
	f.gyro_x_rate_integral = flow.integrated_xgyro;
	f.gyro_y_rate_integral = flow.integrated_ygyro;
	f.gyro_z_rate_integral = flow.integrated_zgyro;
	f.time_since_last_sonar_update = flow.time_delta_distance_us;
	f.ground_distance_m = flow.distance;
	f.quality = flow.quality;
	f.sensor_id = flow.sensor_id;
	f.gyro_temperature = flow.temperature;

	_flow_pub.publish(f);

	/* Use distance value for distance sensor topic */
	distance_sensor_s d{};

	d.timestamp = hrt_absolute_time();
	d.min_distance = 0.3f;
	d.max_distance = 5.0f;
	d.current_distance = flow.distance; /* both are in m */
	d.type = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
	d.id = 0;
	d.orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
	d.variance = 0.0;

	_flow_distance_sensor_pub.publish(d);
}

void
MavlinkReceiver::handle_message_set_mode(mavlink_message_t *msg)
{
	mavlink_set_mode_t new_mode;
	mavlink_msg_set_mode_decode(msg, &new_mode);

	union px4_custom_mode custom_mode;
	custom_mode.data = new_mode.custom_mode;

	vehicle_command_s vcmd{};

	vcmd.timestamp = hrt_absolute_time();

	/* copy the content of mavlink_command_long_t cmd_mavlink into command_t cmd */
	vcmd.param1 = (float)new_mode.base_mode;
	vcmd.param2 = (float)custom_mode.main_mode;
	vcmd.param3 = (float)custom_mode.sub_mode;

	vcmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	vcmd.target_system = new_mode.target_system;
	vcmd.target_component = MAV_COMP_ID_ALL;
	vcmd.source_system = msg->sysid;
	vcmd.source_component = msg->compid;
	vcmd.confirmation = true;
	vcmd.from_external = true;

	_cmd_pub.publish(vcmd);
}

void
MavlinkReceiver::handle_message_distance_sensor(mavlink_message_t *msg)
{
	mavlink_distance_sensor_t dist_sensor;
	mavlink_msg_distance_sensor_decode(msg, &dist_sensor);

	distance_sensor_s ds{};

	ds.timestamp        = hrt_absolute_time(); /* Use system time for now, don't trust sender to attach correct timestamp */
	ds.min_distance     = static_cast<float>(dist_sensor.min_distance) * 1e-2f;     /* cm to m */
	ds.max_distance     = static_cast<float>(dist_sensor.max_distance) * 1e-2f;     /* cm to m */
	ds.current_distance = static_cast<float>(dist_sensor.current_distance) * 1e-2f; /* cm to m */
	ds.variance         = dist_sensor.covariance * 1e-4f;                           /* cm^2 to m^2 */
	ds.h_fov            = dist_sensor.horizontal_fov;
	ds.v_fov            = dist_sensor.vertical_fov;
	ds.q[0]             = dist_sensor.quaternion[0];
	ds.q[1]             = dist_sensor.quaternion[1];
	ds.q[2]             = dist_sensor.quaternion[2];
	ds.q[3]             = dist_sensor.quaternion[3];
	ds.signal_quality   = -1; // TODO: A dist_sensor.signal_quality field is missing from the mavlink message definition.
	ds.type             = dist_sensor.type;
	ds.id               = dist_sensor.id;
	ds.orientation      = dist_sensor.orientation;

	_distance_sensor_pub.publish(ds);
}

void
MavlinkReceiver::handle_message_att_pos_mocap(mavlink_message_t *msg)
{
	mavlink_att_pos_mocap_t mocap;
	mavlink_msg_att_pos_mocap_decode(msg, &mocap);

	vehicle_odometry_s mocap_odom{};

	mocap_odom.timestamp = _mavlink_timesync.sync_stamp(mocap.time_usec);
	mocap_odom.x = mocap.x;
	mocap_odom.y = mocap.y;
	mocap_odom.z = mocap.z;
	mocap_odom.q[0] = mocap.q[0];
	mocap_odom.q[1] = mocap.q[1];
	mocap_odom.q[2] = mocap.q[2];
	mocap_odom.q[3] = mocap.q[3];

	const size_t URT_SIZE = sizeof(mocap_odom.pose_covariance) / sizeof(mocap_odom.pose_covariance[0]);
	static_assert(URT_SIZE == (sizeof(mocap.covariance) / sizeof(mocap.covariance[0])),
		      "Odometry Pose Covariance matrix URT array size mismatch");

	for (size_t i = 0; i < URT_SIZE; i++) {
		mocap_odom.pose_covariance[i] = mocap.covariance[i];
	}

	mocap_odom.vx = NAN;
	mocap_odom.vy = NAN;
	mocap_odom.vz = NAN;
	mocap_odom.rollspeed = NAN;
	mocap_odom.pitchspeed = NAN;
	mocap_odom.yawspeed = NAN;
	mocap_odom.velocity_covariance[0] = NAN;

	_mocap_odometry_pub.publish(mocap_odom);
}

void
MavlinkReceiver::handle_message_set_position_target_local_ned(mavlink_message_t *msg)
{
	mavlink_set_position_target_local_ned_t set_position_target_local_ned;
	mavlink_msg_set_position_target_local_ned_decode(msg, &set_position_target_local_ned);

	const bool values_finite =
		PX4_ISFINITE(set_position_target_local_ned.x) &&
		PX4_ISFINITE(set_position_target_local_ned.y) &&
		PX4_ISFINITE(set_position_target_local_ned.z) &&
		PX4_ISFINITE(set_position_target_local_ned.vx) &&
		PX4_ISFINITE(set_position_target_local_ned.vy) &&
		PX4_ISFINITE(set_position_target_local_ned.vz) &&
		PX4_ISFINITE(set_position_target_local_ned.afx) &&
		PX4_ISFINITE(set_position_target_local_ned.afy) &&
		PX4_ISFINITE(set_position_target_local_ned.afz) &&
		PX4_ISFINITE(set_position_target_local_ned.yaw);

	/* Only accept messages which are intended for this system */
	if ((mavlink_system.sysid == set_position_target_local_ned.target_system ||
	     set_position_target_local_ned.target_system == 0) &&
	    (mavlink_system.compid == set_position_target_local_ned.target_component ||
	     set_position_target_local_ned.target_component == 0) &&
	    values_finite) {

		offboard_control_mode_s offboard_control_mode{};

		/* convert mavlink type (local, NED) to uORB offboard control struct */
		offboard_control_mode.ignore_position = (bool)(set_position_target_local_ned.type_mask & 0x7);
		offboard_control_mode.ignore_alt_hold = (bool)(set_position_target_local_ned.type_mask & 0x4);
		offboard_control_mode.ignore_velocity = (bool)(set_position_target_local_ned.type_mask & 0x38);
		offboard_control_mode.ignore_acceleration_force = (bool)(set_position_target_local_ned.type_mask & 0x1C0);
		offboard_control_mode.ignore_attitude = (bool)(set_position_target_local_ned.type_mask & 0x400);
		offboard_control_mode.ignore_bodyrate_x = (bool)(set_position_target_local_ned.type_mask & 0x800);
		offboard_control_mode.ignore_bodyrate_y = (bool)(set_position_target_local_ned.type_mask & 0x800);
		offboard_control_mode.ignore_bodyrate_z = (bool)(set_position_target_local_ned.type_mask & 0x800);

		/* yaw ignore flag mapps to ignore_attitude */
		bool is_force_sp = (bool)(set_position_target_local_ned.type_mask & (1 << 9));

		bool is_takeoff_sp = (bool)(set_position_target_local_ned.type_mask & 0x1000);
		bool is_land_sp = (bool)(set_position_target_local_ned.type_mask & 0x2000);
		bool is_loiter_sp = (bool)(set_position_target_local_ned.type_mask & 0x3000);
		bool is_idle_sp = (bool)(set_position_target_local_ned.type_mask & 0x4000);

		offboard_control_mode.timestamp = hrt_absolute_time();
		_offboard_control_mode_pub.publish(offboard_control_mode);

		/* If we are in offboard control mode and offboard control loop through is enabled
		 * also publish the setpoint topic which is read by the controller */
		if (_mavlink->get_forward_externalsp()) {

			vehicle_control_mode_s control_mode{};
			_control_mode_sub.copy(&control_mode);

			if (control_mode.flag_control_offboard_enabled) {
				if (is_force_sp && offboard_control_mode.ignore_position &&
				    offboard_control_mode.ignore_velocity) {

					PX4_WARN("force setpoint not supported");

				} else {
					/* It's not a pure force setpoint: publish to setpoint triplet  topic */
					position_setpoint_triplet_s pos_sp_triplet{};

					pos_sp_triplet.timestamp = hrt_absolute_time();
					pos_sp_triplet.previous.valid = false;
					pos_sp_triplet.next.valid = false;
					pos_sp_triplet.current.valid = true;

					/* Order of statements matters. Takeoff can override loiter.
					 * See https://github.com/mavlink/mavlink/pull/670 for a broader conversation. */
					if (is_loiter_sp) {
						pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_LOITER;

					} else if (is_takeoff_sp) {
						pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_TAKEOFF;

					} else if (is_land_sp) {
						pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_LAND;

					} else if (is_idle_sp) {
						pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_IDLE;

					} else {
						pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
					}

					/* set the local pos values */
					if (!offboard_control_mode.ignore_position) {

						pos_sp_triplet.current.position_valid = true;
						pos_sp_triplet.current.x = set_position_target_local_ned.x;
						pos_sp_triplet.current.y = set_position_target_local_ned.y;
						pos_sp_triplet.current.z = set_position_target_local_ned.z;

					} else {
						pos_sp_triplet.current.position_valid = false;
					}

					/* set the local vel values */
					if (!offboard_control_mode.ignore_velocity) {

						pos_sp_triplet.current.velocity_valid = true;
						pos_sp_triplet.current.vx = set_position_target_local_ned.vx;
						pos_sp_triplet.current.vy = set_position_target_local_ned.vy;
						pos_sp_triplet.current.vz = set_position_target_local_ned.vz;

						pos_sp_triplet.current.velocity_frame = set_position_target_local_ned.coordinate_frame;

					} else {
						pos_sp_triplet.current.velocity_valid = false;
					}

					if (!offboard_control_mode.ignore_alt_hold) {
						pos_sp_triplet.current.alt_valid = true;
						pos_sp_triplet.current.z = set_position_target_local_ned.z;

					} else {
						pos_sp_triplet.current.alt_valid = false;
					}

					/* set the local acceleration values if the setpoint type is 'local pos' and none
					 * of the accelerations fields is set to 'ignore' */
					if (!offboard_control_mode.ignore_acceleration_force) {

						pos_sp_triplet.current.acceleration_valid = true;
						pos_sp_triplet.current.a_x = set_position_target_local_ned.afx;
						pos_sp_triplet.current.a_y = set_position_target_local_ned.afy;
						pos_sp_triplet.current.a_z = set_position_target_local_ned.afz;
						pos_sp_triplet.current.acceleration_is_force = is_force_sp;

					} else {
						pos_sp_triplet.current.acceleration_valid = false;
					}

					/* set the yaw sp value */
					if (!offboard_control_mode.ignore_attitude) {
						pos_sp_triplet.current.yaw_valid = true;
						pos_sp_triplet.current.yaw = set_position_target_local_ned.yaw;

					} else {
						pos_sp_triplet.current.yaw_valid = false;
					}

					/* set the yawrate sp value */
					if (!(offboard_control_mode.ignore_bodyrate_x ||
					      offboard_control_mode.ignore_bodyrate_y ||
					      offboard_control_mode.ignore_bodyrate_z)) {

						pos_sp_triplet.current.yawspeed_valid = true;
						pos_sp_triplet.current.yawspeed = set_position_target_local_ned.yaw_rate;

					} else {
						pos_sp_triplet.current.yawspeed_valid = false;
					}

					//XXX handle global pos setpoints (different MAV frames)
					_pos_sp_triplet_pub.publish(pos_sp_triplet);
				}
			}
		}
	}
}

void
MavlinkReceiver::handle_message_set_position_target_global_int(mavlink_message_t *msg)
{
	mavlink_set_position_target_global_int_t set_position_target_global_int;
	mavlink_msg_set_position_target_global_int_decode(msg, &set_position_target_global_int);

	const bool values_finite =
		PX4_ISFINITE(set_position_target_global_int.alt) &&
		PX4_ISFINITE(set_position_target_global_int.vx) &&
		PX4_ISFINITE(set_position_target_global_int.vy) &&
		PX4_ISFINITE(set_position_target_global_int.vz) &&
		PX4_ISFINITE(set_position_target_global_int.afx) &&
		PX4_ISFINITE(set_position_target_global_int.afy) &&
		PX4_ISFINITE(set_position_target_global_int.afz) &&
		PX4_ISFINITE(set_position_target_global_int.yaw);

	/* Only accept messages which are intended for this system */
	if ((mavlink_system.sysid == set_position_target_global_int.target_system ||
	     set_position_target_global_int.target_system == 0) &&
	    (mavlink_system.compid == set_position_target_global_int.target_component ||
	     set_position_target_global_int.target_component == 0) &&
	    values_finite) {

		offboard_control_mode_s offboard_control_mode{};

		/* convert mavlink type (local, NED) to uORB offboard control struct */
		offboard_control_mode.ignore_position = (bool)(set_position_target_global_int.type_mask &
							(POSITION_TARGET_TYPEMASK::POSITION_TARGET_TYPEMASK_X_IGNORE
									| POSITION_TARGET_TYPEMASK::POSITION_TARGET_TYPEMASK_Y_IGNORE
									| POSITION_TARGET_TYPEMASK::POSITION_TARGET_TYPEMASK_Z_IGNORE));
		offboard_control_mode.ignore_alt_hold = (bool)(set_position_target_global_int.type_mask & 0x4);
		offboard_control_mode.ignore_velocity = (bool)(set_position_target_global_int.type_mask &
							(POSITION_TARGET_TYPEMASK::POSITION_TARGET_TYPEMASK_VX_IGNORE
									| POSITION_TARGET_TYPEMASK::POSITION_TARGET_TYPEMASK_VY_IGNORE
									| POSITION_TARGET_TYPEMASK::POSITION_TARGET_TYPEMASK_VZ_IGNORE));
		offboard_control_mode.ignore_acceleration_force = (bool)(set_position_target_global_int.type_mask &
				(POSITION_TARGET_TYPEMASK::POSITION_TARGET_TYPEMASK_AX_IGNORE
				 | POSITION_TARGET_TYPEMASK::POSITION_TARGET_TYPEMASK_AY_IGNORE
				 | POSITION_TARGET_TYPEMASK::POSITION_TARGET_TYPEMASK_AZ_IGNORE));
		/* yaw ignore flag mapps to ignore_attitude */
		offboard_control_mode.ignore_attitude = (bool)(set_position_target_global_int.type_mask &
							POSITION_TARGET_TYPEMASK::POSITION_TARGET_TYPEMASK_YAW_IGNORE);
		offboard_control_mode.ignore_bodyrate_x = (bool)(set_position_target_global_int.type_mask &
				POSITION_TARGET_TYPEMASK::POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE);
		offboard_control_mode.ignore_bodyrate_y = (bool)(set_position_target_global_int.type_mask &
				POSITION_TARGET_TYPEMASK::POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE);
		offboard_control_mode.ignore_bodyrate_z = (bool)(set_position_target_global_int.type_mask &
				POSITION_TARGET_TYPEMASK::POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE);

		bool is_force_sp = (bool)(set_position_target_global_int.type_mask & (1 << 9));

		offboard_control_mode.timestamp = hrt_absolute_time();
		_offboard_control_mode_pub.publish(offboard_control_mode);

		/* If we are in offboard control mode and offboard control loop through is enabled
		 * also publish the setpoint topic which is read by the controller */
		if (_mavlink->get_forward_externalsp()) {

			vehicle_control_mode_s control_mode{};
			_control_mode_sub.copy(&control_mode);

			if (control_mode.flag_control_offboard_enabled) {
				if (is_force_sp && offboard_control_mode.ignore_position &&
				    offboard_control_mode.ignore_velocity) {

					PX4_WARN("force setpoint not supported");

				} else {
					/* It's not a pure force setpoint: publish to setpoint triplet  topic */
					position_setpoint_triplet_s pos_sp_triplet{};

					pos_sp_triplet.timestamp = hrt_absolute_time();
					pos_sp_triplet.previous.valid = false;
					pos_sp_triplet.next.valid = false;
					pos_sp_triplet.current.valid = true;

					/* Order of statements matters. Takeoff can override loiter.
					 * See https://github.com/mavlink/mavlink/pull/670 for a broader conversation. */
					if (set_position_target_global_int.type_mask & 0x3000) { //Loiter setpoint
						pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_LOITER;

					} else if (set_position_target_global_int.type_mask & 0x1000) { //Takeoff setpoint
						pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_TAKEOFF;

					} else if (set_position_target_global_int.type_mask & 0x2000) { //Land setpoint
						pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_LAND;

					} else if (set_position_target_global_int.type_mask & 0x4000) { //Idle setpoint
						pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_IDLE;

					} else {
						pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
					}

					/* set the local pos values */
					vehicle_local_position_s local_pos{};

					if (!offboard_control_mode.ignore_position && _vehicle_local_position_sub.copy(&local_pos)) {
						if (!globallocalconverter_initialized()) {
							globallocalconverter_init(local_pos.ref_lat, local_pos.ref_lon,
										  local_pos.ref_alt, local_pos.ref_timestamp);
							pos_sp_triplet.current.position_valid = false;

						} else {
							globallocalconverter_tolocal(set_position_target_global_int.lat_int / 1e7,
										     set_position_target_global_int.lon_int / 1e7, set_position_target_global_int.alt,
										     &pos_sp_triplet.current.x, &pos_sp_triplet.current.y, &pos_sp_triplet.current.z);
							pos_sp_triplet.current.position_valid = true;
						}

					} else {
						pos_sp_triplet.current.position_valid = false;
					}

					/* set the local velocity values */
					if (!offboard_control_mode.ignore_velocity) {

						pos_sp_triplet.current.velocity_valid = true;
						pos_sp_triplet.current.vx = set_position_target_global_int.vx;
						pos_sp_triplet.current.vy = set_position_target_global_int.vy;
						pos_sp_triplet.current.vz = set_position_target_global_int.vz;

						pos_sp_triplet.current.velocity_frame = set_position_target_global_int.coordinate_frame;

					} else {
						pos_sp_triplet.current.velocity_valid = false;
					}

					if (!offboard_control_mode.ignore_alt_hold) {
						pos_sp_triplet.current.alt_valid = true;

					} else {
						pos_sp_triplet.current.alt_valid = false;
					}

					/* set the local acceleration values if the setpoint type is 'local pos' and none
					 * of the accelerations fields is set to 'ignore' */
					if (!offboard_control_mode.ignore_acceleration_force) {

						pos_sp_triplet.current.acceleration_valid = true;
						pos_sp_triplet.current.a_x = set_position_target_global_int.afx;
						pos_sp_triplet.current.a_y = set_position_target_global_int.afy;
						pos_sp_triplet.current.a_z = set_position_target_global_int.afz;
						pos_sp_triplet.current.acceleration_is_force = is_force_sp;

					} else {
						pos_sp_triplet.current.acceleration_valid = false;
					}

					/* set the yaw setpoint */
					if (!offboard_control_mode.ignore_attitude) {
						pos_sp_triplet.current.yaw_valid = true;
						pos_sp_triplet.current.yaw = set_position_target_global_int.yaw;

					} else {
						pos_sp_triplet.current.yaw_valid = false;
					}

					/* set the yawrate sp value */
					if (!(offboard_control_mode.ignore_bodyrate_x ||
					      offboard_control_mode.ignore_bodyrate_y ||
					      offboard_control_mode.ignore_bodyrate_z)) {

						pos_sp_triplet.current.yawspeed_valid = true;
						pos_sp_triplet.current.yawspeed = set_position_target_global_int.yaw_rate;

					} else {
						pos_sp_triplet.current.yawspeed_valid = false;
					}

					_pos_sp_triplet_pub.publish(pos_sp_triplet);
				}
			}
		}
	}
}

void
MavlinkReceiver::handle_message_set_actuator_control_target(mavlink_message_t *msg)
{
	mavlink_set_actuator_control_target_t set_actuator_control_target;
	mavlink_msg_set_actuator_control_target_decode(msg, &set_actuator_control_target);

	bool values_finite =
		PX4_ISFINITE(set_actuator_control_target.controls[0]) &&
		PX4_ISFINITE(set_actuator_control_target.controls[1]) &&
		PX4_ISFINITE(set_actuator_control_target.controls[2]) &&
		PX4_ISFINITE(set_actuator_control_target.controls[3]) &&
		PX4_ISFINITE(set_actuator_control_target.controls[4]) &&
		PX4_ISFINITE(set_actuator_control_target.controls[5]) &&
		PX4_ISFINITE(set_actuator_control_target.controls[6]) &&
		PX4_ISFINITE(set_actuator_control_target.controls[7]);

	if ((mavlink_system.sysid == set_actuator_control_target.target_system ||
	     set_actuator_control_target.target_system == 0) &&
	    (mavlink_system.compid == set_actuator_control_target.target_component ||
	     set_actuator_control_target.target_component == 0) &&
	    values_finite) {

#if defined(ENABLE_LOCKSTEP_SCHEDULER)
		PX4_ERR("SET_ACTUATOR_CONTROL_TARGET not supported with lockstep enabled");
		PX4_ERR("Please disable lockstep for actuator offboard control:");
		PX4_ERR("https://dev.px4.io/master/en/simulation/#disable-lockstep-simulation");
		return;
#endif
		/* Ignore all setpoints except when controlling the gimbal(group_mlx==2) as we are setting raw actuators here */
		bool ignore_setpoints = bool(set_actuator_control_target.group_mlx != 2);

		offboard_control_mode_s offboard_control_mode{};

		offboard_control_mode.ignore_thrust             = ignore_setpoints;
		offboard_control_mode.ignore_attitude           = ignore_setpoints;
		offboard_control_mode.ignore_bodyrate_x         = ignore_setpoints;
		offboard_control_mode.ignore_bodyrate_y         = ignore_setpoints;
		offboard_control_mode.ignore_bodyrate_z         = ignore_setpoints;
		offboard_control_mode.ignore_position           = ignore_setpoints;
		offboard_control_mode.ignore_velocity           = ignore_setpoints;
		offboard_control_mode.ignore_acceleration_force = ignore_setpoints;

		offboard_control_mode.timestamp = hrt_absolute_time();

		_offboard_control_mode_pub.publish(offboard_control_mode);

		/* If we are in offboard control mode, publish the actuator controls */
		vehicle_control_mode_s control_mode{};
		_control_mode_sub.copy(&control_mode);

		if (control_mode.flag_control_offboard_enabled) {

			actuator_controls_s actuator_controls{};
			actuator_controls.timestamp = hrt_absolute_time();

			/* Set duty cycles for the servos in the actuator_controls message */
			for (size_t i = 0; i < 8; i++) {
				actuator_controls.control[i] = set_actuator_control_target.controls[i];
			}

			switch (set_actuator_control_target.group_mlx) {
			case 0:
				_actuator_controls_pubs[0].publish(actuator_controls);
				break;

			case 1:
				_actuator_controls_pubs[1].publish(actuator_controls);
				break;

			case 2:
				_actuator_controls_pubs[2].publish(actuator_controls);
				break;

			case 3:
				_actuator_controls_pubs[3].publish(actuator_controls);
				break;

			default:
				break;
			}
		}
	}

}

void
MavlinkReceiver::handle_message_gps_global_origin(mavlink_message_t *msg)
{
	mavlink_gps_global_origin_t origin;
	mavlink_msg_gps_global_origin_decode(msg, &origin);

	if (!globallocalconverter_initialized()) {
		/* Set reference point conversion of local coordiantes <--> global coordinates */
		globallocalconverter_init((double)origin.latitude * 1.0e-7, (double)origin.longitude * 1.0e-7,
					  (float)origin.altitude * 1.0e-3f, hrt_absolute_time());
		_global_ref_timestamp = hrt_absolute_time();

	}
}

void
MavlinkReceiver::handle_message_vision_position_estimate(mavlink_message_t *msg)
{
	mavlink_vision_position_estimate_t ev;
	mavlink_msg_vision_position_estimate_decode(msg, &ev);

	vehicle_odometry_s visual_odom{};

	visual_odom.timestamp = _mavlink_timesync.sync_stamp(ev.usec);
	visual_odom.x = ev.x;
	visual_odom.y = ev.y;
	visual_odom.z = ev.z;
	matrix::Quatf q(matrix::Eulerf(ev.roll, ev.pitch, ev.yaw));
	q.copyTo(visual_odom.q);

	// TODO:
	// - add a MAV_FRAME_*_OTHER to the Mavlink MAV_FRAME enum IOT define
	// a frame of reference which is not aligned with NED or ENU
	// - add usage on the estimator side
	visual_odom.local_frame = visual_odom.LOCAL_FRAME_NED;

	const size_t URT_SIZE = sizeof(visual_odom.pose_covariance) / sizeof(visual_odom.pose_covariance[0]);
	static_assert(URT_SIZE == (sizeof(ev.covariance) / sizeof(ev.covariance[0])),
		      "Odometry Pose Covariance matrix URT array size mismatch");

	for (size_t i = 0; i < URT_SIZE; i++) {
		visual_odom.pose_covariance[i] = ev.covariance[i];
	}

	visual_odom.vx = NAN;
	visual_odom.vy = NAN;
	visual_odom.vz = NAN;
	visual_odom.rollspeed = NAN;
	visual_odom.pitchspeed = NAN;
	visual_odom.yawspeed = NAN;
	visual_odom.velocity_covariance[0] = NAN;

	_visual_odometry_pub.publish(visual_odom);
}

void
MavlinkReceiver::handle_message_odometry(mavlink_message_t *msg)
{
	mavlink_odometry_t odom;
	mavlink_msg_odometry_decode(msg, &odom);

	vehicle_odometry_s odometry{};

	odometry.timestamp = _mavlink_timesync.sync_stamp(odom.time_usec);

	/* The position is in a local FRD frame */
	odometry.x = odom.x;
	odometry.y = odom.y;
	odometry.z = odom.z;

	/* The quaternion of the ODOMETRY msg represents a rotation from body frame to
	 * a local frame*/
	matrix::Quatf q_body_to_local(odom.q);
	q_body_to_local.normalize();
	q_body_to_local.copyTo(odometry.q);

	// TODO:
	// add usage of this information on the estimator side
	// The heading of the local frame is not aligned with north
	odometry.local_frame = odometry.LOCAL_FRAME_FRD;

	// pose_covariance
	static constexpr size_t POS_URT_SIZE = sizeof(odometry.pose_covariance) / sizeof(odometry.pose_covariance[0]);
	static_assert(POS_URT_SIZE == (sizeof(odom.pose_covariance) / sizeof(odom.pose_covariance[0])),
		      "Odometry Pose Covariance matrix URT array size mismatch");

	// velocity_covariance
	static constexpr size_t VEL_URT_SIZE = sizeof(odometry.velocity_covariance) / sizeof(odometry.velocity_covariance[0]);
	static_assert(VEL_URT_SIZE == (sizeof(odom.velocity_covariance) / sizeof(odom.velocity_covariance[0])),
		      "Odometry Velocity Covariance matrix URT array size mismatch");

	// TODO: create a method to simplify covariance copy
	for (size_t i = 0; i < POS_URT_SIZE; i++) {
		odometry.pose_covariance[i] = odom.pose_covariance[i];
	}

	/*
	 * PX4 expects the body's linear velocity in the local frame,
	 * the linear velocity is rotated from the odom child_frame to the
	 * local NED frame. The angular velocity needs to be expressed in the
	 * body (fcu_frd) frame.
	 */
	if (odom.child_frame_id == MAV_FRAME_BODY_FRD) {
		/* Linear velocity has to be rotated to the local NED frame
		 * Angular velocities are already in the right body frame  */
		const matrix::Dcmf R_body_to_local = matrix::Dcmf(q_body_to_local);

		/* the linear velocities needs to be transformed to the local frame FRD*/
		matrix::Vector3<float> linvel_local(R_body_to_local * matrix::Vector3<float>(odom.vx, odom.vy, odom.vz));

		odometry.vx = linvel_local(0);
		odometry.vy = linvel_local(1);
		odometry.vz = linvel_local(2);

		odometry.rollspeed = odom.rollspeed;
		odometry.pitchspeed = odom.pitchspeed;
		odometry.yawspeed = odom.yawspeed;

		/* the linear velocity's covariance needs to be transformed to the local frame FRD*/
		matrix::Matrix3f lin_vel_cov_body;
		lin_vel_cov_body(0, 0) = odom.velocity_covariance[odometry.COVARIANCE_MATRIX_VX_VARIANCE];
		lin_vel_cov_body(0, 1) = lin_vel_cov_body(1, 0) = odom.velocity_covariance[1];
		lin_vel_cov_body(0, 2) = lin_vel_cov_body(2, 0) = odom.velocity_covariance[2];
		lin_vel_cov_body(1, 1) = odom.velocity_covariance[odometry.COVARIANCE_MATRIX_VY_VARIANCE];
		lin_vel_cov_body(1, 2) = lin_vel_cov_body(2, 1) = odom.velocity_covariance[7];
		lin_vel_cov_body(2, 2) = odom.velocity_covariance[odometry.COVARIANCE_MATRIX_VZ_VARIANCE];
		matrix::Matrix3f lin_vel_cov_local = R_body_to_local * lin_vel_cov_body * R_body_to_local.transpose();

		/* Only the linear velocity variance elements are used */
		for (size_t i = 0; i < VEL_URT_SIZE; i++) {
			odometry.velocity_covariance[i] = NAN;
		}

		odometry.velocity_covariance[odometry.COVARIANCE_MATRIX_VX_VARIANCE] = lin_vel_cov_local(0, 0);
		odometry.velocity_covariance[odometry.COVARIANCE_MATRIX_VY_VARIANCE] = lin_vel_cov_local(1, 1);
		odometry.velocity_covariance[odometry.COVARIANCE_MATRIX_VZ_VARIANCE] = lin_vel_cov_local(2, 2);

	} else {
		PX4_ERR("Body frame %u not supported. Unable to publish velocity", odom.child_frame_id);
	}

	if (odom.frame_id == MAV_FRAME_LOCAL_FRD) {
		_visual_odometry_pub.publish(odometry);

	} else if (odom.frame_id == MAV_FRAME_MOCAP_NED) {
		_mocap_odometry_pub.publish(odometry);

	} else {
		PX4_ERR("Local frame %u not supported. Unable to publish pose and velocity", odom.frame_id);
	}
}

void MavlinkReceiver::fill_thrust(float *thrust_body_array, uint8_t vehicle_type, float thrust)
{
	// Fill correct field by checking frametype
	// TODO: add as needed
	switch (_mavlink->get_system_type()) {
	case MAV_TYPE_GENERIC:
		break;

	case MAV_TYPE_FIXED_WING:
	case MAV_TYPE_GROUND_ROVER:
		thrust_body_array[0] = thrust;
		break;

	case MAV_TYPE_QUADROTOR:
	case MAV_TYPE_HEXAROTOR:
	case MAV_TYPE_OCTOROTOR:
	case MAV_TYPE_TRICOPTER:
	case MAV_TYPE_HELICOPTER:
	case MAV_TYPE_COAXIAL:
		thrust_body_array[2] = -thrust;
		break;

	case MAV_TYPE_SUBMARINE:
		thrust_body_array[0] = thrust;
		break;

	case MAV_TYPE_VTOL_DUOROTOR:
	case MAV_TYPE_VTOL_QUADROTOR:
	case MAV_TYPE_VTOL_TILTROTOR:
	case MAV_TYPE_VTOL_RESERVED2:
	case MAV_TYPE_VTOL_RESERVED3:
	case MAV_TYPE_VTOL_RESERVED4:
	case MAV_TYPE_VTOL_RESERVED5:
		switch (vehicle_type) {
		case vehicle_status_s::VEHICLE_TYPE_FIXED_WING:
			thrust_body_array[0] = thrust;

			break;

		case vehicle_status_s::VEHICLE_TYPE_ROTARY_WING:
			thrust_body_array[2] = -thrust;

			break;

		default:
			// This should never happen
			break;
		}

		break;
	}
}

void
MavlinkReceiver::handle_message_set_attitude_target(mavlink_message_t *msg)
{
	mavlink_set_attitude_target_t set_attitude_target;
	mavlink_msg_set_attitude_target_decode(msg, &set_attitude_target);

	bool values_finite =
		PX4_ISFINITE(set_attitude_target.q[0]) &&
		PX4_ISFINITE(set_attitude_target.q[1]) &&
		PX4_ISFINITE(set_attitude_target.q[2]) &&
		PX4_ISFINITE(set_attitude_target.q[3]) &&
		PX4_ISFINITE(set_attitude_target.thrust) &&
		PX4_ISFINITE(set_attitude_target.body_roll_rate) &&
		PX4_ISFINITE(set_attitude_target.body_pitch_rate) &&
		PX4_ISFINITE(set_attitude_target.body_yaw_rate);

	/* Only accept messages which are intended for this system */
	if ((mavlink_system.sysid == set_attitude_target.target_system ||
	     set_attitude_target.target_system == 0) &&
	    (mavlink_system.compid == set_attitude_target.target_component ||
	     set_attitude_target.target_component == 0) &&
	    values_finite) {

		offboard_control_mode_s offboard_control_mode{};

		/* set correct ignore flags for thrust field: copy from mavlink message */
		offboard_control_mode.ignore_thrust = (bool)(set_attitude_target.type_mask & (1 << 6));

		/*
		 * The tricky part in parsing this message is that the offboard sender *can* set attitude and thrust
		 * using different messages. Eg.: First send set_attitude_target containing the attitude and ignore
		 * bits set for everything else and then send set_attitude_target containing the thrust and ignore bits
		 * set for everything else.
		 */

		/*
		 * if attitude or body rate have been used (not ignored) previously and this message only sends
		 * throttle and has the ignore bits set for attitude and rates don't change the flags for attitude and
		 * body rates to keep the controllers running
		 */
		bool ignore_bodyrate_msg_x = (bool)(set_attitude_target.type_mask & 0x1);
		bool ignore_bodyrate_msg_y = (bool)(set_attitude_target.type_mask & 0x2);
		bool ignore_bodyrate_msg_z = (bool)(set_attitude_target.type_mask & 0x4);
		bool ignore_attitude_msg = (bool)(set_attitude_target.type_mask & (1 << 7));


		if ((ignore_bodyrate_msg_x || ignore_bodyrate_msg_y ||
		     ignore_bodyrate_msg_z) &&
		    ignore_attitude_msg && !offboard_control_mode.ignore_thrust) {

			/* Message want's us to ignore everything except thrust: only ignore if previously ignored */
			offboard_control_mode.ignore_bodyrate_x = ignore_bodyrate_msg_x && offboard_control_mode.ignore_bodyrate_x;
			offboard_control_mode.ignore_bodyrate_y = ignore_bodyrate_msg_y && offboard_control_mode.ignore_bodyrate_y;
			offboard_control_mode.ignore_bodyrate_z = ignore_bodyrate_msg_z && offboard_control_mode.ignore_bodyrate_z;
			offboard_control_mode.ignore_attitude = ignore_attitude_msg && offboard_control_mode.ignore_attitude;

		} else {
			offboard_control_mode.ignore_bodyrate_x = ignore_bodyrate_msg_x;
			offboard_control_mode.ignore_bodyrate_y = ignore_bodyrate_msg_y;
			offboard_control_mode.ignore_bodyrate_z = ignore_bodyrate_msg_z;
			offboard_control_mode.ignore_attitude = ignore_attitude_msg;
		}

		offboard_control_mode.ignore_position = true;
		offboard_control_mode.ignore_velocity = true;
		offboard_control_mode.ignore_acceleration_force = true;

		offboard_control_mode.timestamp = hrt_absolute_time();

		_offboard_control_mode_pub.publish(offboard_control_mode);

		/* If we are in offboard control mode and offboard control loop through is enabled
		 * also publish the setpoint topic which is read by the controller */
		if (_mavlink->get_forward_externalsp()) {

			vehicle_control_mode_s control_mode{};
			_control_mode_sub.copy(&control_mode);

			if (control_mode.flag_control_offboard_enabled) {
				vehicle_status_s vehicle_status{};
				_vehicle_status_sub.copy(&vehicle_status);

				/* Publish attitude setpoint if attitude and thrust ignore bits are not set */
				if (!(offboard_control_mode.ignore_attitude)) {
					vehicle_attitude_setpoint_s att_sp = {};
					att_sp.timestamp = hrt_absolute_time();

					if (!ignore_attitude_msg) { // only copy att sp if message contained new data
						matrix::Quatf q(set_attitude_target.q);
						q.copyTo(att_sp.q_d);

						matrix::Eulerf euler{q};
						att_sp.roll_body = euler.phi();
						att_sp.pitch_body = euler.theta();
						att_sp.yaw_body = euler.psi();
						att_sp.yaw_sp_move_rate = 0.0f;
					}

					if (!offboard_control_mode.ignore_thrust) { // dont't overwrite thrust if it's invalid
						fill_thrust(att_sp.thrust_body, vehicle_status.vehicle_type, set_attitude_target.thrust);
					}

					// Publish attitude setpoint
					if (vehicle_status.is_vtol && (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING)) {
						_mc_virtual_att_sp_pub.publish(att_sp);

					} else if (vehicle_status.is_vtol && (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING)) {
						_fw_virtual_att_sp_pub.publish(att_sp);

					} else {
						_att_sp_pub.publish(att_sp);
					}
				}

				/* Publish attitude rate setpoint if bodyrate and thrust ignore bits are not set */
				if (!offboard_control_mode.ignore_bodyrate_x ||
				    !offboard_control_mode.ignore_bodyrate_y ||
				    !offboard_control_mode.ignore_bodyrate_z) {

					vehicle_rates_setpoint_s rates_sp{};

					rates_sp.timestamp = hrt_absolute_time();

					// only copy att rates sp if message contained new data
					if (!ignore_bodyrate_msg_x) {
						rates_sp.roll = set_attitude_target.body_roll_rate;
					}

					if (!ignore_bodyrate_msg_y) {
						rates_sp.pitch = set_attitude_target.body_pitch_rate;
					}

					if (!ignore_bodyrate_msg_z) {
						rates_sp.yaw = set_attitude_target.body_yaw_rate;
					}

					if (!offboard_control_mode.ignore_thrust) { // dont't overwrite thrust if it's invalid
						fill_thrust(rates_sp.thrust_body, vehicle_status.vehicle_type, set_attitude_target.thrust);
					}

					_rates_sp_pub.publish(rates_sp);
				}
			}
		}
	}
}

void
MavlinkReceiver::handle_message_radio_status(mavlink_message_t *msg)
{
	/* telemetry status supported only on first ORB_MULTI_MAX_INSTANCES mavlink channels */
	if (_mavlink->get_channel() < (mavlink_channel_t)ORB_MULTI_MAX_INSTANCES) {
		mavlink_radio_status_t rstatus;
		mavlink_msg_radio_status_decode(msg, &rstatus);

		radio_status_s status{};

		status.timestamp = hrt_absolute_time();
		status.rssi = rstatus.rssi;
		status.remote_rssi = rstatus.remrssi;
		status.txbuf = rstatus.txbuf;
		status.noise = rstatus.noise;
		status.remote_noise = rstatus.remnoise;
		status.rxerrors = rstatus.rxerrors;
		status.fix = rstatus.fixed;

		_mavlink->update_radio_status(status);

		_radio_status_pub.publish(status);
	}
}

void
MavlinkReceiver::handle_message_ping(mavlink_message_t *msg)
{
	mavlink_ping_t ping;
	mavlink_msg_ping_decode(msg, &ping);

	if ((ping.target_system == 0) &&
	    (ping.target_component == 0)) {	   // This is a ping request. Return it to the system which requested the ping.

		ping.target_system = msg->sysid;
		ping.target_component = msg->compid;
		mavlink_msg_ping_send_struct(_mavlink->get_channel(), &ping);

	} else if ((ping.target_system == mavlink_system.sysid) &&
		   (ping.target_component ==
		    mavlink_system.compid)) { // This is a returned ping message from this system. Calculate latency from it.

		const hrt_abstime now = hrt_absolute_time();

		// Calculate round trip time
		float rtt_ms = (now - ping.time_usec) / 1000.0f;

		// Update ping statistics
		struct Mavlink::ping_statistics_s &pstats = _mavlink->get_ping_statistics();

		pstats.last_ping_time = now;

		if (pstats.last_ping_seq == 0 && ping.seq > 0) {
			// This is the first reply we are receiving from an offboard system.
			// We may have been broadcasting pings for some time before it came online,
			// and these do not count as dropped packets.

			// Reset last_ping_seq counter for correct packet drop detection
			pstats.last_ping_seq = ping.seq - 1;
		}

		// We can only count dropped packets after the first message
		if (ping.seq > pstats.last_ping_seq) {
			pstats.dropped_packets += ping.seq - pstats.last_ping_seq - 1;
		}

		pstats.last_ping_seq = ping.seq;
		pstats.last_rtt = rtt_ms;
		pstats.mean_rtt = (rtt_ms + pstats.mean_rtt) / 2.0f;
		pstats.max_rtt = fmaxf(rtt_ms, pstats.max_rtt);
		pstats.min_rtt = pstats.min_rtt > 0.0f ? fminf(rtt_ms, pstats.min_rtt) : rtt_ms;

		/* Ping status is supported only on first ORB_MULTI_MAX_INSTANCES mavlink channels */
		if (_mavlink->get_channel() < (mavlink_channel_t)ORB_MULTI_MAX_INSTANCES) {

			ping_s uorb_ping_msg{};

			uorb_ping_msg.timestamp = now;
			uorb_ping_msg.ping_time = ping.time_usec;
			uorb_ping_msg.ping_sequence = ping.seq;
			uorb_ping_msg.dropped_packets = pstats.dropped_packets;
			uorb_ping_msg.rtt_ms = rtt_ms;
			uorb_ping_msg.system_id = msg->sysid;
			uorb_ping_msg.component_id = msg->compid;

			_ping_pub.publish(uorb_ping_msg);
		}
	}
}

void
MavlinkReceiver::handle_message_battery_status(mavlink_message_t *msg)
{
	if ((msg->sysid != mavlink_system.sysid) || (msg->compid == mavlink_system.compid)) {
		// ignore battery status coming from other systems or from the autopilot itself
		return;
	}

	// external battery measurements
	mavlink_battery_status_t battery_mavlink;
	mavlink_msg_battery_status_decode(msg, &battery_mavlink);

	battery_status_s battery_status{};
	battery_status.timestamp = hrt_absolute_time();

	float voltage_sum = 0.0f;
	uint8_t cell_count = 0;

	while (battery_mavlink.voltages[cell_count] < UINT16_MAX && cell_count < 10) {
		voltage_sum += (float)(battery_mavlink.voltages[cell_count]) / 1000.0f;
		cell_count++;
	}

	battery_status.voltage_v = voltage_sum;
	battery_status.voltage_filtered_v  = voltage_sum;
	battery_status.current_a = battery_status.current_filtered_a = (float)(battery_mavlink.current_battery) / 100.0f;
	battery_status.current_filtered_a = battery_status.current_a;
	battery_status.remaining = (float)battery_mavlink.battery_remaining / 100.0f;
	battery_status.discharged_mah = (float)battery_mavlink.current_consumed;
	battery_status.cell_count = cell_count;
	battery_status.connected = true;

	// Set the battery warning based on remaining charge.
	//  Note: Smallest values must come first in evaluation.
	if (battery_status.remaining < _param_bat_emergen_thr.get()) {
		battery_status.warning = battery_status_s::BATTERY_WARNING_EMERGENCY;

	} else if (battery_status.remaining < _param_bat_crit_thr.get()) {
		battery_status.warning = battery_status_s::BATTERY_WARNING_CRITICAL;

	} else if (battery_status.remaining < _param_bat_low_thr.get()) {
		battery_status.warning = battery_status_s::BATTERY_WARNING_LOW;
	}

	_battery_pub.publish(battery_status);
}

void
MavlinkReceiver::handle_message_serial_control(mavlink_message_t *msg)
{
	mavlink_serial_control_t serial_control_mavlink;
	mavlink_msg_serial_control_decode(msg, &serial_control_mavlink);

	// we only support shell commands
	if (serial_control_mavlink.device != SERIAL_CONTROL_DEV_SHELL
	    || (serial_control_mavlink.flags & SERIAL_CONTROL_FLAG_REPLY)) {
		return;
	}

	MavlinkShell *shell = _mavlink->get_shell();

	if (shell) {
		// we ignore the timeout, EXCLUSIVE & BLOCKING flags of the SERIAL_CONTROL message
		if (serial_control_mavlink.count > 0) {
			shell->write(serial_control_mavlink.data, serial_control_mavlink.count);
		}

		// if no response requested, assume the shell is no longer used
		if ((serial_control_mavlink.flags & SERIAL_CONTROL_FLAG_RESPOND) == 0) {
			_mavlink->close_shell();
		}
	}
}

void
MavlinkReceiver::handle_message_logging_ack(mavlink_message_t *msg)
{
	mavlink_logging_ack_t logging_ack;
	mavlink_msg_logging_ack_decode(msg, &logging_ack);

	MavlinkULog *ulog_streaming = _mavlink->get_ulog_streaming();

	if (ulog_streaming) {
		ulog_streaming->handle_ack(logging_ack);
	}
}

void
MavlinkReceiver::handle_message_play_tune(mavlink_message_t *msg)
{
	mavlink_play_tune_t play_tune;
	mavlink_msg_play_tune_decode(msg, &play_tune);

	char *tune = play_tune.tune;

	if ((mavlink_system.sysid == play_tune.target_system ||
	     play_tune.target_system == 0) &&
	    (mavlink_system.compid == play_tune.target_component ||
	     play_tune.target_component == 0)) {

		if (*tune == 'M') {
			int fd = px4_open(TONE_ALARM0_DEVICE_PATH, PX4_F_WRONLY);

			if (fd >= 0) {
				px4_write(fd, tune, strlen(tune) + 1);
				px4_close(fd);
			}
		}
	}
}

void
MavlinkReceiver::handle_message_obstacle_distance(mavlink_message_t *msg)
{
	mavlink_obstacle_distance_t mavlink_obstacle_distance;
	mavlink_msg_obstacle_distance_decode(msg, &mavlink_obstacle_distance);

	obstacle_distance_s obstacle_distance{};

	obstacle_distance.timestamp = hrt_absolute_time();
	obstacle_distance.sensor_type = mavlink_obstacle_distance.sensor_type;
	memcpy(obstacle_distance.distances, mavlink_obstacle_distance.distances, sizeof(obstacle_distance.distances));

	if (mavlink_obstacle_distance.increment_f > 0.f) {
		obstacle_distance.increment = mavlink_obstacle_distance.increment_f;

	} else {
		obstacle_distance.increment = (float)mavlink_obstacle_distance.increment;
	}

	obstacle_distance.min_distance = mavlink_obstacle_distance.min_distance;
	obstacle_distance.max_distance = mavlink_obstacle_distance.max_distance;
	obstacle_distance.angle_offset = mavlink_obstacle_distance.angle_offset;
	obstacle_distance.frame = mavlink_obstacle_distance.frame;

	_obstacle_distance_pub.publish(obstacle_distance);
}

void
MavlinkReceiver::handle_message_trajectory_representation_bezier(mavlink_message_t *msg)
{
	mavlink_trajectory_representation_bezier_t trajectory;
	mavlink_msg_trajectory_representation_bezier_decode(msg, &trajectory);

	vehicle_trajectory_bezier_s trajectory_bezier{};

	trajectory_bezier.timestamp =  _mavlink_timesync.sync_stamp(trajectory.time_usec);

	for (int i = 0; i < vehicle_trajectory_bezier_s::NUMBER_POINTS; ++i) {
		trajectory_bezier.control_points[i].position[0] = trajectory.pos_x[i];
		trajectory_bezier.control_points[i].position[1] = trajectory.pos_y[i];
		trajectory_bezier.control_points[i].position[2] = trajectory.pos_z[i];

		trajectory_bezier.control_points[i].delta = trajectory.delta[i];
		trajectory_bezier.control_points[i].yaw = trajectory.pos_yaw[i];
	}

	trajectory_bezier.bezier_order = math::min(trajectory.valid_points, vehicle_trajectory_bezier_s::NUMBER_POINTS);
	_trajectory_bezier_pub.publish(trajectory_bezier);
}

void
MavlinkReceiver::handle_message_trajectory_representation_waypoints(mavlink_message_t *msg)
{
	mavlink_trajectory_representation_waypoints_t trajectory;
	mavlink_msg_trajectory_representation_waypoints_decode(msg, &trajectory);

	vehicle_trajectory_waypoint_s trajectory_waypoint{};

	trajectory_waypoint.timestamp = hrt_absolute_time();
	const int number_valid_points = trajectory.valid_points;

	for (int i = 0; i < vehicle_trajectory_waypoint_s::NUMBER_POINTS; ++i) {
		trajectory_waypoint.waypoints[i].position[0] = trajectory.pos_x[i];
		trajectory_waypoint.waypoints[i].position[1] = trajectory.pos_y[i];
		trajectory_waypoint.waypoints[i].position[2] = trajectory.pos_z[i];

		trajectory_waypoint.waypoints[i].velocity[0] = trajectory.vel_x[i];
		trajectory_waypoint.waypoints[i].velocity[1] = trajectory.vel_y[i];
		trajectory_waypoint.waypoints[i].velocity[2] = trajectory.vel_z[i];

		trajectory_waypoint.waypoints[i].acceleration[0] = trajectory.acc_x[i];
		trajectory_waypoint.waypoints[i].acceleration[1] = trajectory.acc_y[i];
		trajectory_waypoint.waypoints[i].acceleration[2] = trajectory.acc_z[i];

		trajectory_waypoint.waypoints[i].yaw = trajectory.pos_yaw[i];
		trajectory_waypoint.waypoints[i].yaw_speed = trajectory.vel_yaw[i];

		trajectory_waypoint.waypoints[i].type = UINT8_MAX;
	}

	for (int i = 0; i < number_valid_points; ++i) {
		trajectory_waypoint.waypoints[i].point_valid = true;
	}

	for (int i = number_valid_points; i < vehicle_trajectory_waypoint_s::NUMBER_POINTS; ++i) {
		trajectory_waypoint.waypoints[i].point_valid = false;
	}

	_trajectory_waypoint_pub.publish(trajectory_waypoint);
}

switch_pos_t
MavlinkReceiver::decode_switch_pos(uint16_t buttons, unsigned sw)
{
	// This 2-bit method should be used in the future: (buttons >> (sw * 2)) & 3;
	return (buttons & (1 << sw)) ? manual_control_setpoint_s::SWITCH_POS_ON : manual_control_setpoint_s::SWITCH_POS_OFF;
}

int
MavlinkReceiver::decode_switch_pos_n(uint16_t buttons, unsigned sw)
{
	bool on = (buttons & (1 << sw));

	if (sw < MOM_SWITCH_COUNT) {

		bool last_on = (_mom_switch_state & (1 << sw));

		/* first switch is 2-pos, rest is 2 pos */
		unsigned state_count = (sw == 0) ? 3 : 2;

		/* only transition on low state */
		if (!on && (on != last_on)) {

			_mom_switch_pos[sw]++;

			if (_mom_switch_pos[sw] == state_count) {
				_mom_switch_pos[sw] = 0;
			}
		}

		/* state_count - 1 is the number of intervals and 1000 is the range,
		 * with 2 states 0 becomes 0, 1 becomes 1000. With
		 * 3 states 0 becomes 0, 1 becomes 500, 2 becomes 1000,
		 * and so on for more states.
		 */
		return (_mom_switch_pos[sw] * 1000) / (state_count - 1) + 1000;

	} else {
		/* return the current state */
		return on * 1000 + 1000;
	}
}

void
MavlinkReceiver::handle_message_rc_channels_override(mavlink_message_t *msg)
{
	mavlink_rc_channels_override_t man;
	mavlink_msg_rc_channels_override_decode(msg, &man);

	// Check target
	if (man.target_system != 0 && man.target_system != _mavlink->get_system_id()) {
		return;
	}

	// fill uORB message
	input_rc_s rc{};

	// metadata
	rc.timestamp = hrt_absolute_time();
	rc.timestamp_last_signal = rc.timestamp;
	rc.rssi = RC_INPUT_RSSI_MAX;
	rc.rc_failsafe = false;
	rc.rc_lost = false;
	rc.rc_lost_frame_count = 0;
	rc.rc_total_frame_count = 1;
	rc.rc_ppm_frame_length = 0;
	rc.input_source = input_rc_s::RC_INPUT_SOURCE_MAVLINK;

	// channels
	rc.values[0] = man.chan1_raw;
	rc.values[1] = man.chan2_raw;
	rc.values[2] = man.chan3_raw;
	rc.values[3] = man.chan4_raw;
	rc.values[4] = man.chan5_raw;
	rc.values[5] = man.chan6_raw;
	rc.values[6] = man.chan7_raw;
	rc.values[7] = man.chan8_raw;
	rc.values[8] = man.chan9_raw;
	rc.values[9] = man.chan10_raw;
	rc.values[10] = man.chan11_raw;
	rc.values[11] = man.chan12_raw;
	rc.values[12] = man.chan13_raw;
	rc.values[13] = man.chan14_raw;
	rc.values[14] = man.chan15_raw;
	rc.values[15] = man.chan16_raw;
	rc.values[16] = man.chan17_raw;
	rc.values[17] = man.chan18_raw;

	// check how many channels are valid
	for (int i = 17; i >= 0; i--) {
		const bool ignore_max = rc.values[i] == UINT16_MAX; // ignore any channel with value UINT16_MAX
		const bool ignore_zero = (i > 7) && (rc.values[i] == 0); // ignore channel 8-18 if value is 0

		if (ignore_max || ignore_zero) {
			// set all ignored values to zero
			rc.values[i] = 0;

		} else {
			// first channel to not ignore -> set count considering zero-based index
			rc.channel_count = i + 1;
			break;
		}
	}

	// publish uORB message
	_rc_pub.publish(rc);
}

void
MavlinkReceiver::handle_message_manual_control(mavlink_message_t *msg)
{
	mavlink_manual_control_t man;
	mavlink_msg_manual_control_decode(msg, &man);

	// Check target
	if (man.target != 0 && man.target != _mavlink->get_system_id()) {
		return;
	}

	if (_mavlink->should_generate_virtual_rc_input()) {

		input_rc_s rc{};
		rc.timestamp = hrt_absolute_time();
		rc.timestamp_last_signal = rc.timestamp;

		rc.channel_count = 8;
		rc.rc_failsafe = false;
		rc.rc_lost = false;
		rc.rc_lost_frame_count = 0;
		rc.rc_total_frame_count = 1;
		rc.rc_ppm_frame_length = 0;
		rc.input_source = input_rc_s::RC_INPUT_SOURCE_MAVLINK;
		rc.rssi = RC_INPUT_RSSI_MAX;

		rc.values[0] = man.x / 2 + 1500;	// roll
		rc.values[1] = man.y / 2 + 1500;	// pitch
		rc.values[2] = man.r / 2 + 1500;	// yaw
		rc.values[3] = math::constrain(man.z / 0.9f + 800.0f, 1000.0f, 2000.0f);	// throttle

		/* decode all switches which fit into the channel mask */
		unsigned max_switch = (sizeof(man.buttons) * 8);
		unsigned max_channels = (sizeof(rc.values) / sizeof(rc.values[0]));

		if (max_switch > (max_channels - 4)) {
			max_switch = (max_channels - 4);
		}

		/* fill all channels */
		for (unsigned i = 0; i < max_switch; i++) {
			rc.values[i + 4] = decode_switch_pos_n(man.buttons, i);
		}

		_mom_switch_state = man.buttons;

		_rc_pub.publish(rc);

	} else {
		manual_control_setpoint_s manual{};

		manual.timestamp = hrt_absolute_time();
		manual.x = man.x / 1000.0f;
		manual.y = man.y / 1000.0f;
		manual.r = man.r / 1000.0f;
		manual.z = man.z / 1000.0f;
		manual.data_source = manual_control_setpoint_s::SOURCE_MAVLINK_0 + _mavlink->get_instance_id();

		_manual_pub.publish(manual);
	}
}

void
MavlinkReceiver::handle_message_heartbeat(mavlink_message_t *msg)
{
	/* telemetry status supported only on first TELEMETRY_STATUS_ORB_ID_NUM mavlink channels */
	if (_mavlink->get_channel() < (mavlink_channel_t)ORB_MULTI_MAX_INSTANCES) {
		mavlink_heartbeat_t hb;
		mavlink_msg_heartbeat_decode(msg, &hb);

		/* Accept only heartbeats from GCS or ONBOARD Controller, skip heartbeats from other vehicles */
		if ((msg->sysid != mavlink_system.sysid && hb.type == MAV_TYPE_GCS) || (msg->sysid == mavlink_system.sysid
				&& hb.type == MAV_TYPE_ONBOARD_CONTROLLER)) {

			telemetry_status_s &tstatus = _mavlink->get_telemetry_status();

			/* set heartbeat time and topic time and publish -
			 * the telem status also gets updated on telemetry events
			 */
			tstatus.heartbeat_time = hrt_absolute_time();
			tstatus.remote_system_id = msg->sysid;
			tstatus.remote_component_id = msg->compid;
			tstatus.remote_type = hb.type;
			tstatus.remote_system_status = hb.system_status;
		}

	}
}

int
MavlinkReceiver::set_message_interval(int msgId, float interval, int data_rate)
{
	if (msgId == MAVLINK_MSG_ID_HEARTBEAT) {
		return PX4_ERROR;
	}

	if (data_rate > 0) {
		_mavlink->set_data_rate(data_rate);
	}

	// configure_stream wants a rate (msgs/second), so convert here.
	float rate = 0.f;

	if (interval < -0.00001f) {
		rate = 0.f; // stop the stream

	} else if (interval > 0.00001f) {
		rate = 1000000.0f / interval;

	} else {
		rate = -2.f; // set default rate
	}

	bool found_id = false;


	// The interval between two messages is in microseconds.
	// Set to -1 to disable and 0 to request default rate
	if (msgId != 0) {
		const char *stream_name = get_stream_name(msgId);

		if (stream_name != nullptr) {
			_mavlink->configure_stream_threadsafe(stream_name, rate);
			found_id = true;
		}
	}

	return (found_id ? PX4_OK : PX4_ERROR);
}

void
MavlinkReceiver::get_message_interval(int msgId)
{
	unsigned interval = 0;

	for (const auto &stream : _mavlink->get_streams()) {
		if (stream->get_id() == msgId) {
			interval = stream->get_interval();
			break;
		}
	}

	// send back this value...
	mavlink_msg_message_interval_send(_mavlink->get_channel(), msgId, interval);
}

void
MavlinkReceiver::handle_message_hil_sensor(mavlink_message_t *msg)
{
	mavlink_hil_sensor_t imu;
	mavlink_msg_hil_sensor_decode(msg, &imu);

	const uint64_t timestamp = hrt_absolute_time();

	/* airspeed */
	{
		airspeed_s airspeed{};

		float ias = calc_IAS(imu.diff_pressure * 1e2f);
		float scale = 1.0f; //assume no instrument or pitot placement errors
		float eas = calc_EAS_from_IAS(ias, scale);
		float tas = calc_TAS_from_EAS(eas, imu.abs_pressure * 100, imu.temperature);

		airspeed.timestamp = timestamp;
		airspeed.indicated_airspeed_m_s = ias;
		airspeed.true_airspeed_m_s = tas;

		_airspeed_pub.publish(airspeed);
	}

	/* gyro */
	{
		if (_px4_gyro == nullptr) {
			// 2294028: DRV_GYR_DEVTYPE_GYROSIM, BUS: 1, ADDR: 2, TYPE: SIMULATION
			_px4_gyro = new PX4Gyroscope(2294028);

			if (_px4_gyro == nullptr) {
				PX4_ERR("PX4Gyroscope alloc failed");
			}
		}

		if (_px4_gyro != nullptr) {
			_px4_gyro->set_temperature(imu.temperature);
			_px4_gyro->update(timestamp, imu.xgyro, imu.ygyro, imu.zgyro);
		}
	}

	/* accelerometer */
	{
		if (_px4_accel == nullptr) {
			// 1311244: DRV_ACC_DEVTYPE_ACCELSIM, BUS: 1, ADDR: 1, TYPE: SIMULATION
			_px4_accel = new PX4Accelerometer(1311244);

			if (_px4_accel == nullptr) {
				PX4_ERR("PX4Accelerometer alloc failed");
			}
		}

		if (_px4_accel != nullptr) {
			_px4_accel->set_temperature(imu.temperature);
			_px4_accel->update(timestamp, imu.xacc, imu.yacc, imu.zacc);
		}
	}

	/* magnetometer */
	{
		if (_px4_mag == nullptr) {
			// 197388: DRV_MAG_DEVTYPE_MAGSIM, BUS: 3, ADDR: 1, TYPE: SIMULATION
			_px4_mag = new PX4Magnetometer(197388);

			if (_px4_mag == nullptr) {
				PX4_ERR("PX4Magnetometer alloc failed");
			}
		}

		if (_px4_mag != nullptr) {
			_px4_mag->set_temperature(imu.temperature);
			_px4_mag->update(timestamp, imu.xmag, imu.ymag, imu.zmag);
		}
	}

	/* baro */
	{
		if (_px4_baro == nullptr) {
			// 6620172: DRV_BARO_DEVTYPE_BAROSIM, BUS: 1, ADDR: 4, TYPE: SIMULATION
			_px4_baro = new PX4Barometer(6620172);

			if (_px4_baro == nullptr) {
				PX4_ERR("PX4Barometer alloc failed");
			}
		}

		if (_px4_baro != nullptr) {
			_px4_baro->set_temperature(imu.temperature);
			_px4_baro->update(timestamp, imu.abs_pressure);
		}
	}

	/* battery status */
	{
		battery_status_s hil_battery_status{};

		hil_battery_status.timestamp = timestamp;
		hil_battery_status.voltage_v = 11.5f;
		hil_battery_status.voltage_filtered_v = 11.5f;
		hil_battery_status.current_a = 10.0f;
		hil_battery_status.discharged_mah = -1.0f;

		_battery_pub.publish(hil_battery_status);
	}
}

void
MavlinkReceiver::handle_message_hil_gps(mavlink_message_t *msg)
{
	mavlink_hil_gps_t gps;
	mavlink_msg_hil_gps_decode(msg, &gps);

	const uint64_t timestamp = hrt_absolute_time();

	struct vehicle_gps_position_s hil_gps = {};

	hil_gps.timestamp_time_relative = 0;
	hil_gps.time_utc_usec = gps.time_usec;

	hil_gps.timestamp = timestamp;
	hil_gps.lat = gps.lat;
	hil_gps.lon = gps.lon;
	hil_gps.alt = gps.alt;
	hil_gps.eph = (float)gps.eph * 1e-2f; // from cm to m
	hil_gps.epv = (float)gps.epv * 1e-2f; // from cm to m

	hil_gps.s_variance_m_s = 0.1f;

	hil_gps.vel_m_s = (float)gps.vel * 1e-2f; // from cm/s to m/s
	hil_gps.vel_n_m_s = gps.vn * 1e-2f; // from cm to m
	hil_gps.vel_e_m_s = gps.ve * 1e-2f; // from cm to m
	hil_gps.vel_d_m_s = gps.vd * 1e-2f; // from cm to m
	hil_gps.vel_ned_valid = true;
	hil_gps.cog_rad = ((gps.cog == 65535) ? NAN : wrap_2pi(math::radians(gps.cog * 1e-2f)));

	hil_gps.fix_type = gps.fix_type;
	hil_gps.satellites_used = gps.satellites_visible;  //TODO: rename mavlink_hil_gps_t sats visible to used?

	hil_gps.heading = NAN;
	hil_gps.heading_offset = NAN;

	_gps_pub.publish(hil_gps);
}

void
MavlinkReceiver::handle_message_follow_target(mavlink_message_t *msg)
{
	mavlink_follow_target_t follow_target_msg;
	mavlink_msg_follow_target_decode(msg, &follow_target_msg);

	follow_target_s follow_target_topic{};

	follow_target_topic.timestamp = hrt_absolute_time();
	follow_target_topic.lat = follow_target_msg.lat * 1e-7;
	follow_target_topic.lon = follow_target_msg.lon * 1e-7;
	follow_target_topic.alt = follow_target_msg.alt;

	_follow_target_pub.publish(follow_target_topic);
}

void
MavlinkReceiver::handle_message_landing_target(mavlink_message_t *msg)
{
	mavlink_landing_target_t landing_target;
	mavlink_msg_landing_target_decode(msg, &landing_target);

	if (landing_target.position_valid && landing_target.frame == MAV_FRAME_LOCAL_NED) {
		landing_target_pose_s landing_target_pose{};

		landing_target_pose.timestamp = _mavlink_timesync.sync_stamp(landing_target.time_usec);
		landing_target_pose.abs_pos_valid = true;
		landing_target_pose.x_abs = landing_target.x;
		landing_target_pose.y_abs = landing_target.y;
		landing_target_pose.z_abs = landing_target.z;

		_landing_target_pose_pub.publish(landing_target_pose);
	}
}

void
MavlinkReceiver::handle_message_cellular_status(mavlink_message_t *msg)
{
	mavlink_cellular_status_t status;
	mavlink_msg_cellular_status_decode(msg, &status);

	cellular_status_s cellular_status{};

	cellular_status.timestamp = hrt_absolute_time();
	cellular_status.status = status.status;
	cellular_status.type = status.type;
	cellular_status.quality = status.quality;
	cellular_status.mcc = status.mcc;
	cellular_status.mnc = status.mnc;
	cellular_status.lac = status.lac;
	cellular_status.cid = status.cid;

	_cellular_status_pub.publish(cellular_status);
}

void
MavlinkReceiver::handle_message_adsb_vehicle(mavlink_message_t *msg)
{
	mavlink_adsb_vehicle_t adsb;
	mavlink_msg_adsb_vehicle_decode(msg, &adsb);

	transponder_report_s t{};

	t.timestamp = hrt_absolute_time();

	t.icao_address = adsb.ICAO_address;
	t.lat = adsb.lat * 1e-7;
	t.lon = adsb.lon * 1e-7;
	t.altitude_type = adsb.altitude_type;
	t.altitude = adsb.altitude / 1000.0f;
	t.heading = adsb.heading / 100.0f / 180.0f * M_PI_F - M_PI_F;
	t.hor_velocity = adsb.hor_velocity / 100.0f;
	t.ver_velocity = adsb.ver_velocity / 100.0f;
	memcpy(&t.callsign[0], &adsb.callsign[0], sizeof(t.callsign));
	t.emitter_type = adsb.emitter_type;
	t.tslc = adsb.tslc;
	t.squawk = adsb.squawk;

	t.flags = transponder_report_s::PX4_ADSB_FLAGS_RETRANSLATE;  //Unset in receiver already broadcast its messages

	if (adsb.flags & ADSB_FLAGS_VALID_COORDS) { t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS; }

	if (adsb.flags & ADSB_FLAGS_VALID_ALTITUDE) { t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE; }

	if (adsb.flags & ADSB_FLAGS_VALID_HEADING) { t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING; }

	if (adsb.flags & ADSB_FLAGS_VALID_VELOCITY) { t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY; }

	if (adsb.flags & ADSB_FLAGS_VALID_CALLSIGN) { t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN; }

	if (adsb.flags & ADSB_FLAGS_VALID_SQUAWK) { t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_SQUAWK; }

	//PX4_INFO("code: %d callsign: %s, vel: %8.4f, tslc: %d", (int)t.ICAO_address, t.callsign, (double)t.hor_velocity, (int)t.tslc);

	_transponder_report_pub.publish(t);
}

void
MavlinkReceiver::handle_message_utm_global_position(mavlink_message_t *msg)
{
	mavlink_utm_global_position_t utm_pos;
	mavlink_msg_utm_global_position_decode(msg, &utm_pos);

	px4_guid_t px4_guid;
#ifndef BOARD_HAS_NO_UUID
	board_get_px4_guid(px4_guid);
#else
	// TODO Fill ID with something reasonable
	memset(&px4_guid[0], 0, sizeof(px4_guid));
#endif /* BOARD_HAS_NO_UUID */


	//Ignore selfpublished UTM messages
	if (sizeof(px4_guid) == sizeof(utm_pos.uas_id) && memcmp(px4_guid, utm_pos.uas_id, sizeof(px4_guid_t)) != 0) {

		// Convert cm/s to m/s
		float vx = utm_pos.vx / 100.0f;
		float vy = utm_pos.vy / 100.0f;
		float vz = utm_pos.vz / 100.0f;

		transponder_report_s t{};
		t.timestamp = hrt_absolute_time();
		mav_array_memcpy(t.uas_id, utm_pos.uas_id, sizeof(px4_guid_t));
		t.lat = utm_pos.lat * 1e-7;
		t.lon = utm_pos.lon * 1e-7;
		t.altitude = utm_pos.alt / 1000.0f;
		t.altitude_type = ADSB_ALTITUDE_TYPE_GEOMETRIC;
		// UTM_GLOBAL_POSIION uses NED (north, east, down) coordinates for velocity, in cm / s.
		t.heading = atan2f(vy, vx);
		t.hor_velocity = sqrtf(vy * vy + vx * vx);
		t.ver_velocity = -vz;
		// TODO: Callsign
		// For now, set it to all 0s. This is a null-terminated string, so not explicitly giving it a null
		// terminator could cause problems.
		memset(&t.callsign[0], 0, sizeof(t.callsign));
		t.emitter_type = ADSB_EMITTER_TYPE_UAV;  // TODO: Is this correct?x2?

		// The Mavlink docs do not specify what to do if tslc (time since last communication) is out of range of
		// an 8-bit int, or if this is the first communication.
		// Here, I assume that if this is the first communication, tslc = 0.
		// If tslc > 255, then tslc = 255.
		unsigned long time_passed = (t.timestamp - _last_utm_global_pos_com) / 1000000;

		if (_last_utm_global_pos_com == 0) {
			time_passed = 0;

		} else if (time_passed > UINT8_MAX) {
			time_passed = UINT8_MAX;
		}

		t.tslc = (uint8_t) time_passed;

		t.flags = 0;

		if (utm_pos.flags & UTM_DATA_AVAIL_FLAGS_POSITION_AVAILABLE) {
			t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS;
		}

		if (utm_pos.flags & UTM_DATA_AVAIL_FLAGS_ALTITUDE_AVAILABLE) {
			t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE;
		}

		if (utm_pos.flags & UTM_DATA_AVAIL_FLAGS_HORIZONTAL_VELO_AVAILABLE) {
			t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING;
			t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY;
		}

		// Note: t.flags has deliberately NOT set VALID_CALLSIGN or VALID_SQUAWK, because UTM_GLOBAL_POSITION does not
		// provide these.
		_transponder_report_pub.publish(t);

		_last_utm_global_pos_com = t.timestamp;
	}
}

void
MavlinkReceiver::handle_message_collision(mavlink_message_t *msg)
{
	mavlink_collision_t collision;
	mavlink_msg_collision_decode(msg, &collision);

	collision_report_s collision_report{};

	collision_report.timestamp = hrt_absolute_time();
	collision_report.src = collision.src;
	collision_report.id = collision.id;
	collision_report.action = collision.action;
	collision_report.threat_level = collision.threat_level;
	collision_report.time_to_minimum_delta = collision.time_to_minimum_delta;
	collision_report.altitude_minimum_delta = collision.altitude_minimum_delta;
	collision_report.horizontal_minimum_delta = collision.horizontal_minimum_delta;

	_collision_report_pub.publish(collision_report);
}

void
MavlinkReceiver::handle_message_gps_rtcm_data(mavlink_message_t *msg)
{
	mavlink_gps_rtcm_data_t gps_rtcm_data_msg;
	mavlink_msg_gps_rtcm_data_decode(msg, &gps_rtcm_data_msg);

	gps_inject_data_s gps_inject_data_topic{};

	gps_inject_data_topic.len = math::min((int)sizeof(gps_rtcm_data_msg.data),
					      (int)sizeof(uint8_t) * gps_rtcm_data_msg.len);
	gps_inject_data_topic.flags = gps_rtcm_data_msg.flags;
	memcpy(gps_inject_data_topic.data, gps_rtcm_data_msg.data,
	       math::min((int)sizeof(gps_inject_data_topic.data), (int)sizeof(uint8_t) * gps_inject_data_topic.len));

	_gps_inject_data_pub.publish(gps_inject_data_topic);
}

void
MavlinkReceiver::handle_message_hil_state_quaternion(mavlink_message_t *msg)
{
	mavlink_hil_state_quaternion_t hil_state;
	mavlink_msg_hil_state_quaternion_decode(msg, &hil_state);

	const uint64_t timestamp = hrt_absolute_time();

	/* airspeed */
	{
		airspeed_s airspeed{};

		airspeed.timestamp = timestamp;
		airspeed.indicated_airspeed_m_s = hil_state.ind_airspeed * 1e-2f;
		airspeed.true_airspeed_m_s = hil_state.true_airspeed * 1e-2f;

		_airspeed_pub.publish(airspeed);
	}

	/* attitude */
	{
		vehicle_attitude_s hil_attitude{};

		hil_attitude.timestamp = timestamp;

		matrix::Quatf q(hil_state.attitude_quaternion);
		q.copyTo(hil_attitude.q);

		_attitude_pub.publish(hil_attitude);
	}

	/* global position */
	{
		vehicle_global_position_s hil_global_pos{};

		hil_global_pos.timestamp = timestamp;
		hil_global_pos.lat = hil_state.lat / ((double)1e7);
		hil_global_pos.lon = hil_state.lon / ((double)1e7);
		hil_global_pos.alt = hil_state.alt / 1000.0f;
		hil_global_pos.eph = 2.0f;
		hil_global_pos.epv = 4.0f;

		_global_pos_pub.publish(hil_global_pos);
	}

	/* local position */
	{
		double lat = hil_state.lat * 1e-7;
		double lon = hil_state.lon * 1e-7;

		if (!_hil_local_proj_inited) {
			_hil_local_proj_inited = true;
			_hil_local_alt0 = hil_state.alt / 1000.0f;

			map_projection_init(&_hil_local_proj_ref, lat, lon);
		}

		float x = 0.0f;
		float y = 0.0f;
		map_projection_project(&_hil_local_proj_ref, lat, lon, &x, &y);

		vehicle_local_position_s hil_local_pos{};
		hil_local_pos.timestamp = timestamp;

		hil_local_pos.ref_timestamp = _hil_local_proj_ref.timestamp;
		hil_local_pos.ref_lat = math::radians(_hil_local_proj_ref.lat_rad);
		hil_local_pos.ref_lon = math::radians(_hil_local_proj_ref.lon_rad);
		hil_local_pos.ref_alt = _hil_local_alt0;
		hil_local_pos.xy_valid = true;
		hil_local_pos.z_valid = true;
		hil_local_pos.v_xy_valid = true;
		hil_local_pos.v_z_valid = true;
		hil_local_pos.x = x;
		hil_local_pos.y = y;
		hil_local_pos.z = _hil_local_alt0 - hil_state.alt / 1000.0f;
		hil_local_pos.vx = hil_state.vx / 100.0f;
		hil_local_pos.vy = hil_state.vy / 100.0f;
		hil_local_pos.vz = hil_state.vz / 100.0f;

		matrix::Eulerf euler{matrix::Quatf(hil_state.attitude_quaternion)};
		hil_local_pos.yaw = euler.psi();
		hil_local_pos.xy_global = true;
		hil_local_pos.z_global = true;
		hil_local_pos.vxy_max = INFINITY;
		hil_local_pos.vz_max = INFINITY;
		hil_local_pos.hagl_min = INFINITY;
		hil_local_pos.hagl_max = INFINITY;

		_local_pos_pub.publish(hil_local_pos);
	}

	/* accelerometer */
	{
		if (_px4_accel == nullptr) {
			// 1311244: DRV_ACC_DEVTYPE_ACCELSIM, BUS: 1, ADDR: 1, TYPE: SIMULATION
			_px4_accel = new PX4Accelerometer(1311244);

			if (_px4_accel == nullptr) {
				PX4_ERR("PX4Accelerometer alloc failed");
			}
		}

		if (_px4_accel != nullptr) {
			// accel in mG
			_px4_accel->set_scale(CONSTANTS_ONE_G / 1000.0f);
			_px4_accel->update(timestamp, hil_state.xacc, hil_state.yacc, hil_state.zacc);
		}
	}

	/* gyroscope */
	{
		if (_px4_gyro == nullptr) {
			// 2294028: DRV_GYR_DEVTYPE_GYROSIM, BUS: 1, ADDR: 2, TYPE: SIMULATION
			_px4_gyro = new PX4Gyroscope(2294028);

			if (_px4_gyro == nullptr) {
				PX4_ERR("PX4Gyroscope alloc failed");
			}
		}

		if (_px4_gyro != nullptr) {
			_px4_gyro->update(timestamp, hil_state.rollspeed, hil_state.pitchspeed, hil_state.yawspeed);
		}
	}

	/* battery status */
	{
		battery_status_s hil_battery_status{};

		hil_battery_status.timestamp = timestamp;
		hil_battery_status.voltage_v = 11.1f;
		hil_battery_status.voltage_filtered_v = 11.1f;
		hil_battery_status.current_a = 10.0f;
		hil_battery_status.discharged_mah = -1.0f;

		_battery_pub.publish(hil_battery_status);
	}
}

void
MavlinkReceiver::handle_message_named_value_float(mavlink_message_t *msg)
{
	mavlink_named_value_float_t debug_msg;
	mavlink_msg_named_value_float_decode(msg, &debug_msg);

	debug_key_value_s debug_topic{};

	debug_topic.timestamp = hrt_absolute_time();
	memcpy(debug_topic.key, debug_msg.name, sizeof(debug_topic.key));
	debug_topic.key[sizeof(debug_topic.key) - 1] = '\0'; // enforce null termination
	debug_topic.value = debug_msg.value;

	_debug_key_value_pub.publish(debug_topic);
}

void
MavlinkReceiver::handle_message_debug(mavlink_message_t *msg)
{
	mavlink_debug_t debug_msg;
	mavlink_msg_debug_decode(msg, &debug_msg);

	debug_value_s debug_topic{};

	debug_topic.timestamp = hrt_absolute_time();
	debug_topic.ind = debug_msg.ind;
	debug_topic.value = debug_msg.value;

	_debug_value_pub.publish(debug_topic);
}

void
MavlinkReceiver::handle_message_debug_vect(mavlink_message_t *msg)
{
	mavlink_debug_vect_t debug_msg;
	mavlink_msg_debug_vect_decode(msg, &debug_msg);

	debug_vect_s debug_topic{};

	debug_topic.timestamp = hrt_absolute_time();
	memcpy(debug_topic.name, debug_msg.name, sizeof(debug_topic.name));
	debug_topic.name[sizeof(debug_topic.name) - 1] = '\0'; // enforce null termination
	debug_topic.x = debug_msg.x;
	debug_topic.y = debug_msg.y;
	debug_topic.z = debug_msg.z;

	_debug_vect_pub.publish(debug_topic);
}

void
MavlinkReceiver::handle_message_debug_float_array(mavlink_message_t *msg)
{
	mavlink_debug_float_array_t debug_msg;
	mavlink_msg_debug_float_array_decode(msg, &debug_msg);

	debug_array_s debug_topic{};

	debug_topic.timestamp = hrt_absolute_time();
	debug_topic.id = debug_msg.array_id;
	memcpy(debug_topic.name, debug_msg.name, sizeof(debug_topic.name));
	debug_topic.name[sizeof(debug_topic.name) - 1] = '\0'; // enforce null termination

	for (size_t i = 0; i < debug_array_s::ARRAY_SIZE; i++) {
		debug_topic.data[i] = debug_msg.data[i];
	}

	_debug_array_pub.publish(debug_topic);
}

void
MavlinkReceiver::handle_message_onboard_computer_status(mavlink_message_t *msg)
{
	mavlink_onboard_computer_status_t status_msg;
	mavlink_msg_onboard_computer_status_decode(msg, &status_msg);

	onboard_computer_status_s onboard_computer_status_topic{};

	onboard_computer_status_topic.timestamp = hrt_absolute_time();
	onboard_computer_status_topic.uptime = status_msg.uptime;

	onboard_computer_status_topic.type = status_msg.type;

	memcpy(onboard_computer_status_topic.cpu_cores, status_msg.cpu_cores, sizeof(status_msg.cpu_cores));
	memcpy(onboard_computer_status_topic.cpu_combined, status_msg.cpu_combined, sizeof(status_msg.cpu_combined));
	memcpy(onboard_computer_status_topic.gpu_cores, status_msg.gpu_cores, sizeof(status_msg.gpu_cores));
	memcpy(onboard_computer_status_topic.gpu_combined, status_msg.gpu_combined, sizeof(status_msg.gpu_combined));
	onboard_computer_status_topic.temperature_board = status_msg.temperature_board;
	memcpy(onboard_computer_status_topic.temperature_core, status_msg.temperature_core,
	       sizeof(status_msg.temperature_core));
	memcpy(onboard_computer_status_topic.fan_speed, status_msg.fan_speed, sizeof(status_msg.fan_speed));
	onboard_computer_status_topic.ram_usage = status_msg.ram_usage;
	onboard_computer_status_topic.ram_total = status_msg.ram_total;
	memcpy(onboard_computer_status_topic.storage_type, status_msg.storage_type, sizeof(status_msg.storage_type));
	memcpy(onboard_computer_status_topic.storage_usage, status_msg.storage_usage, sizeof(status_msg.storage_usage));
	memcpy(onboard_computer_status_topic.storage_total, status_msg.storage_total, sizeof(status_msg.storage_total));
	memcpy(onboard_computer_status_topic.link_type, status_msg.link_type, sizeof(status_msg.link_type));
	memcpy(onboard_computer_status_topic.link_tx_rate, status_msg.link_tx_rate, sizeof(status_msg.link_tx_rate));
	memcpy(onboard_computer_status_topic.link_rx_rate, status_msg.link_rx_rate, sizeof(status_msg.link_rx_rate));
	memcpy(onboard_computer_status_topic.link_tx_max, status_msg.link_tx_max, sizeof(status_msg.link_tx_max));
	memcpy(onboard_computer_status_topic.link_rx_max, status_msg.link_rx_max, sizeof(status_msg.link_rx_max));

	_onboard_computer_status_pub.publish(onboard_computer_status_topic);
}

void MavlinkReceiver::handle_message_statustext(mavlink_message_t *msg)
{
	if (msg->sysid == mavlink_system.sysid) {
		// log message from the same system

		mavlink_statustext_t statustext;
		mavlink_msg_statustext_decode(msg, &statustext);

		log_message_s log_message{};

		log_message.severity = statustext.severity;
		log_message.timestamp = hrt_absolute_time();

		snprintf(log_message.text, sizeof(log_message.text),
			 "[mavlink: component %d] %." STRINGIFY(MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN) "s", msg->compid, statustext.text);

		_log_message_pub.publish(log_message);
	}
}

/**
 * Receive data from UART/UDP
 */
void
MavlinkReceiver::Run()
{
	/* set thread name */
	{
		char thread_name[17];
		snprintf(thread_name, sizeof(thread_name), "mavlink_rcv_if%d", _mavlink->get_instance_id());
		px4_prctl(PR_SET_NAME, thread_name, px4_getpid());
	}

	// make sure mavlink app has booted before we start processing anything (parameter sync, etc)
	while (!_mavlink->boot_complete()) {
		if (hrt_elapsed_time(&_mavlink->get_first_start_time()) > 20_s) {
			PX4_ERR("system boot did not complete in 20 seconds");
			_mavlink->set_boot_complete();
		}

		px4_usleep(100000);
	}

	// poll timeout in ms. Also defines the max update frequency of the mission & param manager, etc.
	const int timeout = 10;

#if defined(__PX4_POSIX)
	/* 1500 is the Wifi MTU, so we make sure to fit a full packet */
	uint8_t buf[1600 * 5];
#elif defined(CONFIG_NET)
	/* 1500 is the Wifi MTU, so we make sure to fit a full packet */
	uint8_t buf[1000];
#else
	/* the serial port buffers internally as well, we just need to fit a small chunk */
	uint8_t buf[64];
#endif
	mavlink_message_t msg;

	struct pollfd fds[1] = {};

	if (_mavlink->get_protocol() == Protocol::SERIAL) {
		fds[0].fd = _mavlink->get_uart_fd();
		fds[0].events = POLLIN;
	}

#if defined(MAVLINK_UDP)
	struct sockaddr_in srcaddr = {};
	socklen_t addrlen = sizeof(srcaddr);

	if (_mavlink->get_protocol() == Protocol::UDP) {
		fds[0].fd = _mavlink->get_socket_fd();
		fds[0].events = POLLIN;
	}

#endif // MAVLINK_UDP

	ssize_t nread = 0;
	hrt_abstime last_send_update = 0;

	while (!_mavlink->_task_should_exit) {

		// check for parameter updates
		if (_parameter_update_sub.updated()) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			updateParams();
		}

		int ret = poll(&fds[0], 1, timeout);

		if (ret > 0) {
			if (_mavlink->get_protocol() == Protocol::SERIAL) {
				/* non-blocking read. read may return negative values */
				nread = ::read(fds[0].fd, buf, sizeof(buf));

				if (nread == -1 && errno == ENOTCONN) { // Not connected (can happen for USB)
					usleep(100000);
				}
			}

#if defined(MAVLINK_UDP)

			else if (_mavlink->get_protocol() == Protocol::UDP) {
				if (fds[0].revents & POLLIN) {
					nread = recvfrom(_mavlink->get_socket_fd(), buf, sizeof(buf), 0, (struct sockaddr *)&srcaddr, &addrlen);
				}

				struct sockaddr_in &srcaddr_last = _mavlink->get_client_source_address();

				int localhost = (127 << 24) + 1;

				if (!_mavlink->get_client_source_initialized()) {

					// set the address either if localhost or if 3 seconds have passed
					// this ensures that a GCS running on localhost can get a hold of
					// the system within the first N seconds
					hrt_abstime stime = _mavlink->get_start_time();

					if ((stime != 0 && (hrt_elapsed_time(&stime) > 3_s))
					    || (srcaddr_last.sin_addr.s_addr == htonl(localhost))) {

						srcaddr_last.sin_addr.s_addr = srcaddr.sin_addr.s_addr;
						srcaddr_last.sin_port = srcaddr.sin_port;

						_mavlink->set_client_source_initialized();

						PX4_INFO("partner IP: %s", inet_ntoa(srcaddr.sin_addr));
					}
				}
			}

			// only start accepting messages on UDP once we're sure who we talk to
			if (_mavlink->get_protocol() != Protocol::UDP || _mavlink->get_client_source_initialized()) {
#endif // MAVLINK_UDP

				/* if read failed, this loop won't execute */
				for (ssize_t i = 0; i < nread; i++) {
					if (mavlink_parse_char(_mavlink->get_channel(), buf[i], &msg, &_status)) {

						/* check if we received version 2 and request a switch. */
						if (!(_mavlink->get_status()->flags & MAVLINK_STATUS_FLAG_IN_MAVLINK1)) {
							/* this will only switch to proto version 2 if allowed in settings */
							_mavlink->set_proto_version(2);
						}

						/* handle generic messages and commands */
						handle_message(&msg);

						/* handle packet with mission manager */
						_mission_manager.handle_message(&msg);


						/* handle packet with parameter component */
						_parameters_manager.handle_message(&msg);

						if (_mavlink->ftp_enabled()) {
							/* handle packet with ftp component */
							_mavlink_ftp.handle_message(&msg);
						}

						/* handle packet with log component */
						_mavlink_log_handler.handle_message(&msg);

						/* handle packet with timesync component */
						_mavlink_timesync.handle_message(&msg);

						/* handle packet with parent object */
						_mavlink->handle_message(&msg);
					}
				}

				/* count received bytes (nread will be -1 on read error) */
				if (nread > 0) {
					_mavlink->count_rxbytes(nread);
				}

#if defined(MAVLINK_UDP)
			}

#endif // MAVLINK_UDP

		} else if (ret == -1) {
			usleep(10000);
		}

		hrt_abstime t = hrt_absolute_time();

		if (t - last_send_update > timeout * 1000) {
			_mission_manager.check_active_mission();
			_mission_manager.send(t);

			_parameters_manager.send(t);

			if (_mavlink->ftp_enabled()) {
				_mavlink_ftp.send(t);
			}

			_mavlink_log_handler.send(t);
			last_send_update = t;
		}

	}
}

void *
MavlinkReceiver::start_helper(void *context)
{
	MavlinkReceiver rcv{(Mavlink *)context};
	rcv.Run();

	return nullptr;
}

void
MavlinkReceiver::receive_start(pthread_t *thread, Mavlink *parent)
{
	pthread_attr_t receiveloop_attr;
	pthread_attr_init(&receiveloop_attr);

	struct sched_param param;
	(void)pthread_attr_getschedparam(&receiveloop_attr, &param);
	param.sched_priority = SCHED_PRIORITY_MAX - 80;
	(void)pthread_attr_setschedparam(&receiveloop_attr, &param);

	pthread_attr_setstacksize(&receiveloop_attr,
				  PX4_STACK_ADJUSTED(sizeof(MavlinkReceiver) + 2840 + MAVLINK_RECEIVER_NET_ADDED_STACK));

	pthread_create(thread, &receiveloop_attr, MavlinkReceiver::start_helper, (void *)parent);

	pthread_attr_destroy(&receiveloop_attr);
}
