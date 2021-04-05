/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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
 * @file navigator_main.cpp
 *
 * Handles mission items, geo fencing and failsafe navigation behavior.
 * Published the position setpoint triplet for the position controller.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Jean Cyr <jean.m.cyr@gmail.com>
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

#include "navigator.h"

#include <float.h>
#include <sys/stat.h>

#include <dataman/dataman.h>
#include <drivers/drv_hrt.h>
#include <lib/ecl/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <systemlib/mavlink_log.h>

/**
 * navigator app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int navigator_main(int argc, char *argv[]);

#define GEOFENCE_CHECK_INTERVAL 200000

using namespace time_literals;

namespace navigator
{
Navigator	*g_navigator;
}

Navigator::Navigator() :
	ModuleParams(nullptr),
	_loop_perf(perf_alloc(PC_ELAPSED, "navigator")),
	_geofence(this),
	_mission(this),
	_loiter(this),
	_takeoff(this),
	_land(this),
	_precland(this),
	_rtl(this),
	_engineFailure(this),
	_gpsFailure(this),
	_follow_target(this)
{
	/* Create a list of our possible navigation types */
	_navigation_mode_array[0] = &_mission;
	_navigation_mode_array[1] = &_loiter;
	_navigation_mode_array[2] = &_rtl;
	_navigation_mode_array[3] = &_engineFailure;
	_navigation_mode_array[4] = &_gpsFailure;
	_navigation_mode_array[5] = &_takeoff;
	_navigation_mode_array[6] = &_land;
	_navigation_mode_array[7] = &_precland;
	_navigation_mode_array[8] = &_follow_target;

	_handle_back_trans_dec_mss = param_find("VT_B_DEC_MSS");
	_handle_reverse_delay = param_find("VT_B_REV_DEL");

	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));

	reset_triplets();
}

Navigator::~Navigator()
{
	perf_free(_loop_perf);
	orb_unsubscribe(_local_pos_sub);
	orb_unsubscribe(_vehicle_status_sub);
}

void
Navigator::params_update()
{
	updateParams();

	if (_handle_back_trans_dec_mss != PARAM_INVALID) {
		param_get(_handle_back_trans_dec_mss, &_param_back_trans_dec_mss);
	}

	if (_handle_reverse_delay != PARAM_INVALID) {
		param_get(_handle_reverse_delay, &_param_reverse_delay);
	}
}

void
Navigator::run()
{
	bool have_geofence_position_data = false;

	/* Try to load the geofence:
	 * if /fs/microsd/etc/geofence.txt load from this file */
	struct stat buffer;

	if (stat(GEOFENCE_FILENAME, &buffer) == 0) {
		PX4_INFO("Loading geofence from %s", GEOFENCE_FILENAME);
		_geofence.loadFromFile(GEOFENCE_FILENAME);
	}

	params_update();

	/* wakeup source(s) */
	px4_pollfd_struct_t fds[2] {};

	/* Setup of loop */
	fds[0].fd = _local_pos_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _vehicle_status_sub;
	fds[1].events = POLLIN;

	/* rate-limit position subscription to 20 Hz / 50 ms */
	orb_set_interval(_local_pos_sub, 50);

	hrt_abstime last_geofence_check = 0;

	while (!should_exit()) {

		/* wait for up to 1000ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 1000);

		if (pret == 0) {
			/* Let the loop run anyway, don't do `continue` here. */

		} else if (pret < 0) {
			/* this is undesirable but not much we can do - might want to flag unhappy status */
			PX4_ERR("poll error %d, %d", pret, errno);
			px4_usleep(10000);
			continue;

		} else {
			if (fds[0].revents & POLLIN) {
				/* success, local pos is available */
				orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
			}
		}

		perf_begin(_loop_perf);

		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vstatus);

		/* gps updated */
		if (_gps_pos_sub.updated()) {
			_gps_pos_sub.copy(&_gps_pos);

			if (_geofence.getSource() == Geofence::GF_SOURCE_GPS) {
				have_geofence_position_data = true;
			}
		}

		/* global position updated */
		if (_global_pos_sub.updated()) {
			_global_pos_sub.copy(&_global_pos);

			if (_geofence.getSource() == Geofence::GF_SOURCE_GLOBALPOS) {
				have_geofence_position_data = true;
			}
		}

		// check for parameter updates
		if (_parameter_update_sub.updated()) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			params_update();
		}

		_land_detected_sub.update(&_land_detected);
		_position_controller_status_sub.update();
		_home_pos_sub.update(&_home_pos);

		if (_vehicle_command_sub.updated()) {
			vehicle_command_s cmd{};
			_vehicle_command_sub.copy(&cmd);

			if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_GO_AROUND) {

				// DO_GO_AROUND is currently handled by the position controller (unacknowledged)
				// TODO: move DO_GO_AROUND handling to navigator
				publish_vehicle_command_ack(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);

			} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_REPOSITION) {

				position_setpoint_triplet_s *rep = get_reposition_triplet();
				position_setpoint_triplet_s *curr = get_position_setpoint_triplet();

				// store current position as previous position and goal as next
				rep->previous.yaw = get_global_position()->yaw;
				rep->previous.lat = get_global_position()->lat;
				rep->previous.lon = get_global_position()->lon;
				rep->previous.alt = get_global_position()->alt;

				rep->current.loiter_radius = get_loiter_radius();
				rep->current.loiter_direction = 1;
				rep->current.type = position_setpoint_s::SETPOINT_TYPE_LOITER;

				// If no argument for ground speed, use default value.
				if (cmd.param1 <= 0 || !PX4_ISFINITE(cmd.param1)) {
					rep->current.cruising_speed = get_cruising_speed();

				} else {
					rep->current.cruising_speed = cmd.param1;
				}

				rep->current.cruising_throttle = get_cruising_throttle();
				rep->current.acceptance_radius = get_acceptance_radius();

				// Go on and check which changes had been requested
				if (PX4_ISFINITE(cmd.param4)) {
					rep->current.yaw = cmd.param4;
					rep->current.yaw_valid = true;

				} else {
					rep->current.yaw = NAN;
					rep->current.yaw_valid = false;
				}

				if (PX4_ISFINITE(cmd.param5) && PX4_ISFINITE(cmd.param6)) {

					// Position change with optional altitude change
					rep->current.lat = cmd.param5;
					rep->current.lon = cmd.param6;

					if (PX4_ISFINITE(cmd.param7)) {
						rep->current.alt = cmd.param7;

					} else {
						rep->current.alt = get_global_position()->alt;
					}

				} else if (PX4_ISFINITE(cmd.param7) && curr->current.valid
					   && PX4_ISFINITE(curr->current.lat)
					   && PX4_ISFINITE(curr->current.lon)) {

					// Altitude without position change
					rep->current.lat = curr->current.lat;
					rep->current.lon = curr->current.lon;
					rep->current.alt = cmd.param7;

				} else {
					// All three set to NaN - hold in current position
					rep->current.lat = get_global_position()->lat;
					rep->current.lon = get_global_position()->lon;
					rep->current.alt = get_global_position()->alt;
				}

				rep->previous.valid = true;
				rep->previous.timestamp = hrt_absolute_time();

				rep->current.valid = true;
				rep->current.timestamp = hrt_absolute_time();

				rep->next.valid = false;

				// CMD_DO_REPOSITION is acknowledged by commander

			} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF) {
				position_setpoint_triplet_s *rep = get_takeoff_triplet();

				// store current position as previous position and goal as next
				rep->previous.yaw = get_local_position()->yaw;
				rep->previous.lat = get_global_position()->lat;
				rep->previous.lon = get_global_position()->lon;
				rep->previous.alt = get_global_position()->alt;

				rep->current.loiter_radius = get_loiter_radius();
				rep->current.loiter_direction = 1;
				rep->current.type = position_setpoint_s::SETPOINT_TYPE_TAKEOFF;

				if (home_position_valid()) {
					rep->current.yaw = cmd.param4;

					rep->previous.valid = true;
					rep->previous.timestamp = hrt_absolute_time();

				} else {
					rep->current.yaw = get_local_position()->yaw;
					rep->previous.valid = false;
				}

				if (PX4_ISFINITE(cmd.param5) && PX4_ISFINITE(cmd.param6)) {
					rep->current.lat = cmd.param5;
					rep->current.lon = cmd.param6;

				} else {
					// If one of them is non-finite, reset both
					rep->current.lat = (double)NAN;
					rep->current.lon = (double)NAN;
				}

				rep->current.alt = cmd.param7;

				rep->current.valid = true;
				rep->current.timestamp = hrt_absolute_time();

				rep->next.valid = false;

				// CMD_NAV_TAKEOFF is acknowledged by commander

			} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_LAND_START) {

				/* find NAV_CMD_DO_LAND_START in the mission and
				 * use MAV_CMD_MISSION_START to start the mission there
				 */
				if (_mission.land_start()) {
					vehicle_command_s vcmd = {};
					vcmd.command = vehicle_command_s::VEHICLE_CMD_MISSION_START;
					vcmd.param1 = _mission.get_land_start_index();
					publish_vehicle_cmd(&vcmd);

				} else {
					PX4_WARN("planned mission landing not available");
				}

				publish_vehicle_command_ack(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);

			} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_MISSION_START) {
				if (_mission_result.valid && PX4_ISFINITE(cmd.param1) && (cmd.param1 >= 0)) {
					if (!_mission.set_current_mission_index(cmd.param1)) {
						PX4_WARN("CMD_MISSION_START failed");
					}
				}

				// CMD_MISSION_START is acknowledged by commander

			} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_CHANGE_SPEED) {
				if (cmd.param2 > FLT_EPSILON) {
					// XXX not differentiating ground and airspeed yet
					set_cruising_speed(cmd.param2);

				} else {
					set_cruising_speed();

					/* if no speed target was given try to set throttle */
					if (cmd.param3 > FLT_EPSILON) {
						set_cruising_throttle(cmd.param3 / 100);

					} else {
						set_cruising_throttle();
					}
				}

				// TODO: handle responses for supported DO_CHANGE_SPEED options?
				publish_vehicle_command_ack(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);

			} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_SET_ROI
				   || cmd.command == vehicle_command_s::VEHICLE_CMD_NAV_ROI
				   || cmd.command == vehicle_command_s::VEHICLE_CMD_DO_SET_ROI_LOCATION
				   || cmd.command == vehicle_command_s::VEHICLE_CMD_DO_SET_ROI_WPNEXT_OFFSET
				   || cmd.command == vehicle_command_s::VEHICLE_CMD_DO_SET_ROI_NONE) {
				_vroi = {};

				switch (cmd.command) {
				case vehicle_command_s::VEHICLE_CMD_DO_SET_ROI:
				case vehicle_command_s::VEHICLE_CMD_NAV_ROI:
					_vroi.mode = cmd.param1;
					break;

				case vehicle_command_s::VEHICLE_CMD_DO_SET_ROI_LOCATION:
					_vroi.mode = vehicle_command_s::VEHICLE_ROI_LOCATION;
					_vroi.lat = cmd.param5;
					_vroi.lon = cmd.param6;
					_vroi.alt = cmd.param7;
					break;

				case vehicle_command_s::VEHICLE_CMD_DO_SET_ROI_WPNEXT_OFFSET:
					_vroi.mode = vehicle_command_s::VEHICLE_ROI_WPNEXT;
					_vroi.pitch_offset = (float)cmd.param5 * M_DEG_TO_RAD_F;
					_vroi.roll_offset = (float)cmd.param6 * M_DEG_TO_RAD_F;
					_vroi.yaw_offset = (float)cmd.param7 * M_DEG_TO_RAD_F;
					break;

				case vehicle_command_s::VEHICLE_CMD_DO_SET_ROI_NONE:
					_vroi.mode = vehicle_command_s::VEHICLE_ROI_NONE;
					break;

				default:
					_vroi.mode = vehicle_command_s::VEHICLE_ROI_NONE;
					break;
				}

				_vroi.timestamp = hrt_absolute_time();

				_vehicle_roi_pub.publish(_vroi);

				publish_vehicle_command_ack(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);
			}
		}

		/* Check for traffic */
		check_traffic();

		/* Check geofence violation */
		if (have_geofence_position_data &&
		    (_geofence.getGeofenceAction() != geofence_result_s::GF_ACTION_NONE) &&
		    (hrt_elapsed_time(&last_geofence_check) > GEOFENCE_CHECK_INTERVAL)) {

			bool inside = _geofence.check(_global_pos, _gps_pos, _home_pos,
						      home_position_valid());
			last_geofence_check = hrt_absolute_time();
			have_geofence_position_data = false;

			_geofence_result.timestamp = hrt_absolute_time();
			_geofence_result.geofence_action = _geofence.getGeofenceAction();
			_geofence_result.home_required = _geofence.isHomeRequired();

			if (!inside) {
				/* inform other apps via the mission result */
				_geofence_result.geofence_violated = true;

				/* Issue a warning about the geofence violation once */
				if (!_geofence_violation_warning_sent) {
					mavlink_log_critical(&_mavlink_log_pub, "Geofence violation");
					_geofence_violation_warning_sent = true;
				}

			} else {
				/* inform other apps via the mission result */
				_geofence_result.geofence_violated = false;

				/* Reset the _geofence_violation_warning_sent field */
				_geofence_violation_warning_sent = false;
			}

			_geofence_result_pub.publish(_geofence_result);
		}

		/* Do stuff according to navigation state set by commander */
		NavigatorMode *navigation_mode_new{nullptr};

		switch (_vstatus.nav_state) {
		case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
			_pos_sp_triplet_published_invalid_once = false;

			_mission.set_execution_mode(mission_result_s::MISSION_EXECUTION_MODE_NORMAL);
			navigation_mode_new = &_mission;

			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
			_pos_sp_triplet_published_invalid_once = false;
			navigation_mode_new = &_loiter;
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL: {
				_pos_sp_triplet_published_invalid_once = false;

				const bool rtl_activated = _previous_nav_state != vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;

				switch (rtl_type()) {
				case RTL::RTL_LAND: // use mission landing
				case RTL::RTL_CLOSEST:
					if (rtl_activated) {
						if (rtl_type() == RTL::RTL_LAND) {
							mavlink_and_console_log_info(get_mavlink_log_pub(), "RTL LAND activated");

						} else {
							mavlink_and_console_log_info(get_mavlink_log_pub(), "RTL Closest landing point activated");
						}

					}

					// if RTL is set to use a mission landing and mission has a planned landing, then use MISSION to fly there directly
					if (on_mission_landing() && !get_land_detected()->landed) {
						_mission.set_execution_mode(mission_result_s::MISSION_EXECUTION_MODE_FAST_FORWARD);
						navigation_mode_new = &_mission;

					} else {
						navigation_mode_new = &_rtl;
					}

					break;

				case RTL::RTL_MISSION:
					if (_mission.get_land_start_available() && !get_land_detected()->landed) {
						// the mission contains a landing spot
						_mission.set_execution_mode(mission_result_s::MISSION_EXECUTION_MODE_FAST_FORWARD);

						if (_navigation_mode != &_mission) {
							if (_navigation_mode == nullptr) {
								// switching from an manual mode, go to landing if not already landing
								if (!on_mission_landing()) {
									start_mission_landing();
								}

							} else {
								// switching from an auto mode, continue the mission from the closest item
								_mission.set_closest_item_as_current();
							}
						}

						if (rtl_activated) {
							mavlink_and_console_log_info(get_mavlink_log_pub(), "RTL Mission activated, continue mission");
						}

						navigation_mode_new = &_mission;

					} else {
						// fly the mission in reverse if switching from a non-manual mode
						_mission.set_execution_mode(mission_result_s::MISSION_EXECUTION_MODE_REVERSE);

						if ((_navigation_mode != nullptr && (_navigation_mode != &_rtl || _mission.get_mission_changed())) &&
						    (! _mission.get_mission_finished()) &&
						    (!get_land_detected()->landed)) {
							// determine the closest mission item if switching from a non-mission mode, and we are either not already
							// mission mode or the mission waypoints changed.
							// The seconds condition is required so that when no mission was uploaded and one is available the closest
							// mission item is determined and also that if the user changes the active mission index while rtl is active
							// always that waypoint is tracked first.
							if ((_navigation_mode != &_mission) && (rtl_activated || _mission.get_mission_waypoints_changed())) {
								_mission.set_closest_item_as_current();
							}

							if (rtl_activated) {
								mavlink_and_console_log_info(get_mavlink_log_pub(), "RTL Mission activated, fly mission in reverse");
							}

							navigation_mode_new = &_mission;

						} else {
							if (rtl_activated) {
								mavlink_and_console_log_info(get_mavlink_log_pub(), "RTL Mission activated, fly to home");
							}

							navigation_mode_new = &_rtl;
						}
					}

					break;

				default:
					if (rtl_activated) {
						mavlink_and_console_log_info(get_mavlink_log_pub(), "RTL HOME activated");
					}

					navigation_mode_new = &_rtl;
					break;

				}

				break;
			}

		case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
			_pos_sp_triplet_published_invalid_once = false;
			navigation_mode_new = &_takeoff;
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:
			_pos_sp_triplet_published_invalid_once = false;
			navigation_mode_new = &_land;
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND:
			_pos_sp_triplet_published_invalid_once = false;
			navigation_mode_new = &_precland;
			_precland.set_mode(PrecLandMode::Required);
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL:
			_pos_sp_triplet_published_invalid_once = false;
			navigation_mode_new = &_engineFailure;
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_LANDGPSFAIL:
			_pos_sp_triplet_published_invalid_once = false;
			navigation_mode_new = &_gpsFailure;
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
			_pos_sp_triplet_published_invalid_once = false;
			navigation_mode_new = &_follow_target;
			break;

		case vehicle_status_s::NAVIGATION_STATE_MANUAL:
		case vehicle_status_s::NAVIGATION_STATE_ACRO:
		case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
		case vehicle_status_s::NAVIGATION_STATE_POSCTL:
		case vehicle_status_s::NAVIGATION_STATE_DESCEND:
		case vehicle_status_s::NAVIGATION_STATE_TERMINATION:
		case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
		case vehicle_status_s::NAVIGATION_STATE_STAB:
		default:
			navigation_mode_new = nullptr;
			_can_loiter_at_sp = false;
			break;
		}

		// Do not execute any state machine while we are disarmed
		if (_vstatus.arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
			navigation_mode_new = nullptr;
		}

		// update the vehicle status
		_previous_nav_state = _vstatus.nav_state;

		/* we have a new navigation mode: reset triplet */
		if (_navigation_mode != navigation_mode_new) {
			// We don't reset the triplet if we just did an auto-takeoff and are now
			// going to loiter. Otherwise, we lose the takeoff altitude and end up lower
			// than where we wanted to go.
			//
			// FIXME: a better solution would be to add reset where they are needed and remove
			//        this general reset here.
			if (!(_navigation_mode == &_takeoff &&
			      navigation_mode_new == &_loiter)) {
				reset_triplets();
			}
		}

		_navigation_mode = navigation_mode_new;

		/* iterate through navigation modes and set active/inactive for each */
		for (unsigned int i = 0; i < NAVIGATOR_MODE_ARRAY_SIZE; i++) {
			_navigation_mode_array[i]->run(_navigation_mode == _navigation_mode_array[i]);
		}

		/* if nothing is running, set position setpoint triplet invalid once */
		if (_navigation_mode == nullptr && !_pos_sp_triplet_published_invalid_once) {
			_pos_sp_triplet_published_invalid_once = true;
			reset_triplets();
		}

		if (_pos_sp_triplet_updated) {
			publish_position_setpoint_triplet();
		}

		if (_mission_result_updated) {
			publish_mission_result();
		}

		perf_end(_loop_perf);
	}
}

int Navigator::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("navigator",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_NAVIGATION,
				      1800,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

Navigator *Navigator::instantiate(int argc, char *argv[])
{
	Navigator *instance = new Navigator();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

int
Navigator::print_status()
{
	PX4_INFO("Running");

	_geofence.printStatus();
	return 0;
}

void
Navigator::publish_position_setpoint_triplet()
{
	_pos_sp_triplet.timestamp = hrt_absolute_time();
	_pos_sp_triplet_pub.publish(_pos_sp_triplet);
	_pos_sp_triplet_updated = false;
}

float
Navigator::get_default_acceptance_radius()
{
	return _param_nav_acc_rad.get();
}

float
Navigator::get_acceptance_radius()
{
	return get_acceptance_radius(_param_nav_acc_rad.get());
}

float
Navigator::get_default_altitude_acceptance_radius()
{
	if (get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
		return _param_nav_fw_alt_rad.get();

	} else if (get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROVER) {
		return INFINITY;

	} else {
		float alt_acceptance_radius = _param_nav_mc_alt_rad.get();

		const position_controller_status_s &pos_ctrl_status = _position_controller_status_sub.get();

		if ((pos_ctrl_status.timestamp > _pos_sp_triplet.timestamp)
		    && pos_ctrl_status.altitude_acceptance > alt_acceptance_radius) {
			alt_acceptance_radius = pos_ctrl_status.altitude_acceptance;
		}

		return alt_acceptance_radius;
	}
}

float
Navigator::get_altitude_acceptance_radius()
{
	if (get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
		const position_setpoint_s &next_sp = get_position_setpoint_triplet()->next;

		if (next_sp.type == position_setpoint_s::SETPOINT_TYPE_LAND && next_sp.valid) {
			// Use separate (tighter) altitude acceptance for clean altitude starting point before landing
			return _param_nav_fw_altl_rad.get();
		}
	}

	return get_default_altitude_acceptance_radius();
}

float
Navigator::get_cruising_speed()
{
	/* there are three options: The mission-requested cruise speed, or the current hover / plane speed */
	if (_vstatus.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
		if (is_planned_mission() && _mission_cruising_speed_mc > 0.0f) {
			return _mission_cruising_speed_mc;

		} else {
			return -1.0f;
		}

	} else {
		if (is_planned_mission() && _mission_cruising_speed_fw > 0.0f) {
			return _mission_cruising_speed_fw;

		} else {
			return -1.0f;
		}
	}
}

void
Navigator::set_cruising_speed(float speed)
{
	if (_vstatus.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
		_mission_cruising_speed_mc = speed;

	} else {
		_mission_cruising_speed_fw = speed;
	}
}

void
Navigator::reset_cruising_speed()
{
	_mission_cruising_speed_mc = -1.0f;
	_mission_cruising_speed_fw = -1.0f;
}

void
Navigator::reset_triplets()
{
	reset_position_setpoint(_pos_sp_triplet.previous);
	reset_position_setpoint(_pos_sp_triplet.current);
	reset_position_setpoint(_pos_sp_triplet.next);

	_pos_sp_triplet_updated = true;
}

void
Navigator::reset_position_setpoint(position_setpoint_s &sp)
{
	sp = position_setpoint_s{};
	sp.timestamp = hrt_absolute_time();
	sp.lat = static_cast<double>(NAN);
	sp.lon = static_cast<double>(NAN);;
	sp.loiter_radius = get_loiter_radius();
	sp.acceptance_radius = get_default_acceptance_radius();
	sp.cruising_speed = get_cruising_speed();
	sp.cruising_throttle = get_cruising_throttle();
	sp.valid = false;
	sp.type = position_setpoint_s::SETPOINT_TYPE_IDLE;
	sp.disable_weather_vane = true;
}

float
Navigator::get_cruising_throttle()
{
	/* Return the mission-requested cruise speed, or default FW_THR_CRUISE value */
	if (_mission_throttle > FLT_EPSILON) {
		return _mission_throttle;

	} else {
		return -1.0f;
	}
}

float
Navigator::get_acceptance_radius(float mission_item_radius)
{
	float radius = mission_item_radius;

	// XXX only use navigation capabilities for now
	// when in fixed wing mode
	// this might need locking against a commanded transition
	// so that a stale _vstatus doesn't trigger an accepted mission item.

	const position_controller_status_s &pos_ctrl_status = _position_controller_status_sub.get();

	if (_vstatus.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
	    && (pos_ctrl_status.timestamp > _pos_sp_triplet.timestamp)
	    && pos_ctrl_status.acceptance_radius > radius) {
		radius = pos_ctrl_status.acceptance_radius;
	}

	return radius;
}

float
Navigator::get_yaw_acceptance(float mission_item_yaw)
{
	float yaw = mission_item_yaw;

	const position_controller_status_s &pos_ctrl_status = _position_controller_status_sub.get();

	// if yaw_acceptance from position controller is NaN overwrite the mission item yaw such that
	// the waypoint can be reached from any direction
	if ((pos_ctrl_status.timestamp > _pos_sp_triplet.timestamp) && !PX4_ISFINITE(pos_ctrl_status.yaw_acceptance)) {
		yaw = pos_ctrl_status.yaw_acceptance;
	}

	return yaw;
}

void
Navigator::load_fence_from_file(const char *filename)
{
	_geofence.loadFromFile(filename);
}

/**
 * Creates a fake traffic measurement with supplied parameters.
 *
 */
void Navigator::fake_traffic(const char *callsign, float distance, float direction, float traffic_heading,
			     float altitude_diff, float hor_velocity, float ver_velocity, int emitter_type)
{
	double lat, lon;
	waypoint_from_heading_and_distance(get_global_position()->lat, get_global_position()->lon, direction, distance, &lat,
					   &lon);
	float alt = get_global_position()->alt + altitude_diff;

	// float vel_n = get_global_position()->vel_n;
	// float vel_e = get_global_position()->vel_e;
	// float vel_d = get_global_position()->vel_d;

	transponder_report_s tr{};
	tr.timestamp = hrt_absolute_time();
	tr.icao_address = 1234;
	tr.lat = lat; // Latitude, expressed as degrees
	tr.lon = lon; // Longitude, expressed as degrees
	tr.altitude_type = 0;
	tr.altitude = alt;
	tr.heading = traffic_heading; //-atan2(vel_e, vel_n); // Course over ground in radians
	tr.hor_velocity	= hor_velocity; //sqrtf(vel_e * vel_e + vel_n * vel_n); // The horizontal velocity in m/s
	tr.ver_velocity = ver_velocity; //-vel_d; // The vertical velocity in m/s, positive is up
	strncpy(&tr.callsign[0], callsign, sizeof(tr.callsign) - 1);
	tr.callsign[sizeof(tr.callsign) - 1] = 0;
	tr.emitter_type = emitter_type; // Type from ADSB_EMITTER_TYPE enum
	tr.tslc = 2; // Time since last communication in seconds
	tr.flags = transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS | transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING |
		   transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY |
		   transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE |
		   (transponder_report_s::ADSB_EMITTER_TYPE_UAV & emitter_type ? 0 :
		    transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN); // Flags to indicate various statuses including valid data fields
	tr.squawk = 6667;




#ifndef BOARD_HAS_NO_UUID
	px4_guid_t px4_guid;
	board_get_px4_guid(px4_guid);
	memcpy(tr.uas_id, px4_guid, sizeof(px4_guid_t)); //simulate own GUID
#else

	for (int i = 0; i < PX4_GUID_BYTE_LENGTH ; i++) {
		tr.uas_id[i] = 0xe0 + i; //simulate GUID
	}

#endif /* BOARD_HAS_NO_UUID */

	uORB::PublicationQueued<transponder_report_s> tr_pub{ORB_ID(transponder_report)};
	tr_pub.publish(tr);
}

void Navigator::check_traffic()
{
	double lat = get_global_position()->lat;
	double lon = get_global_position()->lon;
	float alt = get_global_position()->alt;

	// TODO for non-multirotors predicting the future
	// position as accurately as possible will become relevant
	// float vel_n = get_global_position()->vel_n;
	// float vel_e = get_global_position()->vel_e;
	// float vel_d = get_global_position()->vel_d;

	bool changed = _traffic_sub.updated();

	char uas_id[11]; //GUID of incoming UTM messages

	float NAVTrafficAvoidUnmanned = _param_nav_traff_a_radu.get();
	float NAVTrafficAvoidManned = _param_nav_traff_a_radm.get();
	float horizontal_separation = NAVTrafficAvoidManned;
	float vertical_separation = NAVTrafficAvoidManned;

	while (changed) {

		//vehicle_status_s vs{};
		transponder_report_s tr{};
		_traffic_sub.copy(&tr);

		uint16_t required_flags = transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS |
					  transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING |
					  transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY | transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE;

		if ((tr.flags & required_flags) != required_flags) {
			changed = _traffic_sub.updated();
			continue;
		}

		//convert UAS_id byte array to char array for User Warning
		for (int i = 0; i < 5; i++) {
			snprintf(&uas_id[i * 2], sizeof(uas_id) - i * 2, "%02x", tr.uas_id[PX4_GUID_BYTE_LENGTH - 5 + i]);
		}

		//Manned/Unmanned Vehicle Seperation Distance
		if (tr.emitter_type == transponder_report_s::ADSB_EMITTER_TYPE_UAV) {
			horizontal_separation = NAVTrafficAvoidUnmanned;
			vertical_separation = NAVTrafficAvoidUnmanned;
		}

		float d_hor, d_vert;
		get_distance_to_point_global_wgs84(lat, lon, alt,
						   tr.lat, tr.lon, tr.altitude, &d_hor, &d_vert);


		// predict final altitude (positive is up) in prediction time frame
		float end_alt = tr.altitude + (d_vert / tr.hor_velocity) * tr.ver_velocity;

		// Predict until the vehicle would have passed this system at its current speed
		float prediction_distance = d_hor + 1000.0f;

		// If the altitude is not getting close to us, do not calculate
		// the horizontal separation.
		// Since commercial flights do most of the time keep flight levels
		// check for the current and for the predicted flight level.
		// we also make the implicit assumption that this system is on the lowest
		// flight level close to ground in the
		// (end_alt - horizontal_separation < alt) condition. If this system should
		// ever be used in normal airspace this implementation would anyway be
		// inappropriate as it should be replaced with a TCAS compliant solution.

		if ((fabsf(alt - tr.altitude) < vertical_separation) || ((end_alt - horizontal_separation) < alt)) {

			double end_lat, end_lon;
			waypoint_from_heading_and_distance(tr.lat, tr.lon, tr.heading, prediction_distance, &end_lat, &end_lon);

			struct crosstrack_error_s cr;

			if (!get_distance_to_line(&cr, lat, lon, tr.lat, tr.lon, end_lat, end_lon)) {

				if (!cr.past_end && (fabsf(cr.distance) < horizontal_separation)) {

					// direction of traffic in human-readable 0..360 degree in earth frame
					int traffic_direction = math::degrees(tr.heading) + 180;
					int traffic_seperation = (int)fabsf(cr.distance);

					switch (_param_nav_traff_avoid.get()) {

					case 0: {
							/* Ignore */
							PX4_WARN("TRAFFIC %s! dst %d, hdg %d",
								 tr.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN ? tr.callsign : uas_id,
								 traffic_seperation,
								 traffic_direction);
							break;
						}

					case 1: {
							/* Warn only */
							mavlink_log_critical(&_mavlink_log_pub, "Warning TRAFFIC %s! dst %d, hdg %d",
									     tr.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN ? tr.callsign : uas_id,
									     traffic_seperation,
									     traffic_direction);
							break;
						}

					case 2: {
							/* RTL Mode */
							mavlink_log_critical(&_mavlink_log_pub, "TRAFFIC: %s Returning home! dst %d, hdg %d",
									     tr.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN ? tr.callsign : uas_id,
									     traffic_seperation,
									     traffic_direction);

							// set the return altitude to minimum
							_rtl.set_return_alt_min(true);

							// ask the commander to execute an RTL
							vehicle_command_s vcmd = {};
							vcmd.command = vehicle_command_s::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH;
							publish_vehicle_cmd(&vcmd);
							break;
						}

					case 3: {
							/* Land Mode */
							mavlink_log_critical(&_mavlink_log_pub, "TRAFFIC: %s Landing! dst %d, hdg % d",
									     tr.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN ? tr.callsign : uas_id,
									     traffic_seperation,
									     traffic_direction);

							// ask the commander to land
							vehicle_command_s vcmd = {};
							vcmd.command = vehicle_command_s::VEHICLE_CMD_NAV_LAND;
							publish_vehicle_cmd(&vcmd);
							break;

						}

					case 4: {
							/* Position hold */
							mavlink_log_critical(&_mavlink_log_pub, "TRAFFIC: %s Holding position! dst %d, hdg %d",
									     tr.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN ? tr.callsign : uas_id,
									     traffic_seperation,
									     traffic_direction);

							// ask the commander to Loiter
							vehicle_command_s vcmd = {};
							vcmd.command = vehicle_command_s::VEHICLE_CMD_NAV_LOITER_UNLIM;
							publish_vehicle_cmd(&vcmd);
							break;

						}
					}
				}
			}
		}

		changed = _traffic_sub.updated();
	}
}

bool
Navigator::abort_landing()
{
	// only abort if currently landing and position controller status updated
	bool should_abort = false;

	if (_pos_sp_triplet.current.valid
	    && _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {

		if (_pos_ctrl_landing_status_sub.updated()) {
			position_controller_landing_status_s landing_status{};

			// landing status from position controller must be newer than navigator's last position setpoint
			if (_pos_ctrl_landing_status_sub.copy(&landing_status)) {
				if (landing_status.timestamp > _pos_sp_triplet.timestamp) {
					should_abort = landing_status.abort_landing;
				}
			}
		}
	}

	return should_abort;
}

bool
Navigator::force_vtol()
{
	return _vstatus.is_vtol &&
	       (_vstatus.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING || _vstatus.in_transition_to_fw)
	       && _param_nav_force_vt.get();
}

int Navigator::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	if (!strcmp(argv[0], "fencefile")) {
		get_instance()->load_fence_from_file(GEOFENCE_FILENAME);
		return 0;

	} else if (!strcmp(argv[0], "fake_traffic")) {
		get_instance()->fake_traffic("LX007", 500, 1.0f, -1.0f, 100.0f, 90.0f, 0.001f,
					     transponder_report_s::ADSB_EMITTER_TYPE_LIGHT);
		get_instance()->fake_traffic("LX55", 1000, 0, 0, 100.0f, 90.0f, 0.001f, transponder_report_s::ADSB_EMITTER_TYPE_SMALL);
		get_instance()->fake_traffic("LX20", 15000, 1.0f, -1.0f, 280.0f, 90.0f, 0.001f,
					     transponder_report_s::ADSB_EMITTER_TYPE_LARGE);
		get_instance()->fake_traffic("UAV", 10, 1.0f, -2.0f, 10.0f, 10.0f, 0.01f, transponder_report_s::ADSB_EMITTER_TYPE_UAV);
		return 0;
	}

	return print_usage("unknown command");
}

int navigator_main(int argc, char *argv[])
{
	return Navigator::main(argc, argv);
}

void
Navigator::publish_mission_result()
{
	_mission_result.timestamp = hrt_absolute_time();

	/* lazily publish the mission result only once available */
	_mission_result_pub.publish(_mission_result);

	/* reset some of the flags */
	_mission_result.item_do_jump_changed = false;
	_mission_result.item_changed_index = 0;
	_mission_result.item_do_jump_remaining = 0;

	_mission_result_updated = false;
}

void
Navigator::set_mission_failure(const char *reason)
{
	if (!_mission_result.failure) {
		_mission_result.failure = true;
		set_mission_result_updated();
		mavlink_log_critical(&_mavlink_log_pub, "%s", reason);
	}
}

void
Navigator::publish_vehicle_cmd(vehicle_command_s *vcmd)
{
	vcmd->timestamp = hrt_absolute_time();
	vcmd->source_system = _vstatus.system_id;
	vcmd->source_component = _vstatus.component_id;
	vcmd->target_system = _vstatus.system_id;
	vcmd->confirmation = false;
	vcmd->from_external = false;

	// The camera commands are not processed on the autopilot but will be
	// sent to the mavlink links to other components.
	switch (vcmd->command) {
	case NAV_CMD_IMAGE_START_CAPTURE:
	case NAV_CMD_IMAGE_STOP_CAPTURE:
	case NAV_CMD_VIDEO_START_CAPTURE:
	case NAV_CMD_VIDEO_STOP_CAPTURE:
		vcmd->target_component = 100; // MAV_COMP_ID_CAMERA
		break;

	default:
		vcmd->target_component = _vstatus.component_id;
		break;
	}

	_vehicle_cmd_pub.publish(*vcmd);
}

void
Navigator::publish_vehicle_command_ack(const vehicle_command_s &cmd, uint8_t result)
{
	vehicle_command_ack_s command_ack = {};

	command_ack.timestamp = hrt_absolute_time();
	command_ack.command = cmd.command;
	command_ack.target_system = cmd.source_system;
	command_ack.target_component = cmd.source_component;
	command_ack.from_external = false;

	command_ack.result = result;
	command_ack.result_param1 = 0;
	command_ack.result_param2 = 0;

	_vehicle_cmd_ack_pub.publish(command_ack);
}

int Navigator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Module that is responsible for autonomous flight modes. This includes missions (read from dataman),
takeoff and RTL.
It is also responsible for geofence violation checking.

### Implementation
The different internal modes are implemented as separate classes that inherit from a common base class `NavigatorMode`.
The member `_navigation_mode` contains the current active mode.

Navigator publishes position setpoint triplets (`position_setpoint_triplet_s`), which are then used by the position
controller.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("navigator", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("fencefile", "load a geofence file from SD card, stored at etc/geofence.txt");
	PRINT_MODULE_USAGE_COMMAND_DESCR("fake_traffic", "publishes 4 fake transponder_report_s uORB messages");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}
