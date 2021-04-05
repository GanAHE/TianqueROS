/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
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
 * @file navigator_mission.cpp
 *
 * Helper class to access missions
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Ban Siesta <bansiesta@gmail.com>
 * @author Simon Wilks <simon@uaventure.com>
 * @author Andreas Antener <andreas@uaventure.com>
 * @author Sander Smeets <sander@droneslab.com>
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include "mission.h"
#include "navigator.h"

#include <string.h>
#include <drivers/drv_hrt.h>
#include <dataman/dataman.h>
#include <systemlib/mavlink_log.h>
#include <systemlib/err.h>
#include <lib/ecl/geo/geo.h>
#include <navigator/navigation.h>
#include <uORB/uORB.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/mission_result.h>

Mission::Mission(Navigator *navigator) :
	MissionBlock(navigator),
	ModuleParams(navigator)
{
}

void
Mission::on_inactive()
{
	/* We need to reset the mission cruising speed, otherwise the
	 * mission velocity which might have been set using mission items
	 * is used for missions such as RTL. */
	_navigator->set_cruising_speed();

	/* Without home a mission can't be valid yet anyway, let's wait. */
	if (!_navigator->home_position_valid()) {
		return;
	}

	if (_inited) {
		if (_mission_sub.updated()) {
			update_mission();

			if (_mission_type == MISSION_TYPE_NONE && _mission.count > 0) {
				_mission_type = MISSION_TYPE_MISSION;
			}
		}

		/* reset the current mission if needed */
		if (need_to_reset_mission()) {
			reset_mission(_mission);
			update_mission();
			_navigator->reset_cruising_speed();
		}

	} else {

		/* load missions from storage */
		mission_s mission_state = {};

		dm_lock(DM_KEY_MISSION_STATE);

		/* read current state */
		int read_res = dm_read(DM_KEY_MISSION_STATE, 0, &mission_state, sizeof(mission_s));

		dm_unlock(DM_KEY_MISSION_STATE);

		if (read_res == sizeof(mission_s)) {
			_mission.dataman_id = mission_state.dataman_id;
			_mission.count = mission_state.count;
			_current_mission_index = mission_state.current_seq;

			// find and store landing start marker (if available)
			find_mission_land_start();
		}

		/* On init let's check the mission, maybe there is already one available. */
		check_mission_valid(false);

		_inited = true;
	}

	/* require takeoff after non-loiter or landing */
	if (!_navigator->get_can_loiter_at_sp() || _navigator->get_land_detected()->landed) {
		_need_takeoff = true;
	}

	/* reset so current mission item gets restarted if mission was paused */
	_work_item_type = WORK_ITEM_TYPE_DEFAULT;

	/* reset so MISSION_ITEM_REACHED isn't published */
	_navigator->get_mission_result()->seq_reached = -1;
}

void
Mission::on_inactivation()
{
	// Disable camera trigger
	vehicle_command_s cmd = {};
	cmd.command = vehicle_command_s::VEHICLE_CMD_DO_TRIGGER_CONTROL;
	// Pause trigger
	cmd.param1 = -1.0f;
	cmd.param3 = 1.0f;
	_navigator->publish_vehicle_cmd(&cmd);
}

void
Mission::on_activation()
{
	if (_mission_waypoints_changed) {
		// do not set the closest mission item in the normal mission mode
		if (_mission_execution_mode != mission_result_s::MISSION_EXECUTION_MODE_NORMAL) {
			_current_mission_index = index_closest_mission_item();
		}

		_mission_waypoints_changed = false;
	}

	// we already reset the mission items
	_execution_mode_changed = false;

	set_mission_items();

	// unpause triggering if it was paused
	vehicle_command_s cmd = {};
	cmd.command = vehicle_command_s::VEHICLE_CMD_DO_TRIGGER_CONTROL;
	// unpause trigger
	cmd.param1 = -1.0f;
	cmd.param3 = 0.0f;
	_navigator->publish_vehicle_cmd(&cmd);
}

void
Mission::on_active()
{
	if (_work_item_type == WORK_ITEM_TYPE_PRECISION_LAND) {
		// switch out of precision land once landed
		if (_navigator->get_land_detected()->landed) {
			_navigator->get_precland()->on_inactivation();
			_work_item_type = WORK_ITEM_TYPE_DEFAULT;

		} else {
			_navigator->get_precland()->on_active();
		}
	}

	check_mission_valid(false);

	/* check if anything has changed */
	bool mission_sub_updated = _mission_sub.updated();

	if (mission_sub_updated) {
		_navigator->reset_triplets();
		update_mission();
	}

	/* mission is running (and we are armed), need reset after disarm */
	_need_mission_reset = true;

	_mission_changed = false;

	/* reset mission items if needed */
	if (mission_sub_updated || _mission_waypoints_changed || _execution_mode_changed) {
		if (_mission_waypoints_changed) {
			// do not set the closest mission item in the normal mission mode
			if (_mission_execution_mode != mission_result_s::MISSION_EXECUTION_MODE_NORMAL) {
				_current_mission_index = index_closest_mission_item();
			}

			_mission_waypoints_changed = false;
		}

		_execution_mode_changed = false;
		set_mission_items();
	}

	/* lets check if we reached the current mission item */
	if (_mission_type != MISSION_TYPE_NONE && is_mission_item_reached()) {
		/* If we just completed a takeoff which was inserted before the right waypoint,
		   there is no need to report that we reached it because we didn't. */
		if (_work_item_type != WORK_ITEM_TYPE_TAKEOFF) {
			set_mission_item_reached();
		}

		if (_mission_item.autocontinue) {
			/* switch to next waypoint if 'autocontinue' flag set */
			advance_mission();
			set_mission_items();
		}

	} else if (_mission_type != MISSION_TYPE_NONE && _param_mis_altmode.get() == MISSION_ALTMODE_FOH) {

		altitude_sp_foh_update();

	} else {
		/* if waypoint position reached allow loiter on the setpoint */
		if (_waypoint_position_reached && _mission_item.nav_cmd != NAV_CMD_IDLE) {
			_navigator->set_can_loiter_at_sp(true);
		}
	}

	/* check if a cruise speed change has been commanded */
	if (_mission_type != MISSION_TYPE_NONE) {
		cruising_speed_sp_update();
	}

	/* see if we need to update the current yaw heading */
	if (!_param_mis_mnt_yaw_ctl.get()
	    && (_navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING)
	    && (_navigator->get_vroi().mode != vehicle_roi_s::ROI_NONE)
	    && !(_mission_item.nav_cmd == NAV_CMD_TAKEOFF
		 || _mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF
		 || _mission_item.nav_cmd == NAV_CMD_DO_VTOL_TRANSITION
		 || _mission_item.nav_cmd == NAV_CMD_LAND
		 || _mission_item.nav_cmd == NAV_CMD_VTOL_LAND
		 || _work_item_type == WORK_ITEM_TYPE_ALIGN)) {
		// Mount control is disabled If the vehicle is in ROI-mode, the vehicle
		// needs to rotate such that ROI is in the field of view.
		// ROI only makes sense for multicopters.
		heading_sp_update();
	}

	// TODO: Add vtol heading update method if required.
	// Question: Why does vtol ever have to update heading?

	/* check if landing needs to be aborted */
	if ((_mission_item.nav_cmd == NAV_CMD_LAND)
	    && (_navigator->abort_landing())) {

		do_abort_landing();
	}
}

bool
Mission::set_current_mission_index(uint16_t index)
{
	if (_navigator->get_mission_result()->valid &&
	    (index != _current_mission_index) && (index < _mission.count)) {

		_current_mission_index = index;

		// a mission index is set manually which has the higher priority than the closest mission item
		// as it is set by the user
		_mission_waypoints_changed = false;

		// update mission items if already in active mission
		if (_navigator->is_planned_mission()) {
			// prevent following "previous - current" line
			_navigator->get_position_setpoint_triplet()->previous.valid = false;
			_navigator->get_position_setpoint_triplet()->current.valid = false;
			_navigator->get_position_setpoint_triplet()->next.valid = false;
			set_mission_items();
		}

		return true;
	}

	return false;
}

void
Mission::set_closest_item_as_current()
{
	_current_mission_index = index_closest_mission_item();
}

void
Mission::set_execution_mode(const uint8_t mode)
{
	if (_mission_execution_mode != mode) {
		_execution_mode_changed = true;
		_navigator->get_mission_result()->execution_mode = mode;


		switch (_mission_execution_mode) {
		case mission_result_s::MISSION_EXECUTION_MODE_NORMAL:
		case mission_result_s::MISSION_EXECUTION_MODE_FAST_FORWARD:
			if (mode == mission_result_s::MISSION_EXECUTION_MODE_REVERSE) {
				// command a transition if in vtol mc mode
				if (_navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING &&
				    _navigator->get_vstatus()->is_vtol &&
				    !_navigator->get_land_detected()->landed) {

					set_vtol_transition_item(&_mission_item, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);

					position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
					pos_sp_triplet->previous = pos_sp_triplet->current;
					generate_waypoint_from_heading(&pos_sp_triplet->current, _mission_item.yaw);
					_navigator->set_position_setpoint_triplet_updated();
					issue_command(_mission_item);
				}

				if (_mission_type == MISSION_TYPE_NONE && _mission.count > 0) {
					_mission_type = MISSION_TYPE_MISSION;
				}

				if (_current_mission_index > _mission.count - 1) {
					_current_mission_index = _mission.count - 1;

				} else if (_current_mission_index > 0) {
					--_current_mission_index;
				}

				_work_item_type = WORK_ITEM_TYPE_DEFAULT;
			}

			break;

		case mission_result_s::MISSION_EXECUTION_MODE_REVERSE:
			if ((mode == mission_result_s::MISSION_EXECUTION_MODE_NORMAL) ||
			    (mode == mission_result_s::MISSION_EXECUTION_MODE_FAST_FORWARD)) {
				// handle switch from reverse to forward mission
				if (_current_mission_index < 0) {
					_current_mission_index = 0;

				} else if (_current_mission_index < _mission.count - 1) {
					++_current_mission_index;
				}

				_work_item_type = WORK_ITEM_TYPE_DEFAULT;
			}

			break;

		}

		_mission_execution_mode = mode;
	}
}

bool
Mission::find_mission_land_start()
{
	/* return true if a MAV_CMD_DO_LAND_START, NAV_CMD_VTOL_LAND or NAV_CMD_LAND is found and internally save the index
	 *  return false if not found
	 */

	const dm_item_t dm_current = (dm_item_t)_mission.dataman_id;
	struct mission_item_s missionitem = {};
	struct mission_item_s missionitem_prev = {}; //to store mission item before currently checked on, needed to get pos of wp before NAV_CMD_DO_LAND_START

	for (size_t i = 1; i < _mission.count; i++) {
		const ssize_t len = sizeof(missionitem);
		missionitem_prev = missionitem; // store the last mission item before reading a new one

		if (dm_read(dm_current, i, &missionitem, len) != len) {
			/* not supposed to happen unless the datamanager can't access the SD card, etc. */
			PX4_ERR("dataman read failure");
			break;
		}

		// first check for DO_LAND_START marker
		if ((missionitem.nav_cmd == NAV_CMD_DO_LAND_START) && (missionitem_prev.nav_cmd == NAV_CMD_WAYPOINT)) {

			_land_start_available = true;
			_land_start_index = i;
			// the DO_LAND_START marker contains no position sp, so take them from the previous mission item
			_landing_lat = missionitem_prev.lat;
			_landing_lon = missionitem_prev.lon;
			_landing_alt = missionitem_prev.altitude;
			return true;

			// if no DO_LAND_START marker available, also check for VTOL_LAND or normal LAND

		} else if (((missionitem.nav_cmd == NAV_CMD_VTOL_LAND) && _navigator->get_vstatus()->is_vtol) ||
			   (missionitem.nav_cmd == NAV_CMD_LAND)) {

			_land_start_available = true;
			_land_start_index = i;
			_landing_lat = missionitem.lat;
			_landing_lon = missionitem.lon;
			_landing_alt = missionitem.altitude;
			return true;
		}
	}

	_land_start_available = false;
	return false;
}

bool
Mission::land_start()
{
	// if not currently landing, jump to do_land_start
	if (_land_start_available) {
		if (landing()) {
			return true;

		} else {
			set_current_mission_index(get_land_start_index());
			return landing();
		}
	}

	return false;
}

bool
Mission::landing()
{
	// vehicle is currently landing if
	//  mission valid, still flying, and in the landing portion of mission

	const bool mission_valid = _navigator->get_mission_result()->valid;
	const bool on_landing_stage = _land_start_available && (_current_mission_index >= get_land_start_index());

	return mission_valid && on_landing_stage;
}

void
Mission::update_mission()
{

	bool failed = true;

	/* Reset vehicle_roi
	 * Missions that do not explicitly configure ROI would not override
	 * an existing ROI setting from previous missions */
	_navigator->reset_vroi();

	const mission_s old_mission = _mission;

	if (_mission_sub.copy(&_mission)) {
		/* determine current index */
		if (_mission.current_seq >= 0 && _mission.current_seq < (int)_mission.count) {
			_current_mission_index = _mission.current_seq;

		} else {
			/* if less items available, reset to first item */
			if (_current_mission_index >= (int)_mission.count) {
				_current_mission_index = 0;

			} else if (_current_mission_index < 0) {
				/* if not initialized, set it to 0 */
				_current_mission_index = 0;
			}

			/* otherwise, just leave it */
		}

		check_mission_valid(true);

		failed = !_navigator->get_mission_result()->valid;

		if (!failed) {
			/* reset mission failure if we have an updated valid mission */
			_navigator->get_mission_result()->failure = false;

			/* reset sequence info as well */
			_navigator->get_mission_result()->seq_reached = -1;
			_navigator->get_mission_result()->seq_total = _mission.count;

			/* reset work item if new mission has been accepted */
			_work_item_type = WORK_ITEM_TYPE_DEFAULT;
			_mission_changed = true;
		}

		/* check if the mission waypoints changed while the vehicle is in air
		 * TODO add a flag to mission_s which actually tracks if the position of the waypoint changed */
		if (((_mission.count != old_mission.count) ||
		     (_mission.dataman_id != old_mission.dataman_id)) &&
		    !_navigator->get_land_detected()->landed) {
			_mission_waypoints_changed = true;
		}

	} else {
		PX4_ERR("mission update failed");
	}

	if (failed) {
		// only warn if the check failed on merit
		if ((int)_mission.count > 0) {
			PX4_WARN("mission check failed");
		}

		// reset the mission
		_mission.count = 0;
		_mission.current_seq = 0;
		_current_mission_index = 0;
	}

	// find and store landing start marker (if available)
	find_mission_land_start();

	set_current_mission_item();
}


void
Mission::advance_mission()
{
	/* do not advance mission item if we're processing sub mission work items */
	if (_work_item_type != WORK_ITEM_TYPE_DEFAULT) {
		return;
	}

	switch (_mission_type) {
	case MISSION_TYPE_MISSION:
		switch (_mission_execution_mode) {
		case mission_result_s::MISSION_EXECUTION_MODE_NORMAL:
		case mission_result_s::MISSION_EXECUTION_MODE_FAST_FORWARD: {
				_current_mission_index++;
				break;
			}

		case mission_result_s::MISSION_EXECUTION_MODE_REVERSE: {
				// find next position item in reverse order
				dm_item_t dm_current = (dm_item_t)(_mission.dataman_id);

				for (int32_t i = _current_mission_index - 1; i >= 0; i--) {
					struct mission_item_s missionitem = {};
					const ssize_t len = sizeof(missionitem);

					if (dm_read(dm_current, i, &missionitem, len) != len) {
						/* not supposed to happen unless the datamanager can't access the SD card, etc. */
						PX4_ERR("dataman read failure");
						break;
					}

					if (item_contains_position(missionitem)) {
						_current_mission_index = i;
						return;
					}
				}

				// finished flying back the mission
				_current_mission_index = -1;
				break;
			}

		default:
			_current_mission_index++;
		}

		break;

	case MISSION_TYPE_NONE:
	default:
		break;
	}
}

void
Mission::set_mission_items()
{
	/* reset the altitude foh (first order hold) logic, if altitude foh is enabled (param) a new foh element starts now */
	_min_current_sp_distance_xy = FLT_MAX;

	/* the home dist check provides user feedback, so we initialize it to this */
	bool user_feedback_done = false;

	/* mission item that comes after current if available */
	struct mission_item_s mission_item_next_position;
	struct mission_item_s mission_item_after_next_position;
	bool has_next_position_item = false;
	bool has_after_next_position_item = false;

	work_item_type new_work_item_type = WORK_ITEM_TYPE_DEFAULT;

	if (prepare_mission_items(&_mission_item, &mission_item_next_position, &has_next_position_item,
				  &mission_item_after_next_position, &has_after_next_position_item)) {
		/* if mission type changed, notify */
		if (_mission_type != MISSION_TYPE_MISSION) {
			mavlink_log_info(_navigator->get_mavlink_log_pub(),
					 _mission_execution_mode == mission_result_s::MISSION_EXECUTION_MODE_REVERSE ? "Executing Reverse Mission" :
					 "Executing Mission");
			user_feedback_done = true;
		}

		_mission_type = MISSION_TYPE_MISSION;

	} else {
		/* no mission available or mission finished, switch to loiter */
		if (_mission_type != MISSION_TYPE_NONE) {

			if (_navigator->get_land_detected()->landed) {
				mavlink_log_info(_navigator->get_mavlink_log_pub(),
						 _mission_execution_mode == mission_result_s::MISSION_EXECUTION_MODE_REVERSE ? "Reverse Mission finished, landed" :
						 "Mission finished, landed.");

			} else {
				/* https://en.wikipedia.org/wiki/Loiter_(aeronautics) */
				mavlink_log_info(_navigator->get_mavlink_log_pub(),
						 _mission_execution_mode == mission_result_s::MISSION_EXECUTION_MODE_REVERSE ? "Reverse Mission finished, loitering" :
						 "Mission finished, loitering.");

				/* use last setpoint for loiter */
				_navigator->set_can_loiter_at_sp(true);
			}

			user_feedback_done = true;
		}

		_mission_type = MISSION_TYPE_NONE;

		/* set loiter mission item and ensure that there is a minimum clearance from home */
		set_loiter_item(&_mission_item, _navigator->get_takeoff_min_alt());

		/* update position setpoint triplet  */
		position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
		pos_sp_triplet->previous.valid = false;
		mission_apply_limitation(_mission_item);
		mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);
		pos_sp_triplet->next.valid = false;

		/* reuse setpoint for LOITER only if it's not IDLE */
		_navigator->set_can_loiter_at_sp(pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_LOITER);

		// set mission finished
		_navigator->get_mission_result()->finished = true;
		_navigator->set_mission_result_updated();

		if (!user_feedback_done) {
			/* only tell users that we got no mission if there has not been any
			 * better, more specific feedback yet
			 * https://en.wikipedia.org/wiki/Loiter_(aeronautics)
			 */

			if (_navigator->get_land_detected()->landed) {
				/* landed, refusing to take off without a mission */
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "No valid mission available, refusing takeoff.");

			} else {
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "No valid mission available, loitering.");
			}

			user_feedback_done = true;
		}

		_navigator->set_position_setpoint_triplet_updated();
		return;
	}

	/*********************************** handle mission item *********************************************/

	/* handle mission items depending on the mode */

	const position_setpoint_s current_setpoint_copy = _navigator->get_position_setpoint_triplet()->current;

	if (item_contains_position(_mission_item)) {
		switch (_mission_execution_mode) {
		case mission_result_s::MISSION_EXECUTION_MODE_NORMAL:
		case mission_result_s::MISSION_EXECUTION_MODE_FAST_FORWARD: {
				/* force vtol land */
				if (_navigator->force_vtol() && _mission_item.nav_cmd == NAV_CMD_LAND) {
					_mission_item.nav_cmd = NAV_CMD_VTOL_LAND;
				}

				position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

				/* do takeoff before going to setpoint if needed and not already in takeoff */
				/* in fixed-wing this whole block will be ignored and a takeoff item is always propagated */
				if (do_need_vertical_takeoff() &&
				    _work_item_type == WORK_ITEM_TYPE_DEFAULT &&
				    new_work_item_type == WORK_ITEM_TYPE_DEFAULT) {

					new_work_item_type = WORK_ITEM_TYPE_TAKEOFF;

					/* use current mission item as next position item */
					mission_item_next_position = _mission_item;
					mission_item_next_position.nav_cmd = NAV_CMD_WAYPOINT;
					has_next_position_item = true;

					float takeoff_alt = calculate_takeoff_altitude(&_mission_item);

					mavlink_log_info(_navigator->get_mavlink_log_pub(), "Takeoff to %.1f meters above home.",
							 (double)(takeoff_alt - _navigator->get_home_position()->alt));

					_mission_item.nav_cmd = NAV_CMD_TAKEOFF;
					_mission_item.lat = _navigator->get_global_position()->lat;
					_mission_item.lon = _navigator->get_global_position()->lon;
					/* hold heading for takeoff items */
					_mission_item.yaw = _navigator->get_global_position()->yaw;
					_mission_item.altitude = takeoff_alt;
					_mission_item.altitude_is_relative = false;
					_mission_item.autocontinue = true;
					_mission_item.time_inside = 0.0f;

				} else if (_mission_item.nav_cmd == NAV_CMD_TAKEOFF
					   && _work_item_type == WORK_ITEM_TYPE_DEFAULT
					   && new_work_item_type == WORK_ITEM_TYPE_DEFAULT
					   && _navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {

					/* if there is no need to do a takeoff but we have a takeoff item, treat is as waypoint */
					_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
					/* ignore yaw here, otherwise it might yaw before heading_sp_update takes over */
					_mission_item.yaw = NAN;
					/* since _mission_item.time_inside and and _mission_item.pitch_min build a union, we need to set time_inside to zero
					 * since in NAV_CMD_TAKEOFF mode there is currently no time_inside.
					 * Note also that resetting time_inside to zero will cause pitch_min to be zero as well.
					 */
					_mission_item.time_inside = 0.0f;

				} else if (_mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF
					   && _work_item_type == WORK_ITEM_TYPE_DEFAULT
					   && new_work_item_type == WORK_ITEM_TYPE_DEFAULT) {

					if (_navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
						/* haven't transitioned yet, trigger vtol takeoff logic below */
						_work_item_type = WORK_ITEM_TYPE_TAKEOFF;

					} else {
						/* already in fixed-wing, go to waypoint */
						_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
					}

					/* ignore yaw here, otherwise it might yaw before heading_sp_update takes over */
					_mission_item.yaw = NAN;
				}

				/* if we just did a normal takeoff navigate to the actual waypoint now */
				if (_mission_item.nav_cmd == NAV_CMD_TAKEOFF &&
				    _work_item_type == WORK_ITEM_TYPE_TAKEOFF &&
				    new_work_item_type == WORK_ITEM_TYPE_DEFAULT) {

					_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
					/* ignore yaw here, otherwise it might yaw before heading_sp_update takes over */
					_mission_item.yaw = NAN;
					/* since _mission_item.time_inside and and _mission_item.pitch_min build a union, we need to set time_inside to zero
					 * since in NAV_CMD_TAKEOFF mode there is currently no time_inside.
					 */
					_mission_item.time_inside = 0.0f;
				}

				/* if we just did a VTOL takeoff, prepare transition */
				if (_mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF &&
				    _work_item_type == WORK_ITEM_TYPE_TAKEOFF &&
				    new_work_item_type == WORK_ITEM_TYPE_DEFAULT &&
				    _navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING &&
				    !_navigator->get_land_detected()->landed) {

					/* disable weathervane before front transition for allowing yaw to align */
					pos_sp_triplet->current.disable_weather_vane = true;

					/* set yaw setpoint to heading of VTOL_TAKEOFF wp against current position */
					_mission_item.yaw = get_bearing_to_next_waypoint(
								    _navigator->get_global_position()->lat, _navigator->get_global_position()->lon,
								    _mission_item.lat, _mission_item.lon);

					_mission_item.force_heading = true;

					new_work_item_type = WORK_ITEM_TYPE_ALIGN;

					/* set position setpoint to current while aligning */
					_mission_item.lat = _navigator->get_global_position()->lat;
					_mission_item.lon = _navigator->get_global_position()->lon;
				}

				/* heading is aligned now, prepare transition */
				if (_mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF &&
				    _work_item_type == WORK_ITEM_TYPE_ALIGN &&
				    _navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING &&
				    !_navigator->get_land_detected()->landed) {

					/* re-enable weather vane again after alignment */
					pos_sp_triplet->current.disable_weather_vane = false;

					/* check if the vtol_takeoff waypoint is on top of us */
					if (do_need_move_to_takeoff()) {
						new_work_item_type = WORK_ITEM_TYPE_TRANSITON_AFTER_TAKEOFF;
					}

					set_vtol_transition_item(&_mission_item, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);

					if (has_next_position_item) {
						/* got next mission item, update setpoint triplet */
						mission_item_to_position_setpoint(mission_item_next_position, &pos_sp_triplet->current);

					} else {
						_mission_item.yaw = _navigator->get_global_position()->yaw;

						/* set position setpoint to target during the transition */
						generate_waypoint_from_heading(&pos_sp_triplet->current, _mission_item.yaw);
					}
				}

				/* takeoff completed and transitioned, move to takeoff wp as fixed wing */
				if (_mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF
				    && _work_item_type == WORK_ITEM_TYPE_TRANSITON_AFTER_TAKEOFF
				    && new_work_item_type == WORK_ITEM_TYPE_DEFAULT) {

					new_work_item_type = WORK_ITEM_TYPE_DEFAULT;
					_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
					_mission_item.autocontinue = true;
					_mission_item.time_inside = 0.0f;
				}

				/* move to land wp as fixed wing */
				if (_mission_item.nav_cmd == NAV_CMD_VTOL_LAND
				    && (_work_item_type == WORK_ITEM_TYPE_DEFAULT || _work_item_type == WORK_ITEM_TYPE_TRANSITON_AFTER_TAKEOFF)
				    && new_work_item_type == WORK_ITEM_TYPE_DEFAULT
				    && !_navigator->get_land_detected()->landed) {

					new_work_item_type = WORK_ITEM_TYPE_MOVE_TO_LAND;

					/* use current mission item as next position item */
					mission_item_next_position = _mission_item;
					has_next_position_item = true;

					float altitude = _navigator->get_global_position()->alt;

					if (pos_sp_triplet->current.valid && pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_POSITION) {
						altitude = pos_sp_triplet->current.alt;
					}

					_mission_item.altitude = altitude;
					_mission_item.altitude_is_relative = false;
					_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
					_mission_item.autocontinue = true;
					_mission_item.time_inside = 0.0f;
					_mission_item.vtol_back_transition = true;
				}

				/* transition to MC */
				if (_mission_item.nav_cmd == NAV_CMD_VTOL_LAND
				    && _work_item_type == WORK_ITEM_TYPE_MOVE_TO_LAND
				    && new_work_item_type == WORK_ITEM_TYPE_DEFAULT
				    && _navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING
				    && !_navigator->get_land_detected()->landed) {

					set_vtol_transition_item(&_mission_item, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
					_mission_item.altitude = _navigator->get_global_position()->alt;
					_mission_item.altitude_is_relative = false;

					new_work_item_type = WORK_ITEM_TYPE_MOVE_TO_LAND_AFTER_TRANSITION;
				}

				/* move to landing waypoint before descent if necessary */
				if (do_need_move_to_land() &&
				    (_work_item_type == WORK_ITEM_TYPE_DEFAULT ||
				     _work_item_type == WORK_ITEM_TYPE_MOVE_TO_LAND_AFTER_TRANSITION) &&
				    new_work_item_type == WORK_ITEM_TYPE_DEFAULT) {

					new_work_item_type = WORK_ITEM_TYPE_MOVE_TO_LAND;

					/* use current mission item as next position item */
					mission_item_next_position = _mission_item;
					has_next_position_item = true;

					/*
					 * Ignoring waypoint altitude:
					 * Set altitude to the same as we have now to prevent descending too fast into
					 * the ground. Actual landing will descend anyway until it touches down.
					 * XXX: We might want to change that at some point if it is clear to the user
					 * what the altitude means on this waypoint type.
					 */
					float altitude = _navigator->get_global_position()->alt;

					if (pos_sp_triplet->current.valid
					    && pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_POSITION) {
						altitude = pos_sp_triplet->current.alt;
					}

					_mission_item.altitude = altitude;
					_mission_item.altitude_is_relative = false;
					_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
					_mission_item.autocontinue = true;
					_mission_item.time_inside = 0.0f;

				} else if (_mission_item.nav_cmd == NAV_CMD_LAND && _work_item_type == WORK_ITEM_TYPE_DEFAULT) {
					if (_mission_item.land_precision > 0 && _mission_item.land_precision < 3) {
						new_work_item_type = WORK_ITEM_TYPE_PRECISION_LAND;

						if (_mission_item.land_precision == 1) {
							_navigator->get_precland()->set_mode(PrecLandMode::Opportunistic);

						} else { //_mission_item.land_precision == 2
							_navigator->get_precland()->set_mode(PrecLandMode::Required);
						}

						_navigator->get_precland()->on_activation();

					}
				}

				/* we just moved to the landing waypoint, now descend */
				if (_work_item_type == WORK_ITEM_TYPE_MOVE_TO_LAND &&
				    new_work_item_type == WORK_ITEM_TYPE_DEFAULT) {

					if (_mission_item.land_precision > 0 && _mission_item.land_precision < 3) {
						new_work_item_type = WORK_ITEM_TYPE_PRECISION_LAND;

						if (_mission_item.land_precision == 1) {
							_navigator->get_precland()->set_mode(PrecLandMode::Opportunistic);

						} else { //_mission_item.land_precision == 2
							_navigator->get_precland()->set_mode(PrecLandMode::Required);
						}

						_navigator->get_precland()->on_activation();

					}

				}

				/* ignore yaw for landing items */
				/* XXX: if specified heading for landing is desired we could add another step before the descent
				 * that aligns the vehicle first */
				if (_mission_item.nav_cmd == NAV_CMD_LAND || _mission_item.nav_cmd == NAV_CMD_VTOL_LAND) {
					_mission_item.yaw = NAN;
				}


				// for fast forward convert certain types to simple waypoint
				// XXX: add other types which should be ignored in fast forward
				if (_mission_execution_mode == mission_result_s::MISSION_EXECUTION_MODE_FAST_FORWARD &&
				    ((_mission_item.nav_cmd == NAV_CMD_LOITER_UNLIMITED) ||
				     (_mission_item.nav_cmd == NAV_CMD_LOITER_TIME_LIMIT))) {
					_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
					_mission_item.autocontinue = true;
					_mission_item.time_inside = 0.0f;
				}

				break;
			}

		case mission_result_s::MISSION_EXECUTION_MODE_REVERSE: {
				if (item_contains_position(_mission_item)) {
					// convert mission item to a simple waypoint
					_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
					_mission_item.autocontinue = true;
					_mission_item.time_inside = 0.0f;

				} else {
					mavlink_log_critical(_navigator->get_mavlink_log_pub(), "MissionReverse: Got a non-position mission item, ignoring it");
				}

				break;
			}
		}

	} else {
		/* handle non-position mission items such as commands */
		switch (_mission_execution_mode) {
		case mission_result_s::MISSION_EXECUTION_MODE_NORMAL:
		case mission_result_s::MISSION_EXECUTION_MODE_FAST_FORWARD: {
				position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

				/* turn towards next waypoint before MC to FW transition */
				if (_mission_item.nav_cmd == NAV_CMD_DO_VTOL_TRANSITION
				    && _work_item_type == WORK_ITEM_TYPE_DEFAULT
				    && new_work_item_type == WORK_ITEM_TYPE_DEFAULT
				    && _navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
				    && !_navigator->get_land_detected()->landed
				    && has_next_position_item) {

					/* disable weathervane before front transition for allowing yaw to align */
					pos_sp_triplet->current.disable_weather_vane = true;

					new_work_item_type = WORK_ITEM_TYPE_ALIGN;

					set_align_mission_item(&_mission_item, &mission_item_next_position);

					/* set position setpoint to target during the transition */
					mission_apply_limitation(_mission_item);
					mission_item_to_position_setpoint(mission_item_next_position, &pos_sp_triplet->current);
				}

				/* yaw is aligned now */
				if (_work_item_type == WORK_ITEM_TYPE_ALIGN &&
				    new_work_item_type == WORK_ITEM_TYPE_DEFAULT) {

					new_work_item_type = WORK_ITEM_TYPE_DEFAULT;

					/* set position setpoint to target during the transition */
					pos_sp_triplet->previous = pos_sp_triplet->current;
					generate_waypoint_from_heading(&pos_sp_triplet->current, pos_sp_triplet->current.yaw);
				}

				/* don't advance mission after FW to MC command */
				if (_mission_item.nav_cmd == NAV_CMD_DO_VTOL_TRANSITION
				    && _work_item_type == WORK_ITEM_TYPE_DEFAULT
				    && new_work_item_type == WORK_ITEM_TYPE_DEFAULT
				    && _navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING
				    && !_navigator->get_land_detected()->landed
				    && pos_sp_triplet->current.valid) {

					new_work_item_type = WORK_ITEM_TYPE_CMD_BEFORE_MOVE;
				}

				/* after FW to MC transition finish moving to the waypoint */
				if (_work_item_type == WORK_ITEM_TYPE_CMD_BEFORE_MOVE &&
				    new_work_item_type == WORK_ITEM_TYPE_DEFAULT
				    && pos_sp_triplet->current.valid) {

					new_work_item_type = WORK_ITEM_TYPE_DEFAULT;

					_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
					copy_position_if_valid(&_mission_item, &pos_sp_triplet->current);
					_mission_item.autocontinue = true;
					_mission_item.time_inside = 0.0f;
				}

				// ignore certain commands in mission fast forward
				if ((_mission_execution_mode == mission_result_s::MISSION_EXECUTION_MODE_FAST_FORWARD) &&
				    (_mission_item.nav_cmd == NAV_CMD_DELAY)) {
					_mission_item.autocontinue = true;
					_mission_item.time_inside = 0.0f;
				}

				break;
			}

		case mission_result_s::MISSION_EXECUTION_MODE_REVERSE: {
				// nothing to do, all commands are ignored
				break;
			}
		}

		if (_mission_item.nav_cmd == NAV_CMD_CONDITION_GATE) {
			_mission_item.autocontinue = true;
			_mission_item.time_inside = 0;
		}
	}

	/*********************************** set setpoints and check next *********************************************/

	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	// The logic in this section establishes the tracking between the current waypoint
	// which we are approaching and the next waypoint, which will tell us in which direction
	// we will change our trajectory right after reaching it.

	// Because actions, gates and jump labels can be interleaved with waypoints,
	// we are searching around the current mission item in the list to find the closest
	// gate and the closest waypoint. We then store them separately.

	// Check if the mission item is a gate
	// along the current trajectory
	if (item_contains_gate(_mission_item)) {

		// The mission item is a gate, let's check if the next item in the list provides
		// a position to go towards.

		// TODO Precision land needs to be refactored: https://github.com/PX4/Firmware/issues/14320
		if (has_next_position_item) {
			// We have a position, convert it to the setpoint and update setpoint triplet
			mission_apply_limitation(mission_item_next_position);
			mission_item_to_position_setpoint(mission_item_next_position, &pos_sp_triplet->current);
		}

		// ELSE: The current position setpoint stays unchanged.

	} else {
		// The mission item is not a gate, set the current position setpoint from mission item (is protected against non-position items)
		// TODO Precision land needs to be refactored: https://github.com/PX4/Firmware/issues/14320
		if (new_work_item_type != WORK_ITEM_TYPE_PRECISION_LAND) {
			mission_apply_limitation(_mission_item);
			mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);
		}

		// ELSE: The current position setpoint stays unchanged.
	}

	// Only set the previous position item if the current one really changed
	// TODO Precision land needs to be refactored: https://github.com/PX4/Firmware/issues/14320
	if ((_work_item_type != WORK_ITEM_TYPE_MOVE_TO_LAND) &&
	    !position_setpoint_equal(&pos_sp_triplet->current, &current_setpoint_copy)) {
		pos_sp_triplet->previous = current_setpoint_copy;
	}

	/* issue command if ready (will do nothing for position mission items) */
	issue_command(_mission_item);

	/* set current work item type */
	_work_item_type = new_work_item_type;

	/* require takeoff after landing or idle */
	if (pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_LAND
	    || pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_IDLE) {

		_need_takeoff = true;
	}

	_navigator->set_can_loiter_at_sp(false);
	reset_mission_item_reached();

	if (_mission_type == MISSION_TYPE_MISSION) {
		set_current_mission_item();
	}

	// If the mission item under evaluation contains a gate,
	// then we need to check if we have a next position item so
	// the controller can fly the correct line between the
	// current and next setpoint
	if (item_contains_gate(_mission_item)) {
		if (has_after_next_position_item) {
			/* got next mission item, update setpoint triplet */
			mission_apply_limitation(mission_item_next_position);
			mission_item_to_position_setpoint(mission_item_next_position, &pos_sp_triplet->next);

		} else {
			pos_sp_triplet->next.valid = false;
		}

	} else {
		if (_mission_item.autocontinue && get_time_inside(_mission_item) < FLT_EPSILON) {
			/* try to process next mission item */
			if (has_next_position_item) {
				/* got next mission item, update setpoint triplet */
				mission_item_to_position_setpoint(mission_item_next_position, &pos_sp_triplet->next);

			} else {
				/* next mission item is not available */
				pos_sp_triplet->next.valid = false;
			}

		} else {
			/* vehicle will be paused on current waypoint, don't set next item */
			pos_sp_triplet->next.valid = false;
		}
	}

	/* Save the distance between the current sp and the previous one */
	if (pos_sp_triplet->current.valid && pos_sp_triplet->previous.valid) {

		_distance_current_previous = get_distance_to_next_waypoint(
						     pos_sp_triplet->current.lat, pos_sp_triplet->current.lon,
						     pos_sp_triplet->previous.lat, pos_sp_triplet->previous.lon);
	}

	_navigator->set_position_setpoint_triplet_updated();
}

bool
Mission::do_need_vertical_takeoff()
{
	if (_navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {

		float takeoff_alt = calculate_takeoff_altitude(&_mission_item);

		if (_navigator->get_land_detected()->landed) {
			/* force takeoff if landed (additional protection) */
			_need_takeoff = true;

		} else if (_navigator->get_global_position()->alt > takeoff_alt - _navigator->get_altitude_acceptance_radius()) {
			/* if in-air and already above takeoff height, don't do takeoff */
			_need_takeoff = false;

		} else if (_navigator->get_global_position()->alt <= takeoff_alt - _navigator->get_altitude_acceptance_radius()
			   && (_mission_item.nav_cmd == NAV_CMD_TAKEOFF
			       || _mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF)) {
			/* if in-air but below takeoff height and we have a takeoff item */
			_need_takeoff = true;
		}

		/* check if current mission item is one that requires takeoff before */
		if (_need_takeoff && (
			    _mission_item.nav_cmd == NAV_CMD_TAKEOFF ||
			    _mission_item.nav_cmd == NAV_CMD_WAYPOINT ||
			    _mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF ||
			    _mission_item.nav_cmd == NAV_CMD_LOITER_TIME_LIMIT ||
			    _mission_item.nav_cmd == NAV_CMD_LOITER_UNLIMITED)) {

			_need_takeoff = false;
			return true;
		}
	}

	return false;
}

bool
Mission::do_need_move_to_land()
{
	if (_navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
	    && (_mission_item.nav_cmd == NAV_CMD_LAND || _mission_item.nav_cmd == NAV_CMD_VTOL_LAND)) {

		float d_current = get_distance_to_next_waypoint(_mission_item.lat, _mission_item.lon,
				  _navigator->get_global_position()->lat, _navigator->get_global_position()->lon);

		return d_current > _navigator->get_acceptance_radius();
	}

	return false;
}

bool
Mission::do_need_move_to_takeoff()
{
	if (_navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
	    && _mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF) {

		float d_current = get_distance_to_next_waypoint(_mission_item.lat, _mission_item.lon,
				  _navigator->get_global_position()->lat, _navigator->get_global_position()->lon);

		return d_current > _navigator->get_acceptance_radius();
	}

	return false;
}

void
Mission::copy_position_if_valid(struct mission_item_s *mission_item, struct position_setpoint_s *setpoint)
{
	if (setpoint->valid && setpoint->type == position_setpoint_s::SETPOINT_TYPE_POSITION) {
		mission_item->lat = setpoint->lat;
		mission_item->lon = setpoint->lon;
		mission_item->altitude = setpoint->alt;

	} else {
		mission_item->lat = _navigator->get_global_position()->lat;
		mission_item->lon = _navigator->get_global_position()->lon;
		mission_item->altitude = _navigator->get_global_position()->alt;
	}

	mission_item->altitude_is_relative = false;
}

void
Mission::set_align_mission_item(struct mission_item_s *mission_item, struct mission_item_s *mission_item_next)
{
	mission_item->nav_cmd = NAV_CMD_WAYPOINT;
	copy_position_if_valid(mission_item, &(_navigator->get_position_setpoint_triplet()->current));
	mission_item->altitude_is_relative = false;
	mission_item->autocontinue = true;
	mission_item->time_inside = 0.0f;
	mission_item->yaw = get_bearing_to_next_waypoint(
				    _navigator->get_global_position()->lat, _navigator->get_global_position()->lon,
				    mission_item_next->lat, mission_item_next->lon);
	mission_item->force_heading = true;
}

float
Mission::calculate_takeoff_altitude(struct mission_item_s *mission_item)
{
	/* calculate takeoff altitude */
	float takeoff_alt = get_absolute_altitude_for_item(*mission_item);

	/* takeoff to at least MIS_TAKEOFF_ALT above home/ground, even if first waypoint is lower */
	if (_navigator->get_land_detected()->landed) {
		takeoff_alt = fmaxf(takeoff_alt, _navigator->get_global_position()->alt + _navigator->get_takeoff_min_alt());

	} else {
		takeoff_alt = fmaxf(takeoff_alt, _navigator->get_home_position()->alt + _navigator->get_takeoff_min_alt());
	}

	return takeoff_alt;
}

void
Mission::heading_sp_update()
{
	struct position_setpoint_triplet_s *pos_sp_triplet =
		_navigator->get_position_setpoint_triplet();

	// Only update if current triplet is valid
	if (pos_sp_triplet->current.valid) {

		double point_from_latlon[2] = { _navigator->get_global_position()->lat,
						_navigator->get_global_position()->lon
					      };
		double point_to_latlon[2] = { _navigator->get_global_position()->lat,
					      _navigator->get_global_position()->lon
					    };
		float yaw_offset = 0.0f;

		// Depending on ROI-mode, update heading
		switch (_navigator->get_vroi().mode) {
		case vehicle_roi_s::ROI_LOCATION: {
				// ROI is a fixed location. Vehicle needs to point towards that location
				point_to_latlon[0] = _navigator->get_vroi().lat;
				point_to_latlon[1] = _navigator->get_vroi().lon;
				// No yaw offset required
				yaw_offset = 0.0f;
				break;
			}

		case vehicle_roi_s::ROI_WPNEXT: {
				// ROI is current waypoint. Vehcile needs to point towards current waypoint
				point_to_latlon[0] = pos_sp_triplet->current.lat;
				point_to_latlon[1] = pos_sp_triplet->current.lon;
				// Add the gimbal's yaw offset
				yaw_offset = _navigator->get_vroi().yaw_offset;
				break;
			}

		case vehicle_roi_s::ROI_NONE:
		case vehicle_roi_s::ROI_WPINDEX:
		case vehicle_roi_s::ROI_TARGET:
		case vehicle_roi_s::ROI_ENUM_END:
		default: {
			}
		}

		// Get desired heading and update it.
		// However, only update if distance to desired heading is
		// larger than acceptance radius to prevent excessive yawing
		float d_current = get_distance_to_next_waypoint(point_from_latlon[0],
				  point_from_latlon[1], point_to_latlon[0], point_to_latlon[1]);

		if (d_current > _navigator->get_acceptance_radius()) {
			float yaw = matrix::wrap_pi(
					    get_bearing_to_next_waypoint(point_from_latlon[0],
							    point_from_latlon[1], point_to_latlon[0],
							    point_to_latlon[1]) + yaw_offset);

			_mission_item.yaw = yaw;
			pos_sp_triplet->current.yaw = _mission_item.yaw;
			pos_sp_triplet->current.yaw_valid = true;
		}

		// we set yaw directly so we can run this in parallel to the FOH update
		_navigator->set_position_setpoint_triplet_updated();
	}
}

void
Mission::altitude_sp_foh_update()
{
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	/* Don't change setpoint if last and current waypoint are not valid
	 * or if the previous altitude isn't from a position or loiter setpoint or
	 * if rotary wing since that is handled in the mc_pos_control
	 */


	if (!pos_sp_triplet->previous.valid || !pos_sp_triplet->current.valid || !PX4_ISFINITE(pos_sp_triplet->previous.alt)
	    || !(pos_sp_triplet->previous.type == position_setpoint_s::SETPOINT_TYPE_POSITION ||
		 pos_sp_triplet->previous.type == position_setpoint_s::SETPOINT_TYPE_LOITER) ||
	    _navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {

		return;
	}

	// Calculate acceptance radius, i.e. the radius within which we do not perform a first order hold anymore
	float acc_rad = _navigator->get_acceptance_radius(_mission_item.acceptance_radius);

	if (pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_LOITER) {
		acc_rad = _navigator->get_acceptance_radius(fabsf(_mission_item.loiter_radius) * 1.2f);
	}

	/* Do not try to find a solution if the last waypoint is inside the acceptance radius of the current one */
	if (_distance_current_previous - acc_rad < FLT_EPSILON) {
		return;
	}

	/* Don't do FOH for non-missions, landing and takeoff waypoints, the ground may be near
	 * and the FW controller has a custom landing logic
	 *
	 * NAV_CMD_LOITER_TO_ALT doesn't change altitude until reaching desired lat/lon
	 * */
	if (_mission_item.nav_cmd == NAV_CMD_LAND
	    || _mission_item.nav_cmd == NAV_CMD_VTOL_LAND
	    || _mission_item.nav_cmd == NAV_CMD_TAKEOFF
	    || _mission_item.nav_cmd == NAV_CMD_LOITER_TO_ALT
	    || !_navigator->is_planned_mission()) {

		return;
	}

	/* Calculate distance to current waypoint */
	float d_current = get_distance_to_next_waypoint(_mission_item.lat, _mission_item.lon,
			  _navigator->get_global_position()->lat, _navigator->get_global_position()->lon);

	/* Save distance to waypoint if it is the smallest ever achieved, however make sure that
	 * _min_current_sp_distance_xy is never larger than the distance between the current and the previous wp */
	_min_current_sp_distance_xy = math::min(math::min(d_current, _min_current_sp_distance_xy),
						_distance_current_previous);

	/* if the minimal distance is smaller then the acceptance radius, we should be at waypoint alt
	 * navigator will soon switch to the next waypoint item (if there is one) as soon as we reach this altitude */
	if (_min_current_sp_distance_xy < acc_rad) {
		pos_sp_triplet->current.alt = get_absolute_altitude_for_item(_mission_item);

	} else {
		/* update the altitude sp of the 'current' item in the sp triplet, but do not update the altitude sp
		 * of the mission item as it is used to check if the mission item is reached
		 * The setpoint is set linearly and such that the system reaches the current altitude at the acceptance
		 * radius around the current waypoint
		 **/
		float delta_alt = (get_absolute_altitude_for_item(_mission_item) - pos_sp_triplet->previous.alt);
		float grad = -delta_alt / (_distance_current_previous - acc_rad);
		float a = pos_sp_triplet->previous.alt - grad * _distance_current_previous;
		pos_sp_triplet->current.alt = a + grad * _min_current_sp_distance_xy;
	}

	// we set altitude directly so we can run this in parallel to the heading update
	_navigator->set_position_setpoint_triplet_updated();
}

void
Mission::cruising_speed_sp_update()
{
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	const float cruising_speed = _navigator->get_cruising_speed();

	/* Don't change setpoint if the current waypoint is not valid */
	if (!pos_sp_triplet->current.valid ||
	    fabsf(pos_sp_triplet->current.cruising_speed - cruising_speed) < FLT_EPSILON) {
		return;
	}

	pos_sp_triplet->current.cruising_speed = cruising_speed;

	_navigator->set_position_setpoint_triplet_updated();
}

void
Mission::do_abort_landing()
{
	// Abort FW landing
	//  first climb out then loiter over intended landing location

	if (_mission_item.nav_cmd != NAV_CMD_LAND) {
		return;
	}

	// loiter at the larger of MIS_LTRMIN_ALT above the landing point
	//  or 2 * FW_CLMBOUT_DIFF above the current altitude
	const float alt_landing = get_absolute_altitude_for_item(_mission_item);
	const float alt_sp = math::max(alt_landing + _navigator->get_loiter_min_alt(),
				       _navigator->get_global_position()->alt + 20.0f);

	// turn current landing waypoint into an indefinite loiter
	_mission_item.nav_cmd = NAV_CMD_LOITER_UNLIMITED;
	_mission_item.altitude_is_relative = false;
	_mission_item.altitude = alt_sp;
	_mission_item.loiter_radius = _navigator->get_loiter_radius();
	_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
	_mission_item.autocontinue = false;
	_mission_item.origin = ORIGIN_ONBOARD;

	mission_apply_limitation(_mission_item);
	mission_item_to_position_setpoint(_mission_item, &_navigator->get_position_setpoint_triplet()->current);
	_navigator->set_position_setpoint_triplet_updated();

	mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "Holding at %d m above landing.",
				     (int)(alt_sp - alt_landing));

	// reset mission index to start of landing
	if (_land_start_available) {
		_current_mission_index = get_land_start_index();

	} else {
		// move mission index back (landing approach point)
		_current_mission_index -= 1;
	}

	// send reposition cmd to get out of mission
	vehicle_command_s vcmd = {};

	vcmd.command = vehicle_command_s::VEHICLE_CMD_DO_REPOSITION;
	vcmd.param1 = -1;
	vcmd.param2 = 1;
	vcmd.param5 = _mission_item.lat;
	vcmd.param6 = _mission_item.lon;
	vcmd.param7 = alt_sp;

	_navigator->publish_vehicle_cmd(&vcmd);
}

bool
Mission::prepare_mission_items(struct mission_item_s *mission_item,
			       struct mission_item_s *next_position_mission_item, bool *has_next_position_item,
			       struct mission_item_s *after_next_position_mission_item, bool *has_after_next_position_item)
{
	*has_next_position_item = false;
	bool first_res = false;
	int offset = 1;

	if (_mission_execution_mode == mission_result_s::MISSION_EXECUTION_MODE_REVERSE) {
		offset = -1;
	}

	if (read_mission_item(0, mission_item)) {

		first_res = true;

		/* trying to find next position mission item */
		while (read_mission_item(offset, next_position_mission_item)) {
			if (_mission_execution_mode == mission_result_s::MISSION_EXECUTION_MODE_REVERSE) {
				offset--;

			} else {
				offset++;
			}

			if (item_contains_position(*next_position_mission_item)) {
				*has_next_position_item = true;
				break;
			}
		}

		if (_mission_execution_mode != mission_result_s::MISSION_EXECUTION_MODE_REVERSE &&
		    after_next_position_mission_item && has_after_next_position_item) {
			/* trying to find next next position mission item */
			while (read_mission_item(offset, after_next_position_mission_item)) {
				offset++;

				if (item_contains_position(*after_next_position_mission_item)) {
					*has_after_next_position_item = true;
					break;
				}
			}
		}
	}

	return first_res;
}

bool
Mission::read_mission_item(int offset, struct mission_item_s *mission_item)
{
	/* select mission */
	const int current_index = _current_mission_index;
	int index_to_read = current_index + offset;

	int *mission_index_ptr = (offset == 0) ? &_current_mission_index : &index_to_read;
	const dm_item_t dm_item = (dm_item_t)_mission.dataman_id;

	/* do not work on empty missions */
	if (_mission.count == 0) {
		return false;
	}

	/* Repeat this several times in case there are several DO JUMPS that we need to follow along, however, after
	 * 10 iterations we have to assume that the DO JUMPS are probably cycling and give up. */
	for (int i = 0; i < 10; i++) {
		if (*mission_index_ptr < 0 || *mission_index_ptr >= (int)_mission.count) {
			/* mission item index out of bounds - if they are equal, we just reached the end */
			if ((*mission_index_ptr != (int)_mission.count) && (*mission_index_ptr != -1)) {
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission item index out of bound, index: %d, max: %d.",
						     *mission_index_ptr, _mission.count);
			}

			return false;
		}

		const ssize_t len = sizeof(struct mission_item_s);

		/* read mission item to temp storage first to not overwrite current mission item if data damaged */
		struct mission_item_s mission_item_tmp;

		/* read mission item from datamanager */
		if (dm_read(dm_item, *mission_index_ptr, &mission_item_tmp, len) != len) {
			/* not supposed to happen unless the datamanager can't access the SD card, etc. */
			mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Waypoint could not be read.");
			return false;
		}

		/* check for DO_JUMP item, and whether it hasn't not already been repeated enough times */
		if (mission_item_tmp.nav_cmd == NAV_CMD_DO_JUMP) {
			const bool execute_jumps = _mission_execution_mode == mission_result_s::MISSION_EXECUTION_MODE_NORMAL;

			/* do DO_JUMP as many times as requested if not in reverse mode */
			if ((mission_item_tmp.do_jump_current_count < mission_item_tmp.do_jump_repeat_count) && execute_jumps) {

				/* only raise the repeat count if this is for the current mission item
				 * but not for the read ahead mission item */
				if (offset == 0) {
					(mission_item_tmp.do_jump_current_count)++;

					/* save repeat count */
					if (dm_write(dm_item, *mission_index_ptr, DM_PERSIST_POWER_ON_RESET, &mission_item_tmp, len) != len) {
						/* not supposed to happen unless the datamanager can't access the dataman */
						mavlink_log_critical(_navigator->get_mavlink_log_pub(), "DO JUMP waypoint could not be written.");
						return false;
					}

					report_do_jump_mission_changed(*mission_index_ptr, mission_item_tmp.do_jump_repeat_count);
				}

				/* set new mission item index and repeat
				 * we don't have to validate here, if it's invalid, we should realize this later .*/
				*mission_index_ptr = mission_item_tmp.do_jump_mission_index;

			} else {
				if (offset == 0 && execute_jumps) {
					mavlink_log_info(_navigator->get_mavlink_log_pub(), "DO JUMP repetitions completed.");
				}

				/* no more DO_JUMPS, therefore just try to continue with next mission item */
				if (_mission_execution_mode == mission_result_s::MISSION_EXECUTION_MODE_REVERSE) {
					(*mission_index_ptr)--;

				} else {
					(*mission_index_ptr)++;
				}
			}

		} else {
			/* if it's not a DO_JUMP, then we were successful */
			memcpy(mission_item, &mission_item_tmp, sizeof(struct mission_item_s));
			return true;
		}
	}

	/* we have given up, we don't want to cycle forever */
	mavlink_log_critical(_navigator->get_mavlink_log_pub(), "DO JUMP is cycling, giving up.");
	return false;
}

void
Mission::save_mission_state()
{
	mission_s mission_state = {};

	/* lock MISSION_STATE item */
	int dm_lock_ret = dm_lock(DM_KEY_MISSION_STATE);

	if (dm_lock_ret != 0) {
		PX4_ERR("DM_KEY_MISSION_STATE lock failed");
	}

	/* read current state */
	int read_res = dm_read(DM_KEY_MISSION_STATE, 0, &mission_state, sizeof(mission_s));

	if (read_res == sizeof(mission_s)) {
		/* data read successfully, check dataman ID and items count */
		if (mission_state.dataman_id == _mission.dataman_id && mission_state.count == _mission.count) {
			/* navigator may modify only sequence, write modified state only if it changed */
			if (mission_state.current_seq != _current_mission_index) {
				mission_state.current_seq = _current_mission_index;
				mission_state.timestamp = hrt_absolute_time();

				if (dm_write(DM_KEY_MISSION_STATE, 0, DM_PERSIST_POWER_ON_RESET, &mission_state,
					     sizeof(mission_s)) != sizeof(mission_s)) {

					PX4_ERR("Can't save mission state");
				}
			}
		}

	} else {
		/* invalid data, this must not happen and indicates error in mission publisher */
		mission_state.timestamp = hrt_absolute_time();
		mission_state.dataman_id = _mission.dataman_id;
		mission_state.count = _mission.count;
		mission_state.current_seq = _current_mission_index;

		mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Invalid mission state.");

		/* write modified state only if changed */
		if (dm_write(DM_KEY_MISSION_STATE, 0, DM_PERSIST_POWER_ON_RESET, &mission_state,
			     sizeof(mission_s)) != sizeof(mission_s)) {

			PX4_ERR("Can't save mission state");
		}
	}

	/* unlock MISSION_STATE item */
	if (dm_lock_ret == 0) {
		dm_unlock(DM_KEY_MISSION_STATE);
	}
}

void
Mission::report_do_jump_mission_changed(int index, int do_jumps_remaining)
{
	/* inform about the change */
	_navigator->get_mission_result()->item_do_jump_changed = true;
	_navigator->get_mission_result()->item_changed_index = index;
	_navigator->get_mission_result()->item_do_jump_remaining = do_jumps_remaining;

	_navigator->set_mission_result_updated();
}

void
Mission::set_mission_item_reached()
{
	_navigator->get_mission_result()->seq_reached = _current_mission_index;
	_navigator->set_mission_result_updated();

	reset_mission_item_reached();
}

void
Mission::set_current_mission_item()
{
	_navigator->get_mission_result()->finished = false;
	_navigator->get_mission_result()->seq_current = _current_mission_index;

	_navigator->set_mission_result_updated();

	save_mission_state();
}

void
Mission::check_mission_valid(bool force)
{
	if ((!_home_inited && _navigator->home_position_valid()) || force) {

		MissionFeasibilityChecker _missionFeasibilityChecker(_navigator);

		_navigator->get_mission_result()->valid =
			_missionFeasibilityChecker.checkMissionFeasible(_mission,
					_param_mis_dist_1wp.get(),
					_param_mis_dist_wps.get(),
					_navigator->mission_landing_required());

		_navigator->get_mission_result()->seq_total = _mission.count;
		_navigator->increment_mission_instance_count();
		_navigator->set_mission_result_updated();
		_home_inited = _navigator->home_position_valid();

		// find and store landing start marker (if available)
		find_mission_land_start();
	}
}

void
Mission::reset_mission(struct mission_s &mission)
{
	dm_lock(DM_KEY_MISSION_STATE);

	if (dm_read(DM_KEY_MISSION_STATE, 0, &mission, sizeof(mission_s)) == sizeof(mission_s)) {
		if (mission.dataman_id == DM_KEY_WAYPOINTS_OFFBOARD_0 || mission.dataman_id == DM_KEY_WAYPOINTS_OFFBOARD_1) {
			/* set current item to 0 */
			mission.current_seq = 0;

			/* reset jump counters */
			if (mission.count > 0) {
				const dm_item_t dm_current = (dm_item_t)mission.dataman_id;

				for (unsigned index = 0; index < mission.count; index++) {
					struct mission_item_s item;
					const ssize_t len = sizeof(struct mission_item_s);

					if (dm_read(dm_current, index, &item, len) != len) {
						PX4_WARN("could not read mission item during reset");
						break;
					}

					if (item.nav_cmd == NAV_CMD_DO_JUMP) {
						item.do_jump_current_count = 0;

						if (dm_write(dm_current, index, DM_PERSIST_POWER_ON_RESET, &item, len) != len) {
							PX4_WARN("could not save mission item during reset");
							break;
						}
					}
				}
			}

		} else {
			mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Could not read mission.");

			/* initialize mission state in dataman */
			mission.timestamp = hrt_absolute_time();
			mission.dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
			mission.count = 0;
			mission.current_seq = 0;
		}

		dm_write(DM_KEY_MISSION_STATE, 0, DM_PERSIST_POWER_ON_RESET, &mission, sizeof(mission_s));
	}

	dm_unlock(DM_KEY_MISSION_STATE);
}

bool
Mission::need_to_reset_mission()
{
	/* reset mission state when disarmed */
	if (_navigator->get_vstatus()->arming_state != vehicle_status_s::ARMING_STATE_ARMED && _need_mission_reset) {
		_need_mission_reset = false;
		return true;
	}

	return false;
}

void
Mission::generate_waypoint_from_heading(struct position_setpoint_s *setpoint, float yaw)
{
	waypoint_from_heading_and_distance(
		_navigator->get_global_position()->lat, _navigator->get_global_position()->lon,
		yaw, 1000000.0f,
		&(setpoint->lat), &(setpoint->lon));
	setpoint->type = position_setpoint_s::SETPOINT_TYPE_POSITION;
	setpoint->yaw = yaw;
}

int32_t
Mission::index_closest_mission_item() const
{
	int32_t min_dist_index(0);
	float min_dist(FLT_MAX), dist_xy(FLT_MAX), dist_z(FLT_MAX);

	dm_item_t dm_current = (dm_item_t)(_mission.dataman_id);

	for (size_t i = 0; i < _mission.count; i++) {
		struct mission_item_s missionitem = {};
		const ssize_t len = sizeof(missionitem);

		if (dm_read(dm_current, i, &missionitem, len) != len) {
			/* not supposed to happen unless the datamanager can't access the SD card, etc. */
			PX4_ERR("dataman read failure");
			break;
		}

		if (item_contains_position(missionitem)) {
			// do not consider land waypoints for a fw
			if (!((missionitem.nav_cmd == NAV_CMD_LAND) &&
			      (_navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) &&
			      (!_navigator->get_vstatus()->is_vtol))) {
				float dist = get_distance_to_point_global_wgs84(missionitem.lat, missionitem.lon,
						get_absolute_altitude_for_item(missionitem),
						_navigator->get_global_position()->lat,
						_navigator->get_global_position()->lon,
						_navigator->get_global_position()->alt,
						&dist_xy, &dist_z);

				if (dist < min_dist) {
					min_dist = dist;
					min_dist_index = i;
				}
			}
		}
	}

	// for mission reverse also consider the home position
	if (_mission_execution_mode == mission_result_s::MISSION_EXECUTION_MODE_REVERSE) {
		float dist = get_distance_to_point_global_wgs84(
				     _navigator->get_home_position()->lat,
				     _navigator->get_home_position()->lon,
				     _navigator->get_home_position()->alt,
				     _navigator->get_global_position()->lat,
				     _navigator->get_global_position()->lon,
				     _navigator->get_global_position()->alt,
				     &dist_xy, &dist_z);

		if (dist < min_dist) {
			min_dist = dist;
			min_dist_index = -1;
		}
	}

	return min_dist_index;
}

bool Mission::position_setpoint_equal(const position_setpoint_s *p1, const position_setpoint_s *p2) const
{
	return ((p1->valid == p2->valid) &&
		(p1->type == p2->type) &&
		(fabsf(p1->x - p2->x) < FLT_EPSILON) &&
		(fabsf(p1->y - p2->y) < FLT_EPSILON) &&
		(fabsf(p1->z - p2->z) < FLT_EPSILON) &&
		(p1->position_valid == p2->position_valid) &&
		(fabsf(p1->vx - p2->vx) < FLT_EPSILON) &&
		(fabsf(p1->vy - p2->vy) < FLT_EPSILON) &&
		(fabsf(p1->vz - p2->vz) < FLT_EPSILON) &&
		(p1->velocity_valid == p2->velocity_valid) &&
		(p1->velocity_frame == p2->velocity_frame) &&
		(p1->alt_valid == p2->alt_valid) &&
		(fabs(p1->lat - p2->lat) < DBL_EPSILON) &&
		(fabs(p1->lon - p2->lon) < DBL_EPSILON) &&
		(fabsf(p1->alt - p2->alt) < FLT_EPSILON) &&
		((fabsf(p1->yaw - p2->yaw) < FLT_EPSILON) || (!PX4_ISFINITE(p1->yaw) && !PX4_ISFINITE(p2->yaw))) &&
		(p1->yaw_valid == p2->yaw_valid) &&
		(fabsf(p1->yawspeed - p2->yawspeed) < FLT_EPSILON) &&
		(p1->yawspeed_valid == p2->yawspeed_valid) &&
		(fabsf(p1->loiter_radius - p2->loiter_radius) < FLT_EPSILON) &&
		(p1->loiter_direction == p2->loiter_direction) &&
		(fabsf(p1->pitch_min - p2->pitch_min) < FLT_EPSILON) &&
		(fabsf(p1->a_x - p2->a_x) < FLT_EPSILON) &&
		(fabsf(p1->a_y - p2->a_y) < FLT_EPSILON) &&
		(fabsf(p1->a_z - p2->a_z) < FLT_EPSILON) &&
		(p1->acceleration_valid == p2->acceleration_valid) &&
		(p1->acceleration_is_force == p2->acceleration_is_force) &&
		(fabsf(p1->acceptance_radius - p2->acceptance_radius) < FLT_EPSILON) &&
		(fabsf(p1->cruising_speed - p2->cruising_speed) < FLT_EPSILON) &&
		(fabsf(p1->cruising_throttle - p2->cruising_throttle) < FLT_EPSILON));

}
