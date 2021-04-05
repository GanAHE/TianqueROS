/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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
 * @file rtl.cpp
 *
 * Helper class to access RTL
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include "rtl.h"
#include "navigator.h"
#include <dataman/dataman.h>


static constexpr float DELAY_SIGMA = 0.01f;

RTL::RTL(Navigator *navigator) :
	MissionBlock(navigator),
	ModuleParams(navigator)
{
}

void
RTL::on_inactive()
{
	// Reset RTL state.
	_rtl_state = RTL_STATE_NONE;

	find_RTL_destination();

}

void
RTL::find_RTL_destination()
{
	// get home position:
	home_position_s &home_landing_position = *_navigator->get_home_position();
	// get global position
	const vehicle_global_position_s &global_position = *_navigator->get_global_position();

	// set destination to home per default, then check if other valid landing spot is closer
	_destination.set(home_landing_position);
	// get distance to home position
	double dlat = home_landing_position.lat - global_position.lat;
	double dlon = home_landing_position.lon - global_position.lon;
	double min_dist_squared = dlat * dlat + dlon * dlon;

	_destination.type = RTL_DESTINATION_HOME;

	// consider the mission landing if not RTL_HOME type set
	if (rtl_type() != RTL_HOME && _navigator->get_mission_start_land_available()) {
		double mission_landing_lat = _navigator->get_mission_landing_lat();
		double mission_landing_lon = _navigator->get_mission_landing_lon();

		// compare home position to landing position to decide which is closer
		dlat = mission_landing_lat - global_position.lat;
		dlon = mission_landing_lon - global_position.lon;
		double dist_squared = dlat * dlat + dlon * dlon;

		// set destination to mission landing if closest or in RTL_LAND or RTL_MISSION (so not in RTL_CLOSEST)
		if (dist_squared < min_dist_squared || rtl_type() != RTL_CLOSEST) {
			min_dist_squared = dist_squared;
			_destination.lat = _navigator->get_mission_landing_lat();
			_destination.lon = _navigator->get_mission_landing_lon();
			_destination.alt = _navigator->get_mission_landing_alt();
			_destination.type = RTL_DESTINATION_MISSION_LANDING;

		}
	}

	// do not consider rally point if RTL type is set to RTL_MISSION, so exit function and use either home or mission landing
	if (rtl_type() == RTL_MISSION) {
		return;
	}

	// compare to safe landing positions
	mission_safe_point_s closest_safe_point {} ;
	mission_stats_entry_s stats;
	int ret = dm_read(DM_KEY_SAFE_POINTS, 0, &stats, sizeof(mission_stats_entry_s));
	int num_safe_points = 0;

	if (ret == sizeof(mission_stats_entry_s)) {
		num_safe_points = stats.num_items;
	}

	// check if a safe point is closer than home or landing
	int closest_index = 0;

	for (int current_seq = 1; current_seq <= num_safe_points; ++current_seq) {
		mission_safe_point_s mission_safe_point;

		if (dm_read(DM_KEY_SAFE_POINTS, current_seq, &mission_safe_point, sizeof(mission_safe_point_s)) !=
		    sizeof(mission_safe_point_s)) {
			PX4_ERR("dm_read failed");
			continue;
		}

		// TODO: take altitude into account for distance measurement
		dlat = mission_safe_point.lat - global_position.lat;
		dlon = mission_safe_point.lon - global_position.lon;
		double dist_squared = dlat * dlat + dlon * dlon;

		if (dist_squared < min_dist_squared) {
			closest_index = current_seq;
			min_dist_squared = dist_squared;
			closest_safe_point = mission_safe_point;
		}
	}

	if (closest_index > 0) {
		_destination.type = RTL_DESTINATION_SAFE_POINT;

		// There is a safe point closer than home/mission landing
		// TODO: handle all possible mission_safe_point.frame cases
		switch (closest_safe_point.frame) {
		case 0: // MAV_FRAME_GLOBAL
			_destination.lat = closest_safe_point.lat;
			_destination.lon = closest_safe_point.lon;
			_destination.alt = closest_safe_point.alt;
			_destination.yaw = home_landing_position.yaw;
			break;

		case 3: // MAV_FRAME_GLOBAL_RELATIVE_ALT
			_destination.lat = closest_safe_point.lat;
			_destination.lon = closest_safe_point.lon;
			_destination.alt = closest_safe_point.alt + home_landing_position.alt; // alt of safe point is rel to home
			_destination.yaw = home_landing_position.yaw;
			break;

		default:
			mavlink_log_critical(_navigator->get_mavlink_log_pub(), "RTL: unsupported MAV_FRAME");
			break;
		}
	}

}

int
RTL::rtl_type() const
{
	return _param_rtl_type.get();
}

void
RTL::on_activation()
{

	// output the correct message, depending on where the RTL destination is
	switch (_destination.type) {
	case RTL_DESTINATION_HOME:
		mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "RTL: landing at home position.");
		break;

	case RTL_DESTINATION_MISSION_LANDING:
		mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "RTL: landing at mission landing.");
		break;

	case RTL_DESTINATION_SAFE_POINT:
		mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "RTL: landing at safe landing point.");
		break;
	}

	const vehicle_global_position_s &global_position = *_navigator->get_global_position();

	_rtl_alt = calculate_return_alt_from_cone_half_angle((float)_param_rtl_cone_half_angle_deg.get());

	if (_navigator->get_land_detected()->landed) {
		// For safety reasons don't go into RTL if landed.
		_rtl_state = RTL_STATE_LANDED;

	} else if ((_destination.type == RTL_DESTINATION_MISSION_LANDING) && _navigator->on_mission_landing()) {
		// RTL straight to RETURN state, but mission will takeover for landing.

	} else if ((global_position.alt < _destination.alt + _param_rtl_return_alt.get()) || _rtl_alt_min) {

		// If lower than return altitude, climb up first.
		// If rtl_alt_min is true then forcing altitude change even if above.
		_rtl_state = RTL_STATE_CLIMB;

	} else {
		// Otherwise go straight to return
		_rtl_state = RTL_STATE_RETURN;
	}

	set_rtl_item();
}

void
RTL::on_active()
{
	if (_rtl_state != RTL_STATE_LANDED && is_mission_item_reached()) {
		advance_rtl();
		set_rtl_item();
	}
}

void
RTL::set_return_alt_min(bool min)
{
	_rtl_alt_min = min;
}

void
RTL::set_rtl_item()
{
	// RTL_TYPE: mission landing.
	// Landing using planned mission landing, fly to DO_LAND_START instead of returning _destination.
	// After reaching DO_LAND_START, do nothing, let navigator takeover with mission landing.
	if (_destination.type == RTL_DESTINATION_MISSION_LANDING) {
		if (_rtl_state > RTL_STATE_CLIMB) {
			if (_navigator->start_mission_landing()) {
				mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "RTL: using mission landing");
				return;

			} else {
				// Otherwise use regular RTL.
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "RTL: unable to use mission landing");
			}
		}
	}

	_navigator->set_can_loiter_at_sp(false);

	const vehicle_global_position_s &gpos = *_navigator->get_global_position();

	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	// Check if we are pretty close to the destination already.
	const float destination_dist = get_distance_to_next_waypoint(_destination.lat, _destination.lon, gpos.lat, gpos.lon);

	// Compute the loiter altitude.
	const float loiter_altitude = math::min(_destination.alt + _param_rtl_descend_alt.get(), gpos.alt);

	switch (_rtl_state) {
	case RTL_STATE_CLIMB: {

			_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
			_mission_item.lat = gpos.lat;
			_mission_item.lon = gpos.lon;
			_mission_item.altitude = _rtl_alt;
			_mission_item.altitude_is_relative = false;
			_mission_item.yaw = _navigator->get_local_position()->yaw;
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;

			mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "RTL: climb to %d m (%d m above destination)",
						     (int)ceilf(_rtl_alt), (int)ceilf(_rtl_alt - _destination.alt));
			break;
		}

	case RTL_STATE_RETURN: {

			// Don't change altitude.
			_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
			_mission_item.lat = _destination.lat;
			_mission_item.lon = _destination.lon;
			_mission_item.altitude = _rtl_alt;
			_mission_item.altitude_is_relative = false;

			// Use destination yaw if close to _destination.
			// Check if we are pretty close to the destination already.
			if (destination_dist < _param_rtl_min_dist.get()) {
				_mission_item.yaw = _destination.yaw;

			} else {
				// Use current heading to _destination.
				_mission_item.yaw = get_bearing_to_next_waypoint(gpos.lat, gpos.lon, _destination.lat, _destination.lon);
			}

			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;

			mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "RTL: return at %d m (%d m above destination)",
						     (int)ceilf(_mission_item.altitude), (int)ceilf(_mission_item.altitude - _destination.alt));

			break;
		}

	case RTL_STATE_TRANSITION_TO_MC: {
			set_vtol_transition_item(&_mission_item, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
			break;
		}

	case RTL_STATE_DESCEND: {
			_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
			_mission_item.lat = _destination.lat;
			_mission_item.lon = _destination.lon;
			_mission_item.altitude = loiter_altitude;
			_mission_item.altitude_is_relative = false;

			// Except for vtol which might be still off here and should point towards this location.
			const float d_current = get_distance_to_next_waypoint(gpos.lat, gpos.lon, _mission_item.lat, _mission_item.lon);

			if (_navigator->get_vstatus()->is_vtol && (d_current > _navigator->get_acceptance_radius())) {
				_mission_item.yaw = get_bearing_to_next_waypoint(gpos.lat, gpos.lon, _mission_item.lat, _mission_item.lon);

			} else {
				_mission_item.yaw = _destination.yaw;
			}

			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;

			// Disable previous setpoint to prevent drift.
			pos_sp_triplet->previous.valid = false;

			mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "RTL: descend to %d m (%d m above destination)",
						     (int)ceilf(_mission_item.altitude), (int)ceilf(_mission_item.altitude - _destination.alt));
			break;
		}

	case RTL_STATE_LOITER: {
			const bool autoland = (_param_rtl_land_delay.get() > FLT_EPSILON);

			// Don't change altitude.
			_mission_item.lat = _destination.lat;
			_mission_item.lon = _destination.lon;
			_mission_item.altitude = loiter_altitude;
			_mission_item.altitude_is_relative = false;
			_mission_item.yaw = _destination.yaw;
			_mission_item.loiter_radius = _navigator->get_loiter_radius();
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = math::max(_param_rtl_land_delay.get(), 0.0f);
			_mission_item.autocontinue = autoland;
			_mission_item.origin = ORIGIN_ONBOARD;

			_navigator->set_can_loiter_at_sp(true);

			if (autoland && (get_time_inside(_mission_item) > FLT_EPSILON)) {
				_mission_item.nav_cmd = NAV_CMD_LOITER_TIME_LIMIT;
				mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "RTL: loiter %.1fs",
							     (double)get_time_inside(_mission_item));

			} else {
				_mission_item.nav_cmd = NAV_CMD_LOITER_UNLIMITED;
				mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "RTL: completed, loitering");
			}

			break;
		}

	case RTL_STATE_LAND: {
			// Land at destination.
			_mission_item.nav_cmd = NAV_CMD_LAND;
			_mission_item.lat = _destination.lat;
			_mission_item.lon = _destination.lon;
			_mission_item.yaw = _destination.yaw;
			_mission_item.altitude = _destination.alt;
			_mission_item.altitude_is_relative = false;
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;

			mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "RTL: land at destination");
			break;
		}

	case RTL_STATE_LANDED: {
			set_idle_item(&_mission_item);
			set_return_alt_min(false);
			break;
		}

	default:
		break;
	}

	reset_mission_item_reached();

	// Execute command if set. This is required for commands like VTOL transition.
	if (!item_contains_position(_mission_item)) {
		issue_command(_mission_item);
	}

	// Convert mission item to current position setpoint and make it valid.
	mission_apply_limitation(_mission_item);

	if (mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current)) {
		_navigator->set_position_setpoint_triplet_updated();
	}
}

void
RTL::advance_rtl()
{
	switch (_rtl_state) {
	case RTL_STATE_CLIMB:
		_rtl_state = RTL_STATE_RETURN;
		break;

	case RTL_STATE_RETURN:

		// Descend to desired altitude if delay is set, directly land otherwise
		if (_param_rtl_land_delay.get() < -DELAY_SIGMA || _param_rtl_land_delay.get() > DELAY_SIGMA) {
			_rtl_state = RTL_STATE_DESCEND;

		} else {
			_rtl_state = RTL_STATE_LAND;
		}

		if (_navigator->get_vstatus()->is_vtol
		    && _navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
			_rtl_state = RTL_STATE_TRANSITION_TO_MC;
		}

		break;

	case RTL_STATE_TRANSITION_TO_MC:
		_rtl_state = RTL_STATE_RETURN;
		break;

	case RTL_STATE_DESCEND:

		// Only go to land if autoland is enabled.
		if (_param_rtl_land_delay.get() < -DELAY_SIGMA || _param_rtl_land_delay.get() > DELAY_SIGMA) {
			_rtl_state = RTL_STATE_LOITER;

		} else {
			_rtl_state = RTL_STATE_LAND;
		}

		break;

	case RTL_STATE_LOITER:
		_rtl_state = RTL_STATE_LAND;
		break;

	case RTL_STATE_LAND:
		_rtl_state = RTL_STATE_LANDED;
		break;

	default:
		break;
	}
}


float RTL::calculate_return_alt_from_cone_half_angle(float cone_half_angle_deg)
{
	const vehicle_global_position_s &gpos = *_navigator->get_global_position();

	// horizontal distance to destination
	const float destination_dist = get_distance_to_next_waypoint(_destination.lat, _destination.lon, gpos.lat, gpos.lon);

	float rtl_altitude;

	if (destination_dist <= _param_rtl_min_dist.get()) {
		rtl_altitude = _destination.alt + _param_rtl_descend_alt.get();

	} else if (gpos.alt > _destination.alt + _param_rtl_return_alt.get() || cone_half_angle_deg >= 90.0f) {
		rtl_altitude = gpos.alt;

	} else if (cone_half_angle_deg <= 0) {
		rtl_altitude = _destination.alt + _param_rtl_return_alt.get();

	} else {

		// constrain cone half angle to meaningful values. All other cases are already handled above.
		const float cone_half_angle_rad = math::radians(math::constrain(cone_half_angle_deg, 1.0f, 89.0f));

		// minimum height above destination required
		float height_above_destination_min = destination_dist / tanf(cone_half_angle_rad);

		// minimum altitude we need in order to be within the user defined cone
		const float altitude_min = math::constrain(height_above_destination_min + _destination.alt, _destination.alt,
					   _destination.alt + _param_rtl_return_alt.get());

		if (gpos.alt < altitude_min) {
			rtl_altitude = altitude_min;

		} else {
			rtl_altitude = gpos.alt;
		}
	}

	// always demand altitude which is higher or equal the RTL descend altitude
	rtl_altitude = math::max(rtl_altitude, _destination.alt + _param_rtl_descend_alt.get());

	return rtl_altitude;
}
