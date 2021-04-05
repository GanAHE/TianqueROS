#include "FlightTask.hpp"
#include <mathlib/mathlib.h>
#include <lib/ecl/geo/geo.h>

constexpr uint64_t FlightTask::_timeout;
// First index of empty_setpoint corresponds to time-stamp and requires a finite number.
const vehicle_local_position_setpoint_s FlightTask::empty_setpoint = {0, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {}};

const vehicle_constraints_s FlightTask::empty_constraints = {0, NAN, NAN, NAN, NAN, NAN, NAN, NAN, false, {}};
const landing_gear_s FlightTask::empty_landing_gear_default_keep = {0, landing_gear_s::GEAR_KEEP, {}};

bool FlightTask::activate(vehicle_local_position_setpoint_s last_setpoint)
{
	_resetSetpoints();
	_setDefaultConstraints();
	_initEkfResetCounters();
	_time_stamp_activate = hrt_absolute_time();
	_gear = empty_landing_gear_default_keep;
	return true;
}

void FlightTask::reActivate()
{
	activate(getPositionSetpoint());
}

bool FlightTask::updateInitialize()
{
	_time_stamp_current = hrt_absolute_time();
	_time = (_time_stamp_current - _time_stamp_activate) / 1e6f;
	_deltatime  = math::min((_time_stamp_current - _time_stamp_last), _timeout) / 1e6f;
	_time_stamp_last = _time_stamp_current;

	_sub_vehicle_local_position.update();
	_sub_attitude.update();
	_sub_home_position.update();

	_evaluateVehicleLocalPosition();
	_evaluateDistanceToGround();
	_checkEkfResetCounters();
	return true;
}

void FlightTask::_initEkfResetCounters()
{
	_reset_counters.xy = _sub_vehicle_local_position.get().xy_reset_counter;
	_reset_counters.vxy = _sub_vehicle_local_position.get().vxy_reset_counter;
	_reset_counters.z = _sub_vehicle_local_position.get().z_reset_counter;
	_reset_counters.vz = _sub_vehicle_local_position.get().vz_reset_counter;
	_reset_counters.quat = _sub_attitude.get().quat_reset_counter;
}

void FlightTask::_checkEkfResetCounters()
{
	// Check if a reset event has happened
	if (_sub_vehicle_local_position.get().xy_reset_counter != _reset_counters.xy) {
		_ekfResetHandlerPositionXY();
		_reset_counters.xy = _sub_vehicle_local_position.get().xy_reset_counter;
	}

	if (_sub_vehicle_local_position.get().vxy_reset_counter != _reset_counters.vxy) {
		_ekfResetHandlerVelocityXY();
		_reset_counters.vxy = _sub_vehicle_local_position.get().vxy_reset_counter;
	}

	if (_sub_vehicle_local_position.get().z_reset_counter != _reset_counters.z) {
		_ekfResetHandlerPositionZ();
		_reset_counters.z = _sub_vehicle_local_position.get().z_reset_counter;
	}

	if (_sub_vehicle_local_position.get().vz_reset_counter != _reset_counters.vz) {
		_ekfResetHandlerVelocityZ();
		_reset_counters.vz = _sub_vehicle_local_position.get().vz_reset_counter;
	}

	if (_sub_attitude.get().quat_reset_counter != _reset_counters.quat) {
		float delta_psi = matrix::Eulerf(matrix::Quatf(_sub_attitude.get().delta_q_reset)).psi();
		_ekfResetHandlerHeading(delta_psi);
		_reset_counters.quat = _sub_attitude.get().quat_reset_counter;
	}
}

const vehicle_local_position_setpoint_s FlightTask::getPositionSetpoint()
{
	/* fill position setpoint message */
	vehicle_local_position_setpoint_s vehicle_local_position_setpoint{};
	vehicle_local_position_setpoint.timestamp = hrt_absolute_time();

	vehicle_local_position_setpoint.x = _position_setpoint(0);
	vehicle_local_position_setpoint.y = _position_setpoint(1);
	vehicle_local_position_setpoint.z = _position_setpoint(2);

	vehicle_local_position_setpoint.vx = _velocity_setpoint(0);
	vehicle_local_position_setpoint.vy = _velocity_setpoint(1);
	vehicle_local_position_setpoint.vz = _velocity_setpoint(2);

	_acceleration_setpoint.copyTo(vehicle_local_position_setpoint.acceleration);
	_jerk_setpoint.copyTo(vehicle_local_position_setpoint.jerk);
	vehicle_local_position_setpoint.yaw = _yaw_setpoint;
	vehicle_local_position_setpoint.yawspeed = _yawspeed_setpoint;

	// deprecated, only kept for output logging
	matrix::Vector3f(NAN, NAN, NAN).copyTo(vehicle_local_position_setpoint.thrust);

	return vehicle_local_position_setpoint;
}

void FlightTask::_resetSetpoints()
{
	_position_setpoint.setNaN();
	_velocity_setpoint.setNaN();
	_acceleration_setpoint.setNaN();
	_jerk_setpoint.setNaN();
	_yaw_setpoint = _yawspeed_setpoint = NAN;
}

void FlightTask::_evaluateVehicleLocalPosition()
{
	_position.setAll(NAN);
	_velocity.setAll(NAN);
	_yaw = NAN;
	_dist_to_bottom = NAN;

	if ((_time_stamp_current - _sub_attitude.get().timestamp) < _timeout) {
		// yaw
		_yaw = matrix::Eulerf(matrix::Quatf(_sub_attitude.get().q)).psi();
	}

	// Only use vehicle-local-position topic fields if the topic is received within a certain timestamp
	if ((_time_stamp_current - _sub_vehicle_local_position.get().timestamp) < _timeout) {

		// position
		if (_sub_vehicle_local_position.get().xy_valid) {
			_position(0) = _sub_vehicle_local_position.get().x;
			_position(1) = _sub_vehicle_local_position.get().y;
		}

		if (_sub_vehicle_local_position.get().z_valid) {
			_position(2) = _sub_vehicle_local_position.get().z;
		}

		// velocity
		if (_sub_vehicle_local_position.get().v_xy_valid) {
			_velocity(0) = _sub_vehicle_local_position.get().vx;
			_velocity(1) = _sub_vehicle_local_position.get().vy;
		}

		if (_sub_vehicle_local_position.get().v_z_valid) {
			_velocity(2) = _sub_vehicle_local_position.get().vz;
		}

		// distance to bottom
		if (_sub_vehicle_local_position.get().dist_bottom_valid
		    && PX4_ISFINITE(_sub_vehicle_local_position.get().dist_bottom)) {
			_dist_to_bottom =  _sub_vehicle_local_position.get().dist_bottom;
		}

		// global frame reference coordinates to enable conversions
		if (_sub_vehicle_local_position.get().xy_global && _sub_vehicle_local_position.get().z_global) {
			globallocalconverter_init(_sub_vehicle_local_position.get().ref_lat, _sub_vehicle_local_position.get().ref_lon,
						  _sub_vehicle_local_position.get().ref_alt, _sub_vehicle_local_position.get().ref_timestamp);
		}
	}
}

void FlightTask::_evaluateDistanceToGround()
{
	// Altitude above ground is local z-position or altitude above home or distance sensor altitude depending on what's available
	_dist_to_ground = -_position(2);

	if (PX4_ISFINITE(_dist_to_bottom)) {
		_dist_to_ground = _dist_to_bottom;

	} else if (_sub_home_position.get().valid_alt) {
		_dist_to_ground = -(_position(2) - _sub_home_position.get().z);
	}
}

void FlightTask::_setDefaultConstraints()
{
	_constraints.speed_xy = _param_mpc_xy_vel_max.get();
	_constraints.speed_up = _param_mpc_z_vel_max_up.get();
	_constraints.speed_down = _param_mpc_z_vel_max_dn.get();
	_constraints.tilt = math::radians(_param_mpc_tiltmax_air.get());
	_constraints.min_distance_to_ground = NAN;
	_constraints.max_distance_to_ground = NAN;
	_constraints.want_takeoff = false;
}

bool FlightTask::_checkTakeoff()
{
	// position setpoint above the minimum altitude
	bool position_triggered_takeoff = false;

	if (PX4_ISFINITE(_position_setpoint(2))) {
		// minimal altitude either 20cm or what is necessary for correct estimation e.g. optical flow
		float min_altitude = 0.2f;
		const float min_distance_to_ground = _sub_vehicle_local_position.get().hagl_min;

		if (PX4_ISFINITE(min_distance_to_ground)) {
			min_altitude = min_distance_to_ground + 0.05f;
		}

		position_triggered_takeoff = _position_setpoint(2) < (_position(2) - min_altitude);
	}

	// upwards velocity setpoint
	bool velocity_triggered_takeoff = false;

	if (PX4_ISFINITE(_velocity_setpoint(2))) {
		velocity_triggered_takeoff = _velocity_setpoint(2) < -0.3f;
	}

	return position_triggered_takeoff || velocity_triggered_takeoff;
}
