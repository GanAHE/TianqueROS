/****************************************************************************
*
*   Copyright (c) 2016-2017 PX4 Development Team. All rights reserved.
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
 * @file input_mavlink.h
 * @author Beat Küng <beat-kueng@gmx.net>
 *
 */

#pragma once

#include "input.h"
#include "input_rc.h"
#include <cstdint>

#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_roi.h>

namespace vmount
{
/**
 ** class InputMavlinkROI
 ** Input based on the vehicle_roi topic
 */
class InputMavlinkROI : public InputBase
{
public:
	InputMavlinkROI() = default;
	virtual ~InputMavlinkROI();

	virtual void print_status();

protected:
	virtual int update_impl(unsigned int timeout_ms, ControlData **control_data, bool already_active);
	virtual int initialize();

private:
	void _read_control_data_from_position_setpoint_sub();

	int _vehicle_roi_sub = -1;
	int _position_setpoint_triplet_sub = -1;
	uint8_t _cur_roi_mode = vehicle_roi_s::ROI_NONE;
};


/**
 ** class InputMavlinkCmdMount
 ** Input based on the VEHICLE_CMD_DO_MOUNT_CONTROL mavlink command
 */
class InputMavlinkCmdMount : public InputBase
{
public:
	InputMavlinkCmdMount(bool stabilize);
	virtual ~InputMavlinkCmdMount();

	virtual void print_status();

protected:
	virtual int update_impl(unsigned int timeout_ms, ControlData **control_data, bool already_active);
	virtual int initialize();

private:
	void _ack_vehicle_command(vehicle_command_s *cmd);

	int _vehicle_command_sub = -1;
	bool _stabilize[3] = { false, false, false };

	int32_t _mav_sys_id{1}; ///< our mavlink system id
	int32_t _mav_comp_id{1}; ///< our mavlink component id
};


} /* namespace vmount */
