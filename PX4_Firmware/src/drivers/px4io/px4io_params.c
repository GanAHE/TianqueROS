/****************************************************************************
 *
 *   Copyright (c) 2015-2018 PX4 Development Team. All rights reserved.
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
 * @file px4io_params.c
 *
 * Parameters defined by the PX4IO driver
 *
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include <px4_platform_common/px4_config.h>
#include <parameters/param.h>

/**
 * Set usage of IO board
 *
 * Can be used to use a standard startup script but with a FMU only set-up. Set to 0 to force the FMU only set-up.
 *
 * @boolean
 * @min 0
 * @max 1
 * @reboot_required true
 * @group System
 */
PARAM_DEFINE_INT32(SYS_USE_IO, 1);

/**
 * S.BUS out
 *
 * Set to 1 to enable S.BUS version 1 output instead of RSSI.
 *
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_SBUS_MODE, 0);

/**
 * PWM input channel that provides RSSI.
 *
 * 0: do not read RSSI from input channel
 * 1-18: read RSSI from specified input channel
 *
 * Specify the range for RSSI input with RC_RSSI_PWM_MIN and RC_RSSI_PWM_MAX parameters.
 *
 * @min 0
 * @max 18
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 * @group Radio Calibration
 *
 */
PARAM_DEFINE_INT32(RC_RSSI_PWM_CHAN, 0);

/**
 * Max input value for RSSI reading.
 *
 * Only used if RC_RSSI_PWM_CHAN > 0
 *
 * @min 0
 * @max 2000
 * @group Radio Calibration
 *
 */
PARAM_DEFINE_INT32(RC_RSSI_PWM_MAX, 1000);

/**
 * Min input value for RSSI reading.
 *
 * Only used if RC_RSSI_PWM_CHAN > 0
 *
 * @min 0
 * @max 2000
 * @group Radio Calibration
 *
 */
PARAM_DEFINE_INT32(RC_RSSI_PWM_MIN, 2000);
