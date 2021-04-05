/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file temp_comp_params_accel.c
 *
 * Parameters required for temperature compensation of rate Accelerometers.
 *
 * @author Paul Riseborough <gncsolns@gmail.com>
 */

/**
 * Thermal compensation for accelerometer sensors.
 *
 * @group Thermal Compensation
 * @min 0
 * @max 1
 * @boolean
 */
PARAM_DEFINE_INT32(TC_A_ENABLE, 0);

/* Accelerometer 0 */

/**
 * ID of Accelerometer that the calibration is for.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_INT32(TC_A0_ID, 0);

/**
 * Accelerometer offset temperature ^3 polynomial coefficient - X axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A0_X3_0, 0.0f);

/**
 * Accelerometer offset temperature ^3 polynomial coefficient - Y axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A0_X3_1, 0.0f);

/**
 * Accelerometer offset temperature ^3 polynomial coefficient - Z axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A0_X3_2, 0.0f);

/**
 * Accelerometer offset temperature ^2 polynomial coefficient - X axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A0_X2_0, 0.0f);

/**
 * Accelerometer offset temperature ^2 polynomial coefficient - Y axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A0_X2_1, 0.0f);

/**
 * Accelerometer offset temperature ^2 polynomial coefficient - Z axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A0_X2_2, 0.0f);

/**
 * Accelerometer offset temperature ^1 polynomial coefficient - X axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A0_X1_0, 0.0f);

/**
 * Accelerometer offset temperature ^1 polynomial coefficient - Y axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A0_X1_1, 0.0f);

/**
 * Accelerometer offset temperature ^1 polynomial coefficient - Z axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A0_X1_2, 0.0f);

/**
 * Accelerometer offset temperature ^0 polynomial coefficient - X axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A0_X0_0, 0.0f);

/**
 * Accelerometer offset temperature ^0 polynomial coefficient - Y axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A0_X0_1, 0.0f);

/**
 * Accelerometer offset temperature ^0 polynomial coefficient - Z axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A0_X0_2, 0.0f);

/**
 * Accelerometer scale factor - X axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A0_SCL_0, 1.0f);

/**
 * Accelerometer scale factor - Y axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A0_SCL_1, 1.0f);

/**
 * Accelerometer scale factor - Z axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A0_SCL_2, 1.0f);

/**
 * Accelerometer calibration reference temperature.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A0_TREF, 25.0f);

/**
 * Accelerometer calibration minimum temperature.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A0_TMIN, 0.0f);

/**
 * Accelerometer calibration maximum temperature.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A0_TMAX, 100.0f);

/* Accelerometer 1 */

/**
 * ID of Accelerometer that the calibration is for.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_INT32(TC_A1_ID, 0);

/**
 * Accelerometer offset temperature ^3 polynomial coefficient - X axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A1_X3_0, 0.0f);

/**
 * Accelerometer offset temperature ^3 polynomial coefficient - Y axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A1_X3_1, 0.0f);

/**
 * Accelerometer offset temperature ^3 polynomial coefficient - Z axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A1_X3_2, 0.0f);

/**
 * Accelerometer offset temperature ^2 polynomial coefficient - X axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A1_X2_0, 0.0f);

/**
 * Accelerometer offset temperature ^2 polynomial coefficient - Y axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A1_X2_1, 0.0f);

/**
 * Accelerometer offset temperature ^2 polynomial coefficient - Z axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A1_X2_2, 0.0f);

/**
 * Accelerometer offset temperature ^1 polynomial coefficient - X axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A1_X1_0, 0.0f);

/**
 * Accelerometer offset temperature ^1 polynomial coefficient - Y axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A1_X1_1, 0.0f);

/**
 * Accelerometer offset temperature ^1 polynomial coefficient - Z axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A1_X1_2, 0.0f);

/**
 * Accelerometer offset temperature ^0 polynomial coefficient - X axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A1_X0_0, 0.0f);

/**
 * Accelerometer offset temperature ^0 polynomial coefficient - Y axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A1_X0_1, 0.0f);

/**
 * Accelerometer offset temperature ^0 polynomial coefficient - Z axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A1_X0_2, 0.0f);

/**
 * Accelerometer scale factor - X axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A1_SCL_0, 1.0f);

/**
 * Accelerometer scale factor - Y axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A1_SCL_1, 1.0f);

/**
 * Accelerometer scale factor - Z axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A1_SCL_2, 1.0f);

/**
 * Accelerometer calibration reference temperature.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A1_TREF, 25.0f);

/**
 * Accelerometer calibration minimum temperature.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A1_TMIN, 0.0f);

/**
 * Accelerometer calibration maximum temperature.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A1_TMAX, 100.0f);

/* Accelerometer 2 */

/**
 * ID of Accelerometer that the calibration is for.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_INT32(TC_A2_ID, 0);

/**
 * Accelerometer offset temperature ^3 polynomial coefficient - X axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A2_X3_0, 0.0f);

/**
 * Accelerometer offset temperature ^3 polynomial coefficient - Y axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A2_X3_1, 0.0f);

/**
 * Accelerometer offset temperature ^3 polynomial coefficient - Z axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A2_X3_2, 0.0f);

/**
 * Accelerometer offset temperature ^2 polynomial coefficient - X axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A2_X2_0, 0.0f);

/**
 * Accelerometer offset temperature ^2 polynomial coefficient - Y axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A2_X2_1, 0.0f);

/**
 * Accelerometer offset temperature ^2 polynomial coefficient - Z axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A2_X2_2, 0.0f);

/**
 * Accelerometer offset temperature ^1 polynomial coefficient - X axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A2_X1_0, 0.0f);

/**
 * Accelerometer offset temperature ^1 polynomial coefficient - Y axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A2_X1_1, 0.0f);

/**
 * Accelerometer offset temperature ^1 polynomial coefficient - Z axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A2_X1_2, 0.0f);

/**
 * Accelerometer offset temperature ^0 polynomial coefficient - X axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A2_X0_0, 0.0f);

/**
 * Accelerometer offset temperature ^0 polynomial coefficient - Y axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A2_X0_1, 0.0f);

/**
 * Accelerometer offset temperature ^0 polynomial coefficient - Z axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A2_X0_2, 0.0f);

/**
 * Accelerometer scale factor - X axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A2_SCL_0, 1.0f);

/**
 * Accelerometer scale factor - Y axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A2_SCL_1, 1.0f);

/**
 * Accelerometer scale factor - Z axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A2_SCL_2, 1.0f);

/**
 * Accelerometer calibration reference temperature.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A2_TREF, 25.0f);

/**
 * Accelerometer calibration minimum temperature.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A2_TMIN, 0.0f);

/**
 * Accelerometer calibration maximum temperature.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_A2_TMAX, 100.0f);
