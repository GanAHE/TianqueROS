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
 * @file temp_comp_params_gyro.c
 *
 * Parameters required for temperature compensation of rate gyros.
 *
 * @author Paul Riseborough <gncsolns@gmail.com>
 */

/**
 * Thermal compensation for rate gyro sensors.
 *
 * @group Thermal Compensation
 * @min 0
 * @max 1
 * @boolean
 */
PARAM_DEFINE_INT32(TC_G_ENABLE, 0);

/* Gyro 0 */

/**
 * ID of Gyro that the calibration is for.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_INT32(TC_G0_ID, 0);

/**
 * Gyro rate offset temperature ^3 polynomial coefficient - X axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G0_X3_0, 0.0f);

/**
 * Gyro rate offset temperature ^3 polynomial coefficient - Y axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G0_X3_1, 0.0f);

/**
 * Gyro rate offset temperature ^3 polynomial coefficient - Z axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G0_X3_2, 0.0f);

/**
 * Gyro rate offset temperature ^2 polynomial coefficient - X axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G0_X2_0, 0.0f);

/**
 * Gyro rate offset temperature ^2 polynomial coefficient - Y axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G0_X2_1, 0.0f);

/**
 * Gyro rate offset temperature ^2 polynomial coefficient - Z axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G0_X2_2, 0.0f);

/**
 * Gyro rate offset temperature ^1 polynomial coefficient - X axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G0_X1_0, 0.0f);

/**
 * Gyro rate offset temperature ^1 polynomial coefficient - Y axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G0_X1_1, 0.0f);

/**
 * Gyro rate offset temperature ^1 polynomial coefficient - Z axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G0_X1_2, 0.0f);

/**
 * Gyro rate offset temperature ^0 polynomial coefficient - X axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G0_X0_0, 0.0f);

/**
 * Gyro rate offset temperature ^0 polynomial coefficient - Y axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G0_X0_1, 0.0f);

/**
 * Gyro rate offset temperature ^0 polynomial coefficient - Z axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G0_X0_2, 0.0f);

/**
 * Gyro scale factor - X axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G0_SCL_0, 1.0f);

/**
 * Gyro scale factor - Y axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G0_SCL_1, 1.0f);

/**
 * Gyro scale factor - Z axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G0_SCL_2, 1.0f);

/**
 * Gyro calibration reference temperature.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G0_TREF, 25.0f);

/**
 * Gyro calibration minimum temperature.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G0_TMIN, 0.0f);

/**
 * Gyro calibration maximum temperature.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G0_TMAX, 100.0f);

/* Gyro 1 */

/**
 * ID of Gyro that the calibration is for.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_INT32(TC_G1_ID, 0);

/**
 * Gyro rate offset temperature ^3 polynomial coefficient - X axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G1_X3_0, 0.0f);

/**
 * Gyro rate offset temperature ^3 polynomial coefficient - Y axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G1_X3_1, 0.0f);

/**
 * Gyro rate offset temperature ^3 polynomial coefficient - Z axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G1_X3_2, 0.0f);

/**
 * Gyro rate offset temperature ^2 polynomial coefficient - X axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G1_X2_0, 0.0f);

/**
 * Gyro rate offset temperature ^2 polynomial coefficient - Y axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G1_X2_1, 0.0f);

/**
 * Gyro rate offset temperature ^2 polynomial coefficient - Z axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G1_X2_2, 0.0f);

/**
 * Gyro rate offset temperature ^1 polynomial coefficient - X axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G1_X1_0, 0.0f);

/**
 * Gyro rate offset temperature ^1 polynomial coefficient - Y axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G1_X1_1, 0.0f);

/**
 * Gyro rate offset temperature ^1 polynomial coefficient - Z axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G1_X1_2, 0.0f);

/**
 * Gyro rate offset temperature ^0 polynomial coefficient - X axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G1_X0_0, 0.0f);

/**
 * Gyro rate offset temperature ^0 polynomial coefficient - Y axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G1_X0_1, 0.0f);

/**
 * Gyro rate offset temperature ^0 polynomial coefficient - Z axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G1_X0_2, 0.0f);

/**
 * Gyro scale factor - X axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G1_SCL_0, 1.0f);

/**
 * Gyro scale factor - Y axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G1_SCL_1, 1.0f);

/**
 * Gyro scale factor - Z axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G1_SCL_2, 1.0f);

/**
 * Gyro calibration reference temperature.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G1_TREF, 25.0f);

/**
 * Gyro calibration minimum temperature.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G1_TMIN, 0.0f);

/**
 * Gyro calibration maximum temperature.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G1_TMAX, 100.0f);

/* Gyro 2 */

/**
 * ID of Gyro that the calibration is for.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_INT32(TC_G2_ID, 0);

/**
 * Gyro rate offset temperature ^3 polynomial coefficient - X axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G2_X3_0, 0.0f);

/**
 * Gyro rate offset temperature ^3 polynomial coefficient - Y axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G2_X3_1, 0.0f);

/**
 * Gyro rate offset temperature ^3 polynomial coefficient - Z axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G2_X3_2, 0.0f);

/**
 * Gyro rate offset temperature ^2 polynomial coefficient - X axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G2_X2_0, 0.0f);

/**
 * Gyro rate offset temperature ^2 polynomial coefficient - Y axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G2_X2_1, 0.0f);

/**
 * Gyro rate offset temperature ^2 polynomial coefficient - Z axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G2_X2_2, 0.0f);

/**
 * Gyro rate offset temperature ^1 polynomial coefficient - X axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G2_X1_0, 0.0f);

/**
 * Gyro rate offset temperature ^1 polynomial coefficient - Y axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G2_X1_1, 0.0f);

/**
 * Gyro rate offset temperature ^1 polynomial coefficient - Z axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G2_X1_2, 0.0f);

/**
 * Gyro rate offset temperature ^0 polynomial coefficient - X axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G2_X0_0, 0.0f);

/**
 * Gyro rate offset temperature ^0 polynomial coefficient - Y axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G2_X0_1, 0.0f);

/**
 * Gyro rate offset temperature ^0 polynomial coefficient - Z axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G2_X0_2, 0.0f);

/**
 * Gyro scale factor - X axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G2_SCL_0, 1.0f);

/**
 * Gyro scale factor - Y axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G2_SCL_1, 1.0f);

/**
 * Gyro scale factor - Z axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G2_SCL_2, 1.0f);

/**
 * Gyro calibration reference temperature.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G2_TREF, 25.0f);

/**
 * Gyro calibration minimum temperature.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G2_TMIN, 0.0f);

/**
 * Gyro calibration maximum temperature.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_G2_TMAX, 100.0f);
