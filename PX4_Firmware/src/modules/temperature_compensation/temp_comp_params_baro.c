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
 * @file temp_comp_params_baro.c
 *
 * Parameters required for temperature compensation of barometers.
 *
 * @author Paul Riseborough <gncsolns@gmail.com>
 */

/**
 * Thermal compensation for barometric pressure sensors.
 *
 * @group Thermal Compensation
 * @min 0
 * @max 1
 * @boolean
 */
PARAM_DEFINE_INT32(TC_B_ENABLE, 0);

/* Barometer 0 */

/**
 * ID of Barometer that the calibration is for.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_INT32(TC_B0_ID, 0);

/**
 * Barometer offset temperature ^5 polynomial coefficient.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_B0_X5, 0.0f);

/**
 * Barometer offset temperature ^4 polynomial coefficient.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_B0_X4, 0.0f);

/**
 * Barometer offset temperature ^3 polynomial coefficient.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_B0_X3, 0.0f);

/**
 * Barometer offset temperature ^2 polynomial coefficient.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_B0_X2, 0.0f);

/**
 * Barometer offset temperature ^1 polynomial coefficients.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_B0_X1, 0.0f);

/**
 * Barometer offset temperature ^0 polynomial coefficient.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_B0_X0, 0.0f);

/**
 * Barometer scale factor - X axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_B0_SCL, 1.0f);

/**
 * Barometer calibration reference temperature.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_B0_TREF, 40.0f);

/**
 * Barometer calibration minimum temperature.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_B0_TMIN, 5.0f);

/**
 * Barometer calibration maximum temperature.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_B0_TMAX, 75.0f);

/* Barometer 1 */

/**
 * ID of Barometer that the calibration is for.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_INT32(TC_B1_ID, 0);

/**
 * Barometer offset temperature ^5 polynomial coefficient.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_B1_X5, 0.0f);

/**
 * Barometer offset temperature ^4 polynomial coefficient.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_B1_X4, 0.0f);

/**
 * Barometer offset temperature ^3 polynomial coefficient.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_B1_X3, 0.0f);

/**
 * Barometer offset temperature ^2 polynomial coefficient.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_B1_X2, 0.0f);

/**
 * Barometer offset temperature ^1 polynomial coefficients.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_B1_X1, 0.0f);

/**
 * Barometer offset temperature ^0 polynomial coefficient.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_B1_X0, 0.0f);

/**
 * Barometer scale factor - X axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_B1_SCL, 1.0f);

/**
 * Barometer calibration reference temperature.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_B1_TREF, 40.0f);

/**
 * Barometer calibration minimum temperature.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_B1_TMIN, 5.0f);

/**
 * Barometer calibration maximum temperature.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_B1_TMAX, 75.0f);

/* Barometer 2 */

/**
 * ID of Barometer that the calibration is for.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_INT32(TC_B2_ID, 0);

/**
 * Barometer offset temperature ^5 polynomial coefficient.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_B2_X5, 0.0f);

/**
 * Barometer offset temperature ^4 polynomial coefficient.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_B2_X4, 0.0f);

/**
 * Barometer offset temperature ^3 polynomial coefficient.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_B2_X3, 0.0f);

/**
 * Barometer offset temperature ^2 polynomial coefficient.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_B2_X2, 0.0f);

/**
 * Barometer offset temperature ^1 polynomial coefficients.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_B2_X1, 0.0f);

/**
 * Barometer offset temperature ^0 polynomial coefficient.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_B2_X0, 0.0f);

/**
 * Barometer scale factor - X axis.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_B2_SCL, 1.0f);

/**
 * Barometer calibration reference temperature.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_B2_TREF, 40.0f);

/**
 * Barometer calibration minimum temperature.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_B2_TMIN, 5.0f);

/**
 * Barometer calibration maximum temperature.
 *
 * @group Thermal Compensation
 * @category system
 */
PARAM_DEFINE_FLOAT(TC_B2_TMAX, 75.0f);
