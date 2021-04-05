/****************************************************************************
 *
 *  Copyright (C) 2016-2019 PX4 Development Team. All rights reserved.
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
 * @file test_autodeclination.cpp
 * Test for autodeclination values.
 */

#include <unit_test.h>

#include <drivers/drv_hrt.h>
#include <lib/ecl/geo/geo.h>
#include <ecl/geo_lookup/geo_mag_declination.h>
#include <px4iofirmware/px4io.h>
#include <systemlib/err.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

class AutoDeclinationTest : public UnitTest
{
public:
	virtual bool run_tests();

private:
	bool autodeclination_check();
};

bool AutoDeclinationTest::autodeclination_check()
{
	// Test world data using data from Magnetic Model: IGRF12 (calculator version 0.5.0.7)
	// Decimal Year 2018.05753

	// ut_compare_float last argument is precision
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(60, -180), 2.76391, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(60, -170), 7.65238, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(60, -160), 12.17294, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(60, -150), 16.01107, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(60, -140), 18.72801, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(60, -130), 19.73724, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(60, -120), 18.25112, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(60, -110), 13.26654, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(60, -100), 4.13317, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(60, -90), -7.62673, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(60, -80), -17.84456, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(60, -70), -23.58845, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(60, -60), -25.01681, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(60, -50), -23.47267, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(60, -40), -20.44728, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(60, -30), -15.8376, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(60, -20), -11.04886, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(60, -10), -6.13803, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(60, 0), -1.38702, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(60, 10), 3.0231, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(60, 20), 7.04485, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(60, 30), 10.68855, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(60, 40), 13.86345, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(60, 50), 16.27102, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(60, 60), 17.41732, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(60, 70), 16.69502, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(60, 80), 13.51634, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(60, 90), 7.64291, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(60, 100), -0.46206, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(60, 110), -7.83102, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(60, 120), -13.14815, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(60, 130), -15.19585, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(60, 140), -14.314, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(60, 150), -11.33504, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(60, 160), -7.09405, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(60, 170), -2.24693, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(60, 180), 2.76391, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(50, -180), 3.39297, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(50, -170), 7.64688, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(50, -160), 11.46075, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(50, -150), 14.55849, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(50, -140), 16.58104, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(50, -130), 17.12886, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(50, -120), 15.76953, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(50, -110), 11.9804, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(50, -100), 5.33907, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(50, -90), -3.51221, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(50, -80), -12.05701, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(50, -70), -17.6109, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(50, -60), -19.46549, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(50, -50), -18.43554, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(50, -40), -15.64288, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(50, -30), -11.94753, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(50, -20), -7.88526, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(50, -10), -3.85052, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(50, 0), -0.21495, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(50, 10), 2.80897, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(50, 20), 5.29489, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(50, 30), 7.40813, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(50, 40), 9.12158, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(50, 50), 10.20849, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(50, 60), 10.40516, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(50, 70), 9.47967, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(50, 80), 7.22842, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(50, 90), 3.55584, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(50, 100), -1.29281, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(50, 110), -6.43861, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(50, 120), -10.49228, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(50, 130), -12.34449, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(50, 140), -11.76562, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(50, 150), -9.22608, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(50, 160), -5.4404, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(50, 170), -1.07169, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(50, 180), 3.39297, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(40, -180), 4.73304, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(40, -170), 8.06967, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(40, -160), 10.84965, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(40, -150), 13.03592, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(40, -140), 14.4382, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(40, -130), 14.73187, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(40, -120), 13.57059, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(40, -110), 10.56659, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(40, -100), 5.33695, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(40, -90), -1.83952, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(40, -80), -9.22582, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(40, -70), -14.50199, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(40, -60), -16.59645, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(40, -50), -15.90646, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(40, -40), -13.42777, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(40, -30), -10.05355, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(40, -20), -6.33304, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(40, -10), -2.70446, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(40, 0), 0.32852, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(40, 10), 2.53857, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(40, 20), 4.16517, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(40, 30), 5.40477, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(40, 40), 6.09072, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(40, 50), 6.10984, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(40, 60), 5.65289, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(40, 70), 4.84976, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(40, 80), 3.54739, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(40, 90), 1.50963, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(40, 100), -1.37218, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(40, 110), -4.8133, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(40, 120), -7.86308, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(40, 130), -9.35691, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(40, 140), -8.79624, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(40, 150), -6.4712, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(40, 160), -3.00633, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(40, 170), 0.92526, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(40, 180), 4.73304, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(30, -180), 6.51208, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(30, -170), 8.60794, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(30, -160), 10.40201, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(30, -150), 11.35813, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(30, -140), 12.28963, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(30, -130), 12.46921, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(30, -120), 11.53386, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(30, -110), 9.18036, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(30, -100), 5.02502, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(30, -90), -0.91666, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(30, -80), -7.48211, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(30, -70), -12.75024, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(30, -60), -15.41473, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(30, -50), -15.34632, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(30, -40), -13.23131, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(30, -30), -9.97999, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(30, -20), -6.26609, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(30, -10), -2.67489, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(30, 0), 0.42859, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(30, 10), 1.98719, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(30, 20), 3.35693, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(30, 30), 4.30231, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(30, 40), 4.34228, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(30, 50), 3.53408, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(30, 60), 2.61371, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(30, 70), 1.94408, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(30, 80), 1.27365, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(30, 90), 0.30968, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(30, 100), -1.17718, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(30, 110), -3.29027, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(30, 120), -5.41121, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(30, 130), -6.35606, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(30, 140), -5.52501, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(30, 150), -3.19361, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(30, 160), 0.06272, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(30, 170), 3.54892, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(30, 180), 6.51208, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(20, -180), 8.05455, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(20, -170), 8.9018, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(20, -160), 9.28221, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(20, -150), 9.82187, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(20, -140), 10.36589, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(20, -130), 10.39581, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(20, -120), 9.69265, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(20, -110), 8.06385, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(20, -100), 4.97456, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(20, -90), 0.08873, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(20, -80), -5.94379, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(20, -70), -11.56551, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(20, -60), -15.26153, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(20, -50), -16.24801, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(20, -40), -14.66383, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(20, -30), -11.38606, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(20, -20), -7.39534, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(20, -10), -3.57892, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(20, 0), -0.73934, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(20, 10), 1.12744, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(20, 20), 2.64851, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(20, 30), 3.6306, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(20, 40), 3.29003, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(20, 50), 1.9185, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(20, 60), 0.68749, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(20, 70), 0.05693, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(20, 80), -0.26716, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(20, 90), -0.47945, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(20, 100), -0.85743, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(20, 110), -1.87811, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(20, 120), -3.17942, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(20, 130), -3.50309, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(20, 140), -2.25553, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(20, 150), 0.44973, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(20, 160), 3.15526, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(20, 170), 6.0726, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(20, 180), 8.05455, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(10, -180), 8.96811, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(10, -170), 9.00326, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(10, -160), 8.80988, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(10, -150), 8.96581, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(10, -140), 9.20037, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(10, -130), 9.0534, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(10, -120), 8.5718, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(10, -110), 7.65493, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(10, -100), 5.53227, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(10, -90), 1.47753, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(10, -80), -4.32444, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(10, -70), -10.57652, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(10, -60), -15.55305, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(10, -50), -17.91817, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(10, -40), -17.19243, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(10, -30), -14.02084, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(10, -20), -9.68895, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(10, -10), -5.47648, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(10, 0), -2.29568, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(10, 10), -0.0338, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(10, 20), 1.88618, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(10, 30), 2.9443, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(10, 40), 2.24776, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(10, 50), 0.43529, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(10, 60), -1.02026, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(10, 70), -1.62575, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(10, 80), -1.61789, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(10, 90), -1.07428, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(10, 100), -0.43078, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(10, 110), -0.57046, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(10, 120), -1.25424, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(10, 130), -1.04428, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(10, 140), 0.58806, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(10, 150), 3.00104, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(10, 160), 5.63284, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(10, 170), 7.86621, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(10, 180), 8.96811, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(0, -180), 9.65068, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(0, -170), 9.46759, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(0, -160), 9.21732, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(0, -150), 9.26704, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(0, -140), 9.31917, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(0, -130), 9.03556, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(0, -120), 8.63887, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(0, -110), 8.14347, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(0, -100), 6.70051, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(0, -90), 3.24587, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(0, -80), -2.53824, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(0, -70), -9.5519, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(0, -60), -15.78848, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(0, -50), -19.60215, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(0, -40), -20.46832, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(0, -30), -17.73341, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(0, -20), -13.52156, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(0, -10), -8.9561, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(0, 0), -5.01498, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(0, 10), -1.82178, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(0, 20), 0.69154, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(0, 30), 1.62948, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(0, 40), 0.31861, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(0, 50), -2.02581, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(0, 60), -3.63856, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(0, 70), -4.02958, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(0, 80), -3.41245, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(0, 90), -1.87563, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(0, 100), -0.45464, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(0, 110), 0.48183, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(0, 120), 0.27308, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(0, 130), 0.85887, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(0, 140), 2.71814, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(0, 150), 5.06264, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(0, 160), 7.34057, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(0, 170), 9.03413, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(0, 180), 9.65068, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-10, -180), 10.85622, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-10, -170), 10.83952, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-10, -160), 10.78586, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-10, -150), 10.86624, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-10, -140), 10.82903, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-10, -130), 10.44786, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-10, -120), 9.97844, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-10, -110), 9.5716, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-10, -100), 8.51933, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-10, -90), 5.5268, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-10, -80), -0.25975, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-10, -70), -8.02128, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-10, -60), -15.46443, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-10, -50), -20.65426, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-10, -40), -22.77406, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-10, -30), -21.95047, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-10, -20), -18.99256, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-10, -10), -14.69503, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-10, 0), -9.76978, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-10, 10), -5.16636, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-10, 20), -2.08706, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-10, 30), -1.81675, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-10, 40), -4.23367, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-10, 50), -7.17513, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-10, 60), -8.6249, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-10, 70), -8.28432, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-10, 80), -6.54836, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-10, 90), -3.6667, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-10, 100), -0.68901, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-10, 110), 0.85877, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-10, 120), 1.26385, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-10, 130), 2.23753, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-10, 140), 4.26718, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-10, 150), 6.62102, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-10, 160), 8.78297, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-10, 170), 10.29336, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-10, 180), 10.85622, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-20, -180), 13.11268, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-20, -170), 13.37579, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-20, -160), 13.53407, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-20, -150), 13.64149, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-20, -140), 13.52041, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-20, -130), 13.0848, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-20, -120), 12.59068, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-20, -110), 12.22644, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-20, -100), 11.40926, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-20, -90), 8.75137, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-20, -80), 3.04903, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-20, -70), -5.25076, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-20, -60), -13.82106, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-20, -50), -20.32136, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-20, -40), -23.88942, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-20, -30), -24.94171, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-20, -20), -24.16991, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-20, -10), -21.45897, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-20, 0), -16.6749, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-20, 10), -11.59204, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-20, 20), -9.18683, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-20, 30), -10.67507, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-20, 40), -14.34292, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-20, 50), -17.27975, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-20, 60), -17.78731, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-20, 70), -15.96469, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-20, 80), -12.46302, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-20, 90), -7.75001, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-20, 100), -3.08968, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-20, 110), -0.40163, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-20, 120), 1.485, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-20, 130), 3.24596, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-20, 140), 5.70537, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-20, 150), 8.34017, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-20, 160), 10.69411, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-20, 170), 12.34361, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-20, 180), 13.11268, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-30, -180), 16.76996, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-30, -170), 17.25561, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-30, -160), 17.48308, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-30, -150), 17.5041, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-30, -140), 17.28097, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-30, -130), 16.92057, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-30, -120), 16.6716, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-30, -110), 16.5491, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-30, -100), 15.87065, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-30, -90), 13.296, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-30, -80), 7.67066, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-30, -70), -0.78849, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-30, -60), -10.06988, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-30, -50), -17.66004, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-30, -40), -22.38439, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-30, -30), -24.71024, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-30, -20), -25.51542, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-30, -10), -24.76053, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-30, 0), -22.32543, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-30, 10), -20.28166, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-30, 20), -21.40386, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-30, 30), -25.30151, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-30, 40), -29.46545, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-30, 50), -31.84686, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-30, 60), -31.4949, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-30, 70), -28.43575, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-30, 80), -23.14502, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-30, 90), -16.2011, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-30, 100), -9.03586, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-30, 110), -3.45507, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-30, 120), 0.46804, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-30, 130), 3.93373, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-30, 140), 7.48026, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-30, 150), 10.84895, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-30, 160), 13.6994, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-30, 170), 15.69862, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-30, 180), 16.76996, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-40, -180), 22.27152, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-40, -170), 22.79516, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-40, -160), 22.88573, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-40, -150), 22.72737, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-40, -140), 22.47926, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-40, -130), 22.36228, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-40, -120), 22.48086, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-40, -110), 22.51851, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-40, -100), 21.626, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-40, -90), 18.71198, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-40, -80), 13.05342, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-40, -70), 4.93397, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-40, -60), -4.10539, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-40, -50), -11.9663, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-40, -40), -17.34478, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-40, -30), -20.32802, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-40, -20), -21.74287, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-40, -10), -22.31022, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-40, 0), -23.00178, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-40, 10), -25.48173, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-40, 20), -30.36043, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-40, 30), -36.2258, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-40, 40), -41.34601, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-40, 50), -44.67424, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-40, 60), -45.61906, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-40, 70), -43.82595, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-40, 80), -39.06141, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-40, 90), -31.25601, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-40, 100), -21.32532, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-40, 110), -11.44307, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-40, 120), -3.04971, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-40, 130), 3.96484, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-40, 140), 9.91892, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-40, 150), 14.80155, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-40, 160), 18.51373, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-40, 170), 20.96666, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-40, 180), 22.27152, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-50, -180), 30.84148, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-50, -170), 31.06389, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-50, -160), 30.81915, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-50, -150), 30.43597, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-50, -140), 30.44219, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-50, -130), 30.03663, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-50, -120), 29.94126, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-50, -110), 29.32525, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-50, -100), 27.46901, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-50, -90), 23.78201, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-50, -80), 18.09163, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-50, -70), 10.83361, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-50, -60), 3.05909, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-50, -50), -3.92584, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-50, -40), -9.21168, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-50, -30), -12.73883, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-50, -20), -15.21664, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-50, -10), -17.72082, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-50, 0), -21.32895, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-50, 10), -26.66104, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-50, 20), -33.39547, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-50, 30), -40.49868, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-50, 40), -46.96513, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-50, 50), -52.16043, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-50, 60), -55.7058, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-50, 70), -57.28183, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-50, 80), -56.43297, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-50, 90), -52.34375, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-50, 100), -43.87223, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-50, 110), -30.47619, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-50, 120), -13.97494, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-50, 130), 1.68192, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-50, 140), 13.72947, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-50, 150), 21.87931, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-50, 160), 26.91704, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-50, 170), 29.66768, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-50, 180), 30.84148, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-60, -180), 47.4512, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-60, -170), 46.19458, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-60, -160), 44.76274, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-60, -150), 43.37347, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-60, -140), 42.06866, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-60, -130), 40.71814, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-60, -120), 39.02767, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-60, -110), 36.61839, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-60, -100), 33.16884, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-60, -90), 28.54321, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-60, -80), 22.85854, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-60, -70), 16.49065, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-60, -60), 10.00377, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-60, -50), 3.97552, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-60, -40), -1.24758, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-60, -30), -5.73404, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-60, -20), -9.95543, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-60, -10), -14.56373, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-60, 0), -20.08398, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-60, 10), -26.67759, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-60, 20), -34.08849, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-60, 30), -41.80175, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-60, 40), -49.29185, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-60, 50), -56.18184, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-60, 60), -62.26058, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-60, 70), -67.41773, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-60, 80), -71.54529, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-60, 90), -74.4018, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-60, 100), -75.36581, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-60, 110), -72.73199, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-60, 120), -60.82632, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-60, 130), -21.58385, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-60, 140), 25.6129, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-60, 150), 42.23883, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-60, 160), 47.11516, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-60, 170), 48.07206, 0.3);
	ut_compare_float("declination differs from IGRF12 Magnetic Model", get_mag_declination(-60, 180), 47.4512, 0.3);

	return true;
}

bool AutoDeclinationTest::run_tests()
{
	ut_run_test(autodeclination_check);

	return (_tests_failed == 0);
}

ut_declare_test_c(test_autodeclination, AutoDeclinationTest)
