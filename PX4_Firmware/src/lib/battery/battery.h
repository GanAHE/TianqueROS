/****************************************************************************
 *
 *   Copyright (c) 2019-2020 PX4 Development Team. All rights reserved.
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
 * @file battery.h
 *
 * Library calls for battery functionality.
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Timothy Scott <timothy@auterion.com>
 */

#pragma once

#include <uORB/uORB.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/battery_status.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/module_params.h>
#include <parameters/param.h>
#include <drivers/drv_adc.h>
#include <board_config.h>
#include <px4_platform_common/board_common.h>
#include <math.h>
#include <float.h>

/**
 * BatteryBase is a base class for any type of battery.
 *
 * You can use this class on its own. Or, if you need to implement a custom battery type,
 * you can inherit from this class. See, for example, src/modules/battery_status/AnalogBattery.h
 */
class Battery : public ModuleParams
{
public:
	Battery(int index, ModuleParams *parent);

	~Battery();

	/**
	 * Reset all battery stats and report invalid/nothing.
	 */
	void reset();

	/**
	 * Get the battery cell count
	 */
	int cell_count() { return _params.n_cells; }

	/**
	 * Get the empty voltage per cell
	 */
	float empty_cell_voltage() { return _params.v_empty; }

	/**
	 * Get the full voltage per cell
	 */
	float full_cell_voltage() { return _params.v_charged; }

	/**
	 * Update current battery status message.
	 *
	 * @param voltage_raw: Battery voltage, in Volts
	 * @param current_raw: Battery current, in Amps
	 * @param timestamp: Time at which the ADC was read (use hrt_absolute_time())
	 * @param source: Source type in relation to BAT%d_SOURCE param.
	 * @param priority: The brick number -1. The term priority refers to the Vn connection on the LTC4417
	 * @param throttle_normalized: Throttle of the vehicle, between 0 and 1
	 */
	void updateBatteryStatus(hrt_abstime timestamp, float voltage_v, float current_a, bool connected,
				 int source, int priority, float throttle_normalized);

	/**
	 * Publishes the uORB battery_status message with the most recently-updated data.
	 */
	void publish();

protected:
	struct {
		param_t v_empty;
		param_t v_charged;
		param_t n_cells;
		param_t capacity;
		param_t v_load_drop;
		param_t r_internal;
		param_t low_thr;
		param_t crit_thr;
		param_t emergen_thr;
		param_t source;

		// TODO: These parameters are depracated. They can be removed entirely once the
		//  new version of Firmware has been around for long enough.
		param_t v_empty_old;
		param_t v_charged_old;
		param_t n_cells_old;
		param_t capacity_old;
		param_t v_load_drop_old;
		param_t r_internal_old;
		param_t source_old;
	} _param_handles;

	struct {
		float v_empty;
		float v_charged;
		int n_cells;
		float capacity;
		float v_load_drop;
		float r_internal;
		float low_thr;
		float crit_thr;
		float emergen_thr;
		int source;

		// TODO: These parameters are depracated. They can be removed entirely once the
		//  new version of Firmware has been around for long enough.
		float v_empty_old;
		float v_charged_old;
		int n_cells_old;
		float capacity_old;
		float v_load_drop_old;
		float r_internal_old;
		int source_old;
	} _params;

	battery_status_s _battery_status;

	const int _index;

	bool _first_parameter_update{false};
	virtual void updateParams() override;

	/**
	 * This function helps with migrating to new parameters. It performs several tasks:
	 *  - Update both the old and new parameter values using `param_get(...)`
	 *  - Check if either parameter changed just now
	 *    - If so, display a warning if the deprecated parameter was used
	 *    - Copy the new value over to the other parameter
	 *  - If this is the first time the parameters are fetched, check if they are equal
	 *    - If not, display a warning and copy the value of the deprecated parameter over to the new one
	 * @tparam T Type of the parameter (int or float)
	 * @param old_param Handle to the old deprecated parameter (for example, param_find("BAT_N_CELLS")
	 * @param new_param Handle to the new replacement parameter (for example, param_find("BAT1_N_CELLS")
	 * @param old_val Pointer to the value of the old deprecated parameter
	 * @param new_val Pointer to the value of the new replacement parameter
	 * @param firstcall If true, then this function will not check to see if the values have changed
	 * 					  (Since the old values are uninitialized)
	 * @return True iff either of these parameters changed just now and the migration was done.
	 */
	template<typename T>
	bool migrateParam(param_t old_param, param_t new_param, T *old_val, T *new_val, bool firstcall)
	{

		T previous_old_val = *old_val;
		T previous_new_val = *new_val;

		param_get(old_param, old_val);
		param_get(new_param, new_val);

		if (!firstcall) {
			if ((float) fabs((float) *old_val - (float) previous_old_val) > FLT_EPSILON
			    && (float) fabs((float) *old_val - (float) *new_val) > FLT_EPSILON) {
				param_set_no_notification(new_param, old_val);
				param_get(new_param, new_val);
				return true;

			} else if ((float) fabs((float) *new_val - (float) previous_new_val) > FLT_EPSILON
				   && (float) fabs((float) *old_val - (float) *new_val) > FLT_EPSILON) {
				param_set_no_notification(old_param, new_val);
				param_get(old_param, old_val);
				return true;
			}

		} else {
			if ((float) fabs((float) *old_val - (float) *new_val) > FLT_EPSILON) {
				param_set_no_notification(new_param, old_val);
				param_get(new_param, new_val);
				return true;
			}
		}

		return false;
	}

private:
	void filterVoltage(float voltage_v);
	void filterThrottle(float throttle);
	void filterCurrent(float current_a);
	void sumDischarged(hrt_abstime timestamp, float current_a);
	void estimateRemaining(float voltage_v, float current_a, float throttle);
	void determineWarning(bool connected);
	void computeScale();

	bool _battery_initialized = false;
	float _voltage_filtered_v = -1.f;
	float _throttle_filtered = -1.f;
	float _current_filtered_a = -1.f;
	float _discharged_mah = 0.f;
	float _discharged_mah_loop = 0.f;
	float _remaining_voltage = -1.f;		///< normalized battery charge level remaining based on voltage
	float _remaining = -1.f;			///< normalized battery charge level, selected based on config param
	float _scale = 1.f;
	uint8_t _warning;
	hrt_abstime _last_timestamp;

	orb_advert_t _orb_advert{nullptr};
	int _orb_instance;
};
