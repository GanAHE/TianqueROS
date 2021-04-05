#ifdef __PX4_NUTTX

#include <sys/ioctl.h>
#include <lib/mathlib/mathlib.h>

#include "drivers/drv_pwm_trigger.h"
#include "seagull_map2.h"

// PWM levels of the interface to Seagull MAP 2 converter to
// Multiport (http://www.seagulluav.com/manuals/Seagull_MAP2-Manual.pdf)
#define PWM_CAMERA_DISARMED			900
#define PWM_CAMERA_NEUTRAL			1500
#define PWM_1_CAMERA_ON				1100
#define PWM_1_CAMERA_AUTOFOCUS_SHOOT	1300
#define PWM_1_CAMERA_INSTANT_SHOOT	1700
#define PWM_1_CAMERA_OFF				1900
#define PWM_2_CAMERA_KEEP_ALIVE		1700
#define PWM_2_CAMERA_ON_OFF			1900

CameraInterfaceSeagull::CameraInterfaceSeagull():
	CameraInterface(),
	_camera_is_on(false)
{
	get_pins();
	setup();
}

CameraInterfaceSeagull::~CameraInterfaceSeagull()
{
	// Deinitialise trigger channels
	up_pwm_trigger_deinit();
}

void CameraInterfaceSeagull::setup()
{
	for (unsigned i = 0; i < arraySize(_pins); i = i + 2) {
		if (_pins[i] >= 0 && _pins[i + 1] >= 0) {

			// Initialize the interface
			uint8_t pin_bitmask = (1 << _pins[i + 1]) | (1 << _pins[i]);
			up_pwm_trigger_init(pin_bitmask);

			// Set both interface pins to disarmed
			up_pwm_trigger_set(_pins[i + 1], PWM_CAMERA_DISARMED);
			up_pwm_trigger_set(_pins[i], PWM_CAMERA_DISARMED);

			// We only support 2 consecutive pins while using the Seagull MAP2
			return;
		}
	}

	PX4_ERR("Bad pin configuration - Seagull MAP2 requires 2 consecutive pins for control.");
}

void CameraInterfaceSeagull::trigger(bool trigger_on_true)
{

	if (!_camera_is_on) {
		return;
	}

	for (unsigned i = 0; i < arraySize(_pins); i = i + 2) {
		if (_pins[i] >= 0 && _pins[i + 1] >= 0) {
			// Set channel 1 to shoot or neutral levels
			up_pwm_trigger_set(_pins[i + 1], trigger_on_true ? PWM_1_CAMERA_INSTANT_SHOOT : PWM_CAMERA_NEUTRAL);
		}
	}
}

void CameraInterfaceSeagull::send_keep_alive(bool enable)
{
	// This should alternate between enable and !enable to keep the camera alive

	if (!_camera_is_on) {
		return;
	}

	for (unsigned i = 0; i < arraySize(_pins); i = i + 2) {
		if (_pins[i] >= 0 && _pins[i + 1] >= 0) {
			// Set channel 2 pin to keep_alive or netural signal
			up_pwm_trigger_set(_pins[i], enable ? PWM_2_CAMERA_KEEP_ALIVE : PWM_CAMERA_NEUTRAL);
		}
	}
}

void CameraInterfaceSeagull::send_toggle_power(bool enable)
{

	// This should alternate between enable and !enable to toggle camera power

	for (unsigned i = 0; i < arraySize(_pins); i = i + 2) {
		if (_pins[i] >= 0 && _pins[i + 1] >= 0) {
			// Set channel 1 to neutral
			up_pwm_trigger_set(_pins[i + 1], PWM_CAMERA_NEUTRAL);
			// Set channel 2 to on_off or neutral signal
			up_pwm_trigger_set(_pins[i], enable ? PWM_2_CAMERA_ON_OFF : PWM_CAMERA_NEUTRAL);
		}
	}

	if (!enable) { _camera_is_on = !_camera_is_on; }
}

void CameraInterfaceSeagull::info()
{
	PX4_INFO("PWM trigger mode (Seagull MAP2) , pins enabled : [%d][%d][%d][%d][%d][%d]",
		 _pins[5], _pins[4], _pins[3], _pins[2], _pins[1], _pins[0]);
}

#endif /* ifdef __PX4_NUTTX */
