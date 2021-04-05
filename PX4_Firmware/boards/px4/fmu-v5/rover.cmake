
px4_add_board(
	PLATFORM nuttx
	VENDOR px4
	MODEL fmu-v5
	LABEL rover
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m7
	ROMFSROOT px4fmu_common
	IO px4_io-v2_default
	UAVCAN_INTERFACES 2
	SERIAL_PORTS
		GPS1:/dev/ttyS0
		TEL1:/dev/ttyS1
		TEL2:/dev/ttyS2
		TEL4:/dev/ttyS3
	DRIVERS
		adc
		barometer # all available barometer drivers
		batt_smbus
		camera_capture
		camera_trigger
		distance_sensor # all available distance sensor drivers
		gps
		imu/adis16448
		imu/adis16477
		imu/adis16497
		imu/bmi055
		imu/invensense/icm20602
		imu/invensense/icm20689
		#imu/mpu6000 # legacy icm20602/icm20689 driver
		lights/rgbled
		lights/rgbled_ncp5623c
		lights/rgbled_pwm
		magnetometer # all available magnetometer drivers
		mkblctrl
		optical_flow # all available optical flow drivers
		pca9685
		pwm_input
		pwm_out_sim
		pwm_out
		px4io
		rc_input
		roboclaw
		safety_button
		telemetry # all available telemetry drivers
		tone_alarm
		uavcan
	MODULES
		battery_status
		camera_feedback
		commander
		dataman
		ekf2
		events
		land_detector
		load_mon
		logger
		mavlink
		navigator
		rc_update
		rover_pos_control
		sensors
		temperature_compensation
		vmount
	SYSTEMCMDS
		bl_update
		config
		dmesg
		dumpfile
		esc_calib
		hardfault_log
		i2cdetect
		led_control
		mixer
		motor_ramp
		motor_test
		mtd
		nshterm
		param
		perf
		pwm
		reboot
		reflect
		sd_bench
		shutdown
		top
		topic_listener
		tune_control
		usb_connected
		ver
		work_queue
	)
