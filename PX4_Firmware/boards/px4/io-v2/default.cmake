
px4_add_board(
	PLATFORM nuttx
	VENDOR px4
	MODEL io-v2
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m3
	DRIVERS
	MODULES
		px4iofirmware
	)
