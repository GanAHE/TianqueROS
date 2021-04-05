/****************************************************************************
 *   Copyright (c) 2015 Mark Charlebois. All rights reserved.
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
 * 3. Neither the name ATLFlight nor the names of its contributors may be
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

#pragma once

/**
 * @file
 * The declarations in this file are released to DspAL users and are used to
 * make file I/O call's for GPIO access.  Many of the data structures
 * are used in the parameter to a particular IOCTL function.
 *
 * @warning
 * The functions specified in this header file have received limited testing
 * and may not function properly.  A subsequent update to this release will include
 * fixes to bugs identified during testing, still ongoing.
 *
 * @par
 * Sample source files are referenced to demonstrate how a particular IOCTL
 * or data structure is used in the context of the POSIX standard file I/O functions
 * (open/close/read/write/ioctl).
 *
 * @par Opening/closing GPIO device
 * Use POSIX open()/close() to open and close the GPIO device. Unlike normal file
 * operations, to have GPIO device function, some configurations need to be
 * done with ioctl() function.
 *
 * @par Configuring GPIO device mode using ioctl
 * DspAL GPIO driver allows users to configure GPIO device into the following
 * modes:
 * - general purpose Input/Output GPIO device
 * - GPIO as interrupt source. In this mode, the GPIO device direction is
 *   explictly configured as input device.
 * Besides, DspAL GPIO driver provides a set of IOCTL options to configure the
 * GPIO device.
 *
 * @par Reading from GPIO input port
 * Use POSIX read() to get the current value of GPIO device. This works for the
 * GPIO device that has been configured into general purpose I/O mode. This
 * works for both input and output device.
 *
 * @par Writing to GPIO output port
 * Use POSIX write() to set the signal on the GPIO device. This works for the
 * GPIO device that has been configured into general purpose I/O mode. This
 * works only on the output device
 *
 * @par Using GPIO as interrupt source
 * Use ioctl() to configure GPIO device as an interrupt source. Users can
 * regsiter and de-register interrupt service handler through ioctl argument.
 *
 * @par
 * Sample source code for read/write data to a GPIO device and using GPIO
 * as interrupt source  is included below:
 * @include gpio_test_imp.c
 */

/**
 * @brief
 * The GPIO device path uses the following format:
 * /dev/gpio-{device number}
 * Device numbers start at 1 and go to up to the max number of GPIO supported
 * by the SoC. There is no indication if the user tries to open a GPIO that has already
 * been mapped to a different function.
 */
#define DEV_FS_GPIO_DEVICE_TYPE_STRING  "/dev/gpio-"
#define DEV_FS_GPIO_SSC_DEVICE_TYPE_STRING  "/dev/gpio_ssc-"

/**
 * @brief
 * GPIO function mode that can be configured through ioctl call
 */
enum DSPAL_GPIO_MODE {
	DSPAL_GPIO_MODE_IO = 0,    /**< general purpose IO mode */
	DSPAL_GPIO_MODE_INT,       /**< interrupt mode  */
	DSPAL_GPIO_MODE_MAX_NUM,   /**< maximum number of DspAL GPIO modes defined*/
};

/**
 * @brief
 * Direction enumerations for configuration: an INPUT to the MSM or an OUTPUT
 * from the MSM
 */
enum DSPAL_GPIO_DIRECTION_TYPE {
	DSPAL_GPIO_DIRECTION_INPUT   = 0,     /**< SET TO INPUT */
	DSPAL_GPIO_DIRECTION_OUTPUT  = 1,     /**< SET TO OUTPU */
	DSPAL_GPIO_DIRECTION_MAX_NUM,         /**< maximum number of DspAL GPIO direction defined */
};

/**
 * @brief
 * Pull value for a GPIO configuration
 */
enum DSPAL_GPIO_PULL_TYPE {
	DSPAL_GPIO_NO_PULL    = 0,    /**< -- Do not specify a pull. */
	DSPAL_GPIO_PULL_DOWN  = 0x1,  /**< -- Pull the GPIO down. */
	DSPAL_GPIO_KEEPER     = 0x2,  /**< -- Designate as a Keeper. */
	DSPAL_GPIO_PULL_UP    = 0x3,  /**< -- Pull the GPIO up. */
	DSPAL_GPIO_PULL_TYPE_MAX_NUM, /**< -- maximum number of DspAL GPIO pull types defined */
};

/**
 * @brief
 * Drive strength to use in the configuration of a GPIO
 */
enum  DSPAL_GPIO_DRIVE_TYPE {
	DSPAL_GPIO_2MA     = 0,    /**< -- Specify a 2 mA drive. */
	DSPAL_GPIO_4MA     = 0x1,  /**< -- Specify a 4 mA drive. */
	DSPAL_GPIO_6MA     = 0x2,  /**< -- Specify a 6 mA drive. */
	DSPAL_GPIO_8MA     = 0x3,  /**< -- Specify an 8 mA drive. */
	DSPAL_GPIO_10MA    = 0x4,  /**< -- Specify a 10 mA drive. */
	DSPAL_GPIO_12MA    = 0x5,  /**< -- Specify a 12 mA drive. */
	DSPAL_GPIO_14MA    = 0x6,  /**< -- Specify a 14 mA drive. */
	DSPAL_GPIO_16MA    = 0x7,  /**< -- Specify a 16 mA drive. */
	DSPAL_GPIO_DRIVE_MAX_NUM,  /**< -- maximum number of DspAL GPIO drive strengths defined. */
};

/**
 * @brief
 * Output value specification for general purpose I/O.
 */
enum DSPAL_GPIO_VALUE_TYPE {
	DSPAL_GPIO_LOW_VALUE = 0,  /**< Drive the output LOW */
	DSPAL_GPIO_HIGH_VALUE = 1, /**< Drive the output HIGH */
	DSPAL_GPIO_VALUE_TYPE_MAX_NUM, /**< maximum number of DspAL GPIO value types defined */
};

/**
 * @brief
 * This enum is used to define the trigger condition of the GPIO interrupt.
 */
enum DSPAL_GPIO_INT_TRIGGER_TYPE {
	DSPAL_GPIOINT_TRIGGER_HIGH = 0,   /**< The GPIO interrupt will trigger only if the input signal is high */
	DSPAL_GPIOINT_TRIGGER_LOW,  /**< The GPIO interrupt will trigger only if the input signal is low */
	DSPAL_GPIOINT_TRIGGER_RISING, /**< The GPIO interrupt will trigger only if the input signal level transitions from low to high */
	DSPAL_GPIOINT_TRIGGER_FALLING, /**< The GPIO interrupt will trigger only if the input signal level transitions from high to low */
	DSPAL_GPIOINT_TRIGGER_DUAL_EDGE, /**< The GPIO interrupt will trigger only if the input signal level transitions from high to low or from low to high.*/
	DSPAL_GPIOINT_TRIGGER_TYPE_MAX_NUM, /**< maximum number of DspAL GPIO interrtup trigger types defined */
};


/**
 * @brief
 * ioctl codes used to extend the functionality of the standard read/write file
 * semantics for the gpio.
 */
enum DSPAL_GPIO_IOCTLS {
	DSPAL_GPIO_IOCTL_INVALID = -1, /**< invalid IOCTL code, used to return an error */
	DSPAL_GPIO_IOCTL_CONFIG_IO,    /**< configure GPIO device into general purpose I/O mode */
	DSPAL_GPIO_IOCTL_CONFIG_REG_INT,   /**< configure GPIO device into interrupt mode */
	DSPAL_GPIO_IOCTL_CONFIG_DEREG_INT,   /**< configure GPIO device into interrupt mode. No argument required for this option */
	DSPAL_GPIO_IOCTL_MAX_NUM,      /**< number of valid IOCTL codes defined for the GPIO */
};

/**
 * @brief
 * Structure passed to the DSPAL_GPIO_IOCTL_CONFIG_IO IOCTL call.  Specifies all
 * the required settings for general purpose I/O GPIO function
 */
struct dspal_gpio_ioctl_config_io {
	enum DSPAL_GPIO_DIRECTION_TYPE direction; /**< direction type indicating if the device is used as INPUT or OUTPUT */
	enum DSPAL_GPIO_PULL_TYPE pull;  /**< the pull type of the GPIO device */
	enum DSPAL_GPIO_DRIVE_TYPE drive; /**< the drive strength */
};


/**
 * @brief
 * This is the parameter to be passed to DSPAL_GPIO_INT_ISR when the gpio
 * interrupt fires.
 * This is a pointer to the context argument, which is used to pass
 * user custom data to callback.
 */
typedef uint32_t DSPAL_GPIO_INT_ISR_CTX;
/**
 * @brief
 * This is the void function pointer client callback to be registered with
 * DSPAL_GPIO_IOCTL_CONFIG_REG_INT IOCTL call. This function is called when
 * the GPIO interrupt fires.
 */
typedef void *(*DSPAL_GPIO_INT_ISR)(DSPAL_GPIO_INT_ISR_CTX);

/**
 * @brief
 * Structure passed to the DSPAL_GPIO_IOCTL_CONFIG_REG_INT IOCTL call.  Specifies all
 * the required settings for interrupt function
 */
struct dspal_gpio_ioctl_reg_int {
	enum DSPAL_GPIO_INT_TRIGGER_TYPE trigger; /**< the interrupt trigger type */
	DSPAL_GPIO_INT_ISR isr; /**< ISR functor */
	DSPAL_GPIO_INT_ISR_CTX isr_ctx;  /**< the context argument passed to isr */
};
