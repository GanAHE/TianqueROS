/****************************************************************************
 * Copyright (c) 2015 Mark Charlebois. All rights reserved.
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

#pragma once

#include <stddef.h>

/**
 * @file
 * The declarations in this file are released to DSPAL users and are used to
 * make file I/O call's for UART device access.  Many of the data structures
 * are used in the parameter of the IOCTL functions to define the
 * behavior of the particular IOCTL.
 *
 * Sample source files are referenced to demonstrate how a particular IOCTL
 * or data structure is used in the context of the POSIX standard file I/O functions
 * (open/close/read/write/ioctl).
 *
 * @par Reading UART Data
 * To read data that has accumulated since the last call to read (see the rx_func_ptr_t to define an
 * optional receive data callback) the buffer parameter of the read function must reference a buffer
 * large enough to contain all of the accumulated data.  If the buffer is not large enough, some portion of the
 * accumulated will be copied to the buffer.  The actual length of the data copied to the caller's buffer is
 * specified in the return value of the read function.
 *
 * @par Writing UART Data
 * To write data to the serial port a buffer parameter containing the data to be transmitted must be passed
 * to the write function.  After the data is queued for transmit, the write function will return immediately
 * to the caller, unless the is_tx_data_synchronous member of the dspal_serial_open_options structure is set
 * to true.  If set to true the transmit function will only return when all data in the transmit queue has
 * been transmitted.
 *
 * The tx_data_callback member of the dspal_serial_open_options structure can be used to be receive
 * notification of when all queued data has been transmitted.  This can be used as an alternative to
 * setting the is_tx_data_synchronous member to true.
 *
 * @par
 * Sample source code for read/write data to a serial port is included below:
 * @include serial_test_imp.c
 */

/**
 * @brief
 * The serial device path uses the following format:
 * /dev/tty-{number}
 * Device numbers start at 1 and go to up to 4.
 */
#define DEV_FS_UART_DEVICE_TYPE_STRING "/dev/tty-"

/* Forward references: */
struct dspal_serial_buffer_item;

/**
 * The signature for the optional callback function used to indicate when
 * new data is received, or when all of the data in the transmit queue
 * has been transmitted.
 * @note
 * The buffer passed to the rx_func_ptr_t callback function will be deallocated
 * upon return.  The buffer must be copied to retain the data for subsequent processing
 * after returning from the callback function.
 * @param buffer
 * The address of the buffer containing the received bytes.
 * @param num_bytes
 * The number of bytes referenced by the buffer parameter.
 */
typedef void (*serial_rx_func_ptr_t)(void *context, char *buffer, size_t num_bytes);
typedef void (*serial_tx_func_ptr_t)(void);

/**
 * @brief
 * DSPAL ID's for the serial bit rate. These values should be identical
 * to those used in the aDSP SIO port code.
 */
enum DSPAL_SERIAL_BITRATES {
	DSPAL_SIO_BITRATE_ILLEGAL_1,            /**< Illegal bit-rate 1*/
	DSPAL_SIO_BITRATE_ILLEGAL_3,            /**< Illegal bit-rate 2*/
	DSPAL_SIO_BITRATE_ILLEGAL_4,            /**< Illegal bit-rate 4*/
	DSPAL_SIO_BITRATE_ILLEGAL_5,            /**< Illegal bit-rate 5*/
	DSPAL_SIO_BITRATE_ILLEGAL_6,            /**< Illegal bit-rate 6*/
	DSPAL_SIO_BITRATE_ILLEGAL_7,            /**< Illegal bit-rate 7*/
	DSPAL_SIO_BITRATE_300,                  /**< 300  bit-rate     */
	DSPAL_SIO_BITRATE_600,                  /**< 600  bit-rate     */
	DSPAL_SIO_BITRATE_1200,                 /**< 1200 bit-rate     */
	DSPAL_SIO_BITRATE_2400,                 /**< 2400 bit-rate     */
	DSPAL_SIO_BITRATE_4800,                 /**< 4800 bit-rate     */
	DSPAL_SIO_BITRATE_9600,                 /**< 9600 bit-rate     */
	DSPAL_SIO_BITRATE_14400,                /**< 14400 bit-rate    */
	DSPAL_SIO_BITRATE_ILLEGAL_8 = DSPAL_SIO_BITRATE_14400,
	DSPAL_SIO_BITRATE_19200,                /**< 19200  bit-rate   */
	DSPAL_SIO_BITRATE_38400,                /**< 38400  bit-rate   */
	DSPAL_SIO_BITRATE_57600,                /**< 57600  bit-rate   */
	DSPAL_SIO_BITRATE_76800,                /**< 76800  bit-rate   */
	DSPAL_SIO_BITRATE_ILLEGAL_9 = DSPAL_SIO_BITRATE_76800,
	DSPAL_SIO_BITRATE_115200,               /**< 115200 bit-rate   */
	DSPAL_SIO_BITRATE_230400,               /**< 230400 bit-rate   */
	DSPAL_SIO_BITRATE_ILLEGAL_2 = DSPAL_SIO_BITRATE_230400,
	DSPAL_SIO_BITRATE_250000,
	DSPAL_SIO_BITRATE_460800,               /**< 460800 bit-rate   */
	DSPAL_SIO_BITRATE_ILLEGAL_10 = DSPAL_SIO_BITRATE_460800,
	DSPAL_SIO_BITRATE_921600,               /**< 921600 bit-rate   */
	DSPAL_SIO_BITRATE_2000000,              /**< 2000000 bit-rate  */
	DSPAL_SIO_BITRATE_2900000,              /**< 2900000 bit-rate  */
	DSPAL_SIO_BITRATE_3000000,              /**< 3000000 bit-rate  */
	DSPAL_SIO_BITRATE_3200000,              /**< 3200000 bit-rate  */
	DSPAL_SIO_BITRATE_3686400,              /**< 3686400 bit-rate  */
	DSPAL_SIO_BITRATE_4000000,              /**< 4000000 bit-rate  */
	DSPAL_SIO_BITRATE_HS_CUSTOM,            /**< HS custom bit-rate*/
	DSPAL_SIO_BITRATE_ILLEGAL_11 = DSPAL_SIO_BITRATE_HS_CUSTOM,
	DSPAL_SIO_BITRATE_BEST = 0x7FFE,  /**< Best bitrate (default, fastest, etc) */
	DSPAL_SIO_BITRATE_MAX = 0x7FFF    /**< For bounds checking only             */
};

/**
 * @brief
 * DSPAL ID's for the serial flow control. These values should be identical
 * to those used in the aDSP SIO port code.
 */
enum DSPAL_SERIAL_FLOW_CONTROL_METHODS {
	DSPAL_SIO_FCTL_BEST = 0,                /**< Best Flow control method.  Can
                                                mean default or no change. */
	DSPAL_SIO_FCTL_OFF,                     /**< Flow control disabled */
	DSPAL_SIO_XONXOFF_STRIP_FCTL_FS,        /**< Use fail-safe XON/XOFF flow
                                                control but strip XON/XOFF
                                                characters from stream */
	DSPAL_SIO_XONXOFF_STRIP_FCTL_NFS,       /**< Use non-failsafe XON/XOFF flow
                                                control but strip XON/XOFF
                                                characters from stream  */
	DSPAL_SIO_CTSRFR_FCTL,                  /**< Use CTS/RFR flow control*/
	DSPAL_SIO_XONXOFF_NSTRIP_FCTL_FS,       /**< Use fail-safe XON/XOFF flow
                                                control and leave in stream */
	DSPAL_SIO_XONXOFF_NSTRIP_FCTL_NFS,      /**< Use non-failsafe XON/XOFF flow
                                                control and leave in stream */
	DSPAL_SIO_MAX_FLOW,                     /**< For bounds checking only  */
	DSPAL_SIO_CTSRFR_AUTO_FCTL              /**< Use CTS/RFR flow control with
                                                  auto RX RFR signal generation  */
};

/**
 * @brief
 * ioctl codes used to extend the functionality of the standard read/write file
 * semantics for the serial interface.
 */
enum DSPAL_SERIAL_IOCTLS {
	SERIAL_IOCTL_INVALID = -1,     /**< indicates an invalid IOCTL detected */
	SERIAL_IOCTL_OPEN_OPTIONS,     /**< provides callbacks, flow control, data rate, etc. */
	SERIAL_IOCTL_SET_RECEIVE_DATA_CALLBACK,  /**< assigns the receive data callback to the address specified. */
	SERIAL_IOCTL_SET_DATA_RATE,    /**< sets the new data rate on the currently open UART interface. */
	SERIAL_IOCTL_MAX_NUM           /**< maximum number of serial IOCTL's defined */
};

/**
 * @brief
 * DSPAL ID's mapped to the specified aDSP SIO port.
 */
enum DSPAL_SERIAL_PORTS {
	DSPAL_SIO_PORT_UART_MAIN = 1,/**< DSPAL ID for the UART_MAIN port */
	DSPAL_SIO_PORT_UART_AUX,     /**< DSPAL ID for the UART_AUX port */
	DSPAL_SIO_PORT_UART_THIRD,   /**< DSPAL ID for the UART_THIRD port */
	DSPAL_SIO_PORT_UART_FOURTH,  /**< DspAL ID for the UART_FOURTH port */
	DSPAL_SIO_PORT_UART_FIFTH,   /**< DspAL ID for the UART_FIFTH port */
	DSPAL_SIO_PORT_UART_SIXTH,   /**< DspAL ID for the UART_FIFTH port */
	DSPAL_SIO_PORT_UART_SEVENTH,   /**< DspAL ID for the UART_FIFTH port */
	DSPAL_SIO_PORT_UART_EIGHTH,   /**< DspAL ID for the UART_FIFTH port */
	DSPAL_SIO_PORT_UART_NINTH,   /**< DspAL ID for the UART_FIFTH port */
	DSPAL_SIO_PORT_UART_TENTH,   /**< DspAL ID for the UART_FIFTH port */
	DSPAL_SIO_PORT_UART_ELEVENTH,   /**< DspAL ID for the UART_FIFTH port */
	DSPAL_SIO_PORT_UART_TWELFTH,   /**< DspAL ID for the UART_FIFTH port */
	DSPAL_SIO_PORT_UART_CXM,     /**< DspAL ID for the UART_CXM port */
};

/**
 * @brief
 * Structure used to configure how the serial port is opened.
 *
 */
struct dspal_serial_open_options {
	enum DSPAL_SERIAL_BITRATES bit_rate;
	enum DSPAL_SERIAL_FLOW_CONTROL_METHODS tx_flow;
	enum DSPAL_SERIAL_FLOW_CONTROL_METHODS rx_flow;
	serial_rx_func_ptr_t rx_data_callback; 	/**< optional, called when new data is received. */
	serial_tx_func_ptr_t tx_data_callback; 	/**< optional, called when all data in the buffer is transmitted. */
	int is_tx_data_synchronous;	 	/**< if true, causes transmit function to block until all data is transmitted. */
};

/**
 * @brief
 * Structure used to configure the receive callback function pointer and
 * context, included as a parameter to the callback function.
 */
struct dspal_serial_ioctl_receive_data_callback {
	serial_rx_func_ptr_t rx_data_callback_func_ptr;
	/**< pointer to a callback function, called in the ISR context when new data has arrived. */
	void *context; 	/**< the pointer to user defined context data, passed to the callback function */
};

/**
 * @brief
 * Structure used to set the UART data rate
 */
struct dspal_serial_ioctl_data_rate {
	enum DSPAL_SERIAL_BITRATES bit_rate; /**< baud rate in enum DSPAL_SERIAL_BITRATES type */
};
