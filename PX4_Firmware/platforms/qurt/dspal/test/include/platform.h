/****************************************************************************
 *   Copyright (c) 2016 Larry Wang. All rights reserved.
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
#if defined(DSP_TYPE_ADSP)
// Use spi-8 on the ADSP to avoid conflicts with the serial I/O tests.
#define SPI_DEVICE_PATH "/dev/spi-8"
#elif defined(DSP_TYPE_SLPI)
#define SPI_DEVICE_PATH "/dev/spi-10"
#endif

// I2C port definition
#if defined(DSP_TYPE_ADSP)
#define I2C_DEVICE_PATH "/dev/i2c-8"
#define I2C_SLAVE_ADDRESS 0x70
#elif defined(DSP_TYPE_SLPI)
#define I2C_DEVICE_PATH "/dev/iic-3"
#define I2C_SLAVE_ADDRESS 0x76 
#endif

// GPIO port definition
#if defined(DSP_TYPE_ADSP)
#define GPIO_DEVICE_PATH  "/dev/gpio-10"
#define GPIO_DEVICE_PATH_LOOPBACK  "/dev/gpio-11"
#define GPIO_INT_DEVICE_PATH  "/dev/gpio-11"
#elif defined(DSP_TYPE_SLPI)
#define GPIO_DEVICE_PATH  "/dev/gpio_ssc-14"
#define GPIO_DEVICE_PATH_LOOPBACK  "/dev/gpio_ssc-15"
#define GPIO_INT_DEVICE_PATH  "/dev/gpio_ssc-15"
#endif


// UART port definition
#if defined(DSP_TYPE_ADSP)
#define NUM_UART_DEVICE_ENABLED  4
#elif defined(DSP_TYPE_SLPI)
#define NUM_UART_DEVICE_ENABLED  4
#endif
