/************************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_config.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_S32K1XX_S32K1XX_CONFIG_H
#define __ARCH_ARM_SRC_S32K1XX_S32K1XX_CONFIG_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Configuration *********************************************************************/

#undef HAVE_LPUART0
#undef HAVE_LPUART1
#undef HAVE_LPUART2

#ifdef CONFIG_S32K1XX_LPUART0
#  define HAVE_LPUART0 1
#endif
#ifdef CONFIG_S32K1XX_LPUART1
#  define HAVE_LPUART1 1
#endif
#ifdef CONFIG_S32K1XX_LPUART2
#  define HAVE_LPUART2 1
#endif

/* Check if we have a LPUART device */

#undef CONFIG_S32K1XX_HAVE_LPUART
#undef HAVE_LPUART_DEVICE

#if defined(HAVE_LPUART0) || defined(HAVE_LPUART1) || defined(HAVE_LPUART2)
#  define HAVE_LPUART_DEVICE 1
#endif

/* Is there a serial console? There should be at most one defined.  It could be on
 * any LPUARTn, n=0,1,2,3
 */

#undef HAVE_LPUART_CONSOLE

#if defined(CONFIG_LPUART0_SERIAL_CONSOLE) && defined(HAVE_LPUART0)
#  undef CONFIG_LPUART1_SERIAL_CONSOLE
#  undef CONFIG_LPUART2_SERIAL_CONSOLE
#  define HAVE_LPUART_CONSOLE 1
#elif defined(CONFIG_LPUART1_SERIAL_CONSOLE) && defined(HAVE_LPUART1)
#  undef CONFIG_LPUART0_SERIAL_CONSOLE
#  undef CONFIG_LPUART2_SERIAL_CONSOLE
#  define HAVE_LPUART_CONSOLE 1
#elif defined(CONFIG_LPUART2_SERIAL_CONSOLE) && defined(HAVE_LPUART2)
#  undef CONFIG_LPUART0_SERIAL_CONSOLE
#  undef CONFIG_LPUART1_SERIAL_CONSOLE
#  define HAVE_LPUART_CONSOLE 1
#else
#  ifdef CONFIG_DEV_CONSOLE
#    warning "No valid CONFIG_[LP]LPUART[n]_SERIAL_CONSOLE Setting"
#  endif
#  undef CONFIG_LPUART0_SERIAL_CONSOLE
#  undef CONFIG_LPUART1_SERIAL_CONSOLE
#  undef CONFIG_LPUART2_SERIAL_CONSOLE
#endif

/* Check LPUART flow control (Not yet supported) */

# undef CONFIG_LPUART0_FLOWCONTROL
# undef CONFIG_LPUART1_FLOWCONTROL
# undef CONFIG_LPUART2_FLOWCONTROL

/* Ethernet controller configuration */

#ifndef CONFIG_S32K1XX_ENET_NRXBUFFERS
#  define CONFIG_S32K1XX_ENET_NRXBUFFERS 6
#endif

#ifndef CONFIG_S32K1XX_ENET_NTXBUFFERS
#  define CONFIG_S32K1XX_ENET_NTXBUFFERS 2
#endif

#ifndef CONFIG_S32K1XX_ENET_NETHIFS
#  define CONFIG_S32K1XX_ENET_NETHIFS 1
#endif

#define S32K1XX_ENET_HAS_DBSWAP 1

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_S32K1XX_S32K1XX_CONFIG_H */
