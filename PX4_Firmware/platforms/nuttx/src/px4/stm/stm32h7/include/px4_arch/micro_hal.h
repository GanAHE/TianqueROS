/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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


#include "../../../stm32_common/include/px4_arch/micro_hal.h"

__BEGIN_DECLS

#define PX4_SOC_ARCH_ID             PX4_SOC_ARCH_ID_STM32H7
#include <chip.h>
#include <hardware/stm32_flash.h>
#include <up_internal.h> //include up_systemreset() which is included on stm32.h
#include <stm32_bbsram.h>
#define PX4_BBSRAM_SIZE STM32H7_BBSRAM_SIZE
#define PX4_BBSRAM_GETDESC_IOCTL STM32H7_BBSRAM_GETDESC_IOCTL
#define PX4_FLASH_BASE  0x08000000
#define PX4_NUMBER_I2C_BUSES STM32H7_NI2C
#define PX4_ARCH_DCACHE_LINESIZE ARMV7M_DCACHE_LINESIZE //Fix me! REMOVE this The DSHOT added this and needs to be fixed

int stm32h7_flash_lock(size_t addr);
int stm32h7_flash_unlock(size_t addr);
int stm32h7_flash_writeprotect(size_t block, bool enabled);
#define  stm32_flash_lock() stm32h7_flash_lock(PX4_FLASH_BASE)

__END_DECLS
