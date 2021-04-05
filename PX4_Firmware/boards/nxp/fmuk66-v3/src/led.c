/****************************************************************************
 *
 *   Copyright (c) 2016, 2018 PX4 Development Team. All rights reserved.
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
 * @file led.c
 *
 * NXP fmuk66-v3 LED backend.
 */

#include <px4_platform_common/px4_config.h>

#include <stdbool.h>

#include "kinetis.h"
#include "chip.h"
#include "board_config.h"

#include <arch/board/board.h>

/*
 * Ideally we'd be able to get these from up_internal.h,
 * but since we want to be able to disable the NuttX use
 * of leds for system indication at will and there is no
 * separate switch, we need to build independent of the
 * CONFIG_ARCH_LEDS configuration switch.
 */
__BEGIN_DECLS
extern void led_init(void);
extern void led_on(int led);
extern void led_off(int led);
extern void led_toggle(int led);
__END_DECLS


static uint32_t g_ledmap[] = {
	0,      // Indexed by LED_BLUE
	GPIO_LED_1,       // Indexed by LED_RED, LED_AMBER
	GPIO_LED_SAFETY,  // Indexed by LED_SAFETY
	GPIO_LED_2,     // Indexed by LED_GREEN
};

__EXPORT void led_init(void)
{
	/* Configure LED GPIOs for output */
	for (size_t l = 0; l < (sizeof(g_ledmap) / sizeof(g_ledmap[0])); l++) {
		if (g_ledmap[l] != 0) {
			kinetis_pinconfig(g_ledmap[l]);
		}
	}
}

static void phy_set_led(int led, bool state)
{
	/* Drive High to switch on */

	if (g_ledmap[led] != 0) {
		kinetis_gpiowrite(g_ledmap[led], state);
	}
}

static bool phy_get_led(int led)
{

	if (g_ledmap[led] != 0) {
		return kinetis_gpioread(g_ledmap[led]);
	}

	return false;
}

__EXPORT void led_on(int led)
{
	phy_set_led(led, true);
}

__EXPORT void led_off(int led)
{
	phy_set_led(led, false);
}

__EXPORT void led_toggle(int led)
{

	phy_set_led(led, !phy_get_led(led));
}
