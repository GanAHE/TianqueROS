/****************************************************************************
 * boards/arm/kinetis/freedom-k64f/src/k64_autoleds.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/
/* The Freedom K64F has a single RGB LED driven by the K64F as follows:
 *
 *   LED    K64
 *   ------ -------------------------------------------------------
 *   RED    PTB22/SPI2_SOUT/FB_AD29/CMP2_OUT
 *   BLUE   PTB21/SPI2_SCK/FB_AD30/CMP1_OUT
 *   GREEN  PTE26/ENET_1588_CLKIN/UART4_CTS_b/RTC_CLKOUT/USB0_CLKIN
 *
 * If CONFIG_ARCH_LEDs is defined, then NuttX will control the LED on board
 * the Freedom K64F.  The following definitions describe how NuttX controls
 * the LEDs:
 *
 *   SYMBOL                Meaning                 LED state
 *                                                 RED   GREEN  BLUE
 *   -------------------  -----------------------  -----------------
 *   LED_STARTED          NuttX has been started    OFF  OFF  OFF
 *   LED_HEAPALLOCATE     Heap has been allocated   OFF  OFF  ON
 *   LED_IRQSENABLED      Interrupts enabled        OFF  OFF  ON
 *   LED_STACKCREATED     Idle stack created        OFF  ON   OFF
 *   LED_INIRQ            In an interrupt          (no change)
 *   LED_SIGNAL           In a signal handler      (no change)
 *   LED_ASSERTION        An assertion failed      (no change)
 *   LED_PANIC            The system has crashed    FLASH OFF OFF
 *   LED_IDLE             K64 is in sleep mode     (Optional, not used)
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "chip.h"
#include "kinetis.h"
#include "freedom-k64f.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Summary of all possible settings */

#define LED_NOCHANGE      0 /* LED_IRQSENABLED, LED_INIRQ, LED_SIGNAL, LED_ASSERTION */
#define LED_OFF_OFF_OFF   1 /* LED_STARTED */
#define LED_OFF_OFF_ON    2 /* LED_HEAPALLOCATE */
#define LED_OFF_ON_OFF    3 /* LED_STACKCREATED */
#define LED_ON_OFF_OFF    4 /* LED_PANIC */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 *
 * Description:
 *   Initialize the on-board LED
 *
 ****************************************************************************/

void board_autoled_initialize(void)
{
  kinetis_pinconfig(GPIO_LED_R);
  kinetis_pinconfig(GPIO_LED_G);
  kinetis_pinconfig(GPIO_LED_B);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  if (led != LED_NOCHANGE)
    {
      bool redoff   = true;
      bool greenoff = true;
      bool blueoff  = true;

      switch (led)
        {
          default:
          case LED_OFF_OFF_OFF:
            break;

          case LED_OFF_OFF_ON:
            blueoff = false;
            break;

          case LED_OFF_ON_OFF:
            greenoff = false;
            break;

          case LED_ON_OFF_OFF:
            redoff = false;
            break;
        }

      kinetis_gpiowrite(GPIO_LED_R, redoff);
      kinetis_gpiowrite(GPIO_LED_G, greenoff);
      kinetis_gpiowrite(GPIO_LED_B, blueoff);
    }
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  if (led == LED_ON_OFF_OFF)
    {
      kinetis_gpiowrite(GPIO_LED_R, true);
      kinetis_gpiowrite(GPIO_LED_G, true);
      kinetis_gpiowrite(GPIO_LED_B, true);
    }
}

#endif /* CONFIG_ARCH_LEDS */
