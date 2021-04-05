/****************************************************************************
 * arch/z16/src/common/up_doirq.c
 *
 *   Copyright (C) 2008-2009, 2011, 2015 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "chip.h"
#include "up_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_doirq
 *
 * Description:
 *   Interface between low-level IRQ decode logic and the NuttX IRQ dispatch
 *   logic.
 *
 ****************************************************************************/

FAR chipreg_t *up_doirq(int irq, FAR chipreg_t *regs)
{
  FAR chipreg_t *ret = regs;

  board_autoled_on(LED_INIRQ);
#ifdef CONFIG_SUPPRESS_INTERRUPTS
  PANIC();
#else
  if ((unsigned)irq < NR_IRQS)
    {
      FAR chipreg_t *savestate;

      /* Nested interrupts are not supported in this implementation.  If
       * you want to implement nested interrupts, you would have to (1) change
       * the way that g_current_regs is handled and (2) the design associated
       * with CONFIG_ARCH_INTERRUPTSTACK.  The savestate variable will not
       * work for that purpose as implemented here because only the outermost
       * nested interrupt can result in a context switch (it can probably be
       * deleted).
       */

      /* Current regs non-zero indicates that we are processing
       * an interrupt; g_current_regs is also used to manage
       * interrupt level context switches.
       */

      savestate    = (FAR chipreg_t *)g_current_regs;
      g_current_regs = regs;

      /* Acknowledge the interrupt */

      up_ack_irq(irq);

      /* Deliver the IRQ */

      irq_dispatch(irq, regs);

      /* Restore the previous value of g_current_regs.  NULL would indicate that
       * we are no longer in an interrupt handler.  It will be non-NULL if we
       * are returning from a nested interrupt.
       */

      ret          = g_current_regs;
      g_current_regs = savestate;
    }

  board_autoled_off(LED_INIRQ);
#endif

  return ret;
}
