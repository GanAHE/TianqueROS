/****************************************************************************
 * arch/risc-v/src/rv32im/up_dumpstate.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 *   Updated for RISC-V:
 *
 *   Copyright (C) 2016 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit <pettitkd@gmail.com>
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

#include <stdint.h>
#include <stdlib.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "sched/sched.h"
#include "up_internal.h"

#ifdef CONFIG_ARCH_STACKDUMP

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_getsp
 ****************************************************************************/

static inline uint32_t up_getsp(void)
{
  register uint32_t sp;
  __asm__
  (
    "\tadd  %0, x0, x2\n"
    : "=r"(sp)
  );
  return sp;
}

/****************************************************************************
 * Name: up_stackdump
 ****************************************************************************/

static void up_stackdump(uint32_t sp, uint32_t stack_base)
{
  uint32_t stack ;

  for (stack = sp & ~0x1f; stack < stack_base; stack += 32)
    {
      uint32_t *ptr = (uint32_t *)stack;
      _alert("%08x: %08x %08x %08x %08x %08x %08x %08x %08x\n",
             stack, ptr[0], ptr[1], ptr[2], ptr[3],
             ptr[4], ptr[5], ptr[6], ptr[7]);
    }
}

/****************************************************************************
 * Name: up_registerdump
 ****************************************************************************/

static inline void up_registerdump(void)
{
  /* Are user registers available from interrupt processing? */

  if (g_current_regs)
    {
      _alert("EPC:%08x \n",
            g_current_regs[REG_EPC]);
      _alert("A0:%08x A1:%08x A2:%08x A3:%08x A4:%08x A5:%08x A6:%08x A7:%08x\n",
            g_current_regs[REG_A0], g_current_regs[REG_A1], g_current_regs[REG_A2],
            g_current_regs[REG_A3], g_current_regs[REG_A4], g_current_regs[REG_A5],
            g_current_regs[REG_A6], g_current_regs[REG_A7]);
      _alert("T0:%08x T1:%08x T2:%08x T3:%08x T4:%08x T5:%08x T6:%08x\n",
            g_current_regs[REG_T0], g_current_regs[REG_T1], g_current_regs[REG_T2],
            g_current_regs[REG_T3], g_current_regs[REG_T4], g_current_regs[REG_T5],
            g_current_regs[REG_T6]);
      _alert("S0:%08x S1:%08x S2:%08x S3:%08x S4:%08x S5:%08x S6:%08x S7:%08x\n",
            g_current_regs[REG_S0], g_current_regs[REG_S1], g_current_regs[REG_S2],
            g_current_regs[REG_S3], g_current_regs[REG_S4], g_current_regs[REG_S5],
            g_current_regs[REG_S6], g_current_regs[REG_S7]);
      _alert("S8:%08x S9:%08x S10:%08x S11:%08x\n",
            g_current_regs[REG_S8], g_current_regs[REG_S9], g_current_regs[REG_S10],
            g_current_regs[REG_S11]);
#ifdef RISCV_SAVE_GP
      _alert("GP:%08x SP:%08x FP:%08x TP:%08x RA:%08x\n",
            g_current_regs[REG_GP], g_current_regs[REG_SP], g_current_regs[REG_FP],
            g_current_regs[REG_TP], g_current_regs[REG_RA]);
#else
      _alert("SP:%08x FP:%08x TP:%08x RA:%08x\n",
            g_current_regs[REG_SP], g_current_regs[REG_FP], g_current_regs[REG_TP],
            g_current_regs[REG_RA]);
#endif
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_dumpstate
 ****************************************************************************/

void up_dumpstate(void)
{
  struct tcb_s *rtcb = running_task();
  uint32_t sp = up_getsp();
  uint32_t ustackbase;
  uint32_t ustacksize;
#if CONFIG_ARCH_INTERRUPTSTACK > 3
  uint32_t istackbase;
  uint32_t istacksize;
#endif

  /* Dump the registers (if available) */

  up_registerdump();

  /* Get the limits on the user stack memory */

  if (rtcb->pid == 0) /* Check for CPU0 IDLE thread */
    {
      ustackbase = g_idle_topstack - 4;
      ustacksize = CONFIG_IDLETHREAD_STACKSIZE;
    }
  else
    {
      ustackbase = (uint32_t)rtcb->adj_stack_ptr;
      ustacksize = (uint32_t)rtcb->adj_stack_size;
    }

  /* Get the limits on the interrupt stack memory */

#if CONFIG_ARCH_INTERRUPTSTACK > 3
  istackbase = (uint32_t)&g_intstackbase;
  istacksize = (CONFIG_ARCH_INTERRUPTSTACK & ~3) - 4;

  /* Show interrupt stack info */

  _alert("sp:     %08x\n", sp);
  _alert("IRQ stack:\n");
  _alert("  base: %08x\n", istackbase);
  _alert("  size: %08x\n", istacksize);

  /* Does the current stack pointer lie within the interrupt
   * stack?
   */

  if (sp <= istackbase && sp > istackbase - istacksize)
    {
      /* Yes.. dump the interrupt stack */

      up_stackdump(sp, istackbase);

      /* Extract the user stack pointer which should lie
       * at the base of the interrupt stack.
       */

      sp = g_intstackbase;
      _alert("sp:     %08x\n", sp);
    }
  else if (g_current_regs)
    {
      _alert("ERROR: Stack pointer is not within the interrupt stack\n");
      up_stackdump(istackbase - istacksize, istackbase);
    }

  /* Show user stack info */

  _alert("User stack:\n");
  _alert("  base: %08x\n", ustackbase);
  _alert("  size: %08x\n", ustacksize);
#else
  _alert("sp:         %08x\n", sp);
  _alert("stack base: %08x\n", ustackbase);
  _alert("stack size: %08x\n", ustacksize);
#endif

  /* Dump the user stack if the stack pointer lies within the allocated user
   * stack memory.
   */

  if (sp > ustackbase || sp <= ustackbase - ustacksize)
    {
      _alert("ERROR: Stack pointer is not within allocated stack\n");
      up_stackdump(ustackbase - ustacksize, ustackbase);
    }
  else
    {
      up_stackdump(sp, ustackbase);
    }
}

#endif /* CONFIG_ARCH_STACKDUMP */
