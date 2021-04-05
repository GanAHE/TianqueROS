/****************************************************************************
 * arch/z80/src/z8/z8_registerdump.c
 *
 *   Copyright (C) 2008-2009, 2011, 2016 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "chip/switch.h"
#include "z80_internal.h"

#ifdef CONFIG_ARCH_STACKDUMP

/****************************************************************************
 * Private Data
 ****************************************************************************/

static chipreg_t s_last_regs[XCPTCONTEXT_REGS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void z8_dumpregs(FAR chipret_t *regs)
{
  _alert("REGS: %04x %04x %04x %04x %04x %04x %04x %04x\n",
        regs[XCPT_RR0], regs[XCPT_RR2], regs[XCPT_RR4], regs[XCPT_RR6],
        regs[XCPT_RR8], regs[XCPT_RR10], regs[XCPT_RR12], regs[XCPT_RR14]);
}

static inline void z8_dumpstate(chipreg_t sp, chipreg_t pc, uint8_t irqctl,
                                chipreg_t rpflags)
{
  _alert("SP: %04x PC: %04x IRQCTL: %02x RP: %02x FLAGS: %02x\n",
        sp, pc, irqctl & 0xff, rpflags >> 8, rpflags & 0xff);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: z8_registerdump
 ****************************************************************************/

void z8_registerdump(void)
{
  FAR chipret_t *regs;
  chipreg_t      sp;
  uint16_t       rp;

  switch (g_z8irqstate.state)
    {
      case Z8_IRQSTATE_ENTRY:
        /* Calculate the source address based on the saved RP value */

        rp   = g_z8irqstate.regs[Z8_IRQSAVE_RPFLAGS] >> 8;
        regs = (FAR uint16_t*)(rp & 0xf0);

        /* Then dump the register values */

        z8_dumpregs(regs);

        /* Dump the saved machine state:
         * The g_z8irqstate.regs pointer is the value of the stack pointer at
         * the time that z80_doirq() was called.  Therefore, we can calculate
         * the correct value for the stack pointer on return from interrupt:
         */

        sp = ((chipreg_t)g_z8irqstate.regs) + Z8_IRQSAVE_SIZE;
        z8_dumpstate(sp, g_z8irqstate.regs[Z8_IRQSAVE_PC], 0x80,
                     g_z8irqstate.regs[Z8_IRQSAVE_RPFLAGS]);
        break;

      case Z8_IRQSTATE_SAVED:
        regs = g_z8irqstate.regs;
        z8_dumpregs(regs);
        z8_dumpstate(regs[XCPT_SP], regs[XCPT_PC],
                     regs[XCPT_IRQCTL], regs[XCPT_RPFLAGS]);
        break;

      case Z8_IRQSTATE_NONE:
      default:
        z8_saveusercontext(s_last_regs);
        regs = s_last_regs;
        z8_dumpregs(regs);
        z8_dumpstate(regs[XCPT_SP], regs[XCPT_PC],
                     regs[XCPT_IRQCTL], regs[XCPT_RPFLAGS]);
        break;
    }
}

#endif /* CONFIG_ARCH_STACKDUMP */
