/****************************************************************************
 * apps/nshlib/nsh_init.c
 *
 *   Copyright (C) 2007-2012, 2014-2015 Gregory Nutt. All rights reserved.
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

#include <sys/boardctl.h>

#include "system/readline.h"
#include "netutils/netinit.h"
#include "nshlib/nshlib.h"

#include "nsh.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined(CONFIG_NSH_READLINE) && defined(CONFIG_READLINE_TABCOMPLETION) && \
    defined(CONFIG_READLINE_HAVE_EXTMATCH)
static const struct extmatch_vtable_s g_nsh_extmatch =
{
  nsh_extmatch_count,  /* count_matches */
  nsh_extmatch_getname /* getname */
};
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nsh_initialize
 *
 * Description:
 *   This nterfaces is used to initialize the NuttShell (NSH).
 *   nsh_initialize() should be called one during application start-up prior
 *   to executing either nsh_consolemain() or nsh_telnetstart().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nsh_initialize(void)
{
#if defined(CONFIG_NSH_READLINE) && defined(CONFIG_READLINE_TABCOMPLETION)
  /* Configure the NSH prompt */

  (void)readline_prompt(g_nshprompt);

#ifdef CONFIG_READLINE_HAVE_EXTMATCH
  /* Set up for tab completion on NSH commands */

  (void)readline_extmatch(&g_nsh_extmatch);
#endif
#endif

  /* Mount the /etc filesystem */

  (void)nsh_romfsetc();

#ifdef CONFIG_NSH_ARCHINIT
  /* Perform architecture-specific initialization (if configured) */

  (void)boardctl(BOARDIOC_INIT, 0);
#endif

#ifdef CONFIG_NSH_NETINIT
  /* Bring up the network */

  (void)netinit_bringup();
#endif
}
