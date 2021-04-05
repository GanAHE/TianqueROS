/****************************************************************************
 * apps/nshlib/nsh_consolemain.c
 *
 *   Copyright (C) 2007-2009, 2011-2013 Gregory Nutt. All rights reserved.
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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

#include <stdio.h>
#include <assert.h>

#include <sys/boardctl.h>

#include "nsh.h"
#include "nsh_console.h"

#if !defined(CONFIG_NSH_ALTCONDEV) && !defined(HAVE_USB_CONSOLE) && \
    !defined(HAVE_USB_KEYBOARD)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nsh_consolemain (Normal character device version)
 *
 * Description:
 *   This interfaces may be to called or started with task_start to start a
 *   single an NSH instance that operates on stdin and stdout.  This
 *   function does not normally return (see below).
 *
 *   This version of nsh_consolmain handles generic /dev/console character
 *   devices (see nsh_usbconsole.c and usb_usbkeyboard for other versions
 *   for special USB console devices).
 *
 * Input Parameters:
 *   Standard task start-up arguments.  These are not used.  argc may be
 *   zero and argv may be NULL.
 *
 * Returned Values:
 *   This function does not normally return.  exit() is usually called to
 *   terminate the NSH session.  This function will return in the event of
 *   an error.  In that case, a non-zero value is returned (EXIT_FAILURE=1).
 *
 ****************************************************************************/

int nsh_consolemain(int argc, char *argv[])
{
  FAR struct console_stdio_s *pstate = nsh_newconsole();
  int ret;

  DEBUGASSERT(pstate != NULL);

#ifdef CONFIG_NSH_ROMFSETC
  /* Execute the start-up script */

  (void)nsh_initscript(&pstate->cn_vtbl);

#ifndef CONFIG_NSH_DISABLESCRIPT
  /* Reset the option flags */

  pstate->cn_vtbl.np.np_flags = NSH_NP_SET_OPTIONS_INIT;
#endif
#endif

#ifdef CONFIG_NSH_USBDEV_TRACE
  /* Initialize any USB tracing options that were requested */

  usbtrace_enable(TRACE_BITSET);
#endif

#if defined(CONFIG_NSH_ARCHINIT) && defined(CONFIG_BOARDCTL_FINALINIT)
  /* Perform architecture-specific final-initialization (if configured) */

  (void)boardctl(BOARDIOC_FINALINIT, 0);
#endif

  /* Execute the session */

  ret = nsh_session(pstate);

  /* Exit upon return */

  nsh_exit(&pstate->cn_vtbl, ret);
  return ret;
}

#endif /* !HAVE_USB_CONSOLE && !HAVE_USB_KEYBOARD !HAVE_SLCD_CONSOLE */
