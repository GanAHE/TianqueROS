/****************************************************************************
 * examples/nxterm/nxterm_wndo.c
 *
 *   Copyright (C) 2012, 2019 Gregory Nutt. All rights reserved.
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
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nxfonts.h>

#include "nxterm_internal.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void nxwndo_redraw(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
                          bool morem, FAR void *arg);
static void nxwndo_position(NXWINDOW hwnd, FAR const struct nxgl_size_s *size,
                            FAR const struct nxgl_point_s *pos,
                            FAR const struct nxgl_rect_s *bounds,
                            FAR void *arg);
#ifdef CONFIG_NX_XYINPUT
static void nxwndo_mousein(NXWINDOW hwnd, FAR const struct nxgl_point_s *pos,
                           uint8_t buttons, FAR void *arg);
#endif

#ifdef CONFIG_NX_KBD
static void nxwndo_kbdin(NXWINDOW hwnd, uint8_t nch, FAR const uint8_t *ch,
                         FAR void *arg);
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Background window call table */

const struct nx_callback_s g_nxtermcb =
{
  nxwndo_redraw,   /* redraw */
  nxwndo_position  /* position */
#ifdef CONFIG_NX_XYINPUT
  , nxwndo_mousein /* mousein */
#endif
#ifdef CONFIG_NX_KBD
  , nxwndo_kbdin   /* kbdin */
#endif
  , NULL           /* event */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxwndo_redraw
 ****************************************************************************/

static void nxwndo_redraw(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
                          bool more, FAR void *arg)
{
  nxgl_mxpixel_t wcolor[CONFIG_NX_NPLANES];

  ginfo("hwnd=%p rect={(%d,%d),(%d,%d)} more=%s\n",
         hwnd, rect->pt1.x, rect->pt1.y, rect->pt2.x, rect->pt2.y,
         more ? "true" : "false");

  /* Don't attempt to redraw if the driver has not yet been opened */

  if (g_nxterm_vars.hdrvr)
    {
      struct boardioc_nxterm_ioctl_s iocargs;
      struct nxtermioc_redraw_s redraw;

      /* Inform the NX console of the redraw request */

      redraw.handle = g_nxterm_vars.hdrvr;
      redraw.more   = more;
      nxgl_rectcopy(&redraw.rect, rect);

      iocargs.cmd = NXTERMIOC_NXTERM_REDRAW;
      iocargs.arg = (uintptr_t)&redraw;

      (void)boardctl(BOARDIOC_NXTERM_IOCTL, (uintptr_t)&iocargs);
    }
  else
    {
      /* If the driver has not been opened, then just redraw the window color */

      wcolor[0] = CONFIG_EXAMPLES_NXTERM_WCOLOR;
      (void)nxtk_fillwindow(hwnd, rect, wcolor);
    }
}

/****************************************************************************
 * Name: nxwndo_position
 ****************************************************************************/

static void nxwndo_position(NXWINDOW hwnd, FAR const struct nxgl_size_s *size,
                            FAR const struct nxgl_point_s *pos,
                            FAR const struct nxgl_rect_s *bounds,
                            FAR void *arg)
{
  /* Report the position */

  ginfo("hwnd=%p size=(%d,%d) pos=(%d,%d) bounds={(%d,%d),(%d,%d)}\n",
        hwnd, size->w, size->h, pos->x, pos->y,
        bounds->pt1.x, bounds->pt1.y, bounds->pt2.x, bounds->pt2.y);

  /* Have we picked off the window bounds yet? */

  if (!g_nxterm_vars.haveres)
    {
      /* Save the background window handle */

      g_nxterm_vars.hwnd = hwnd;

      /* Save the background window size */

      g_nxterm_vars.wndo.wsize.w = size->w;
      g_nxterm_vars.wndo.wsize.h = size->h;

      /* Save the window limits (these should be the same for all places and
       * all windows)
       */

      g_nxterm_vars.xres = bounds->pt2.x + 1;
      g_nxterm_vars.yres = bounds->pt2.y + 1;

      g_nxterm_vars.haveres = true;
      sem_post(&g_nxterm_vars.eventsem);
      ginfo("Have xres=%d yres=%d\n", g_nxterm_vars.xres, g_nxterm_vars.yres);
    }
}

/****************************************************************************
 * Name: nxwndo_mousein
 ****************************************************************************/

#ifdef CONFIG_NX_XYINPUT
static void nxwndo_mousein(NXWINDOW hwnd, FAR const struct nxgl_point_s *pos,
                           uint8_t buttons, FAR void *arg)
{
  ginfo("hwnd=%p pos=(%d,%d) button=%02x\n",
        hwnd,  pos->x, pos->y, buttons);
}
#endif

/****************************************************************************
 * Name: nxwndo_kbdin
 ****************************************************************************/

#ifdef CONFIG_NX_KBD
static void nxwndo_kbdin(NXWINDOW hwnd, uint8_t nch, FAR const uint8_t *ch,
                         FAR void *arg)
{
  ginfo("hwnd=%p nch=%d\n", hwnd, nch);
  (void)write(1, ch, nch);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/
