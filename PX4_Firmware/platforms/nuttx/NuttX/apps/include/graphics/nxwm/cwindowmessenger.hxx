/****************************************************************************
 * apps/include/graphics/nxwm/cwindowmessenger.hxx
 *
 *   Copyright (C) 2012-2013 Gregory Nutt. All rights reserved.
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
 * 3. Neither the name NuttX, NxWidgets, nor the names of its contributors
 *    me be used to endorse or promote products derived from this software
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

#ifndef __APPS_INCLUDE_GRAPHICS_NXWM_CWINDOWMESSENGER_HXX
#define __APPS_INCLUDE_GRAPHICS_NXWM_CWINDOWMESSENGER_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <nuttx/wqueue.h>
#include <nuttx/nx/nxtk.h>
#include <nuttx/nx/nxterm.h>

#include "graphics/nxwidgets/cwindoweventhandler.hxx"
#include "graphics/nxwidgets/cwidgetstyle.hxx"
#include "graphics/nxwidgets/cwidgetcontrol.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Implementation Classes
 ****************************************************************************/

#if defined(__cplusplus)

namespace NxWM
{
  /**
   * Forward references.
   */

  class IApplication;

  /**
   * The class CWindowMessenger integrates the widget control with some special
   * handling of mouse and keyboard inputs needs by NxWM.  It use used
   * in place of CWidgetControl whenever an NxWM window is created.
   *
   * CWindowMessenger cohabitates with CWidgetControl only because it needs the
   * CWidgetControl as an argument in its messaging.
   */

  class CWindowMessenger : public NXWidgets::CWindowEventHandler,
                           public NXWidgets::CWidgetControl
  {
  private:
    /** Structure that stores data for the work queue callback. */

    struct work_state_t
      {
        work_s work;
        CWindowMessenger *windowMessenger;
        void *instance;

        work_state_t() : work() {}
      };

    /** Work queue callback functions */

    static void inputWorkCallback(FAR void *arg);
    static void destroyWorkCallback(FAR void *arg);

#ifdef CONFIG_NX_XYINPUT
    /**
     * Handle an NX window mouse input event.
     */

    void handleMouseEvent(FAR const struct nxgl_point_s *pos, uint8_t buttons);
#endif

#ifdef CONFIG_NX_KBD
    /**
     * Handle a NX window keyboard input event.
     */

    void handleKeyboardEvent(void);
#endif

    /**
     * Handle a NX window blocked event
     *
     * @param arg - User provided argument (see nx_block or nxtk_block)
     */

    void handleBlockedEvent(FAR void *arg);

  public:

    /**
     * CWindowMessenger Constructor
     *
     * @param style The default style that all widgets on this display
     *   should use.  If this is not specified, the widget will use the
     *   values stored in the defaultCWidgetStyle object.
     */

     CWindowMessenger(FAR const NXWidgets::CWidgetStyle *style =
       (const NXWidgets::CWidgetStyle *)NULL);

    /**
     * CWindowMessenger Destructor.
     */

    ~CWindowMessenger(void);
  };
}
#endif // __cplusplus

#endif // __APPS_INCLUDE_GRAPHICS_NXWM_CWINDOWMESSENGER_HXX
