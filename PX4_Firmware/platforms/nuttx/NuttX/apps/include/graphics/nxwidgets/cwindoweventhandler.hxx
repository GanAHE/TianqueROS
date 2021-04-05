/****************************************************************************
 * apps/include/graphics/nxwidgets/cwindoweventhandler.hxx
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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

#ifndef __APPS_INCLUDE_GRAPHICS_NXWIDGETS_CWINDOWEVENTHANDLER_HXX
#define __APPS_INCLUDE_GRAPHICS_NXWIDGETS_CWINDOWEVENTHANDLER_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "graphics/nxwidgets/nxconfig.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Implementation Classes
 ****************************************************************************/

#if defined(__cplusplus)

namespace NXWidgets
{
  class CNxWidget;

  /**
   * Base CWindowEventHandler class, intended to be subclassed.  Any class that
   * needs to listen for window events should inherit from this class.
   */

  class CWindowEventHandler
  {
  public:
    /**
     * Constructor.
     */

    inline CWindowEventHandler() { }

    /**
     * Destructor.
     */

    virtual inline ~CWindowEventHandler() { }

    /**
     * Handle a NX window redraw request event
     *
     * @param nxRect The region in the window to be redrawn
     * @param more More redraw requests will follow
     */

    virtual void handleRedrawEvent(FAR const nxgl_rect_s *nxRect, bool more)
    {
    }

    /**
     * Handle a NX window position/size change event
     */

    virtual void handleGeometryEvent(void) { }

#ifdef CONFIG_NX_XYINPUT
    /**
     * Handle an NX window mouse input event.
     */

    virtual void handleMouseEvent(FAR const struct nxgl_point_s *pos,
                                  uint8_t buttons)
    {
    }
#endif

#ifdef CONFIG_NX_KBD
    /**
     * Handle a NX window keyboard input event.
     */

    virtual void handleKeyboardEvent(void) { }
#endif

    /**
     * Handle a NX window blocked event
     *
     * @param arg - User provided argument (see nx_block or nxtk_block)
     */

    virtual void handleBlockedEvent(FAR void *arg) { }
  };
}

#endif // __cplusplus

#endif // __APPS_INCLUDE_GRAPHICS_NXWIDGETS_CWINDOWEVENTHANDLER_HXX
