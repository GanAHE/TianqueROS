/****************************************************************************
 * libs/libc/stdio/lib_rawinstream.c
 *
 *   Copyright (C) 2007-2009, 2011-2012, 2014 Gregory Nutt. All rights reserved.
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

#include <unistd.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/fs/fs.h>

#include "libc.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rawinstream_getc
 ****************************************************************************/

static int rawinstream_getc(FAR struct lib_instream_s *this)
{
  FAR struct lib_rawinstream_s *rthis = (FAR struct lib_rawinstream_s *)this;
  int nwritten;
  char ch;

  DEBUGASSERT(this && rthis->fd >= 0);

  /* Attempt to read one character */

  nwritten = _NX_READ(rthis->fd, &ch, 1);
  if (nwritten == 1)
    {
      this->nget++;
      return ch;
    }

  /* Return EOF on any failure to read from the incoming byte stream. The
   * only expected error is EINTR meaning that the read was interrupted
   * by a signal.  A Zero return value would indicated an end-of-file
   * confition.
   */

  return EOF;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_rawinstream
 *
 * Description:
 *   Initializes a stream for use with a file descriptor.
 *
 * Input Parameters:
 *   instream - User allocated, uninitialized instance of struct
 *              lib_rawinstream_s to be initialized.
 *   fd       - User provided file/socket descriptor (must have been opened
 *              for the correct access).
 *
 * Returned Value:
 *   None (User allocated instance initialized).
 *
 ****************************************************************************/

void lib_rawinstream(FAR struct lib_rawinstream_s *instream, int fd)
{
  instream->public.get  = rawinstream_getc;
  instream->public.nget = 0;
  instream->fd          = fd;
}
