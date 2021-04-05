/****************************************************************************
 * libs/libc/libgen/lib_dirname.c
 *
 *   Copyright (C) 2007, 2009, 2011-2012 Gregory Nutt. All rights reserved.
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

#include <string.h>
#include <libgen.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char g_retchar[2];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dirname
 *
 * Description:
 *   dirname() extracts the directory component from a null-terminated
 *   pathname string. In the usual case, dirname() returns the string up
 *   to, but not including, the final '/'. Trailing '/' characters are not
 *   counted as part of the pathname.
 *
 *   If path does not contain a slash, dirname() returns the string ".". If
 *   path is the string "/", then dirname() returns the string "/". If path
 *   is a NULL pointer or points to an empty string, then dirname() returns
 *   the string ".".
 *
 *   dirname() may modify the contents of path, so copies should be passed.
 *   dirname() may return pointers to statically allocated memory which may
 *   be overwritten by subsequent calls.
 *
 * Input Parameters:
 *   path The null-terminated string referring to the path to be decomposed
 *
 * Returned Value:
 *   On success the directory component of the path is returned.
 *
 ****************************************************************************/

FAR char *dirname(FAR char *path)
{
  char *p;
  int   len;
  int   ch;

  /* Handle some corner cases */

  if (!path || *path == '\0')
    {
      ch = '.';
      goto out_retchar;
    }

  /* Check for trailing slash characters */

  len = strlen(path);
  while (path[len-1] == '/')
    {
      /* Remove trailing '/' UNLESS this would make a zero length string */
      if (len > 1)
        {
          path[len-1] = '\0';
          len--;
        }
      else
        {
          ch = '/';
          goto out_retchar;
        }
    }

  /* Get the address of the last '/' which is not at the end of the path and,
   * therefor, must be the end of the directory component.
   */

  p = strrchr(path, '/');
  if (p)
    {
      /* Handle the case where the only '/' in the string is the at the beginning
       * of the path.
       */

      if (p == path)
        {
          ch = '/';
          goto out_retchar;
        }

      /* No, the directory component is the substring before the '/'. */

      *p = '\0';
      return path;
    }

  /* There is no '/' in the path */

  ch = '.';

out_retchar:
  g_retchar[0] = ch;
  g_retchar[1] = '\0';
  return g_retchar;
}
