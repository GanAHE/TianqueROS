/****************************************************************************
 * apps/graphics/traveler/tools/libwld/wld_inifile.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

#ifndef __APPS_GRAPHICS_TRAVELER_TOOSL_LIBWLD_WLD_INIFILE_H
#define __APPS_GRAPHICS_TRAVELER_TOOSL_LIBWLD_WLD_INIFILE_H

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef void *INIHANDLE;

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name:  inifile_initialize
 *
 * Description:
 *   Initialize for access to the INI file 'inifile_name'
 *
 ****************************************************************************/

INIHANDLE inifile_initialize(const char *inifile_name);

/****************************************************************************
 * Name:  inifile_uninitialize
 *
 * Description:
 *   Free resources commit to INI file parsing
 *
 ****************************************************************************/

void inifile_uninitialize(INIHANDLE handle);

/****************************************************************************
 * Name: inifile_read_string
 *
 * Description:
 *   Obtains the specified string value for the specified variable name
 *   within the specified section of the INI file.  The receiver of the
 *   value string should call inifile_free_string when it no longer needs
 *   the memory held by the value string.
 *
 ****************************************************************************/

char *inifile_read_string(INIHANDLE handle,
                          const char *section,
                          const char *variable,
                          const char *defvalue);

/****************************************************************************
 * Name:  inifile_read_integer
 *
 * Description:
 *   Obtains the specified integer value for the specified variable name
 *   within the specified section of the INI file
 *
 ****************************************************************************/

long inifile_read_integer(INIHANDLE handle,
                          const char *section,
                          const char *variable,
                          long defvalue);

/****************************************************************************
 * Name:  inifile_free_string
 *
 * Description:
 *   Release resources allocated for the value string previously obtained
 *   from inifile_read_string.  The purpose of this inline function is to
 *   hide the memory allocator used by this implementation.
 *
 ****************************************************************************/

void inifile_free_string(char *value);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __APPS_GRAPHICS_TRAVELER_TOOSL_LIBWLD_WLD_INIFILE_H */
