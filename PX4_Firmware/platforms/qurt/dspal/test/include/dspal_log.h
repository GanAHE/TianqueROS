/****************************************************************************
 *   Copyright (c) 2015 Mark Charlebois. All rights reserved.
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
 * 3. Neither the name ATLFlight nor the names of its contributors may be
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

#pragma once

#ifdef __hexagon__
/* Use the following macro for debug output on the aDSP. */
#include <HAP_farf.h>

/* Enable medium level debugging. */
//#define FARF_HIGH   1    /* Use a value of 0 to disable the specified debug level. */
//#define FARF_MEDIUM 1
//#define FARF_LOW    1

#define LOG_INFO(...) FARF(ALWAYS, __VA_ARGS__);
#define LOG_ERR(...) FARF(ALWAYS, __VA_ARGS__);
#define LOG_DEBUG(...) FARF(MEDIUM, __VA_ARGS__);
#else
/* Use the following macro for debug output on the Application Processor. */
#include <stdio.h>
#define LOG_INFO(...)	do{ printf(__VA_ARGS__); printf("\n"); }while(0)
#define LOG_ERR(...)	do{ printf(__VA_ARGS__); printf("\n"); }while(0)
#define LOG_DEBUG(...)	do{ printf(__VA_ARGS__); printf("\n"); }while(0)
#endif
