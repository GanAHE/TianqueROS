/****************************************************************************
 * libs/libc/string/lib_psa_getschedparam.c
 *
 *   Copyright (C) 2013, 2015 Gregory Nutt. All rights reserved.
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

#include <sched.h>
#include <spawn.h>
#include <assert.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: posix_spawnattr_getschedparam
 *
 * Description:
 *   The posix_spawnattr_getschedparam() function will obtain the value of
 *   the spawn-schedparam attribute from the attributes object referenced
 *   by attr.
 *
 * Input Parameters:
 *   attr - The address spawn attributes to be queried.
 *   param - The location to return the spawn-schedparam value.
 *
 * Returned Value:
 *   On success, these functions return 0; on failure they return an error
 *   number from <errno.h>.
 *
 ****************************************************************************/

int posix_spawnattr_getschedparam(FAR const posix_spawnattr_t *attr,
                                  FAR struct sched_param *param)
{
  DEBUGASSERT(attr && param);
  param->sched_priority = attr->priority;
#ifdef CONFIG_SCHED_SPORADIC
  param->sched_ss_low_priority        = (int)attr->low_priority;
  param->sched_ss_max_repl            = (int)attr->max_repl;
  param->sched_ss_repl_period.tv_sec  = attr->repl_period.tv_sec;
  param->sched_ss_repl_period.tv_nsec = attr->repl_period.tv_nsec;
  param->sched_ss_init_budget.tv_sec  = attr->budget.tv_sec;
  param->sched_ss_init_budget.tv_nsec = attr->budget.tv_nsec;
#endif
  return OK;
}
