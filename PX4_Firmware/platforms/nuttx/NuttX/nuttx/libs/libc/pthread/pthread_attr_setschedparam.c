/****************************************************************************
 * libs/libc/pthread/pthread_attr_setschedparam.c
 *
 *   Copyright (C) 2007-2009, 2011, 2015 Gregory Nutt. All rights reserved.
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

#include <pthread.h>
#include <string.h>
#include <sched.h>
#include <debug.h>
#include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  pthread_attr_setschedparam
 *
 * Description:
 *
 * Input Parameters:
 *   attr
 *   param
 *
 * Returned Value:
 *   0 if successful.  Otherwise, an error code.
 *
 * Assumptions:
 *
 ****************************************************************************/

int pthread_attr_setschedparam(FAR pthread_attr_t *attr,
                               FAR const struct sched_param *param)
{
  int ret;

  linfo("attr=0x%p param=0x%p\n", attr, param);

  if (!attr || !param)
    {
      ret = EINVAL;
    }
  else
    {
      attr->priority            = (short)param->sched_priority;
#ifdef CONFIG_SCHED_SPORADIC
      attr->low_priority        = (uint8_t)param->sched_ss_low_priority;
      attr->max_repl            = (uint8_t)param->sched_ss_max_repl;
      attr->repl_period.tv_sec  = param->sched_ss_repl_period.tv_sec;
      attr->repl_period.tv_nsec = param->sched_ss_repl_period.tv_nsec;
      attr->budget.tv_sec       = param->sched_ss_init_budget.tv_sec;
      attr->budget.tv_nsec      = param->sched_ss_init_budget.tv_nsec;
#endif
      ret = OK;
    }

  linfo("Returning %d\n", ret);
  return ret;
}
