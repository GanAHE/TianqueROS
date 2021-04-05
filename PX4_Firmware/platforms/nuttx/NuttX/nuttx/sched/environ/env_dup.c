/****************************************************************************
 * sched/environ/env_dup.c
 *
 *   Copyright (C) 2007, 2009, 2011, 2013, 2017 Gregory Nutt. All rights reserved.
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

#ifndef CONFIG_DISABLE_ENVIRON

#include <sys/types.h>
#include <sched.h>
#include <string.h>
#include <errno.h>

#include <nuttx/kmalloc.h>

#include "sched/sched.h"
#include "environ/environ.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: env_dup
 *
 * Description:
 *   Copy the internal environment structure of a task.  This is the action
 *   that is performed when a new task is created: The new task has a private,
 *   exact duplicate of the parent task's environment.
 *
 * Input Parameters:
 *   group - The child task group to receive the newly allocated copy of the
 *           parent task groups environment structure.
 *
 * Returned Value:
 *   zero on success
 *
 * Assumptions:
 *   Not called from an interrupt handler.
 *
 ****************************************************************************/

int env_dup(FAR struct task_group_s *group)
{
  FAR struct tcb_s *ptcb = this_task();
  FAR char *envp = NULL;
  size_t envlen;
  int ret = OK;

  DEBUGASSERT(group != NULL && ptcb != NULL && ptcb->group != NULL);

  /* Pre-emption must be disabled throughout the following because the
   * environment may be shared.
   */

  sched_lock();

  /* Does the parent task have an environment? */

  if (ptcb->group != NULL && ptcb->group->tg_envp != NULL)
    {
      /* Yes.. The parent task has an environment allocation. */

      envlen = ptcb->group->tg_envsize;
      envp   = NULL;

      /* A special case is that the parent has an "empty" environment
       * allocation, i.e., there is an allocation in place but it
       * contains no variable definitions and, hence, envlen == 0.
       */

      if (envlen > 0)
        {
          /* There is an environment, duplicate it */

          envp = (FAR char *)kumm_malloc(envlen);
          if (envp == NULL)
            {
              /* The parent's environment can not be inherited due to a
               * failure in the allocation of the child environment.
               */

              envlen = 0;
              ret    = -ENOMEM;
            }
          else
            {
              /* Duplicate the parent environment. */

              memcpy(envp, ptcb->group->tg_envp, envlen);
            }
        }

      /* Save the size and child environment allocation. */

      group->tg_envsize = envlen;
      group->tg_envp    = envp;
    }

  sched_unlock();
  return ret;
}

#endif /* CONFIG_DISABLE_ENVIRON */
