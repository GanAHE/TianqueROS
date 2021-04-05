/****************************************************************************
 * examples/thttpd/tasks/tasks.c
 *
 *   Copyright (C) 2009, 2011, 2013, 2015 Gregory Nutt. All rights reserved.
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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sched.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char *g_statenames[] =
{
  "INVALID ",
  "PENDING ",
  "READY   ",
  "RUNNING ",
  "INACTIVE",
  "WAITSEM ",
#ifndef CONFIG_DISABLE_MQUEUE
  "WAITSIG ",
#endif
#ifndef CONFIG_DISABLE_MQUEUE
  "MQNEMPTY",
  "MQNFULL "
#endif
};

static const char *g_ttypenames[4] =
{
  "TASK   ",
  "PTHREAD",
  "KTHREAD",
  "--?--  "
};

static FAR const char *g_policynames[4] =
{
  "FIFO",
  "RR  ",
  "SPOR",
  "OTHR"
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* NOTEs:
 *
 * 1. One limitation in the use of NXFLAT is that functions that are
 *    referenced as a pointer-to-a-function must have global scope.  Otherwise
 *    ARM GCC will generate some bad logic.
 * 2. In general, when called back, there is no guarantee to that PIC registers
 *    will be valid and, unless you take special precautions, it could be
 *    dangerous to reference global variables in the callback function.
 */

/* static */ void show_task(FAR struct tcb_s *tcb, FAR void *arg)
{
  FAR const char *policy;
  int i;

  /* Show task/thread status */

  policy = g_policynames[(tcb->flags & TCB_FLAG_POLICY_MASK) >> TCB_FLAG_POLICY_SHIFT];
  printf("%5d %3d %4s %7s%c%c %8s ",
         tcb->pid, tcb->sched_priority, policy,
         g_ttypenames[(tcb->flags & TCB_FLAG_TTYPE_MASK) >> TCB_FLAG_TTYPE_SHIFT],
         tcb->flags & TCB_FLAG_NONCANCELABLE ? 'N' : ' ',
         tcb->flags & TCB_FLAG_CANCEL_PENDING ? 'P' : ' ',
         g_statenames[tcb->task_state]);

  /* Is this a task or a thread? */

#ifndef CONFIG_DISABLE_PTHREAD
  if ((tcb->flags & TCB_FLAG_TTYPE_MASK) == TCB_FLAG_TTYPE_PTHREAD)
    {
      FAR struct pthread_tcb_s *ptcb = (FAR struct pthread_tcb_s *)tcb;

      /* It is a pthread.  Show any name assigned to the pthread via prtcl() along
       * with the startup value.
       */

#if CONFIG_TASK_NAME_SIZE > 0
      printf("%s(%p)\n", tcb->name, ptcb->arg);
#else
      printf("pthread(%p)\n", ptcb->arg);
#endif
    }
  else
#endif
    {
      FAR struct task_tcb_s *ttcb = (FAR struct task_tcb_s *)tcb;

      /* Show task name and arguments */

      printf("%s(", ttcb->argv[0]);

      /* Special case 1st argument (no comma) */

      if (ttcb->argv[1])
        {
         printf("%p", ttcb->argv[1]);
        }

      /* Then any additional arguments */

      for (i = 2; ttcb->argv[i]; i++)
        {
          printf(", %p", ttcb->argv[i]);
         }

      printf(")\n");
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_THTTPD_BINFS
int tasks_main(int argc, char *argv[])
#else
int main(int argc, char *argv[])
#endif
{
  puts(
    "Content-type: text/html\r\n"
    "Status: 200/html\r\n"
    "\r\n"
    "<html>\r\n"
      "<head>\r\n"
        "<title>NuttX Tasks</title>\r\n"
        "<link rel=\"stylesheet\" type=\"text/css\" href=\"/style.css\">\r\n"
      "</head>\r\n"
      "<body bgcolor=\"#fffeec\" text=\"black\">\r\n"
        "<div class=\"menu\">\r\n"
        "<div class=\"menubox\"><a href=\"/index.html\">Front page</a></div>\r\n"
        "<div class=\"menubox\"><a href=\"hello\">Say Hello</a></div>\r\n"
        "<div class=\"menubox\"><a href=\"tasks\">Tasks</a></div>\r\n"
        "<br>\r\n"
        "</div>\r\n"
        "<div class=\"contentblock\">\r\n"
        "<pre>\r\n"
        "PID   PRI SCHD TYPE   NP STATE    NAME\r\n");

  sched_foreach(show_task, NULL);

  puts(
        "</pre>\r\n"
      "</body>\r\n"
   "</html>\r\n");
  return 0;
}
