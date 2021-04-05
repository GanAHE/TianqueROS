/****************************************************************************
 * system/cu/cu_main.c
 *
 *   Copyright (C) 2014 sysmocom - s.f.m.c. GmbH. All rights reserved.
 *   Author: Harald Welte <hwelte@sysmocom.de>
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

#include <sys/types.h>
#include <sys/stat.h>

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <fcntl.h>
#include <errno.h>
#include <stdlib.h>
#include <signal.h>
#include <termios.h>
#include <debug.h>

#include "system/readline.h"

#include "cu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef SIGKILL
#  define SIGKILL 9
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum parity_mode
{
  PARITY_NONE,
  PARITY_EVEN,
  PARITY_ODD,
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct cu_globals_s g_cu;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cu_listener
 *
 * Description:
 *   Entry point for the listener thread.
 *
 ****************************************************************************/

static FAR void *cu_listener(FAR void *parameter)
{
  for (; ; )
    {
      int rc;
      char ch;

      rc = read(g_cu.infd, &ch, 1);
      if (rc <= 0)
        {
          break;
        }

      fputc(ch, stdout);
      fflush(stdout);
    }

  /* Won't get here */

  return NULL;
}

static void sigint(int sig)
{
  pthread_cancel(g_cu.listener);
  tcflush(g_cu.outfd, TCIOFLUSH);
  close(g_cu.outfd);
  close(g_cu.infd);
  exit(0);
}

static int enable_crlf_conversion(int fd)
{
#ifdef CONFIG_SERIAL_TERMIOS
  int rc = 0;
  int ret;
  struct termios tio;

  /* enable \n -> \r\n conversion during write */

  ret = tcgetattr(fd, &tio);
  if (ret)
    {
      fprintf(stderr, "en_crlf_conv: ERROR during tcgetattr(): %d\n", errno);
      rc = -1;
    }

  tio.c_oflag = OPOST | ONLCR;
  ret = tcsetattr(fd, TCSANOW, &tio);
  if (ret)
    {
      fprintf(stderr, "en_crlf_conv: ERROR during tcsetattr(): %d\n", errno);
      rc = -1;
    }

  return rc;
#else
  return -1;
#endif
}

static int set_baudrate(int fd, int rate, enum parity_mode parity, int rtscts)
{
#ifdef CONFIG_SERIAL_TERMIOS
  int rc = 0;
  int ret;
  struct termios tio;

  /* enable \n -> \r\n conversion during write */

  ret = tcgetattr(fd, &tio);
  if (ret)
    {
      fprintf(stderr, "set_baudrate: ERROR during tcgetattr(): %d\n", errno);
      rc = -1;
    }

  if (rate != 0)
    {
      cfsetspeed(&tio, rate);
    }

  switch (parity)
    {
      case PARITY_EVEN:
        tio.c_cflag = PARENB;
        break;

      case PARITY_ODD:
        tio.c_cflag = PARENB | PARODD;
        break;

      case PARITY_NONE:
        break;
    }

  if (rtscts)
    {
      tio.c_cflag |= CRTS_IFLOW | CCTS_OFLOW;
    }

  ret = tcsetattr(fd, TCSANOW, &tio);
  if (ret)
    {
      fprintf(stderr, "set_baudrate: ERROR during tcsetattr(): %d\n", errno);
      rc = -1;
    }

  return rc;
#else
  if (rate == 0)
    {
      return 0;
    }

  return -1;
#endif
}

static void print_help(void)
{
  printf("Usage: cu [options]\n"
         " -l: Use named device (default %s)\n"
         " -e: Set even parity\n"
         " -o: Set odd parity\n"
         " -s: Use given speed (default %d)\n"
         " -r: Disable RTS/CTS flow control (default: on)\n"
         " -?: This help\n",
         CONFIG_SYSTEM_CUTERM_DEFAULT_DEVICE,
         CONFIG_SYSTEM_CUTERM_DEFAULT_BAUD);
}

static void print_escape_help(void)
{
  printf("[Escape sequences]\n"
         "[~. hangup]\n"
         );
}

static int cu_cmd(char bcmd)
{
  switch (bcmd)
    {
    case '?':
      print_escape_help();
      break;

    case '.':
      return 1;

    /* FIXME: implement other commands such as send/receive file */
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cu_main
 *
 * Description:
 *   Main entry point for the serial terminal example.
 *
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  pthread_attr_t attr;
  struct sigaction sa;
  FAR char *devname = CONFIG_SYSTEM_CUTERM_DEFAULT_DEVICE;
  int baudrate = CONFIG_SYSTEM_CUTERM_DEFAULT_BAUD;
  enum parity_mode parity = PARITY_NONE;
  int rtscts = 1;
  int option;
  int ret;
  int bcmd;
  int start_of_line = 1;
  int exitval = EXIT_FAILURE;

  /* Initialize global data */

  memset(&g_cu, 0, sizeof(struct cu_globals_s));

  /* Install signal handlers */

  memset(&sa, 0, sizeof(sa));
  sa.sa_handler = sigint;
  sigaction(SIGKILL, &sa, NULL);

  optind = 0;   /* global that needs to be reset in FLAT mode */
  while ((option = getopt(argc, argv, "l:s:ehor?")) != ERROR)
    {
      switch (option)
        {
          case 'l':
            devname = optarg;
            break;

          case 's':
            baudrate = atoi(optarg);
            break;

          case 'e':
            parity = PARITY_ODD;
            break;

          case 'o':
            parity = PARITY_EVEN;
            break;

          case 'r':
            rtscts = 0;
            break;

          case 'h':
          case '?':
            print_help();
            return EXIT_SUCCESS;

          default:
            return EXIT_FAILURE;
        }
    }

  /* Open the serial device for writing */

  g_cu.outfd = open(devname, O_WRONLY);
  if (g_cu.outfd < 0)
    {
      fprintf(stderr, "cu_main: ERROR: Failed to open %s for writing: %d\n",
              devname, errno);
      goto errout_with_devinit;
    }

  enable_crlf_conversion(g_cu.outfd);
  set_baudrate(g_cu.outfd, baudrate, parity, rtscts);

  /* Open the serial device for reading.  Since we are already connected, this
   * should not fail.
   */

  g_cu.infd = open(devname, O_RDONLY);
  if (g_cu.infd < 0)
    {
      fprintf(stderr, "cu_main: ERROR: Failed to open %s for reading: %d\n",
             devname, errno);
      goto errout_with_outfd;
    }

  /* Start the serial receiver thread */

  ret = pthread_attr_init(&attr);
  if (ret != OK)
    {
      fprintf(stderr, "cu_main: pthread_attr_init failed: %d\n", ret);
      goto errout_with_fds;
    }

  /* Set priority of listener to configured value */

  attr.priority = CONFIG_SYSTEM_CUTERM_PRIORITY;

  ret = pthread_create(&g_cu.listener, &attr,
                       cu_listener, (pthread_addr_t)0);
  if (ret != 0)
    {
      fprintf(stderr, "cu_main: Error in thread creation: %d\n", ret);
      goto errout_with_fds;
    }

  /* Send messages and get responses -- forever */

  for (; ; )
    {
      int ch = getc(stdin);

      if (ch <= 0)
        {
          continue;
        }

      if (start_of_line == 1 && ch == '~')
        {
          /* We've seen and escape (~) character, echo it to local
           * terminal and read the next char from serial
           */

          fputc(ch, stdout);
          bcmd = getc(stdin);
          if (bcmd == ch)
            {
              /* Escaping a tilde: handle like normal char */

              write(g_cu.outfd, &ch, 1);
              continue;
            }
          else
            {
              if (cu_cmd(bcmd) == 1)
                {
                  break;
                }
            }
        }

      /* Normal character */

      write(g_cu.outfd, &ch, 1);

      /* Determine if we are now at the start of a new line or not */

      if (ch == '\n' || ch == '\r')
        {
          start_of_line = 1;
        }
      else
        {
          start_of_line = 0;
        }
    }

  pthread_cancel(g_cu.listener);
  pthread_attr_destroy(&attr);
  exitval = EXIT_SUCCESS;

  /* Error exits */

errout_with_fds:
  close(g_cu.infd);
errout_with_outfd:
  close(g_cu.outfd);
errout_with_devinit:
  return exitval;
}
