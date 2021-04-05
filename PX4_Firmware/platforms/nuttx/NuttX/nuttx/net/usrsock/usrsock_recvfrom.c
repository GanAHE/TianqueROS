/****************************************************************************
 * net/usrsock/usrsock_recvfrom.c
 *
 *  Copyright (C) 2015, 2017 Haltian Ltd. All rights reserved.
 *  Author: Jussi Kivilinna <jussi.kivilinna@haltian.com>
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_USRSOCK)

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/irq.h>

#include <sys/socket.h>
#include <nuttx/semaphore.h>
#include <nuttx/net/net.h>
#include <nuttx/net/usrsock.h>

#include "usrsock/usrsock.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint16_t recvfrom_event(FAR struct net_driver_s *dev,
                               FAR void *pvconn, FAR void *pvpriv,
                               uint16_t flags)
{
  FAR struct usrsock_data_reqstate_s *pstate = pvpriv;
  FAR struct usrsock_conn_s *conn = pvconn;

  if (flags & USRSOCK_EVENT_ABORT)
    {
      ninfo("socket aborted.\n");

      pstate->reqstate.result = -ECONNABORTED;
      pstate->valuelen = 0;
      pstate->valuelen_nontrunc = 0;

      /* Stop further callbacks */

      pstate->reqstate.cb->flags   = 0;
      pstate->reqstate.cb->priv    = NULL;
      pstate->reqstate.cb->event   = NULL;

      /* Wake up the waiting thread */

      nxsem_post(&pstate->reqstate.recvsem);
    }
  else if (flags & USRSOCK_EVENT_REQ_COMPLETE)
    {
      ninfo("request completed.\n");

      pstate->reqstate.result = conn->resp.result;
      if (pstate->reqstate.result < 0)
        {
          pstate->valuelen = 0;
          pstate->valuelen_nontrunc = 0;
        }
      else
        {
          pstate->valuelen = conn->resp.valuelen;
          pstate->valuelen_nontrunc = conn->resp.valuelen_nontrunc;
        }

      if (pstate->reqstate.result >= 0 ||
          pstate->reqstate.result == -EAGAIN)
        {
          /* After reception of data, mark input not ready. Daemon will
           * send event to restore this flag.
           */

          conn->flags &= ~USRSOCK_EVENT_RECVFROM_AVAIL;
        }

      /* Stop further callbacks */

      pstate->reqstate.cb->flags   = 0;
      pstate->reqstate.cb->priv    = NULL;
      pstate->reqstate.cb->event   = NULL;

      /* Wake up the waiting thread */

      nxsem_post(&pstate->reqstate.recvsem);
    }
  else if (flags & USRSOCK_EVENT_REMOTE_CLOSED)
    {
      ninfo("remote closed.\n");

      pstate->reqstate.result = -EPIPE;

      /* Stop further callbacks */

      pstate->reqstate.cb->flags   = 0;
      pstate->reqstate.cb->priv    = NULL;
      pstate->reqstate.cb->event   = NULL;

      /* Wake up the waiting thread */

      nxsem_post(&pstate->reqstate.recvsem);
    }
  else if (flags & USRSOCK_EVENT_RECVFROM_AVAIL)
    {
      ninfo("recvfrom avail.\n");

      flags &= ~USRSOCK_EVENT_RECVFROM_AVAIL;

      /* Stop further callbacks */

      pstate->reqstate.cb->flags   = 0;
      pstate->reqstate.cb->priv    = NULL;
      pstate->reqstate.cb->event   = NULL;

      /* Wake up the waiting thread */

      nxsem_post(&pstate->reqstate.recvsem);
    }

  return flags;
}

/****************************************************************************
 * Name: do_recvfrom_request
 ****************************************************************************/

static int do_recvfrom_request(FAR struct usrsock_conn_s *conn, size_t buflen,
                               socklen_t addrlen)
{
  struct usrsock_request_recvfrom_s req =
  {
  };

  struct iovec bufs[1];

  if (addrlen > UINT16_MAX)
    {
      addrlen = UINT16_MAX;
    }

  if (buflen > UINT16_MAX)
    {
      buflen = UINT16_MAX;
    }

  /* Prepare request for daemon to read. */

  req.head.reqid = USRSOCK_REQUEST_RECVFROM;
  req.usockid = conn->usockid;
  req.max_addrlen = addrlen;
  req.max_buflen = buflen;

  bufs[0].iov_base = (FAR void *)&req;
  bufs[0].iov_len = sizeof(req);

  return usrsockdev_do_request(conn, bufs, ARRAY_SIZE(bufs));
}

/****************************************************************************
 * Name: usrsock_recvfrom
 *
 * Description:
 *   recvfrom() receives messages from a socket, and may be used to receive
 *   data on a socket whether or not it is connection-oriented.
 *
 *   If from is not NULL, and the underlying protocol provides the source
 *   address, this source address is filled in. The argument fromlen
 *   initialized to the size of the buffer associated with from, and modified
 *   on return to indicate the actual size of the address stored there.
 *
 * Input Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   buf      Buffer to receive data
 *   len      Length of buffer
 *   flags    Receive flags (ignored)
 *   from     Address of source (may be NULL)
 *   fromlen  The length of the address structure
 *
 ****************************************************************************/

ssize_t usrsock_recvfrom(FAR struct socket *psock, FAR void *buf, size_t len,
                         int flags, FAR struct sockaddr *from,
                         FAR socklen_t *fromlen)
{
  FAR struct usrsock_conn_s *conn = psock->s_conn;
  struct usrsock_data_reqstate_s state =
  {
  };

  struct iovec inbufs[2];
  socklen_t addrlen = 0;
  socklen_t outaddrlen = 0;
  ssize_t ret;
#ifdef CONFIG_NET_SOCKOPTS
  struct timespec abstime;
#endif
  struct timespec *ptimeo = NULL;

  DEBUGASSERT(conn);

  if (fromlen)
    {
      if (*fromlen > 0 && from == NULL)
        {
          return -EINVAL;
        }

      addrlen = *fromlen;
    }

  net_lock();

  if (conn->state == USRSOCK_CONN_STATE_UNINITIALIZED ||
      conn->state == USRSOCK_CONN_STATE_ABORTED)
    {
      /* Invalid state or closed by daemon. */

      ninfo("usockid=%d; connect() with uninitialized usrsock.\n",
            conn->usockid);

      ret = (conn->state == USRSOCK_CONN_STATE_ABORTED) ? -EPIPE :
            -ECONNRESET;
      goto errout_unlock;
    }

  if (conn->type == SOCK_STREAM || conn->type == SOCK_SEQPACKET)
    {
      if (!conn->connected)
        {
          if (conn->state == USRSOCK_CONN_STATE_CONNECTING)
            {
              /* Connecting. */

              ninfo("usockid=%d; socket still connecting.\n",
                    conn->usockid);

              ret = -EAGAIN;
              goto errout_unlock;
            }
          else
            {
              /* Not connected. */

              ninfo("usockid=%d; socket not connected.\n",
                    conn->usockid);

              ret = -ENOTCONN;
              goto errout_unlock;
            }
        }
    }

  if (conn->state == USRSOCK_CONN_STATE_CONNECTING)
    {
      /* Non-blocking connecting. */

      ninfo("usockid=%d; socket still connecting.\n",
            conn->usockid);

      ret = -EAGAIN;
      goto errout_unlock;
    }

#ifdef CONFIG_NET_SOCKOPTS
  if (psock->s_rcvtimeo != 0)
    {
      DEBUGVERIFY(clock_gettime(CLOCK_REALTIME, &abstime));

      /* Prepare timeout value for recvfrom. */

      abstime.tv_sec += psock->s_rcvtimeo / DSEC_PER_SEC;
      abstime.tv_nsec += (psock->s_rcvtimeo % DSEC_PER_SEC) * NSEC_PER_DSEC;
      if (abstime.tv_nsec >= NSEC_PER_SEC)
        {
          abstime.tv_sec++;
          abstime.tv_nsec -= NSEC_PER_SEC;
        }

      ptimeo = &abstime;
    }
#endif

  do
    {
      /* Check if remote end has closed connection. */

      if (conn->flags & USRSOCK_EVENT_REMOTE_CLOSED)
        {
          ninfo("usockid=%d; remote closed (EOF).\n", conn->usockid);

          ret = 0;
          goto errout_unlock;
        }

      /* Check if need to wait for receive data to become available. */

      if (!(conn->flags & USRSOCK_EVENT_RECVFROM_AVAIL))
        {
          if (_SS_ISNONBLOCK(psock->s_flags))
            {
              /* Nothing to receive from daemon side. */

              ret = -EAGAIN;
              goto errout_unlock;
            }

          /* Wait recv to become avail. */

          ret = usrsock_setup_data_request_callback(
              conn, &state, recvfrom_event,
              USRSOCK_EVENT_ABORT | USRSOCK_EVENT_RECVFROM_AVAIL |
              USRSOCK_EVENT_REMOTE_CLOSED);
          if (ret < 0)
            {
              nwarn("usrsock_setup_request_callback failed: %d\n", ret);
              goto errout_unlock;
            }

          /* Wait for receive-avail (or abort, or timeout, or signal). */

          ret = net_timedwait(&state.reqstate.recvsem, ptimeo);
          if (ret < 0)
            {
              if (ret == -ETIMEDOUT)
                {
                  ninfo("recvfrom timedout\n");

                  ret = -EAGAIN;
                }
              else if (ret == -EINTR)
                {
                  ninfo("recvfrom interrupted\n");
                }
              else
                {
                  nerr("net_timedwait errno: %d\n", ret);
                  DEBUGASSERT(false);
                }
            }

          usrsock_teardown_data_request_callback(&state);

          /* Did wait timeout or got signal? */

          if (ret != 0)
            {
              goto errout_unlock;
            }

          /* Was socket aborted? */

          if (conn->state == USRSOCK_CONN_STATE_ABORTED)
            {
              ret = -EPIPE;
              goto errout_unlock;
            }

          /* Did remote disconnect? */

          if (conn->flags & USRSOCK_EVENT_REMOTE_CLOSED)
            {
              ret = 0;
              goto errout_unlock;
            }

          DEBUGASSERT(conn->flags & USRSOCK_EVENT_RECVFROM_AVAIL);
        }

      /* Set up event callback for usrsock. */

      ret = usrsock_setup_data_request_callback(
          conn, &state, recvfrom_event,
          USRSOCK_EVENT_ABORT | USRSOCK_EVENT_REQ_COMPLETE);
      if (ret < 0)
        {
          nwarn("usrsock_setup_request_callback failed: %d\n", ret);
          goto errout_unlock;
        }

      inbufs[0].iov_base = (FAR void *)from;
      inbufs[0].iov_len = addrlen;
      inbufs[1].iov_base = (FAR void *)buf;
      inbufs[1].iov_len = len;

      usrsock_setup_datain(conn, inbufs, ARRAY_SIZE(inbufs));

      /* Request user-space daemon to close socket. */

      ret = do_recvfrom_request(conn, len, addrlen);
      if (ret >= 0)
        {
          /* Wait for completion of request. */

          while ((ret = net_lockedwait(&state.reqstate.recvsem)) < 0)
            {
              DEBUGASSERT(ret == -EINTR || ret == -ECANCELED);
            }

          ret = state.reqstate.result;

          DEBUGASSERT(ret <= (ssize_t)len);
          DEBUGASSERT(state.valuelen <= addrlen);
          DEBUGASSERT(state.valuelen <= state.valuelen_nontrunc);

          if (ret >= 0)
            {
              /* Store length of 'from' address that was available at
               * daemon-side.
               */

              outaddrlen = state.valuelen_nontrunc;
            }
        }

      usrsock_teardown_datain(conn);
      usrsock_teardown_data_request_callback(&state);
    }
  while (ret == -EAGAIN);

errout_unlock:
  net_unlock();

  if (fromlen)
    {
      *fromlen = outaddrlen;
    }

  return ret;
}

#endif /* CONFIG_NET && CONFIG_NET_USRSOCK */
