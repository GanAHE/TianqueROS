/****************************************************************************
 * netutils/netlib/netlib_getpanid.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include <net/if.h>
#include <nuttx/wireless/ieee802154/ieee802154_mac.h>

#include "netutils/netlib.h"

#if defined(CONFIG_NET_6LOWPAN) || defined(CONFIG_NET_IEEE802154)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netlib_getpanid
 *
 * Description:
 *   Return the current PAN ID
 *
 * Parameters:
 *   ifname The name of the interface to use
 *   panid  The location to return the current PAN ID
 *
 * Return:
 *   0 on success; -1 on failure.  errno will be set on failure.
 *
 ****************************************************************************/

int netlib_getpanid(FAR const char *ifname, FAR uint8_t *panid)
{
  struct ieee802154_netmac_s arg;
  int ret = ERROR;

  if (ifname != NULL && panid != NULL)
    {
      /* Get a socket (only so that we get access to the INET subsystem) */

      int sockfd = socket(PF_INET6, NETLIB_SOCK_TYPE, 0);
      if (sockfd >= 0)
        {
          /* Perform the IOCTL */

         strncpy(arg.ifr_name, ifname, IFNAMSIZ);
         arg.u.getreq.attr = IEEE802154_ATTR_MAC_PANID;

         ret = ioctl(sockfd, MAC802154IOC_MLME_GET_REQUEST,
                     (unsigned long)((uintptr_t)&arg));
         if (ret < 0)
           {
             ret = -errno;
             fprintf(stderr, "MAC802154IOC_MLME_GET_REQUEST failed: %d\n",
                     ret);
           }

          close(sockfd);
          IEEE802154_PANIDCOPY(panid, arg.u.getreq.attrval.mac.panid);
        }
    }

  return ret;
}

#endif /* CONFIG_NET_6LOWPAN || CONFIG_NET_IEEE802154 */

