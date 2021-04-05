/****************************************************************************
 * netutils/netlib/netlib_setmacaddr.c
 *
 *   Copyright (C) 2007-2009, 2011-2012, 2017-2018 Gregory Nutt. All rights
 *     reserved.
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
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include <netinet/in.h>
#include <net/if.h>

#include "netutils/netlib.h"

#ifdef CONFIG_NET

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* The address family that we used to create the socket and in the IOCTL
 * data really does not matter.  It should, however, be valid in the current
 * configuration.
 */

#if defined(CONFIG_NET_IPv4)
#  define PF_FAMILY  PF_INET
#  define AF_FAMILY  AF_INET
#elif defined(CONFIG_NET_IPv6)
#  define PF_FAMILY  PF_INET6
#  define AF_FAMILY  AF_INET6
#elif defined(CONFIG_NET_PKT)
#  define PF_FAMILY  PF_PACKET
#  define AF_FAMILY  AF_PACKET
#elif defined(CONFIG_NET_IEEE802154)
#  define PF_FAMILY  PF_IEEE802154
#  define AF_FAMILY  AF_IEEE802154
#elif defined(CONFIG_NET_BLUETOOTH)
#  define PF_FAMILY  PF_BLUETOOTH
#  define AF_FAMILY  AF_BLUETOOTH
#elif defined(CONFIG_NET_USRSOCK)
#  define PF_FAMILY  PF_INET
#  define AF_FAMILY  AF_INET
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netlib_setmacaddr
 *
 * Description:
 *   Set the network driver MAC address
 *
 * Parameters:
 *   ifname   The name of the interface to use
 *   macaddr  MAC address to set, size must be IFHWADDRLEN
 *
 * Return:
 *   0 on success; -1 on failure
 *
 ****************************************************************************/

int netlib_setmacaddr(const char *ifname, const uint8_t *macaddr)
{
  int ret = ERROR;

  if (ifname && macaddr)
    {
      /* Get a socket (only so that we get access to the INET subsystem) */

      int sockfd = socket(PF_FAMILY, NETLIB_SOCK_TYPE, 0);
      if (sockfd >= 0)
        {
          struct ifreq req;

          /* Put the driver name into the request */

          strncpy(req.ifr_name, ifname, IFNAMSIZ);

          /* Put the new MAC address into the request */

          req.ifr_hwaddr.sa_family = AF_FAMILY;
          memcpy(&req.ifr_hwaddr.sa_data, macaddr, IFHWADDRLEN);

          /* Perform the ioctl to set the MAC address */

          ret = ioctl(sockfd, SIOCSIFHWADDR, (unsigned long)&req);
          close(sockfd);
        }
    }

  return ret;
}

#endif /* CONFIG_NET */

