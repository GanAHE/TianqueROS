/****************************************************************************
 * examples/udp/udp_netinit.c
 *
 *   Copyright (C) 2007, 2011, 2015, 2017 Gregory Nutt. All rights reserved.
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

#include "config.h"

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>

#include <arpa/inet.h>
#include <netinet/in.h>

#include "netutils/netlib.h"

#include "udp.h"

#ifdef CONFIG_EXAMPLES_UDP_NETINIT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_EXAMPLES_UDP_DEVNAME
#  define DEVNAME CONFIG_EXAMPLES_UDP_DEVNAME
#else
#  define DEVNAME "eth0"
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined(CONFIG_EXAMPLES_UDP_IPv6) && !defined(CONFIG_NET_ICMPv6_AUTOCONF)
/* Our host IPv6 address */

static const uint16_t g_ipv6_hostaddr[8] =
{
  HTONS(CONFIG_EXAMPLES_UDP_IPv6ADDR_1),
  HTONS(CONFIG_EXAMPLES_UDP_IPv6ADDR_2),
  HTONS(CONFIG_EXAMPLES_UDP_IPv6ADDR_3),
  HTONS(CONFIG_EXAMPLES_UDP_IPv6ADDR_4),
  HTONS(CONFIG_EXAMPLES_UDP_IPv6ADDR_5),
  HTONS(CONFIG_EXAMPLES_UDP_IPv6ADDR_6),
  HTONS(CONFIG_EXAMPLES_UDP_IPv6ADDR_7),
  HTONS(CONFIG_EXAMPLES_UDP_IPv6ADDR_8),
};

/* Default routine IPv6 address */

static const uint16_t g_ipv6_draddr[8] =
{
  HTONS(CONFIG_EXAMPLES_UDP_DRIPv6ADDR_1),
  HTONS(CONFIG_EXAMPLES_UDP_DRIPv6ADDR_2),
  HTONS(CONFIG_EXAMPLES_UDP_DRIPv6ADDR_3),
  HTONS(CONFIG_EXAMPLES_UDP_DRIPv6ADDR_4),
  HTONS(CONFIG_EXAMPLES_UDP_DRIPv6ADDR_5),
  HTONS(CONFIG_EXAMPLES_UDP_DRIPv6ADDR_6),
  HTONS(CONFIG_EXAMPLES_UDP_DRIPv6ADDR_7),
  HTONS(CONFIG_EXAMPLES_UDP_DRIPv6ADDR_8),
};

/* IPv6 netmask */

static const uint16_t g_ipv6_netmask[8] =
{
  HTONS(CONFIG_EXAMPLES_UDP_IPv6NETMASK_1),
  HTONS(CONFIG_EXAMPLES_UDP_IPv6NETMASK_2),
  HTONS(CONFIG_EXAMPLES_UDP_IPv6NETMASK_3),
  HTONS(CONFIG_EXAMPLES_UDP_IPv6NETMASK_4),
  HTONS(CONFIG_EXAMPLES_UDP_IPv6NETMASK_5),
  HTONS(CONFIG_EXAMPLES_UDP_IPv6NETMASK_6),
  HTONS(CONFIG_EXAMPLES_UDP_IPv6NETMASK_7),
  HTONS(CONFIG_EXAMPLES_UDP_IPv6NETMASK_8),
};
#endif /* CONFIG_EXAMPLES_UDP_IPv6 && !CONFIG_NET_ICMPv6_AUTOCONF */

static bool g_initialized;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * udp_netinit
 ****************************************************************************/

int udp_netinit(void)
{
  if (!g_initialized)
    {
#ifdef CONFIG_EXAMPLES_UDP_IPv6
#ifdef CONFIG_NET_ICMPv6_AUTOCONF
      /* Perform ICMPv6 auto-configuration */

      netlib_icmpv6_autoconfiguration(DEVNAME);

#else /* CONFIG_NET_ICMPv6_AUTOCONF */

      /* Set up our fixed host address */

      netlib_set_ipv6addr(DEVNAME,
                          (FAR const struct in6_addr *)g_ipv6_hostaddr);

      /* Set up the default router address */

      netlib_set_dripv6addr(DEVNAME,
                            (FAR const struct in6_addr *)g_ipv6_draddr);

      /* Setup the subnet mask */

      netlib_set_ipv6netmask(DEVNAME,
                            (FAR const struct in6_addr *)g_ipv6_netmask);

#endif /* CONFIG_NET_ICMPv6_AUTOCONF */
#else /* CONFIG_EXAMPLES_UDP_IPv6 */

      struct in_addr addr;

      /* Set up our host address */

      addr.s_addr = HTONL(CONFIG_EXAMPLES_UDP_IPADDR);
      netlib_set_ipv4addr(DEVNAME, &addr);

      /* Set up the default router address */

      addr.s_addr = HTONL(CONFIG_EXAMPLES_UDP_DRIPADDR);
      netlib_set_dripv4addr(DEVNAME, &addr);

      /* Setup the subnet mask */

      addr.s_addr = HTONL(CONFIG_EXAMPLES_UDP_NETMASK);
      netlib_set_ipv4netmask(DEVNAME, &addr);

#endif /* CONFIG_EXAMPLES_UDP_IPv6 */

      /* New versions of netlib_set_ipvXaddr will not bring the network up,
       * So ensure the network is really up at this point.
       */

      netlib_ifup(DEVNAME);
      g_initialized = true;
    }

  return 0;
}

#endif /* CONFIG_EXAMPLES_UDP_NETINIT */

