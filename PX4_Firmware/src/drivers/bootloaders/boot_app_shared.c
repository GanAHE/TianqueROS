/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
 *       Author: Ben Dyer <ben_dyer@mac.com>
 *               Pavel Kirienko <pavel.kirienko@zubax.com>
 *               David Sidrane <david_s5@nscdg.com>
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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

#include <stdint.h>
#include <string.h>

#include "chip.h"
#include "stm32.h"

#include <errno.h>
#include "boot_app_shared.h"
#include "systemlib/crc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BOOTLOADER_COMMON_APP_SIGNATURE         0xB0A04150u
#define BOOTLOADER_COMMON_BOOTLOADER_SIGNATURE  0xB0A0424Cu


/*  CAN_FiRx where (i=0..27|13, x=1, 2)
 *                      STM32_CAN1_FIR(i,x)
 * Using i = 2 does not requier there block
 * to be enabled nor FINIT in CAN_FMR to be set.
 * todo:Validate this claim on F2, F3
 */

#define crc_HiLOC       STM32_CAN1_FIR(2,1)
#define crc_LoLOC       STM32_CAN1_FIR(2,2)
#define signature_LOC   STM32_CAN1_FIR(3,1)
#define bus_speed_LOC   STM32_CAN1_FIR(3,2)
#define node_id_LOC     STM32_CAN1_FIR(4,1)
#define CRC_H 1
#define CRC_L 0

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: read
 ****************************************************************************/

inline static void read(bootloader_app_shared_t *pshared)
{
	pshared->signature = getreg32(signature_LOC);
	pshared->bus_speed = getreg32(bus_speed_LOC);
	pshared->node_id = getreg32(node_id_LOC);
	pshared->crc.ul[CRC_L] = getreg32(crc_LoLOC);
	pshared->crc.ul[CRC_H] = getreg32(crc_HiLOC);

}

/****************************************************************************
 * Name: write
 ****************************************************************************/

inline static void write(bootloader_app_shared_t *pshared)
{
	putreg32(pshared->signature, signature_LOC);
	putreg32(pshared->bus_speed, bus_speed_LOC);
	putreg32(pshared->node_id, node_id_LOC);
	putreg32(pshared->crc.ul[CRC_L], crc_LoLOC);
	putreg32(pshared->crc.ul[CRC_H], crc_HiLOC);

}

/****************************************************************************
 * Name: calulate_signature
 ****************************************************************************/

static uint64_t calulate_signature(bootloader_app_shared_t *pshared)
{
	uint64_t crc;
	crc = crc64_add_word(CRC64_INITIAL, pshared->signature);
	crc = crc64_add_word(crc, pshared->bus_speed);
	crc = crc64_add_word(crc, pshared->node_id);
	crc ^= CRC64_OUTPUT_XOR;
	return crc;
}

/****************************************************************************
 * Name: bootloader_app_shared_init
 ****************************************************************************/
static void bootloader_app_shared_init(bootloader_app_shared_t *pshared, eRole_t role)
{
	memset(pshared, 0, sizeof(bootloader_app_shared_t));

	if (role != Invalid) {
		pshared->signature =
			(role ==
			 App ? BOOTLOADER_COMMON_APP_SIGNATURE :
			 BOOTLOADER_COMMON_BOOTLOADER_SIGNATURE);
	}

}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name: bootloader_app_shared_read
 *
 * Description:
 *   Based on the role requested, this function will conditionally populate
 *   a bootloader_app_shared_t structure from the physical locations used
 *   to transfer the shared data to/from an application (internal data) .
 *
 *   The functions will only populate the structure and return a status
 *   indicating success, if the internal data has the correct signature as
 *   requested by the Role AND has a valid crc.
 *
 * Input Parameters:
 *   shared - A pointer to a bootloader_app_shared_t return the data in if
 *   the internal data is valid for the requested Role
 *   role   - An eRole_t of App or BootLoader to validate the internal data
 *            against. For a Bootloader this would be the value of App to
 *            read the application passed data.
 *
 * Returned value:
 *   OK     - Indicates that the internal data has been copied to callers
 *            bootloader_app_shared_t structure.
 *
 *  -EBADR  - The Role or crc of the internal data was not valid. The copy
 *            did not occur.
 *
 ****************************************************************************/

__EXPORT
int bootloader_app_shared_read(bootloader_app_shared_t *shared,
			       eRole_t role)
{
	int rv = -EBADR;
	bootloader_app_shared_t working;

	read(&working);

	if ((role == App ? working.signature == BOOTLOADER_COMMON_APP_SIGNATURE
	     : working.signature == BOOTLOADER_COMMON_BOOTLOADER_SIGNATURE)
	    && (working.crc.ull == calulate_signature(&working))) {
		*shared = working;
		rv = OK;
	}

	return rv;
}

/****************************************************************************
 * Name: bootloader_app_shared_write
 *
 * Description:
 *   Based on the role, this function will commit the data passed
 *   into the physical locations used to transfer the shared data to/from
 *   an application (internal data) .
 *
 *   The functions will populate the signature and crc the data
 *   based on the provided Role.
 *
 * Input Parameters:
 *   shared - A pointer to a bootloader_app_shared_t data to commit to
 *   the internal data for passing to/from an application.
 *   role   - An eRole_t of App or BootLoader to use in the internal data
 *            to be passed to/from an application. For a Bootloader this
 *            would be the value of Bootloader to write to the passed data.
 *            to the application via the internal data.
 *
 * Returned value:
 *   None.
 *
 ****************************************************************************/
__EXPORT
void bootloader_app_shared_write(bootloader_app_shared_t *shared,
				 eRole_t role)
{
	bootloader_app_shared_t working = *shared;
	working.signature =
		(role ==
		 App ? BOOTLOADER_COMMON_APP_SIGNATURE :
		 BOOTLOADER_COMMON_BOOTLOADER_SIGNATURE);
	working.crc.ull = calulate_signature(&working);
	write(&working);

}

/****************************************************************************
 * Name: bootloader_app_shared_invalidate
 *
 * Description:
 *   Invalidates the data passed the physical locations used to transfer
 *   the shared data to/from an application (internal data) .
 *
 *   The functions will invalidate the signature and crc and shoulf be used
 *   to prevent deja vu.
 *
 * Input Parameters:
 *   None.
 *
 * Returned value:
 *   None.
 *
 ****************************************************************************/

__EXPORT
void bootloader_app_shared_invalidate(void)
{
	bootloader_app_shared_t working;
	bootloader_app_shared_init(&working, Invalid);
	write(&working);
}
