/****************************************************************************
 *   Copyright (c) 2015 James Wilson. All rights reserved.
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
 * 3. Neither the name ATLFlight nor the names of its contributors may be
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


// dspal_version.h cannot be included here because it would cause the build
// to pick up all the DSP specific types. Instead use --include dspal_version.h

#include "stdbool.h"
#include "adspmsgd.h"
#include "rpcmem.h"
#include "version_test.h"
#include "dspal_log.h"
#include "dspal_version_types.h"

/**
 * @brief Display the DSP version information
 *
 * @return
 * 0 ------ Version information retrieved
 * 1 ------ An error occured
*/

int main()
{
	int status = 0;
	const int heap_id = 22; 	// FIXME - no idea how these are allocated
	const int fastrpc_flags = 0;
	const int buf_size = DSPAL_MAX_LEN_VERSION_INFO_STR * 3;

	rpcmem_init();

        char *versionBuffer = (char *) rpcmem_alloc(heap_id, fastrpc_flags, buf_size);

        int ret = (versionBuffer != NULL) ? true : false;

        if (!ret) {
                LOG_ERR("%s rpcmem_alloc failed! for version string buffer");
                rpcmem_free(versionBuffer);
                return 1;
        }

	char *version_string = &versionBuffer[0];
	char *build_date_string = &versionBuffer[DSPAL_MAX_LEN_VERSION_INFO_STR];
	char *build_time_string = &versionBuffer[DSPAL_MAX_LEN_VERSION_INFO_STR*2];

	status = version_test_get_version_info(
		version_string, DSPAL_MAX_LEN_VERSION_INFO_STR,
		build_date_string, DSPAL_MAX_LEN_VERSION_INFO_STR,
		build_time_string, DSPAL_MAX_LEN_VERSION_INFO_STR);

	if (status != 0) {
		LOG_INFO("Failed to get DSP image version information.");
	}
	else {
		LOG_INFO("version: %s", version_string);
		LOG_INFO("build date: %s", build_date_string);
		LOG_INFO("build time: %s", build_time_string);
	}

	if (versionBuffer != NULL) {
                rpcmem_free(versionBuffer);
                versionBuffer = 0;
        }

	return status;
}

