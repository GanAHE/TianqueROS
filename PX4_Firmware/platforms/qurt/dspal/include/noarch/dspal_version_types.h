/****************************************************************************
 * Copyright (c) 2016 James Wilson. All rights reserved.
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

#pragma once

/**
 * @file
 * API to retrieve the current version of the DSPAL API.
 */

/**
 * The maximum length of each version info string returned by the function
 * get_dspal_version_info().  The output parameters passed to get_dspal_version_info
 * should reference a character string of at least this length.
 */
#define DSPAL_MAX_LEN_VERSION_INFO_STR 40
#define DSPAL_VERSION_INFO_OVERFLOW "####"

/**
 * @brief
 * Structure used for retrieving the version information
 *
 * @par version_string
 * The version information of the DSPAL implementation currently in use, formatted as a key/value pair:
 * DSPAL_VERSION_STRING=[major.minor.update.build_number]. May be left NULL.
 * e.g.: DSPAL_VERSION_STRING=DSPAL-1.0.1.4242
 *
 * @par build_date
 * The build date of the DSPAL implementation currently in use, formatted as a key/value pair:
 * DSPAL_DATE_STRING=[Mmm dd yyyy]. May be left NULL.
 * e.g.: DSPAL_DATE_STRING=Jan 01 2016
 *
 * @par build_time
 * The build time of the DSPAL implementation currently in use, formatted as a key/value pair:
 * BUILD_TIME_STRING=[hh:mm:ss]. May be left NULL.
 * e.g.: BUILD_TIME_STRING=[23:59:01]
 */
struct dspal_version_info {
	char version_string[DSPAL_MAX_LEN_VERSION_INFO_STR];
	char build_date[DSPAL_MAX_LEN_VERSION_INFO_STR];
	char build_time[DSPAL_MAX_LEN_VERSION_INFO_STR];
};

