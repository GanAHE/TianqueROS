/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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

/**
 * @file CDev.cpp
 *
 * Character device base class.
 */

#include "CDev.hpp"

#include <cstring>

#include <px4_platform_common/posix.h>
#include <drivers/drv_device.h>

namespace device
{

CDev::CDev(const char *name, const char *devname) :
	Device(name),
	cdev::CDev(devname)
{
}

int CDev::init()
{
	PX4_DEBUG("CDev::init");

	// base class init first
	int ret = Device::init();

	if (ret != PX4_OK) {
		goto out;
	}

	// now register the driver
	if (get_devname() != nullptr) {
		ret = cdev::CDev::init();

		if (ret != PX4_OK) {
			goto out;
		}
	}

out:
	return ret;
}

int CDev::ioctl(file_t *filep, int cmd, unsigned long arg)
{
	PX4_DEBUG("CDev::ioctl");
	int ret = -ENOTTY;

	switch (cmd) {
	case DEVIOCGDEVICEID:
		ret = (int)_device_id.devid;
		break;

	default:
		break;
	}

	return ret;
}

} // namespace device
