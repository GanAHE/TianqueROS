
#pragma once

#include <inttypes.h>
#include <string.h>

#define ATOMIC_ENTER lock()
#define ATOMIC_LEAVE unlock()

namespace cdev
{

struct file_operations {
	void *op;
};

using px4_file_operations_t = struct file_operations;
using mode_t = uint32_t;

struct file_t {
	int f_oflags{0};
	void *f_priv{nullptr};
	void *vdev{nullptr};

	file_t() = default;
	file_t(int f, void *c) : f_oflags(f), vdev(c) {}
};

} // namespace cdev

extern "C" __EXPORT int register_driver(const char *name, const cdev::px4_file_operations_t *fops,
					cdev::mode_t mode, void *data);
extern "C" __EXPORT int unregister_driver(const char *path);
