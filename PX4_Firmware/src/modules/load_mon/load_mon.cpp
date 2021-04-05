/****************************************************************************
 *
 *   Copyright (c) 2012-2018 PX4 Development Team. All rights reserved.
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
 * @file load_mon.cpp
 *
 * @author Jonathan Challinger <jonathan@3drobotics.com>
 * @author Julian Oes <julian@oes.ch
 * @author Andreas Antener <andreas@uaventure.com>
 */

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <systemlib/cpuload.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationQueued.hpp>
#include <uORB/topics/cpuload.h>
#include <uORB/topics/task_stack_info.h>

#if defined(__PX4_NUTTX) && !defined(CONFIG_SCHED_INSTRUMENTATION)
#  error load_mon support requires CONFIG_SCHED_INSTRUMENTATION
#endif

#define STACK_LOW_WARNING_THRESHOLD 300 ///< if free stack space falls below this, print a warning
#define FDS_LOW_WARNING_THRESHOLD 3 ///< if free file descriptors fall below this, print a warning

namespace load_mon
{

extern "C" __EXPORT int load_mon_main(int argc, char *argv[]);

// Run it at 1 Hz.
const unsigned LOAD_MON_INTERVAL_US = 1000000;

class LoadMon : public ModuleBase<LoadMon>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	LoadMon();
	~LoadMon() override;

	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[])
	{
		return print_usage("unknown command");
	}

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	void start();

private:
	/** Do a compute and schedule the next cycle. */
	void Run() override;

	/** Do a calculation of the CPU load and publish it. */
	void _cpuload();

	/** Calculate the memory usage */
	float _ram_used();

#ifdef __PX4_NUTTX
	/* Calculate stack usage */
	void _stack_usage();

	int _stack_task_index{0};
	uORB::PublicationQueued<task_stack_info_s> _task_stack_info_pub{ORB_ID(task_stack_info)};
#endif

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::SYS_STCK_EN>) _param_sys_stck_en
	)

	uORB::Publication<cpuload_s>  _cpuload_pub{ORB_ID(cpuload)};

	hrt_abstime _last_idle_time{0};
	hrt_abstime _last_idle_time_sample{0};

	perf_counter_t _stack_perf;
};

LoadMon::LoadMon() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default),
	_stack_perf(perf_alloc(PC_ELAPSED, "stack_check"))
{
}

LoadMon::~LoadMon()
{
	ScheduleClear();

	perf_free(_stack_perf);
}

int LoadMon::task_spawn(int argc, char *argv[])
{
	LoadMon *obj = new LoadMon();

	if (!obj) {
		PX4_ERR("alloc failed");
		return -1;
	}

	_object.store(obj);
	_task_id = task_id_is_work_queue;

	/* Schedule a cycle to start things. */
	obj->start();

	return 0;
}

void
LoadMon::start()
{
	ScheduleOnInterval(LOAD_MON_INTERVAL_US);
}

void LoadMon::Run()
{
	_cpuload();

#ifdef __PX4_NUTTX

	if (_param_sys_stck_en.get()) {
		_stack_usage();
	}

#endif

	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
	}
}

void LoadMon::_cpuload()
{
	if (_last_idle_time == 0) {
		/* Just get the time in the first iteration */
		_last_idle_time = system_load.tasks[0].total_runtime;
		return;
	}

	/* compute system load */
	const hrt_abstime total_runtime = system_load.tasks[0].total_runtime;
	const hrt_abstime interval = hrt_elapsed_time(&_last_idle_time_sample);
	const hrt_abstime interval_idletime = total_runtime - _last_idle_time;

	_last_idle_time = total_runtime;
	_last_idle_time_sample = hrt_absolute_time();

	cpuload_s cpuload{};
	cpuload.load = 1.0f - (float)interval_idletime / (float)interval;
	cpuload.ram_usage = _ram_used();
	cpuload.timestamp = hrt_absolute_time();

	_cpuload_pub.publish(cpuload);
}

float LoadMon::_ram_used()
{
#ifdef __PX4_NUTTX
	struct mallinfo mem;

#ifdef CONFIG_CAN_PASS_STRUCTS
	mem = mallinfo();
#else
	(void)mallinfo(&mem);
#endif /* CONFIG_CAN_PASS_STRUCTS */

	// mem.arena: total ram (bytes)
	// mem.uordblks: used (bytes)
	// mem.fordblks: free (bytes)
	// mem.mxordblk: largest remaining block (bytes)

	return (float)mem.uordblks / mem.arena;

#else
	return 0.0f;
#endif
}

#ifdef __PX4_NUTTX
void LoadMon::_stack_usage()
{
	int task_index = 0;

	/* Scan maximum num_tasks_per_cycle tasks to reduce load. */
	const int num_tasks_per_cycle = 2;

	for (int i = _stack_task_index; i < _stack_task_index + num_tasks_per_cycle; i++) {
		task_index = i % CONFIG_MAX_TASKS;
		unsigned stack_free = 0;
		unsigned fds_free = FDS_LOW_WARNING_THRESHOLD + 1;
		bool checked_task = false;

		perf_begin(_stack_perf);
		sched_lock();

		task_stack_info_s task_stack_info = {};

		if (system_load.tasks[task_index].valid && (system_load.tasks[task_index].tcb->pid > 0)) {

			stack_free = up_check_tcbstack_remain(system_load.tasks[task_index].tcb);

			static_assert(sizeof(task_stack_info.task_name) == CONFIG_TASK_NAME_SIZE,
				      "task_stack_info.task_name must match NuttX CONFIG_TASK_NAME_SIZE");
			strncpy((char *)task_stack_info.task_name, system_load.tasks[task_index].tcb->name, CONFIG_TASK_NAME_SIZE - 1);
			task_stack_info.task_name[CONFIG_TASK_NAME_SIZE - 1] = '\0';

#if CONFIG_NFILE_DESCRIPTORS > 0
			FAR struct task_group_s *group = system_load.tasks[task_index].tcb->group;

			unsigned tcb_num_used_fds = 0;

			if (group) {
				for (int fd_index = 0; fd_index < CONFIG_NFILE_DESCRIPTORS; ++fd_index) {
					if (group->tg_filelist.fl_files[fd_index].f_inode) {
						++tcb_num_used_fds;
					}
				}

				fds_free = CONFIG_NFILE_DESCRIPTORS - tcb_num_used_fds;
			}

#endif // CONFIG_NFILE_DESCRIPTORS

			checked_task = true;
		}

		sched_unlock();
		perf_end(_stack_perf);

		if (checked_task) {

			task_stack_info.stack_free = stack_free;
			task_stack_info.timestamp = hrt_absolute_time();

			_task_stack_info_pub.publish(task_stack_info);

			/*
			 * Found task low on stack, report and exit. Continue here in next cycle.
			 */
			if (stack_free < STACK_LOW_WARNING_THRESHOLD) {
				PX4_WARN("%s low on stack! (%i bytes left)", task_stack_info.task_name, stack_free);
				break;
			}

			/*
			 * Found task low on file descriptors, report and exit. Continue here in next cycle.
			 */
			if (fds_free < FDS_LOW_WARNING_THRESHOLD) {
				PX4_WARN("%s low on FDs! (%i FDs left)", task_stack_info.task_name, fds_free);
				break;
			}

		} else {
			/* No task here, check one more task in same cycle. */
			_stack_task_index++;
		}
	}

	/* Continue after last checked task next cycle. */
	_stack_task_index = task_index + 1;
}
#endif

int LoadMon::print_usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Background process running periodically with 1 Hz on the LP work queue to calculate the CPU load and RAM
usage and publish the `cpuload` topic.

On NuttX it also checks the stack usage of each process and if it falls below 300 bytes, a warning is output,
which will also appear in the log file.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("load_mon", "system");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the background task");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}


int load_mon_main(int argc, char *argv[])
{
	return LoadMon::main(argc, argv);
}

} // namespace load_mon
