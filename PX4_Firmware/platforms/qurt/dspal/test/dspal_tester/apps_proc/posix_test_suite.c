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

#include <stdio.h>
#include <string.h>

#include "test_utils.h"
#include "dspal_tester.h"
#include <rpcmem.h>

int run_rpcmem_test()
{
    int test_result = TEST_PASS;
    unsigned char test_string[] = "12345"; 
    unsigned char upd_string[sizeof(test_string)];
    int data_len = sizeof(test_string); 
    int i = 0; 

    rpcmem_init();
    unsigned char* data = (unsigned char*)rpcmem_alloc(22, RPCMEM_FLAG_UNCACHED, data_len);
    
    if (!data) {
        LOG_INFO("tests failed - cannot allocate rpc mem");
        test_result = TEST_FAIL;
        return test_result;
    }
  
    memcpy(data, test_string, data_len);
    dspal_tester_test_rpcmem(data, data_len);

    for (i = 0; i < data_len; i++) {
        upd_string[i] = test_string[i]+1; 
    }

    if ( memcmp(data, upd_string, data_len) != 0 ){
        LOG_INFO("tests failed - not matching expected string");
        test_result = TEST_FAIL;
    }
    
    rpcmem_free(data); 
    return test_result;
}


int run_HAP_power_test()
{
    int test_result = TEST_PASS;
    int len = 5 * 1024;
    rpcmem_init();
    unsigned char*  data = (unsigned char*)rpcmem_alloc(0, RPCMEM_HEAP_DEFAULT, sizeof(int) * len);
    dspal_tester_test_hap_power(data, len);
    rpcmem_free(data); 
    rpcmem_deinit();

    return test_result;
}
int run_pthreads_test_suite()
{
	int test_results = TEST_PASS;

	LOG_INFO("testing malloc size");
    test_results |= display_test_results( dspal_tester_test_malloc(), "malloc size test");
    
    LOG_INFO("HAP power API test");
    test_results |= display_test_results( run_HAP_power_test(), "HAP power API test"); 

	LOG_INFO("testing pthread.h");

	test_results |= display_test_results( dspal_tester_test_pthread_attr_init(), "pthread attr init");
	test_results |= display_test_results( dspal_tester_test_pthread_create(), "pthread create");
	test_results |= display_test_results( dspal_tester_test_pthread_cancel(), "pthread cancel");
	test_results |= display_test_results( dspal_tester_test_pthread_self(), "pthread self");
	test_results |= display_test_results( dspal_tester_test_pthread_exit(), "pthread exit");
	test_results |= display_test_results( dspal_tester_test_pthread_kill(), "pthread kill");
	test_results |= display_test_results( dspal_tester_test_pthread_cond_timedwait(), "pthread condition timed wait");
	test_results |= display_test_results( dspal_tester_test_pthread_mutex_lock(), "pthread mutex lock");
	test_results |= display_test_results( dspal_tester_test_pthread_mutex_lock_thread(), "thread mutex lock thread");
	test_results |= display_test_results( dspal_tester_test_pthread_stack(), "thread large allocation on stack");
	test_results |= display_test_results( dspal_tester_test_pthread_heap(), "thread large allocation on heap");
	test_results |= display_test_results( dspal_tester_test_usleep(), "usleep for two seconds");

	LOG_INFO("testing semaphore.h");

	test_results |= display_test_results( dspal_tester_test_semaphore_wait(), "semaphore wait");

	LOG_INFO("testing C++");

	test_results |= display_test_results( dspal_tester_test_cxx_heap(), "test C++ heap");
	test_results |= display_test_results( dspal_tester_test_cxx_static(), "test C++ static initialization");

    LOG_INFO("testing rpcmem");
    test_results |= display_test_results( run_rpcmem_test(), "RPC memory test"); 

    
	LOG_INFO("pthread tests complete");

	return test_results;
}

int run_timers_test_suite()
{
    int test_results = TEST_PASS;
	LOG_INFO("testing time.h");

	test_results |= display_test_results( dspal_tester_test_clockid(), "clockid values exist");
	test_results |= display_test_results( dspal_tester_test_sigevent(), "sigevent values exist");
	test_results |= display_test_results( dspal_tester_test_time(), "time returns good value");
	test_results |= display_test_results( dspal_tester_test_timer_realtime_sig_none(), "timer realtime");
	test_results |= display_test_results( dspal_tester_test_timer_monotonic_sig_none(), "timer monotonic");
	test_results |= display_test_results( dspal_tester_test_timer_process_cputime_sig_none(), "timer process cputime");
	test_results |= display_test_results( dspal_tester_test_timer_thread_cputime_sig_none(), "timer thread cputime");
	test_results |= display_test_results( dspal_tester_test_time_return_value(), "time return value");
	test_results |= display_test_results( dspal_tester_test_time_param(), "time parameter");
	test_results |= display_test_results( dspal_tester_test_usleep(), "usleep for two seconds");
	test_results |= display_test_results( dspal_tester_test_clock_getres(), "clock_getres");
	test_results |= display_test_results( dspal_tester_test_clock_gettime(), "clock_gettime");
	test_results |= display_test_results( dspal_tester_test_clock_settime(), "clock_settime");
	test_results |= display_test_results( dspal_tester_test_one_shot_timer_cb(), "one shot timer cb");
	test_results |= display_test_results( dspal_tester_test_periodic_timer_cb(), "periodic timer cb");
	test_results |= display_test_results( dspal_tester_test_periodic_timer_signal_cb(), "periodic timer signal cb");
	test_results |= display_test_results( dspal_tester_test_periodic_timer_sigwait(), "periodic timer sigwait");

	LOG_INFO("timer tests complete");
    
	return test_results;
}
