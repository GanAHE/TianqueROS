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

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <dspal_signal.h>
#include <dspal_platform.h>
#include <pthread.h>

#include "test_utils.h"
#include "dspal_tester.h"

#define SKIP_PTHREAD_KILL
#define DSPAL_TESTER_COND_WAIT_TIMEOUT_IN_SECS 3


/**
 * @brief Test if pthread_attr_init and pthread_attr_destroy work.
 * Errors printed to log.
 *
 * @par Test:
 * 1) Try pthread_attr_init
 * 2) Try pthread_attr_destroy
 *
 * @return
 * TEST_PASS ------ Always
*/
int dspal_tester_test_pthread_attr_init(void)
{
	int rv = 0;
	pthread_attr_t attr;

	rv = pthread_attr_init(&attr);

	if (rv != 0) { FAIL("pthread_attr_init returned error"); }

	rv = pthread_attr_destroy(&attr);

	if (rv != 0) { FAIL("pthread_attr_destroy returned error"); }

	return TEST_PASS;
}

/**
 * @brief Helper function for the 'dspal_tester_test_pthread_create' test.
 *
 * @par Takes in an integer and changes its value to a 1 to show that this thread
 * did actually run
 *
 * @param   test_value[out]   pointer to test value (int) to change to 1
 *
 * @return
 * NULL ------ Always
*/
void *test_pthread_create_helper(void *test_value)
{
	int *v = (int *)test_value;
	(*v) = 1;

	return NULL;
}

/**
 * @brief Test if pthread_create and pthread_join work.
 * Errors printed to log.
 *
 * @par Creates a tread (runs test_pthread_create_helper) and checks
 * to see if a test value has changed.  The test value is changed by the
 * thread that is created.  It is passed into the thread during creation via
 * a pointer
 *
 * @par Test:
 * 1) Create a thread that runs the 'test_pthread_create_helper' function
 *    In creation a test variable integer is passed to the thread
 * 2) Joins the newly created thread (aka waits for thread completion)
 * 3) Checks if the test variable has changed value.
 *
 * @return
 * TEST_PASS ------ Always
*/
int dspal_tester_test_pthread_create(void)
{
	int rv = 0;

	int test_value = 0;
	pthread_t thread;

	rv = pthread_create(&thread, NULL, test_pthread_create_helper, &test_value);

	if (rv != 0) { FAIL("thread_create returned error"); }

	rv = pthread_join(thread, NULL);

	if (rv != 0) { FAIL("thread_join returned error"); }

	if (test_value == 0) { FAIL("test value did not change"); }

	return TEST_PASS;
}

void *test_pthread_cancel_helper(void *unused)
{
	while (TRUE) {
		//pause(); TODO: find cancelable method
	}

	return NULL;
}

int dspal_tester_test_pthread_cancel(void)
{
	return TEST_SKIP;

#if 0 // I need to find a cancelable method in order to test this
	int rv = 0;

	int test_value = 0;
	pthread_t thread;

	rv = pthread_create(&thread, NULL, test_pthread_cancel_helper, &test_value);

	if (rv != 0) { FAIL("thread_create returned error"); }

	rv = pthread_cancel(thread);

	if (rv != 0) { FAIL("thread_join returned error"); }

	rv = pthread_join(thread, NULL);

	if (rv != 0) { FAIL("thread_join returned error"); }

	return TEST_PASS;
#endif
}

/**
 * @brief Helper function for the 'dspal_tester_test_pthread_self' test.
 *
 * @par Assigns the parameter variable to be an instance of itself
 *
 * @param   thread_self[out]   pointer to a pthread_t that should be assigned to self instance
 *
 * @return
 * NULL ------ Always
*/
void *test_pthread_self_helper(void *thread_self)
{
	pthread_t *v = (pthread_t *)thread_self;
	(*v) = pthread_self();

	return NULL;
}

/**
 * @brief Test if pthread_self works.
 * Errors printed to log.
 *
 * @par Creates a tread (runs test_pthread_self_helper) and passes a pthread_t
 * variable in.  Inside the newly created thread, the thread assigned the
 * passed in pthread_t variable to be 'pthread_self()' (aka the instance of itself).
 * After the thread is done executing, the main thread checks to see if the thread
 * instance it created and the thread instance returned by the helper thread ('pthread_self()')
 * match.  They should match because they are the same thread instance.
 *
 * @par Test:
 * 1) Create a thread that runs the 'test_pthread_self_helper' function
 *    In creation a pthread_t is passed in as a parameter
 * 2) Joins the newly created thread (aka waits for thread completion)
 * 3) Checks if pthread_t variable passed in matches the created thread value
 *
 * @return
 * TEST_PASS ------ Always
*/
int dspal_tester_test_pthread_self(void)
{
	int rv = 0;

	pthread_t thread;
	pthread_t thread_self;

	rv = pthread_create(&thread, NULL, test_pthread_self_helper, &thread_self);

	if (rv != 0) { FAIL("thread_create returned error"); }

	rv = pthread_join(thread, NULL);

	if (rv != 0) { FAIL("thread_join returned error"); }

	if (thread_self != thread) { FAIL("pthread_self did not return the expected value"); }

	return TEST_PASS;
}

/**
 * @brief Helper function for the 'dspal_tester_test_pthread_exit' test.
 *
 * @par Exits the thread.  Should not execute code that is after the thread exit
 *
 * @param   test_value[out]   pointer to test value (int)
 *
 * @return
 * NULL ------ Always
*/
void *test_pthread_exit_helper(void *test_value)
{
	pthread_exit(NULL);

	int *v = (int *)test_value;
	(*v) = 1;

	return NULL;
}

/**
 * @brief Test if pthread_exit works.
 * Errors printed to log.
 *
 * @par Creates a thread and passes in a mutable test variable into the thread on creation.
 * As that created thread executes, it will execute a 'pthread_exit' call.  Everything after
 * this call should not run.  the code after this call include changing the value of the test
 * variable.  After this created thread has exited, the value of the test variable is checked
 * to ensure that it has not changed (the code after the 'pthread_exit' was not run because
 * the thread exited correctly).
 *
 *
 * @par Test:
 * 1) Create a thread that runs the 'test_pthread_exit_helper' function
 *    In creation a test variable (int) is passed in as a parameter
 * 2) Joins the newly created thread (aka waits for thread completion)
 * 3) Checks if the value of the test variable has changed (should not change)
 *
 * @return
 * TEST_PASS ------ Always
*/
int dspal_tester_test_pthread_exit(void)
{
	int rv = 0;

	int test_value = 0;

	pthread_t thread;

	rv = pthread_create(&thread, NULL, test_pthread_exit_helper, &test_value);

	if (rv != 0) { FAIL("thread_create returned error"); }

	rv = pthread_join(thread, NULL);

	if (rv != 0) { FAIL("thread_join returned error"); }

	if (test_value != 0) { FAIL("test value should not have changed"); }

	return TEST_PASS;
}

int last_signal;

/**
 * @brief Signal handler for the 'dspal_tester_test_pthread_kill' test.
 *
 * @par Sets 'last_signal' variable to the signal number.
 *
 * @param   signo[in]   signal number
*/
void test_pthread_kill_sig_handler(int signo)
{
	last_signal = signo;
	return;
}

/**
 * @brief Helper function for the 'dspal_tester_test_pthread_kill' test.
 *
 * @par Runs the thread until the 'last_signal' variable is set (exit flag)
 * or for a max amount of time (in case signal never arrives)
 *
 * @param   unused[in]   unused variable
 *
 * @return
 * NULL ------ Always
*/
void *test_pthread_kill_helper(void *unused)
{
	const int MAX_RUN_TIME = 2; // in seconds

	time_t stop_time = time(NULL) + MAX_RUN_TIME;

	while (last_signal == 0 && time(NULL) < stop_time);

	return NULL;
}

/**
 * @brief Test if pthread_exit works.
 * Errors printed to log.
 *
 * @par Creates a thread and passes in a mutable test variable into the thread on creation.
 * As that created thread executes, it will execute a 'pthread_exit' call.  Everything after
 * this call should not run.  the code after this call include changing the value of the test
 * variable.  After this created thread has exited, the value of the test variable is checked
 * to ensure that it has not changed (the code after the 'pthread_exit' was not run because
 * the thread exited correctly).
 *
 *
 * @par Test:
 * 1) Setup signal handler to be 'test_pthread_kill_sig_handler'
 * 2) Create a thread that runs the 'test_pthread_kill_helper' function
 * 3) Send a signal to the thread using 'pthread_kill'
 *    This should kill the newly created thread, checks signal as exit flag
 * 4) Joins the newly created thread (aka waits for thread completion)
 * 5) Checks to make sure that 'last_signal' was set to the correct value
 *
 * @return
 * TEST_PASS ------ Always
*/
int dspal_tester_test_pthread_kill(void)
{
#ifdef SKIP_PTHREAD_KILL
	return TEST_SKIP;
#else

	int rv = 0;

	int last_signal = 0;

	struct sigaction actions;

	pthread_t thread;

	memset(&actions, 0, sizeof(actions));
	sigemptyset(&actions.sa_mask);
	actions.sa_flags = 0;
	actions.sa_handler = test_pthread_kill_sig_handler;

	rv = sigaction(SIGALRM, &actions, NULL);

	if (rv != 0) { FAIL("sigaction returned error"); }

	rv = pthread_create(&thread, NULL, test_pthread_kill_helper, NULL);

	if (rv != 0) { FAIL("thread_create returned error"); }

	rv = pthread_kill(thread, SIGALRM);

	if (rv != 0) { FAIL("pthread_kill returned error"); }

	rv = pthread_join(thread, NULL);

	if (rv != 0) { FAIL("thread_join returned error"); }

	if (last_signal != SIGALRM) { FAIL("last signal is not SIGALRM"); }

	return TEST_PASS;

#endif
}

/**
 * @brief Test functionality of pthread mutex using 1 thread
 * Errors printed to log.
 *
 * @par Does locking and unlocking of a mutex in a single thread to ensure
 * that the mutex behaves correctly.
 *
 * @par Test:
 * 1) Create the mutex lock and init it
 * 2) Attempt to lock the mutex (should lock)
 * 3) Attempt to unlock the mutex (should unlock, locked in step 2)
 * 4) Attempt to destroy the mutex
 *
 * @return
 * TEST_PASS ------ Always
*/
int dspal_tester_test_pthread_mutex_lock(void)
{
	int rv = 0;
	pthread_mutex_t lock;

	rv = pthread_mutex_init(&lock, NULL);

	if (rv != 0) { FAIL("pthread_mutex_init returned error"); }

	pthread_mutex_lock(&lock);

	if (rv != 0) { FAIL("pthread_mutex_lock returned error"); }

	rv = pthread_mutex_unlock(&lock);

	if (rv != 0) { FAIL("pthread_mutex_unlock returned error"); }

	rv = pthread_mutex_destroy(&lock);

	if (rv != 0) { FAIL("pthread_mutex_destroy returned error"); }

	return TEST_PASS;
}

typedef struct {
	int *int_value;
	pthread_mutex_t *mutex;
} test_mutex_t;

/**
 * @brief Helper function for the 'dspal_tester_test_pthread_mutex_lock_thread' test.
 *
 * @par Mutates the test variable then locks the mutex.  Mutex will not lock at first
 * since the main thread locked the mutex.  After the main thread unlocks the mutex
 * change the value of the test value again.
 *
 *
 * @param   test_value[in]   test variable (int) whos value should be mutated
 *
 * @return
 * NULL ------ Always
*/
void *test_pthread_mutex_lock_thread_helper(void *test_value)
{
	int rv = 0;
	test_mutex_t *v = (test_mutex_t *)test_value;
	*(v->int_value) = 1;

	pthread_mutex_lock(v->mutex);

	*(v->int_value) = 2;

	rv = pthread_mutex_unlock(v->mutex);

	return NULL;
}

/**
 * @brief Test functionality of pthread mutex using multiple threads.
 * Errors printed to log.
 *
 * @par Creates a mutex and locks it. A thread is then created and mutates the data slightly.
 * The created thread then tries to lock the mutex but cant because it is already locks, it
 * therefore blocks until the mutex is released.  The main thread then checks the data to make
 * sure the created thread only ran the code before it tried to lock the mutex.  The main thread
 * then unlocks the mutex and joins the created thread to the main thread.  It then checks the value
 * of the data to make sure the created thread ran the rest of the code
 *
 * @par Test:
 * 1) Create the mutex lock and init it
 * 2) Lock the mutex (should lock)
 * 3) Create a thread using the 'test_pthread_mutex_lock_thread_helper' function
 * 4) Wait for that thread to mutate the test_value integer or until timeout
 * 5) Unlock the mutex
 * 6) Join the created thread
 * 7) Wait for that thread to mutate the test_value integer or until timeout
 * 8) Destroy the mutex
 *
 * @return
 * TEST_PASS ------ Always
*/
int dspal_tester_test_pthread_mutex_lock_thread(void)
{
	const int MAX_RUN_TIME = 2; // in seconds

	int rv = 0;
	int test_value = 0;

	pthread_mutex_t lock;
	pthread_t thread;

	test_mutex_t test_mutex;
	test_mutex.int_value = &test_value;
	test_mutex.mutex = &lock;

	rv = pthread_mutex_init(&lock, NULL);

	if (rv != 0) { FAIL("pthread_mutex_init returned error"); }

	pthread_mutex_lock(&lock);

	if (rv != 0) { FAIL("pthread_mutex_lock returned error"); }

	rv = pthread_create(&thread, NULL, test_pthread_mutex_lock_thread_helper, &test_mutex);

	if (rv != 0) { FAIL("thread_create returned error"); }

	time_t stop_time = time(NULL) + MAX_RUN_TIME;

	while (test_value != 1 && time(NULL) < stop_time);

	if (test_value != 1) { FAIL("test value did not change"); }

	rv = pthread_mutex_unlock(&lock);

	if (rv != 0) { FAIL("pthread_mutex_unlock returned error"); }

	rv = pthread_join(thread, NULL);

	if (rv != 0) { FAIL("thread_join returned error"); }

	stop_time = time(NULL) + MAX_RUN_TIME;

	while (test_value != 2 && time(NULL) < stop_time);

	if (test_value != 2) { FAIL("test value did not change"); }

	rv = pthread_mutex_destroy(&lock);

	if (rv != 0) { FAIL("pthread_mutex_destroy returned error"); }

	return TEST_PASS;
}

/**
 * @brief Helper function for the 'dspal_tester_test_pthread_stack' test.
 *
 * @par Attempts to allocate memory from the stack.  Checks the stack memory.
 * Sets 'test_value' to 1 if memory allocation worked.  -1 if it didn't.
 *
 * @param   test_value[out]   pointer to test value (int)
 *
 * @return
 * NULL ------ Always
*/
void *test_pthread_stack_helper(void *test_value)
{
	// allocate mem on the stack and see if it crashes

	int *v = (int *)test_value;
	(*v) = -1;

	const int SIZE = 4 * 1024 - 256;

	uint8 mem_on_stack[SIZE];

	for (int i = 0; i < SIZE; i += 1) {
		mem_on_stack[i] = (uint8)i;
	}

	for (int i = 0; i < SIZE; i += 1) {
		if (mem_on_stack[i] != (uint8)i) {
			(*v) = -2;
		}
	}

	(*v) = 1;

	return NULL;
}

/**
 * @brief Test functionality of allocating memory on the stack
 *
 * @par This test launches a thread and tests if it is able to allocate memory on the stack.
 * This is done by creating a thread and changing its stack size.  During creation a mutable
 * variable is passed in.  The created thread sets the variable to -1 and tries to allocate
 * stack memory.  If this fails the thread exits. If not then the created thread sets the
 * mutable variable to 1 and exits.  The main thread checks the value of this mutable variable
 * to see if the stack memory allocation worked
 *
 * @par Test:
 * 1) Set thread attributes
 * 2) Set the thread stack size
 * 3) Launch a new thread while passing in a mutable variable
 * 4) Join the thread
 *
 * In the created thread
 * 5) Set the mutable variable to -1
 * 6) Allocate stack memory and set its value
 * 7) Check the stack memory values
 * 8) set mutable variable to 1
 *
 * @return
 * TEST_PASS ------ Always
*/
int dspal_tester_test_pthread_stack(void)
{
	int rv = 0;

	int test_value = 0;
	pthread_t thread;
	pthread_attr_t attr;
	size_t stacksize = 4 * 1024;

	rv = pthread_attr_init(&attr);

	if (rv != 0) { FAIL("pthread_attr_init returned error"); }

	rv = pthread_attr_setstacksize(&attr, stacksize);

	if (rv != 0) { FAIL("pthread_attr_setstacksize returned error"); }

	rv = pthread_create(&thread, &attr, test_pthread_stack_helper, &test_value);

	if (rv != 0) { FAIL("thread_create returned error"); }

	rv = pthread_join(thread, NULL);

	if (rv != 0) { FAIL("thread_join returned error"); }

	if (test_value != 1) {
		LOG_ERR("test value: %d", test_value);
		FAIL("test value is not passing");
	}

	return TEST_PASS;
}

/**
 * @brief Helper function for the 'dspal_tester_test_pthread_heap' test.
 *
 * @par Attempts to allocate memory from the heap.  Checks the heap memory.
 * Sets 'test_value' to 1 if memory allocation worked.  -1 if it didn't.
 *
 * @param   test_value[out]   pointer to test value (int)
 *
 * @return
 * NULL ------ Always
*/
void *test_pthread_heap_helper(void *test_value)
{
	// allocate mem on the stack and see if it crashes

	int *v = (int *)test_value;
	(*v) = -1;

	const int SIZE = 1024;

	uint8 *mem_on_heap1 = (uint8 *)malloc(SIZE * sizeof(uint8));
	uint8 *mem_on_heap2 = (uint8 *)malloc(SIZE * sizeof(uint8));

	for (int i = 0; i < SIZE; i += 1) {
		mem_on_heap1[i] = (uint8)i;
		mem_on_heap2[i] = (uint8)i;
	}

	for (int i = 0; i < SIZE; i += 1) {
		if (mem_on_heap1[i] != (uint8)i || mem_on_heap2[i] != (uint8)i) {
			(*v) = -2;
		}
	}

	free(mem_on_heap1);
	free(mem_on_heap2);

	(*v) = 1;

	return NULL;
}

/**
 * @brief Test functionality of allocating memory on the heap
 *
 * @par This test launches a thread and tests if it is able to allocate memory on the stack.
 * This is done by creating a thread.  During creation a mutable
 * variable is passed in.  The created thread sets the variable to -1 and tries to allocate
 * heap memory.  If this fails the thread exits. If not then the created thread sets the
 * mutable variable to 1 and exits.  The main thread checks the value of this mutable variable
 * to see if the heap memory allocation worked
 *
 * @par Test:
 * 1) Set thread attributes
 * 2) Launch a new thread while passing in a mutable variable
 * 3) Join the thread
 *
 * In the created thread
 * 4) Set the mutable variable to -1
 * 5) Allocate heap memory and set its value
 * 6) Check the heap memory values
 * 7) set mutable variable to 1
 *
 * @return
 * TEST_PASS ------ Always
*/
int dspal_tester_test_pthread_heap(void)
{
	int rv = 0;

	int test_value = 0;
	pthread_t thread;

	rv = pthread_create(&thread, NULL, test_pthread_heap_helper, &test_value);

	if (rv != 0) { FAIL("thread_create returned error"); }

	rv = pthread_join(thread, NULL);

	if (rv != 0) { FAIL("thread_join returned error"); }

	if (test_value != 1) {
		LOG_ERR("test value: %d", test_value);
		FAIL("test value is not passing");
	}

	return TEST_PASS;
}

/**
 * @brief Helper thread which blocks on a condition variable
 *
 * @par See the documentation on the test for the pthread_cond_timedwait() function.
 *
 * @return
 * error code
 */
void *test_pthread_cond_timedwait_helper(void *test_value)
{
	pthread_mutex_t mutex;
	pthread_mutexattr_t attr;
	pthread_cond_t *cond = (pthread_cond_t *)test_value;
	int mutex_lock_status;
	struct timespec timeout;
	struct timespec now;
	int cond_status, return_status = 0;

	/*
	 * Initialize and lock the mutex used to prevent race conditions when accessing
	 * the condition from multiple threads.  Explicitly set the mutex type to normal
	 * to prevent the use of the default recursive mutex.
	 */
	if (pthread_mutexattr_init(&attr) != 0 ||
	    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_NORMAL) != 0 ||
	    pthread_mutex_init(&mutex, &attr) != 0) {
		LOG_ERR("error: pthread mutex initialization failed");
		return_status = -1;
		goto exit;
	}

	/*
	 * Obtain the current time and add the specified timeout, since
	 * the timeout must be specified as absolute time.
	 */
	clock_gettime(CLOCK_REALTIME, &now);
	timeout.tv_sec = now.tv_sec + DSPAL_TESTER_COND_WAIT_TIMEOUT_IN_SECS;
	timeout.tv_nsec = now.tv_nsec;

	/*
	 * Initialize the parameters to cause a new mutex and cond object
	 * to be instantiated.
	 */
	if ((mutex_lock_status = pthread_mutex_trylock(&mutex)) != 0) {
		LOG_ERR("error: pthread_mutex_trylock indicates that the lock is already established: %d", mutex_lock_status);
	}

	*cond = PTHREAD_COND_INITIALIZER;

	/*
	 * Wait once, long enough for a timeout to occur.
	 */
	LOG_DEBUG("entering first pthread_cond_timedwait call, testing timeout");
	cond_status = pthread_cond_timedwait(cond, &mutex, &timeout);

	if (cond_status != ETIMEDOUT) {
		LOG_ERR("error: pthread_cond_timewait did not time out as expected, cond_status: %d, ETIMEDOUT: %d",
			cond_status, ETIMEDOUT);
		return_status = -1;

	} else {
		LOG_DEBUG("first pthread_cond_timedwait call timed out as expected.");
	}

	/*
	 * Update the time to the new absolute time for the next timeout.
	 */
	clock_gettime(CLOCK_REALTIME, &now);
	timeout.tv_sec = now.tv_sec + DSPAL_TESTER_COND_WAIT_TIMEOUT_IN_SECS;
	timeout.tv_nsec = now.tv_nsec;

	/*
	 * Wait again, but this time expect to be signaled before the timeout
	 * has occurred.
	 */
	LOG_DEBUG("entering second pthread_cond_timedwait call, no timeout expected");
	cond_status = pthread_cond_timedwait(cond, &mutex, &timeout);

	if (cond_status == ETIMEDOUT) {
		LOG_ERR("error: pthread_cond_timewait timed out unexpectedly");
		return_status = -1;

	} else {
		LOG_INFO("second pthread_cond_timedwait did *not* timeout as expected, cond_status: %d, ETIMEDOUT: %d",
			 cond_status, ETIMEDOUT);
	}

	/*
	 * Leave the mutex unlocked since it is no longer needed.
	 */
	pthread_mutex_unlock(&mutex);

	/*
	 * Free the resources allocated by the called function through
	 * the use of the {_}_INITIALIZER constant and direct call to the
	 * _init function.
	 */
	pthread_mutex_destroy(&mutex);
	pthread_cond_destroy(cond);

exit:
	return (void *)return_status;
}

/**
 * @brief Tests timeout functionality of the pthread_cond_timedwait() function.
 *
 * @par This test launches a helper thread which blocks on a condition variable.  The
 * first blocking condition is cleared with a timeout, while the second is cleared
 * when the condition is signaled (not timeout).
 *
 * @return
 * TEST_PASS
 * TEST_FAIL
*/

int dspal_tester_test_pthread_cond_timedwait(void)
{
	int rv = 0;
	pthread_cond_t cond;
	int test_value = TEST_FAIL;
	pthread_t thread;

	pthread_attr_t attr;

    /* 
     * Stack increase is necessary for the newer DSP because of changes 
     * in the timer structures  
     */
	size_t stacksize = 2 * 1024; 

	rv = pthread_attr_init(&attr);

	if (rv != 0) { FAIL("pthread_attr_init returned error"); }

	rv = pthread_attr_setstacksize(&attr, stacksize);

	if (rv != 0) { FAIL("pthread_attr_setstacksize returned error"); }

	/*
	 * Create the thread passing a reference to the cond structure
	 * just initialized.
	 */
	rv = pthread_create(&thread, &attr, test_pthread_cond_timedwait_helper, &cond);

	if (rv != 0) {
		LOG_ERR("error pthread_create: %d", rv);
		goto exit;
	}

	/*
	 * Begin the first test by sleeping long enough for the timeout to
	 * have occurred.
	 */
	usleep((DSPAL_TESTER_COND_WAIT_TIMEOUT_IN_SECS + 2) * 1000000);

	/*
	 * Now trigger the condition to verify that it was detected before
	 * the timeout period has expired.
	 */
	LOG_DEBUG("entering pthread_cond_signal for the next cond wait (after the timeout)");
	rv = pthread_cond_signal(&cond);

	if (rv != 0) {
		LOG_ERR("error pthread_cond_signal: %d", rv);
		goto exit;
	}

	LOG_INFO("pthread_cond_signal has returned, waiting for the helper thread to exit");
	rv = pthread_join(thread, NULL);

	if (rv != 0) {
		LOG_ERR("error pthread_join: %d", rv);
		goto exit;
	}

	test_value = TEST_PASS;

exit:

	if (test_value != TEST_PASS) {
		LOG_ERR("error: dspal_tester_test_pthread_cond_timedwait");

	} else {
		LOG_INFO("success: dspal_tester_test_pthread_cond_timedwait");
	}

	return test_value;
}

int dspal_tester_test_rpcmem(unsigned char* data, int dataLen)
{
    LOG_ERR("Got call: data %x data_len %d", data, dataLen);

    for (int i = 0; i < dataLen; i++) {
        data[i] = data[i]+1; 
    }

    return TEST_PASS;
}

long long measure_time_helper(int * data, int dataLen)
{
   unsigned int sum = 0, run_times = 50000;
   struct timespec now, finish;
   int rv = clock_gettime(CLOCK_MONOTONIC, &now);
   if (rv != 0) { LOG_ERR("clock_gettime returned error"); }

   for (unsigned int j = 0; j < run_times; j++) {
       data[0] = 0;
       for (int i = 1; i < dataLen; i++)
            data[i] = data[i-1] + 1;
       sum = 0;
       for (int i = 0; i < dataLen; i++)
            sum = sum + data[i];
   }
   rv = clock_gettime(CLOCK_MONOTONIC, &finish);
   if (rv != 0) { LOG_ERR("clock_gettime returned error"); }

   long long length = (int) (((finish.tv_sec - now.tv_sec) * 1000000 + (finish.tv_nsec - now.tv_nsec) / 1000)/1000); // in milli-seconds
   return length;   
}
int dspal_tester_test_hap_power(unsigned char* data, int dataLen)
{
    int test_value = TEST_PASS;

    LOG_ERR("Got call: data %x data_len %d", data, dataLen);
    long long t1, t2;
    HAP_power_request(5, 5, 1000);
    t1 = measure_time_helper((int *) data, dataLen/sizeof(int));    
    LOG_ERR("HAP power low: %lld", t1);

    HAP_power_request(100, 100, 1000);
    t2 = measure_time_helper((int *)data, dataLen/sizeof(int));    
    LOG_ERR("HAP power high: %lld", t2);
    if (t2 > (t1 - t2)) test_value = TEST_FAIL;  // if the speed up is not significatnt, fail
    LOG_ERR("HAP power performance speed up %lld (ms)", t1 - t2);
    return test_value;
}

