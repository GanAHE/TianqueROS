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

#include <semaphore.h>

#include "test_utils.h"
#include "dspal_tester.h"

/**
* @brief Tests semaphore functionality (using 1 thread)
*
* @par Test:
* 1) Initialize the semaphore to a value of 0
* 2) Get the value of the semaphore and check that is 0
* 3) Do a try wait and make sure it returns an error (needs to error out because no post yet)
* 4) Do a semaphore post
* 5) Get the value of the semaphore and make sure it is 1 (set to 1 from step 4)
* 6) Do a wait on the semaphore
* 7) Get the value of the semaphore and make sure it is 0, 9 set to 1 from step 4 and 0 from step 6)
* 8) Destroy the Semaphore
*
* @return
* TEST_PASS ------ Always
*/
int dspal_tester_test_semaphore_wait(void)
{
	int rv = 0;

	int sem_value = 0;
	sem_t sem;

	rv = sem_init(&sem, 0, 0);

	if (rv != 0) { FAIL("sem_init returned error"); }

	rv = sem_getvalue(&sem, &sem_value);

	if (rv != 0) { FAIL("sem_getvalue returned error"); }

	if (sem_value != 0) { FAIL("sem_value not as expected"); }

	rv = sem_trywait(&sem);

	if (rv != -1) { FAIL("sem_trywait DID NOT return an error"); }

	rv = sem_post(&sem);

	if (rv != 0) { FAIL("sem_post returned error"); }

	rv = sem_getvalue(&sem, &sem_value);

	if (rv != 0) { FAIL("sem_getvalue returned error"); }

	if (sem_value != 1) { FAIL("sem_value not as expected"); }

	rv = sem_wait(&sem);

	if (rv != 0) { FAIL("sem_wait returned error"); }

	rv = sem_getvalue(&sem, &sem_value);

	if (rv != 0) { FAIL("sem_getvalue returned error"); }

	if (sem_value != 0) { FAIL("sem_value not as expected"); }

	rv = sem_destroy(&sem);

	if (rv != 0) { FAIL("sem_destroy returned error"); }

	return TEST_PASS;
}
