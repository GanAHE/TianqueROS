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
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <dspal_time.h>
#include <dspal_signal.h>
#include <pthread.h>

#include "test_utils.h"
#include "dspal_tester.h"

#define TEST_FILE_PATH  "/dev/fs/test.txt"


/**
* @brief Test file open/close operation
*
* @par Detailed Description:
* This tests opens a file from the adsp side ('/dev/fs/test.txt').  It then
* tries to open the file without the dspal file prefix ('test.txt').

* Test:
* 1) Opens file with dspal path prefix ('/dev/fs/test.txt') in O_RDWR mode
* 2) Close that file
* 3) Open file without dispal path prefix ('test.txt') in O_RDONLY mode
* 4) Close that file
*
* @return
* TEST_PASS ------ on success
* TEST_FAIL ------ on error
*/
int dspal_tester_test_posix_file_open_close(void)
{
	int fd;

	LOG_INFO("%s test", __FUNCTION__);

	// Open TEST_FILE_PATH.  Create the file if it doesn't exist.
	fd = open(TEST_FILE_PATH, O_RDWR | O_CREAT | O_TRUNC);

	if (fd == -1) {
		FAIL("failed to open /dev/fs/test.txt in O_RDWR|O_CREAT|O_TRUNC mode. "
		     "Make sure to have test.txt at $ADSP_LIBRARY_PATH");
	}

	LOG_DEBUG("open /dev/fs/test.txt in O_RDWR|O_CREAT|O_TRUNC mode");

	if (close(fd) != 0) {
		FAIL("failed to close file handle");
	}

	LOG_DEBUG("close /dev/fs/test.txt");

	// test open file path without dspal file path prefix
	fd = open("random_file.txt", O_RDONLY);

	// open() is expected to fail
	if (fd >= 0) {
		close(fd);
		FAIL("open() succ for invalid path test.txt. This should never happen!");
	}

	return TEST_PASS;
}

/**
* @brief Test file read/write operations
*
* @par
* Test:
* 1) open file with dspal path prefix ('/dev/fs/test.txt') in read/write mode
* 2) write timestamp to file
* 3) read the file and compare the content
* 4) close the file
* 5) Opens the same file with read only mode
* 6) read the file and compare with the content written in step 3
* 7) test write() and it should fail
* 8) close the file
*
* @return
* TEST_PASS ------ if all operations succeed
* TEST_FAIL ------ otherwise
*/
int dspal_tester_test_posix_file_read_write(void)
{
	int fd;
	int bytes_read;
	char wbuf[100];
	char rbuf[100];
	uint64_t timestamp = time(NULL);

	LOG_INFO("%s test", __FUNCTION__);

	// Open the file in read/write mode
	fd = open(TEST_FILE_PATH, O_RDWR | O_CREAT | O_TRUNC);

	if (fd < 0) {
		FAIL("failed to open /dev/fs/test.txt in O_RDWR|O_CREAT|O_TRUNC mode.");
	}

	LOG_DEBUG("open /dev/fs/test.txt in O_RDWR|O_CREAT|O_TRUNC mode");

	// write timestamp
	memset(wbuf, 0, 100);
	sprintf(wbuf, "test - timestamp: %llu\n", timestamp);

	if (write(fd, wbuf, strlen(wbuf)) != (int)strlen(wbuf)) {
		FAIL("failed to write /dev/fs/test.txt");
	}

	LOG_DEBUG("written to %s: %s (len: %d)", TEST_FILE_PATH, wbuf,
		  strlen(wbuf));

	close(fd);
	LOG_DEBUG("closed /dev/fs/test.txt");

	// Open the file in read/write mode
	fd = open(TEST_FILE_PATH, O_RDONLY);

	if (fd < 0) {
		FAIL("failed to open /dev/fs/test.txt in O_RDONLY mode.");
	}

	LOG_DEBUG("opened /dev/fs/test.txt in O_RDONLY mode");

	// read the content to ensure the timestamp is properly written to file
	memset(rbuf, 0, 100);
	bytes_read = read(fd, rbuf, sizeof(rbuf) - 1);
	LOG_DEBUG("read %d bytes from /dev/fs/test.txt:\n%s", bytes_read, rbuf);

	if ((!(strncmp(wbuf, rbuf, bytes_read) == 0) && (bytes_read == (int)strlen(wbuf)))) {
		FAIL("file write and read content does not match");
	}

	// test writing file in O_RDONLY mode
	int ret = write(fd, wbuf, strlen(wbuf));

	if (ret >= 0) {
		FAIL("write() succ on file in O_RDONLY mode. This should never happen!");
	}

	LOG_DEBUG("write() failed on file in O_RDONLY mode as expected.");

	close(fd);
	LOG_DEBUG("closed /dev/fs/test.txt");

	return TEST_PASS;
}

/**
 * @brief Test opening file in O_TRUNC mode
 *
 * @par
 * 1) open the file in O_WRONLY mode
 * 2) write timestamp to file
 * 3) close the file
 * 4) open the file in O_TRUNC mode
 * 5) read the file and verify if the file is empty
 * 6) cloes the file
 *
 * @return
 * TEST_PASS ------- file properly truncated
 * TEST_FAIL ------- on error
 */
int dspal_tester_test_posix_file_open_trunc()
{
	int fd;
	int bytes_read;
	char wbuf[100];
	char rbuf[100];
	uint64_t timestamp = time(NULL);

	LOG_INFO("%s test", __FUNCTION__);

	// Open the file in read/write mode
	fd = open(TEST_FILE_PATH, O_RDWR | O_CREAT | O_TRUNC);

	if (fd < 0) {
		FAIL("failed to open /dev/fs/test.txt in O_RDWR|O_CREAT|O_TRUNC mode.");
	}

	LOG_DEBUG("opened /dev/fs/test.txt in O_RDWR|O_CREAT|O_TRUNC mode");

	// write timestamp
	memset(wbuf, 0, 100);
	sprintf(wbuf, "test - timestamp: %llu\n", timestamp);
	LOG_DEBUG("writing to %s: %s (len: %d)", TEST_FILE_PATH, wbuf,
		  strlen(wbuf) + 1);

	close(fd);
	LOG_DEBUG("closed /dev/fs/test.txt");

	// open the file in O_TRUNC mode
	fd = open(TEST_FILE_PATH, O_RDWR | O_TRUNC);

	if (fd < 0) {
		FAIL("failed to open /dev/fs/test.txt in O_RDWR|O_TRUNC mode.");
	}

	LOG_DEBUG("opened /dev/fs/test.txt in O_RDWR|O_TRUNC mode");

	bytes_read = read(fd, rbuf, sizeof(rbuf) - 1);
	LOG_DEBUG("read %d bytes from /dev/fs/test.txt", bytes_read);

	if (bytes_read != 0) {
		FAIL("Failed to truncate the file using O_TRUNC");
	}

	LOG_DEBUG("/dev/fs/test.txt in truncated");

	close(fd);
	LOG_DEBUG("closed /dev/fs/test.txt");

	return TEST_PASS;
}

/**
 * @brief Test opening file in O_APPEND mode
 *
 * @par
 * 1) open the file in O_RDWR|O_APPEND mode
 * 2) read file content
 * 3) write something
 * 4) close the file
 * 5) open the file in O_RDONLY mode
 * 6) read the file and verify the content
 * 7) close the file
 *
 * @return
 * TEST_PASS ------- file properly appended
 * TEST_FAIL ------- on error
 */
int dspal_tester_test_posix_file_open_append()
{
	int fd;
	int bytes_read;
	char rbuf[100];
	const char *old_content = "old content\n";
	const char *new_content = "new content\n";

	LOG_INFO("%s test", __FUNCTION__);

	fd = open(TEST_FILE_PATH, O_RDWR | O_TRUNC | O_CREAT);

	if (fd < 0) {
		FAIL("failed to open /dev/fs/test.txt in O_RDWR|O_TRUNC|O_CREAT mode.");
	}

	LOG_DEBUG("opened /dev/fs/test.txt in O_RDWR|O_CREAT|O_TRUNC mode");

	if (write(fd, old_content, strlen(old_content)) != (int)strlen(old_content)) {
		FAIL("failed to write /dev/fs/test.txt");
	}

	LOG_DEBUG("writing to %s: %s (len: %d)", TEST_FILE_PATH, old_content,
		  strlen(old_content));

	close(fd);
	LOG_DEBUG("closed /dev/fs/test.txt");

	// Open the file in APPEND mode
	fd = open(TEST_FILE_PATH, O_RDWR | O_APPEND);

	if (fd < 0) {
		FAIL("failed to open /dev/fs/test.txt in O_RDWR|O_APPEND mode.");
	}

	LOG_DEBUG("opened /dev/fs/test.txt in O_RDWR|O_APPEND mode");

	if (write(fd, new_content, strlen(new_content)) != (int)strlen(new_content)) {
		FAIL("failed to write /dev/fs/test.txt");
	}

	LOG_DEBUG("writing to %s: %s (len: %d)", TEST_FILE_PATH, new_content,
		  strlen(new_content));

	close(fd);
	LOG_DEBUG("closed /dev/fs/test.txt");

	// open the file in read only mode
	fd = open(TEST_FILE_PATH, O_RDONLY);

	if (fd < 0) {
		FAIL("failed to open /dev/fs/test.txt in O_RDONLY mode.");
	}

	LOG_DEBUG("opened /dev/fs/test.txt in O_RDONLY mode");

	memset(rbuf, 0, 100);
	bytes_read = read(fd, rbuf, sizeof(rbuf));

	if (bytes_read < 0) {
		FAIL("failed to read /dev/fs/test.txt.");

	} else if (bytes_read == 0) {
		FAIL("/dev/fs/test.txt is empty");
	}

	if (!(strstr(rbuf, old_content) == rbuf) &&
	    strstr(rbuf + strlen(old_content), new_content) ==
	    rbuf + strlen(old_content)) {
		FAIL("failed to append writing to /dev/fs/test.txt");
	}

	LOG_DEBUG("succ to append file using O_APPEND");

	close(fd);
	LOG_DEBUG("closed /dev/fs/test.txt");

	return TEST_PASS;
}

/**
* @brief Test to see if ioctl fails (ioctl is not supported for files)
*
* @par
* Test:
* 1) Opens file with dspal path prefix ('/dev/fs/test.txt') in read/write mode with append
* 2) Try ioctl call and make sure it fails
* 3) Close the file
*
* @return
* TEST_PASS ------ Always
*/
int dspal_tester_test_posix_file_ioctl(void)
{
	int fd;

	LOG_INFO("%s test", __FUNCTION__);

	fd = open(TEST_FILE_PATH, O_RDWR | O_TRUNC | O_TRUNC);
	LOG_DEBUG("opened /dev/fs/test.txt in O_RDWR|O_TRUNC|O_TRUNC mode");

	if (fd == -1) {
		FAIL("open test.txt failed. Make sure to have test.txt at $ADSP_LIBRARY_PATH");
	}

	LOG_DEBUG("trying ioctl()");

	if (ioctl(fd, 0, NULL) != -1) {
		FAIL("ioctl() is not supported and should have returned -1.");
	}

	close(fd);
	LOG_DEBUG("closed /dev/fs/test.txt");

	return TEST_PASS;
}

/**
* @brief Test to remove the specified file
*
* @par
* Test:
* 1) Opens file with dspal path prefix ('/dev/fs/test.txt') and create it if
*    it does not exist
* 2) close the file
* 3) remove the file
*
* @return
* TEST_PASS ------ if remove returns 0
* TEST FAIL ------ on error
*/
int dspal_tester_test_posix_file_remove(void)
{
	int fd;

	LOG_INFO("%s test", __FUNCTION__);

	// First create the file if it does not exist yet
	fd = open(TEST_FILE_PATH, O_RDWR | O_CREAT);
	LOG_DEBUG("opened /dev/fs/test.txt in O_RDWR|O_CREAT mode");

	if (fd == -1) {
		FAIL("open test.txt failed. Make sure to have test.txt at $ADSP_LIBRARY_PATH");
	}

	close(fd);
	LOG_DEBUG("closed /dev/fs/test.txt");

	if (remove(TEST_FILE_PATH) != 0) {
		LOG_ERR("failed to remove %s", TEST_FILE_PATH);
		return TEST_FAIL;
	}

	LOG_DEBUG("removed /dev/fs/test.txt");

	// test removing a file with invalid dspal path
	if (remove("test.txt") == 0) {
		LOG_ERR("removed %s. This shouldn't happen", TEST_FILE_PATH);
		return TEST_FAIL;
	}

	LOG_DEBUG("removing file with invalid path failed. expected");

	return TEST_PASS;
}

/**
* @brief Test file fsync operation
*
* @par
* Test:
* 1) open file with dspal path prefix ('/dev/fs/test.txt') in read/write mode
* 2) write timestamp to file
* 3) fsync on the file
* 4) close the file
* 5) Opens the same file with read only mode
* 6) read the file and compare with the content written in step 3
* 7) close the file
*
* @return
* TEST_PASS ------ if all operations succeed
* TEST_FAIL ------ otherwise
*/
int dspal_tester_test_posix_file_fsync(void)
{
	int fd;
	int bytes_read;
	char wbuf[100];
	char rbuf[100];
	uint64_t timestamp = time(NULL);

	LOG_INFO("%s test", __FUNCTION__);

	// Open the file in read/write mode
	fd = open(TEST_FILE_PATH, O_RDWR | O_CREAT | O_TRUNC);

	if (fd < 0) {
		FAIL("failed to open /dev/fs/test.txt in O_RDWR|O_CREAT|O_TRUNC mode.");
	}

	LOG_DEBUG("open /dev/fs/test.txt in O_RDWR|O_CREAT|O_TRUNC mode");

	// write timestamp
	memset(wbuf, 0, 100);
	sprintf(wbuf, "test - timestamp: %llu\n", timestamp);

	if (write(fd, wbuf, strlen(wbuf)) != (int)strlen(wbuf)) {
		FAIL("failed to write /dev/fs/test.txt");
	}

	LOG_DEBUG("written to %s: %s (len: %d)", TEST_FILE_PATH, wbuf,
		  strlen(wbuf));

	// fsync fd to flush the content to storage device
	if (fsync(fd) < 0) {
		FAIL("failed to fsync /dev/fs/test.txt");
	}

	LOG_INFO("fsync() succ");

	close(fd);
	LOG_DEBUG("closed /dev/fs/test.txt");

	// Open the file in read/write mode
	fd = open(TEST_FILE_PATH, O_RDONLY);

	if (fd < 0) {
		FAIL("failed to open /dev/fs/test.txt in O_RDONLY mode.");
	}

	LOG_DEBUG("opened /dev/fs/test.txt in O_RDONLY mode");

	// read the content to ensure the timestamp is properly written to file
	memset(rbuf, 0, 100);
	bytes_read = read(fd, rbuf, sizeof(rbuf) - 1);
	LOG_DEBUG("read %d bytes from /dev/fs/test.txt:\n%s", bytes_read, rbuf);

	if ((!(strncmp(wbuf, rbuf, bytes_read) == 0) && (bytes_read == (int)strlen(wbuf)))) {
		FAIL("file write and read content does not match");
	}

	// test writing file in O_RDONLY mode
	int ret = write(fd, wbuf, strlen(wbuf));

	if (ret >= 0) {
		FAIL("write() succ on file in O_RDONLY mode. This should never happen!");
	}

	LOG_DEBUG("write() failed on file in O_RDONLY mode as expected.");

	close(fd);
	LOG_DEBUG("closed /dev/fs/test.txt");

	return TEST_PASS;
}

/**
* @brief Test to a file can be opened in all supported modes and closed.
*
* @par
* Test:
* 1) Opens file with dspal path prefix ('/dev/fs/test.txt') in one of the
*    fopen supported modes
* 2) Close the file
*
* @return
* TEST_FAIL ------ if fopen failed on certain mode
* TEST_PASS ------ if fopen succeeds on all modes
*/
int dspal_tester_test_fopen_fclose(void)
{
	FILE *fd;
	const char *modes[] = {
		"w", "w+", "r", "r+", "a", "a+"
	};
	int num_modes = sizeof(modes) / sizeof(const char *);

	LOG_INFO("%s test", __FUNCTION__);

	for (int i = 0; i < num_modes; i++) {
		fd = fopen(TEST_FILE_PATH, modes[i]);

		if (fd == NULL) {
			LOG_ERR("fopen() mode %s returned NULL", modes[i]);
			return TEST_FAIL;
		}

		fclose(fd);
		LOG_INFO("fopen()/fclose mode %s succ", modes[i]);
	}

	LOG_INFO("fopen_fclose test passed");

	return TEST_PASS;
}

/**
* @brief Test to a file can be written and read using fwrite() and fread()
*
* @par
* Test:
* 1) Opens file with dspal path prefix ('/dev/fs/test.txt') in write mode
* 2) write timestamp to file
* 3) fflush and fclose the file
* 4) open the file in r mode
* 5) fread() the file content and compare the bytes read and bytes written
* 6) Close the file
*
* @return
* TEST_PASS ------ if fwrite and fread succeed, and the bytes read and bytes
*                  written are identical.
* TEST_FAIL ------ otherwise
*/
int dspal_tester_test_fwrite_fread(void)
{
	FILE *fd;
	char buffer[50] = {0};
	uint64_t timestamp = time(NULL);
	size_t bytes_written;
	size_t bytes_read;
	size_t buffer_len;

	LOG_INFO("%s test", __FUNCTION__);

	fd = fopen(TEST_FILE_PATH, "w");

	if (fd == NULL) {
		LOG_ERR("fopen() mode w returned NULL");
		return TEST_FAIL;
	}

	sprintf(buffer, "test - timestamp: %llu\n", timestamp);
	buffer_len = strlen(buffer) + 1;

	LOG_DEBUG("writing to test.txt: %s (len: %d)", buffer, buffer_len);

	bytes_written = fwrite(buffer, 1, buffer_len, fd);

	if (bytes_written != buffer_len) {
		LOG_ERR("fwrite() %d bytes returned less than expected %d",
			buffer_len, bytes_written);
		return TEST_FAIL;
	}

	fflush(fd);
	fclose(fd);

	fd = fopen(TEST_FILE_PATH, "r");

	if (fd == NULL) {
		LOG_ERR("fopen() mode r returned NULL");
		return TEST_FAIL;
	}

	memset(buffer, 0, 50);
	bytes_read = fread(buffer, 1, buffer_len, fd);

	if (bytes_read != buffer_len) {
		LOG_ERR("fread() %d bytes returned less than expected %d",
			buffer_len, bytes_read);
		return TEST_FAIL;
	}

	LOG_DEBUG("fread() %d bytes: %s", bytes_read, buffer);

	fclose(fd);

	LOG_DEBUG("fwrite_fread test passed");

	return TEST_PASS;
}

/**
* @brief Test file read/write operations, but in a different thread
*
* @par
* Test:
* 1) open file with dspal path prefix ('/dev/fs/test.txt') in read/write mode
* 2) write timestamp to file
* 3) read the file and compare the content
* 4) close the file
* 5) Opens the same file with read only mode
* 6) read the file and compare with the content written in step 3
* 7) test write() and it should fail
* 8) close the file
*
* @return
* TEST_PASS ------ if all operations succeed
* TEST_FAIL ------ otherwise
*/

int dspal_tester_test_posix_file_threading(void)
{
	int rv = 0;

	int test_value = 0;
	pthread_t thread;
	LOG_DEBUG("dspal_tester_test_posix_file_threading");
	pthread_attr_t attr;
	size_t stacksize = 20 * 1024;
	rv = pthread_attr_init(&attr);
	if (rv != 0) { FAIL("pthread_attr_init returned error"); }
	rv = pthread_attr_setstacksize(&attr, stacksize);
	rv = pthread_attr_setthreadname(&attr, "dspal_test_thread_file");
	attr.priority = 235;
	if (rv != 0) { FAIL("pthread_attr_init returned error"); }
	rv = pthread_create(&thread, &attr, (void *)dspal_tester_test_posix_file_open_close, &test_value);
	if (rv != 0) { FAIL("thread_create returned error"); }

	rv = pthread_join(thread, NULL);
	LOG_DEBUG("dspal_tester_test_posix_file_threading joining....done!");


	if (rv != 0) { FAIL("thread_join returned error"); }

	return TEST_PASS;
}

