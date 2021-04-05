/*-
 * Copyright (c) 2005 David Xu <davidxu@freebsd.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#pragma once

#include <sys/cdefs.h>
#include "dspal_types.h"
#include "dspal_signal.h"

struct mq_attr {
	long	mq_flags;	/* Message queue flags. */
	long	mq_maxmsg;	/* Maximum number of messages. */
	long	mq_msgsize;	/* Maximum message size. */
	long	mq_curmsgs;	/* Number of messages currently queued. */
	long    __reserved[4];  /* Ignored for input, zeroed for output */
};

struct timespec;

typedef int mqd_t;

__BEGIN_DECLS
mqd_t mq_open(const char *, int, ...);
int mq_close(mqd_t);
int mq_unlink(const char *);
int mq_send(mqd_t, const char *, size_t, unsigned);
int mq_timedsend(mqd_t, const char *, size_t, unsigned,
		 const struct timespec *);
ssize_t mq_receive(mqd_t, char *, size_t, unsigned *);
ssize_t mq_timedreceive(mqd_t, char *__restrict, size_t,
			unsigned *__restrict, const struct timespec *__restrict);
int mq_getattr(mqd_t, struct mq_attr *);
int mq_setattr(mqd_t, const struct mq_attr *__restrict,
	       struct mq_attr *__restrict);
__END_DECLS
