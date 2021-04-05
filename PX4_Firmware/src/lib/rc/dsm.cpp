/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file dsm.cpp
 *
 * Serial protocol decoder for the Spektrum DSM* family of protocols.
 *
 * Decodes into the global PPM buffer and updates accordingly.
 */

#include <px4_platform_common/px4_config.h>
#include <board_config.h>
#include <px4_platform_common/defines.h>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>

#include "dsm.h"
#include "spektrum_rssi.h"
#include "common_rc.h"
#include <drivers/drv_hrt.h>

#if defined(__PX4_NUTTX)
#include <nuttx/arch.h>
#define dsm_udelay(arg)    up_udelay(arg)
#else
#define dsm_udelay(arg) px4_usleep(arg)
#endif

// #define DSM_DEBUG

static enum DSM_DECODE_STATE {
	DSM_DECODE_STATE_DESYNC = 0,
	DSM_DECODE_STATE_SYNC
} dsm_decode_state = DSM_DECODE_STATE_DESYNC;

static int dsm_fd = -1;						/**< File handle to the DSM UART */
static hrt_abstime dsm_last_rx_time;            /**< Timestamp when we last received data */
static hrt_abstime dsm_last_frame_time;		/**< Timestamp for start of last valid dsm frame */
static dsm_frame_t &dsm_frame = rc_decode_buf.dsm.frame;	/**< DSM_BUFFER_SIZE DSM dsm frame receive buffer */
static dsm_buf_t &dsm_buf = rc_decode_buf.dsm.buf;	/**< DSM_BUFFER_SIZE DSM dsm frame receive buffer */

static uint16_t dsm_chan_buf[DSM_MAX_CHANNEL_COUNT];
static unsigned dsm_partial_frame_count;	/**< Count of bytes received for current dsm frame */
static unsigned dsm_channel_shift = 0;			/**< Channel resolution, 0=unknown, 1=10 bit, 2=11 bit */
static unsigned dsm_frame_drops = 0;			/**< Count of incomplete DSM frames */
static uint16_t dsm_chan_count = 0;         /**< DSM channel count */

static bool
dsm_decode(hrt_abstime frame_time, uint16_t *values, uint16_t *num_values, bool *dsm_11_bit, unsigned max_values,
	   int8_t *rssi_percent);

/**
 * Attempt to decode a single channel raw channel datum
 *
 * The DSM* protocol doesn't provide any explicit framing,
 * so we detect dsm frame boundaries by the inter-dsm frame delay.
 *
 * The minimum dsm frame spacing is 11ms; with 16 bytes at 115200bps
 * dsm frame transmission time is ~1.4ms.
 *
 * We expect to only be called when bytes arrive for processing,
 * and if an interval of more than 5ms passes between calls,
 * the first byte we read will be the first byte of a dsm frame.
 *
 * In the case where byte(s) are dropped from a dsm frame, this also
 * provides a degree of protection. Of course, it would be better
 * if we didn't drop bytes...
 *
 * Upon receiving a full dsm frame we attempt to decode it
 *
 * @param[in] raw 16 bit raw channel value from dsm frame
 * @param[in] shift position of channel number in raw data
 * @param[out] channel pointer to returned channel number
 * @param[out] value pointer to returned channel value
 * @return true=raw value successfully decoded
 */
static bool
dsm_decode_channel(uint16_t raw, unsigned shift, unsigned *channel, unsigned *value)
{

	if (raw == 0xffff) {
		return false;
	}

	*channel = (raw >> shift) & 0xf;

	uint16_t data_mask = (1 << shift) - 1;
	*value = raw & data_mask;

	//debug("DSM: %d 0x%04x -> %d %d", shift, raw, *channel, *value);

	return true;
}

/**
 * Attempt to guess if receiving 10 or 11 bit channel values
 *
 * @param[in] reset true=reset the 10/11 bit state to unknown
 */
static bool
dsm_guess_format(bool reset)
{
	static uint32_t	cs10 = 0;
	static uint32_t	cs11 = 0;
	static unsigned samples = 0;

	/* reset the 10/11 bit sniffed channel masks */
	if (reset) {
		cs10 = 0;
		cs11 = 0;
		samples = 0;
		dsm_channel_shift = 0;
		return false;
	}

	/* scan the channels in the current dsm_frame in both 10- and 11-bit mode */
	for (unsigned i = 0; i < DSM_FRAME_CHANNELS; i++) {

		uint8_t *dp = &dsm_frame[2 + (2 * i)];
		uint16_t raw = (dp[0] << 8) | dp[1];
		unsigned channel, value;

		/* if the channel decodes, remember the assigned number */
		if (dsm_decode_channel(raw, 10, &channel, &value) && (channel < 31)) {
			cs10 |= (1 << channel);
		}

		if (dsm_decode_channel(raw, 11, &channel, &value) && (channel < 31)) {
			cs11 |= (1 << channel);
		}

		/* XXX if we cared, we could look for the phase bit here to decide 1 vs. 2-dsm_frame format */
	}

	samples++;

#ifdef DSM_DEBUG
	printf("dsm guess format: samples: %d %s\n", samples,
	       (reset) ? "RESET" : "");
#endif

	/* wait until we have seen plenty of frames - 5 should normally be enough */
	if (samples < 5) {
		return false;
	}

	/*
	 * Iterate the set of sensible sniffed channel sets and see whether
	 * decoding in 10 or 11-bit mode has yielded anything we recognize.
	 *
	 * XXX Note that due to what seem to be bugs in the DSM2 high-resolution
	 *     stream, we may want to sniff for longer in some cases when we think we
	 *     are talking to a DSM2 receiver in high-resolution mode (so that we can
	 *     reject it, ideally).
	 *     See e.g. http://git.openpilot.org/cru/OPReview-116 for a discussion
	 *     of this issue.
	 */
	static uint32_t masks[] = {
		0x3f,	/* 6 channels (DX6) */
		0x7f,	/* 7 channels (DX7) */
		0xff,	/* 8 channels (DX8) */
		0x1ff,	/* 9 channels (DX9, etc.) */
		0x3ff,	/* 10 channels (DX10) */
		0x1fff,	/* 13 channels (DX10t) */
		0x3fff	/* 18 channels (DX10) */
	};
	unsigned votes10 = 0;
	unsigned votes11 = 0;

	for (unsigned i = 0; i < (sizeof(masks) / sizeof(masks[0])); i++) {

		if (cs10 == masks[i]) {
			votes10++;
		}

		if (cs11 == masks[i]) {
			votes11++;
		}
	}

	if ((votes11 == 1) && (votes10 == 0)) {
		dsm_channel_shift = 11;
#ifdef DSM_DEBUG
		printf("DSM: 11-bit format\n");
#endif
		return true;
	}

	if ((votes10 == 1) && (votes11 == 0)) {
		dsm_channel_shift = 10;
#ifdef DSM_DEBUG
		printf("DSM: 10-bit format\n");
#endif
		return true;
	}

	/* call ourselves to reset our state ... we have to try again */
#ifdef DSM_DEBUG
	printf("DSM: format detect fail, 10: 0x%08x %d 11: 0x%08x %d\n", cs10, votes10, cs11, votes11);
#endif
	dsm_guess_format(true);
	return false;
}

int
dsm_config(int fd)
{
#ifdef SPEKTRUM_POWER
	// enable power on DSM connector
	SPEKTRUM_POWER(true);
#endif

	int ret = -1;

	if (fd >= 0) {

		struct termios t;

		/* 115200bps, no parity, one stop bit */
		tcgetattr(fd, &t);
		cfsetspeed(&t, 115200);
		t.c_cflag &= ~(CSTOPB | PARENB);
		tcsetattr(fd, TCSANOW, &t);

		/* initialise the decoder */
		dsm_partial_frame_count = 0;
		dsm_last_rx_time = hrt_absolute_time();

		/* reset the format detector */
		dsm_guess_format(true);

		ret = 0;
	}

	return ret;
}

void
dsm_proto_init()
{
	dsm_channel_shift = 0;
	dsm_frame_drops = 0;
	dsm_chan_count = 0;
	dsm_decode_state = DSM_DECODE_STATE_DESYNC;

	for (unsigned i = 0; i < DSM_MAX_CHANNEL_COUNT; i++) {
		dsm_chan_buf[i] = 0;
	}
}

/**
 * Initialize the DSM receive functionality
 *
 * Open the UART for receiving DSM frames and configure it appropriately
 *
 * @param[in] device Device name of DSM UART
 */
int
dsm_init(const char *device)
{

	if (dsm_fd < 0) {
		dsm_fd = open(device, O_RDWR | O_NONBLOCK);
	}

	dsm_proto_init();

	int ret = dsm_config(dsm_fd);

	if (!ret) {
		return dsm_fd;

	} else {
		return -1;
	}
}

void
dsm_deinit()
{
	if (dsm_fd >= 0) {
		close(dsm_fd);
	}

	dsm_fd = -1;
}

#if defined(SPEKTRUM_POWER)
/**
 * Handle DSM satellite receiver bind mode handler
 *
 * @param[in] cmd commands - dsm_bind_power_down, dsm_bind_power_up, dsm_bind_set_rx_out, dsm_bind_send_pulses, dsm_bind_reinit_uart
 * @param[in] pulses Number of pulses for dsm_bind_send_pulses command
 */
void
dsm_bind(uint16_t cmd, int pulses)
{
	if (dsm_fd < 0) {
		return;
	}

	switch (cmd) {

	case DSM_CMD_BIND_POWER_DOWN:

		/*power down DSM satellite*/
		SPEKTRUM_POWER(false);
		break;

	case DSM_CMD_BIND_POWER_UP:

		/*power up DSM satellite*/
		SPEKTRUM_POWER(true);
		dsm_guess_format(true);
		break;

	case DSM_CMD_BIND_SET_RX_OUT:

		/*Set UART RX pin to active output mode*/
		SPEKTRUM_RX_AS_GPIO_OUTPUT();
		break;

	case DSM_CMD_BIND_SEND_PULSES:

		/*Pulse RX pin a number of times*/
		for (int i = 0; i < pulses; i++) {
			dsm_udelay(120);
			SPEKTRUM_OUT(false);
			dsm_udelay(120);
			SPEKTRUM_OUT(true);
		}

		break;

	case DSM_CMD_BIND_REINIT_UART:

		/*Restore USART RX pin to RS232 receive mode*/
		SPEKTRUM_RX_AS_UART();
		break;

	}
}
#endif

/**
 * Decode the entire dsm frame (all contained channels)
 *
 * @param[in] frame_time timestamp when this dsm frame was received. Used to detect RX loss in order to reset 10/11 bit guess.
 * @param[out] values pointer to per channel array of decoded values
 * @param[out] num_values pointer to number of raw channel values returned
 * @return true=DSM frame successfully decoded, false=no update
 */
bool
dsm_decode(hrt_abstime frame_time, uint16_t *values, uint16_t *num_values, bool *dsm_11_bit, unsigned max_values,
	   int8_t *rssi_percent)
{
	/*
	debug("DSM dsm_frame %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x",
		dsm_frame[0], dsm_frame[1], dsm_frame[2], dsm_frame[3], dsm_frame[4], dsm_frame[5], dsm_frame[6], dsm_frame[7],
		dsm_frame[8], dsm_frame[9], dsm_frame[10], dsm_frame[11], dsm_frame[12], dsm_frame[13], dsm_frame[14], dsm_frame[15]);
	*/
	/*
	 * If we have lost signal for at least a second, reset the
	 * format guessing heuristic.
	 */
	if (((frame_time - dsm_last_frame_time) > 1000000) && (dsm_channel_shift != 0)) {
		dsm_guess_format(true);
	}

	/* if we don't know the dsm_frame format, update the guessing state machine */
	if (dsm_channel_shift == 0) {
		if (!dsm_guess_format(false)) {
			return false;
		}
	}

	/*
	 * The first byte represents the rssi in dBm on telemetry receivers with updated
	 * firmware, or fades on others. If the value is less than zero, it's rssi.
	 * We have other ways to detect bad link metrics, so ignore positive values,
	 * but rssi dBm is a useful value.
	 */

	if (rssi_percent) {
		if (((int8_t *)dsm_frame)[0] < 0) {
			/*
			 * RSSI is a signed integer between -42dBm and -92dBm
			 * If signal is lost, the value is -128
			 */
			int8_t dbm = (int8_t)dsm_frame[0];

			if (dbm == -128) {
				*rssi_percent = 0;

			} else {
				*rssi_percent = spek_dbm_to_percent(dbm);
			}

		} else {
			/* if we don't know the rssi, anything over 100 will invalidate it */
			*rssi_percent = 127;
		}
	}

	/*
	 * The second byte indicates the protocol and frame rate. We have a
	 * guessing state machine, so we don't need to use this. At any rate,
	 * these are the allowable values:
	 *
	 * 0x01 22MS 1024 DSM2
	 * 0x12 11MS 2048 DSM2
	 * 0xa2 22MS 2048 DSMX
	 * 0xb2 11MS 2048 DSMX
	 */

	/*
	 * Each channel is a 16-bit unsigned value containing either a 10-
	 * or 11-bit channel value and a 4-bit channel number, shifted
	 * either 10 or 11 bits. The MSB may also be set to indicate the
	 * second dsm_frame in variants of the protocol where more than
	 * seven channels are being transmitted.
	 */

	for (unsigned i = 0; i < DSM_FRAME_CHANNELS; i++) {

		uint8_t *dp = &dsm_frame[2 + (2 * i)];
		uint16_t raw = (dp[0] << 8) | dp[1];
		unsigned channel, value;

		if (!dsm_decode_channel(raw, dsm_channel_shift, &channel, &value)) {
			continue;
		}

		/* reset bit guessing state machine if the channel index is out of bounds */
		if (channel > DSM_MAX_CHANNEL_COUNT) {
			dsm_guess_format(true);
			return false;
		}

		/* ignore channels out of range */
		if (channel >= max_values) {
			continue;
		}

		/* update the decoded channel count */
		if (channel >= *num_values) {
			*num_values = channel + 1;
		}

		/* convert 0-1024 / 0-2048 values to 1000-2000 ppm encoding. */
		if (dsm_channel_shift == 10) {
			value *= 2;
		}

		/*
		 * Spektrum scaling is special. There are these basic considerations
		 *
		 *   * Midpoint is 1520 us
		 *   * 100% travel channels are +- 400 us
		 *
		 * We obey the original Spektrum scaling (so a default setup will scale from
		 * 1100 - 1900 us), but we do not obey the weird 1520 us center point
		 * and instead (correctly) center the center around 1500 us. This is in order
		 * to get something useful without requiring the user to calibrate on a digital
		 * link for no reason.
		 */

		/* scaled integer for decent accuracy while staying efficient */
		value = ((((int)value - 1024) * 1000) / 1700) + 1500;

		/*
		 * Store the decoded channel into the R/C input buffer, taking into
		 * account the different ideas about channel assignement that we have.
		 *
		 * Specifically, the first four channels in rc_channel_data are roll, pitch, thrust, yaw,
		 * but the first four channels from the DSM receiver are thrust, roll, pitch, yaw.
		 */
		switch (channel) {
		case 0:
			channel = 2;
			break;

		case 1:
			channel = 0;
			break;

		case 2:
			channel = 1;

		default:
			break;
		}

		values[channel] = value;
	}

	/*
	 * Spektrum likes to send junk in higher channel numbers to fill
	 * their packets. We don't know about a 13 channel model in their TX
	 * lines, so if we get a channel count of 13, we'll return 12 (the last
	 * data index that is stable).
	 */
	if (*num_values == 13) {
		*num_values = 12;
	}

	/* Set the 11-bit data indicator */
	*dsm_11_bit = (dsm_channel_shift == 11);

	/* we have received something we think is a dsm_frame */
	dsm_last_frame_time = frame_time;

	/*
	 * XXX Note that we may be in failsafe here; we need to work out how to detect that.
	 */

#ifdef DSM_DEBUG
	printf("PARSED PACKET\n");
#endif

	/* check all values */
	for (unsigned i = 0; i < *num_values; i++) {
		/* if the value is unrealistic, fail the parsing entirely */
		if (values[i] < 600 || values[i] > 2400) {
#ifdef DSM_DEBUG
			printf("DSM: VALUE RANGE FAIL: %d: %d\n", (int)i, (int)values[i]);
#endif
			*num_values = 0;
			return false;
		}
	}

	return true;
}

/**
 * Called periodically to check for input data from the DSM UART
 *
 * The DSM* protocol doesn't provide any explicit framing,
 * so we detect dsm frame boundaries by the inter-dsm frame delay.
 * The minimum dsm frame spacing is 11ms; with 16 bytes at 115200bps
 * dsm frame transmission time is ~1.4ms.
 * We expect to only be called when bytes arrive for processing,
 * and if an interval of more than 5ms passes between calls,
 * the first byte we read will be the first byte of a dsm frame.
 * In the case where byte(s) are dropped from a dsm frame, this also
 * provides a degree of protection. Of course, it would be better
 * if we didn't drop bytes...
 * Upon receiving a full dsm frame we attempt to decode it.
 *
 * @param[out] values pointer to per channel array of decoded values
 * @param[out] num_values pointer to number of raw channel values returned, high order bit 0:10 bit data, 1:11 bit data
 * @param[out] n_butes number of bytes read
 * @param[out] bytes pointer to the buffer of read bytes
 * @param[out] rssi value in percent, if supported, or 127
 * @return true=decoded raw channel values updated, false=no update
 */
bool
dsm_input(int fd, uint16_t *values, uint16_t *num_values, bool *dsm_11_bit, uint8_t *n_bytes, uint8_t **bytes,
	  int8_t *rssi,
	  unsigned max_values)
{
	int		ret = 1;
	hrt_abstime	now;

	/*
	 * The S.BUS protocol doesn't provide reliable framing,
	 * so we detect frame boundaries by the inter-frame delay.
	 *
	 * The minimum frame spacing is 7ms; with 25 bytes at 100000bps
	 * frame transmission time is ~2ms.
	 *
	 * We expect to only be called when bytes arrive for processing,
	 * and if an interval of more than 3ms passes between calls,
	 * the first byte we read will be the first byte of a frame.
	 *
	 * In the case where byte(s) are dropped from a frame, this also
	 * provides a degree of protection. Of course, it would be better
	 * if we didn't drop bytes...
	 */
	now = hrt_absolute_time();

	/*
	 * Fetch bytes, but no more than we would need to complete
	 * a complete frame.
	 */

	ret = read(fd, &dsm_buf[0], sizeof(dsm_buf) / sizeof(dsm_buf[0]));

	/* if the read failed for any reason, just give up here */
	if (ret < 1) {
		return false;

	} else {
		*n_bytes = ret;
		*bytes = &dsm_buf[0];
	}

	/*
	 * Try to decode something with what we got
	 */
	return dsm_parse(now, &dsm_buf[0], ret, values, num_values, dsm_11_bit, &dsm_frame_drops, rssi, max_values);
}

bool
dsm_parse(const uint64_t now, const uint8_t *frame, const unsigned len, uint16_t *values,
	  uint16_t *num_values, bool *dsm_11_bit, unsigned *frame_drops, int8_t *rssi_percent, uint16_t max_channels)
{

	/* this is set by the decoding state machine and will default to false
	 * once everything that was decodable has been decoded.
	 */
	bool decode_ret = false;

	/* ensure there can be no overflows */
	if (max_channels > sizeof(dsm_chan_buf) / sizeof(dsm_chan_buf[0])) {
		max_channels = sizeof(dsm_chan_buf) / sizeof(dsm_chan_buf[0]);
	}

	/* keep decoding until we have consumed the buffer */
	for (unsigned d = 0; d < len; d++) {

		/* overflow check */
		if (dsm_partial_frame_count == sizeof(dsm_frame) / sizeof(dsm_frame[0])) {
			dsm_partial_frame_count = 0;
			dsm_decode_state = DSM_DECODE_STATE_DESYNC;
#ifdef DSM_DEBUG
			printf("DSM: RESET (BUF LIM)\n");
#endif
		}

		if (dsm_partial_frame_count == DSM_FRAME_SIZE) {
			dsm_partial_frame_count = 0;
			dsm_decode_state = DSM_DECODE_STATE_DESYNC;
#ifdef DSM_DEBUG
			printf("DSM: RESET (PACKET LIM)\n");
#endif
		}

#ifdef DSM_DEBUG
#if 1
		printf("dsm state: %s%s, count: %d, val: %02x\n",
		       (dsm_decode_state == DSM_DECODE_STATE_DESYNC) ? "DSM_DECODE_STATE_DESYNC" : "",
		       (dsm_decode_state == DSM_DECODE_STATE_SYNC) ? "DSM_DECODE_STATE_SYNC" : "",
		       dsm_partial_frame_count,
		       (unsigned)frame[d]);
#endif
#endif

		switch (dsm_decode_state) {
		case DSM_DECODE_STATE_DESYNC:

			/* we are de-synced and only interested in the frame marker */
			if ((now - dsm_last_rx_time) > 5000) {
				dsm_decode_state = DSM_DECODE_STATE_SYNC;
				dsm_partial_frame_count = 0;
				dsm_chan_count = 0;
				dsm_frame[dsm_partial_frame_count++] = frame[d];
			}

			break;

		case DSM_DECODE_STATE_SYNC: {
				dsm_frame[dsm_partial_frame_count++] = frame[d];

				/* decode whatever we got and expect */
				if (dsm_partial_frame_count < DSM_FRAME_SIZE) {
					break;
				}

				/*
				 * Great, it looks like we might have a frame.  Go ahead and
				 * decode it.
				 */
				decode_ret = dsm_decode(now, &dsm_chan_buf[0], &dsm_chan_count, dsm_11_bit, max_channels, rssi_percent);

				/* we consumed the partial frame, reset */
				dsm_partial_frame_count = 0;

				/* if decoding failed, set proto to desync */
				if (!decode_ret) {
					dsm_decode_state = DSM_DECODE_STATE_DESYNC;
					dsm_frame_drops++;
				}
			}
			break;

		default:
#ifdef DSM_DEBUG
			printf("UNKNOWN PROTO STATE");
#endif
			decode_ret = false;
		}
	}

	if (frame_drops) {
		*frame_drops = dsm_frame_drops;
	}

	if (decode_ret) {
		*num_values = dsm_chan_count;

		memcpy(&values[0], &dsm_chan_buf[0], dsm_chan_count * sizeof(dsm_chan_buf[0]));
#ifdef DSM_DEBUG

		for (unsigned i = 0; i < dsm_chan_count; i++) {
			printf("dsm_decode: %u: %u\n", i, values[i]);
		}

#endif
	}

	dsm_last_rx_time = now;

	/* return false as default */
	return decode_ret;
}
