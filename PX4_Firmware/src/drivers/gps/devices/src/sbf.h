/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file sbf.h
 *
 * Septentrio protocol as defined in PPSDK SBF Reference Guide 4.1.8
 *
 * @author Matej Frančeškin <Matej.Franceskin@gmail.com>
 *
 */

#pragma once

#include "gps_helper.h"
#include "base_station.h"
#include "rtcm.h"
#include "../../definitions.h"

#define SBF_CONFIG_FORCE_INPUT "SSSSSSSSSS\n"

#define SBF_CONFIG_BAUDRATE "setCOMSettings, COM1, baud%d\n"

#define SBF_CONFIG_RESET "setSBFOutput, all, COM1, none, off\n"

#define SBF_CONFIG_RECEIVER_DYNAMICS "setReceiverDynamics, %s, UAV\n"

#define SBF_TX_CFG_PRT_BAUDRATE 115200

#define SBF_CONFIG "" \
	"setDataInOut, COM1, Auto, SBF\n" \
	"setPVTMode, Rover, All, auto\n" \
	"setSatelliteTracking, All\n" \
	"setSatelliteUsage, All\n" \
	"setElevationMask, All, 10\n" \
	"setSBFOutput, Stream1, DSK1, Support, msec100\n" \
	"setSBFOutput, Stream2, Dsk1, Event+Comment, OnChange\n" \
	"setSBFOutput, Stream3, COM1, DOP+VelCovGeodetic, sec1\n" \
	"setSBFOutput, Stream4, COM1, PVTGeodetic, msec100\n" \
	"setFileNaming, DSK1, Incremental\n" \
	"setFileNaming, DSK1, , 'px4'\n"

#define SBF_CONFIG_RTCM "" \
	"setDataInOut, USB1, Auto, RTCMv3+SBF\n" \
	"setPVTMode, Rover, All, auto\n" \
	"setSatelliteTracking, All\n" \
	"setSatelliteUsage, All\n" \
	"setElevationMask, All, 10\n" \
	"setReceiverDynamics, Moderate, Automotive\n" \
	"setSBFOutput, Stream1, DSK1, Support, msec100\n" \
	"setSBFOutput, Stream2, USB1, DOP+VelCovGeodetic, sec1\n" \
	"setSBFOutput, Stream3, USB1, PVTGeodetic, msec200\n" \
	"setFileNaming, DSK1, Incremental\n" \
	"setFileNaming, DSK1, , 'px4rtcm'\n"

#define SBF_CONFIG_RTCM_STATIC1 "" \
	"setReceiverDynamics, Low, Static\n"

#define SBF_CONFIG_RTCM_STATIC2 "" \
	"setPVTMode, Static, , Geodetic1\n"

#define SBF_CONFIG_RTCM_STATIC_COORDINATES "" \
	"setStaticPosGeodetic, Geodetic1, %f, %f, %f\n"

#define SBF_CONFIG_RTCM_STATIC_OFFSET "" \
	"setAntennaOffset, Main, %f, %f, %f\n"

#define SBF_CONFIG_RESET_HOT "" \
	SBF_CONFIG_FORCE_INPUT"ExeResetReceiver, soft, none\n"

#define SBF_CONFIG_RESET_WARM "" \
	SBF_CONFIG_FORCE_INPUT"ExeResetReceiver, soft, PVTData\n"

#define SBF_CONFIG_RESET_COLD "" \
	SBF_CONFIG_FORCE_INPUT"ExeResetReceiver, hard, SatData\n"

#define SBF_SYNC1 0x24
#define SBF_SYNC2 0x40

/* Block IDs */
#define SBF_ID_DOP            4001
#define SBF_ID_PVTGeodetic    4007
#define SBF_ID_ChannelStatus  4013
#define SBF_ID_VelCovGeodetic 5908

/*** SBF protocol binary message and payload definitions ***/
#pragma pack(push, 1)

typedef struct {
	uint8_t  mode_type: 4;       /**< Bit ﬁeld indicating the PVT mode type, as follows:
                                     0: No PVT available (the Error ﬁeld indicates the cause of the absence of the PVT solution)
                                     1: Stand-Alone PVT
                                     2: Differential PVT
                                     3: Fixed location
                                     4: RTK with ﬁxed ambiguities
                                     5: RTK with ﬂoat ambiguities
                                     6: SBAS aided PVT
                                     7: moving-base RTK with ﬁxed ambiguities
                                     8: moving-base RTK with ﬂoat ambiguities
                                     10:Precise Point Positioning (PPP) */
	uint8_t  mode_reserved: 2;   /**< Reserved */
	uint8_t  mode_base_fixed: 1; /**< Set if the user has entered the command setPVTMode,base,auto and the receiver
                                     is still in the process of determining its ﬁxed position. */
	uint8_t  mode_2d: 1;        /**< 2D/3D ﬂag: set in 2D mode(height assumed constant and not computed). */
	uint8_t  error;             /**< PVT error code. The following values are deﬁned:
                                     0: No Error
                                     1: Not enough measurements
                                     2: Not enough ephemerides available
                                     3: DOP too large (larger than 15)
                                     4: Sum of squared residuals too large
                                     5: No convergence
                                     6: Not enough measurements after outlier rejection
                                     7: Position output prohibited due to export laws
                                     8: Not enough differential corrections available
                                     9: Base station coordinates unavailable
                                     10:Ambiguities not ﬁxed and user requested to only output RTK-ﬁxed positions
                                     Note: if this ﬁeld has a non-zero value, all following ﬁelds are set to their Do-Not-Use value. */
	double   latitude;          /**< Marker latitude, from −π/2 to +π/2, positive North of Equator */
	double   longitude;         /**< Marker longitude, from −π to +π, positive East of Greenwich */
	double   height;            /**< Marker ellipsoidal height (with respect to the ellipsoid speciﬁed by Datum) */
	float    undulation;        /**< Geoid undulation computed from the global geoid model deﬁned in
                                     the document ’Technical Characteristics of the NAVSTAR GPS, NATO, June 1991’ */
	float    vn;                /**< Velocity in the North direction */
	float    ve;                /**< Velocity in the East direction */
	float    vu;                /**< Velocity in the Up direction */
	float    cog;               /**< Course over ground: this is deﬁned as the angle of the vehicle with respect
                                     to the local level North, ranging from 0 to 360, and increasing towards east.
                                     Set to the do-not-use value when the speed is lower than 0.1m/s. */
	double   rx_clk_bias;       /**< Receiver clock bias relative to system time reported in the Time System ﬁeld.
                                     To transfer the receiver time to the system time, use: tGPS/GST=trx-RxClkBias */
	float    RxClkDrift;        /**< Receiver clock drift relative to system time (relative frequency error) */
	uint8_t  time_system;       /**< Time system of which the offset is provided in this sub-block:
                                     0:GPStime
                                     1:Galileotime
                                     3:GLONASStime */
	uint8_t  datum;             /**< This ﬁeld deﬁnes in which datum the coordinates are expressed:
                                     0: WGS84/ITRS
                                     19: Datum equal to that used by the DGNSS/RTK basestation
                                     30: ETRS89(ETRF2000 realization)
                                     31: NAD83(2011), North American Datum(2011)
                                     32: NAD83(PA11), North American Datum, Paciﬁcplate (2011)
                                     33: NAD83(MA11), North American Datum, Marianas plate(2011)
                                     34: GDA94(2010), Geocentric Datum of Australia (2010)
                                     250:First user-deﬁned datum
                                     251:Second user-deﬁned datum */
	uint8_t  nr_sv;             /**< Total number of satellites used in the PVT computation. */
	uint8_t  wa_corr_info;      /**< Bit ﬁeld providing information about which wide area corrections have been applied:
                                     Bit 0: set if orbit and satellite clock correction information is used
                                     Bit 1: set if range correction information is used
                                     Bit 2: set if ionospheric information is used
                                     Bit 3: set if orbit accuracy information is used(UERE/SISA)
                                     Bit 4: set if DO229 Precision Approach mode is active
                                     Bits 5-7: Reserved */
	uint16_t reference_id;      /**< In case of DGPS or RTK operation, this ﬁeld is to be interpreted as the base station identiﬁer.
                                     In SBAS operation, this ﬁeld is to be interpreted as the PRN of the geostationary satellite
                                     used (from 120 to 158). If multiple base stations or multiple geostationary satellites are used
                                     the value is set to 65534.*/
	uint16_t mean_corr_age;     /**< In case of DGPS or RTK, this ﬁeld is the mean age of the differential corrections.
                                     In case of SBAS operation, this ﬁeld is the mean age of the ’fast corrections’
                                     provided by the SBAS satellites */
	uint32_t signal_info;       /**< Bit ﬁeld indicating the type of GNSS signals having been used in the PVT computations.
                                     If a bit i is set, the signal type having index i has been used. */
	uint8_t alert_flag;         /**< Bit ﬁeld indicating integrity related information */

	// Revision 1
	uint8_t nr_bases;
	uint16_t ppp_info;
	// Revision 2
	uint16_t latency;
	uint16_t h_accuracy;
	uint16_t v_accuracy;
} sbf_payload_pvt_geodetic_t;

typedef struct {
	uint8_t  mode_type: 4;       /**< Bit ﬁeld indicating the PVT mode type, as follows:
                                     0: No PVT available (the Error ﬁeld indicates the cause of the absence of the PVT solution)
                                     1: Stand-Alone PVT
                                     2: Differential PVT
                                     3: Fixed location
                                     4: RTK with ﬁxed ambiguities
                                     5: RTK with ﬂoat ambiguities
                                     6: SBAS aided PVT
                                     7: moving-base RTK with ﬁxed ambiguities
                                     8: moving-base RTK with ﬂoat ambiguities
                                     10:Precise Point Positioning (PPP) */
	uint8_t  mode_reserved: 2;  /**< Reserved */
	uint8_t  mode_base_fixed: 1;/**< Set if the user has entered the command setPVTMode,base,auto and the receiver
                                     is still in the process of determining its ﬁxed position. */
	uint8_t  mode_2d: 1;        /**< 2D/3D ﬂag: set in 2D mode(height assumed constant and not computed). */
	uint8_t  error;             /**< PVT error code. The following values are deﬁned:
                                     0: No Error
                                     1: Not enough measurements
                                     2: Not enough ephemerides available
                                     3: DOP too large (larger than 15)
                                     4: Sum of squared residuals too large
                                     5: No convergence
                                     6: Not enough measurements after outlier rejection
                                     7: Position output prohibited due to export laws
                                     8: Not enough differential corrections available
                                     9: Base station coordinates unavailable
                                     10:Ambiguities not ﬁxed and user requested to only output RTK-ﬁxed positions
                                     Note: if this ﬁeld has a non-zero value, all following ﬁelds are set to their Do-Not-Use value. */
	float cov_vn_vn;            /**< Variance of the north-velocity estimate */
	float cov_ve_ve;            /**< Variance of the east-velocity estimate */
	float cov_vu_vu;            /**< Variance of the up - velocity estimate */
	float cov_dt_dt;            /**< Variance of the clock drift estimate */
	float cov_vn_ve;			/**< Covariance between the north - and east - velocity estimates */
	float cov_vn_vu;			/**< Covariance between the north - and up - velocity estimates */
	float cov_vn_dt;            /**< Covariance between the north - velocity and clock drift estimates */
	float cov_ve_vu;            /**< Covariance between the east - and up - velocity estimates */
	float cov_ve_dt;            /**< Covariance between the east - velocity and clock drift estimates */
	float cov_vu_dt;            /**< Covariance between the up - velocity and clock drift estimates */
} sbf_payload_vel_cov_geodetic_t;

typedef struct {
	uint8_t nr_sv;              /**< Total number of satellites used in the PVT computation. */
	uint8_t reserved;
	uint16_t pDOP;
	uint16_t tDOP;
	uint16_t hDOP;
	uint16_t vDOP;
	float hpl;                  /**< Horizontal Protection Level (see the DO229 standard). */
	float vpl;                  /**< Vertical Protection Level (see the DO229 standard). */
} sbf_payload_dop_t;

typedef struct {
	uint8_t antenna;
	uint8_t reserved;
	uint16_t tracking_status;
	uint16_t pvt_status;
	uint16_t pvt_info;
} sbf_payload_channel_state_info_t;

/* General message and payload buffer union */

typedef struct {
	uint16_t sync;              /** The Sync field is a 2-byte array always set to 0x24, 0x40. The first byte of every SBF block has
									hexadecimal value 24 (decimal 36, ASCII ’$’). The second byte of every SBF block has hexadecimal
									value 40 (decimal 64, ASCII ’@’). */
	uint16_t crc16;				/** The CRC field is the 16-bit CRC of all the bytes in an SBF block from and including the ID field
									to the last byte of the block. The generator polynomial for this CRC is the so-called CRC-CCITT
									polynomial: x 16 + x 12 + x 5 + x 0 . The CRC is computed in the forward direction using a seed of 0, no
									reverse and no final XOR. */
uint16_t msg_id:
	13;        /** The ID ﬁeld is a 2-byte block ID, which uniquely identiﬁes the block type and its contents */
uint8_t msg_revision:
	3;    /** block revision number, starting from 0 at the initial block deﬁnition, and incrementing
                                    each time backwards - compatible changes are performed to the block  */
	uint16_t length;            /** The Length ﬁeld is a 2-byte unsigned integer containing the size of the SBF block.
                                    It is the total number of bytes in the SBF block including the header.
                                    It is always a multiple of 4. */
	uint32_t TOW;               /**< Time-Of-Week: Time-tag, expressed in whole milliseconds from
                                     the beginning of the current Galileo/GPSweek. */
	uint16_t WNc;               /**< The GPS week number associated with the TOW. WNc is a continuous
                                     weekcount (hence the "c"). It is not affected by GPS week roll overs,
                                     which occur every 1024 weeks. By deﬁnition of the Galileo system time,
                                     WNc is also the Galileo week number + 1024. */
	union {
		sbf_payload_pvt_geodetic_t  payload_pvt_geodetic;
		sbf_payload_vel_cov_geodetic_t payload_vel_col_geodetic;
		sbf_payload_dop_t payload_dop;
	};

	uint8_t padding[16];
} sbf_buf_t;

#pragma pack(pop)
/*** END OF SBF protocol binary message and payload definitions ***/

/* Decoder state */
typedef enum {
	SBF_DECODE_SYNC1 = 0,
	SBF_DECODE_SYNC2,
	SBF_DECODE_PAYLOAD,
	SBF_DECODE_RTCM3
} sbf_decode_state_t;

class GPSDriverSBF : public GPSBaseStationSupport
{
public:
	GPSDriverSBF(GPSCallbackPtr callback, void *callback_user,
		     struct vehicle_gps_position_s *gps_position,
		     struct satellite_info_s *satellite_info,
		     uint8_t dynamic_model);

	virtual ~GPSDriverSBF() override;

	int receive(unsigned timeout) override;
	int configure(unsigned &baudrate, OutputMode output_mode) override;
	int reset(GPSRestartType restart_type) override;

private:

	/**
	 * @brief Parse the binary SBF packet
	 */
	int parseChar(const uint8_t b);

	/**
	 * @brief Add payload rx byte
	 */
	int payloadRxAdd(const uint8_t b);

	/**
	 * @brief Finish payload rx
	 */
	int payloadRxDone(void);

	/**
	 * @brief Reset the parse state machine for a fresh start
	 */
	void decodeInit(void);

	/**
	 * @brief Send a message
	 * @return true on success, false on write error (errno set)
	 */
	bool sendMessage(const char *msg);

	/**
	 * @brief Send a message and waits for acknowledge
	 * @return true on success, false on write error (errno set) or ack wait timeout
	 */
	bool sendMessageAndWaitForAck(const char *msg, const int timeout);

	struct vehicle_gps_position_s *_gps_position { nullptr };
	struct satellite_info_s *_satellite_info { nullptr };
	uint8_t _dynamic_model{ 7 };
	uint64_t _last_timestamp_time { 0 };
	bool _configured { false };
	uint8_t _msg_status { 0 };
	sbf_decode_state_t _decode_state { SBF_DECODE_SYNC1 };
	uint16_t _rx_payload_index { 0 };
	sbf_buf_t _buf;
	OutputMode _output_mode { OutputMode::GPS };
	RTCMParsing	*_rtcm_parsing { nullptr };
};

uint16_t crc16(const uint8_t *buf, uint32_t len);

