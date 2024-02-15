/*
 * Copyright (c) 2020-2021, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MINMEA_H
#define MINMEA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>

#define MINMEA_GPS_FLAG 0 //!< GPS flag parameter index
#define MINMEA_GLONASS_FLAG 1 //!< GLONASS flag parameter index
#define MINMEA_GALILEO_FLAG 2 //!< GALILEO flag parameter index
#define MINMEA_GALILEO_FULL_FLAG 3 //!< GALILEO FULL flag parameter index
#define MINMEA_BEIDOU_FLAG 4 //!< BEIDOU flag parameter index

#define MINMEA_GLL_FREQUENCY 0 //!< GLL frequency parameter index
#define MINMEA_RMC_FREQUENCY 1 //!< RMC frequency parameter index
#define MINMEA_VTG_FREQUENCY 2 //!< VTG frequency parameter index
#define MINMEA_GGA_FREQUENCY 3 //!< GGA frequency parameter index
#define MINMEA_GSA_FREQUENCY 4 //!< GSA frequency parameter index
#define MINMEA_GSV_FREQUENCY 5 //!< GSV frequency parameter index

#define MINMEA_NAVIGATION_MODE 0 //!< Navigation mode parameter index

#define MINMEA_STANDBY_MODE 0 //!< Standby mode parameter index

#define MINMEA_PMTK_MAX_LENGTH 100 //!< Maximal Pmtk packet length
#define MINMEA_PMTK_PACKET_DATA_MAX_LENGTH 96 //!< Max PMTK packet data length
#define MINMEA_MAX_LENGTH 120 //!< Maximal nmea packet length

enum minmea_sentence_id {
    MINMEA_INVALID = -1,
    MINMEA_UNKNOWN = 0,
    MINMEA_SENTENCE_RMC,
    MINMEA_SENTENCE_GGA,
    MINMEA_SENTENCE_GSA,
    MINMEA_SENTENCE_GLL,
    MINMEA_SENTENCE_GST,
    MINMEA_SENTENCE_GSV,
    MINMEA_SENTENCE_VTG,
    MINMEA_SENTENCE_ZDA,
    // MINMEA_SENTENCE_GPTXT, // Not implemented
    MINMEA_SENTENCE_PMTK_ACK
};

// MTK NMEA Packet Protocol (extension messages of the NMEA packet protocol)
enum minmea_pmtk_packet_type {
    MINMEA_PMTK_UNKNOWN = 0,
    MINMEA_PMTK_CMD_HOT_START = 101,
    MINMEA_PMTK_CMD_WARM_START = 102,
    MINMEA_PMTK_CMD_COLD_START = 103,
    MINMEA_PMTK_CMD_FULL_COLD_START = 104,
    MINMEA_PMTK_API_SET_POS_FIX = 220,
    MINMEA_PMTK_API_SET_PERIODIC_MODE = 225,
    MINMEA_PMTK_API_SET_NMEA_OUTPUT = 314,
    MINMEA_PMTK_API_SET_GNSS_SEARCH_MODE = 353,
    MINMEA_PMTK_FR_MODE = 886,
};

struct minmea_float {
    int_least32_t value;
    int_least32_t scale;
};

struct minmea_date {
    int day;
    int month;
    int year;
};

struct minmea_time {
    int hours;
    int minutes;
    int seconds;
    int microseconds;
};

struct minmea_sentence_pmtk {
    enum minmea_pmtk_packet_type type;
    char parameters[MINMEA_PMTK_PACKET_DATA_MAX_LENGTH];
    bool ack_expected;
    bool ack_received;
    bool result;
};

enum minmea_pmtk_flag {
    MINMEA_PMTK_FLAG_INVALID = 0,
    MINMEA_PMTK_FLAG_UNSUPPORTED = 1,
    MINMEA_PMTK_FLAG_FAILURE = 2,
    MINMEA_PMTK_FLAG_SUCCESS = 3,
};

struct minmea_sentence_pmtk_ack {
    enum minmea_pmtk_packet_type command;
    enum minmea_pmtk_flag flag;
};

struct minmea_sentence_rmc {
    struct minmea_time time;
    bool valid;
    struct minmea_float latitude;
    struct minmea_float longitude;
    struct minmea_float speed;
    struct minmea_float course;
    struct minmea_date date;
    struct minmea_float variation;
};

struct minmea_sentence_gga {
    struct minmea_time time;
    struct minmea_float latitude;
    struct minmea_float longitude;
    int fix_quality;
    int satellites_tracked;
    struct minmea_float hdop;
    struct minmea_float altitude;
    char altitude_units;
    struct minmea_float height;
    char height_units;
    struct minmea_float dgps_age;
};

enum minmea_gll_status {
    MINMEA_GLL_STATUS_DATA_VALID = 'A',
    MINMEA_GLL_STATUS_DATA_NOT_VALID = 'V',
};

// FAA mode added to some fields in NMEA 2.3.
enum minmea_faa_mode {
    MINMEA_FAA_MODE_AUTONOMOUS = 'A',
    MINMEA_FAA_MODE_DIFFERENTIAL = 'D',
    MINMEA_FAA_MODE_ESTIMATED = 'E',
    MINMEA_FAA_MODE_MANUAL = 'M',
    MINMEA_FAA_MODE_SIMULATED = 'S',
    MINMEA_FAA_MODE_NOT_VALID = 'N',
    MINMEA_FAA_MODE_PRECISE = 'P',
};

struct minmea_sentence_gll {
    struct minmea_float latitude;
    struct minmea_float longitude;
    struct minmea_time time;
    char status;
    char mode;
};

struct minmea_sentence_gst {
    struct minmea_time time;
    struct minmea_float rms_deviation;
    struct minmea_float semi_major_deviation;
    struct minmea_float semi_minor_deviation;
    struct minmea_float semi_major_orientation;
    struct minmea_float latitude_error_deviation;
    struct minmea_float longitude_error_deviation;
    struct minmea_float altitude_error_deviation;
};

enum minmea_gsa_mode {
    MINMEA_GPGSA_MODE_AUTO = 'A',
    MINMEA_GPGSA_MODE_FORCED = 'M',
};

enum minmea_gsa_fix_type {
    MINMEA_GPGSA_FIX_NONE = 1,
    MINMEA_GPGSA_FIX_2D = 2,
    MINMEA_GPGSA_FIX_3D = 3,
};

struct minmea_sentence_gsa {
    char mode;
    int fix_type;
    int sats[12];
    struct minmea_float pdop;
    struct minmea_float hdop;
    struct minmea_float vdop;
};

struct minmea_sat_info {
    int nr;
    int elevation;
    int azimuth;
    int snr;
};

struct minmea_sentence_gsv {
    int total_msgs;
    int msg_nr;
    int total_sats;
    struct minmea_sat_info sats[4];
};

struct minmea_sentence_vtg {
    struct minmea_float true_track_degrees;
    struct minmea_float magnetic_track_degrees;
    struct minmea_float speed_knots;
    struct minmea_float speed_kph;
    enum minmea_faa_mode faa_mode;
};

struct minmea_sentence_zda {
    struct minmea_time time;
    struct minmea_date date;
    int hour_offset;
    int minute_offset;
};

struct minmea_satellite_system {
    bool gps;
    bool glonass;
    bool galileo;
    bool galileo_full;
    bool beidou;
};

struct minmea_nmea_output {
    int gll_frequency;
    int rmc_frequency;
    int vtg_frequency;
    int gga_frequency;
    int gsa_frequency;
    int gsv_frequency;
};

/**
 * Calculate raw sentence checksum. Does not check sentence integrity.
 */
uint8_t minmea_checksum(const char *sentence);

/**
 * Check sentence validity and checksum. Returns true for valid sentences.
 */
bool minmea_check(const char *sentence, bool strict);

/**
 * Determine talker identifier.
 */
bool minmea_talker_id(char talker[3], const char *sentence);

/**
 * Determine sentence identifier.
 */
enum minmea_sentence_id minmea_sentence_id(const char *sentence, bool strict);

/**
 * Scanf-like processor for NMEA sentences. Supports the following formats:
 * c - single character (char *)
 * d - direction, returned as 1/-1, default 0 (int *)
 * f - fractional, returned as value + scale (int *, int *)
 * i - decimal, default zero (int *)
 * s - string (char *)
 * t - talker identifier and type (char *)
 * T - date/time stamp (int *, int *, int *)
 * Returns true on success. See library source code for details.
 */
bool minmea_scan(const char *sentence, const char *format, ...);

/*
 * Parse a specific type of sentence. Return true on success.
 */
bool minmea_parse_pmtk_ack(struct minmea_sentence_pmtk_ack *frame, const char *sentence);
bool minmea_parse_rmc(struct minmea_sentence_rmc *frame, const char *sentence);
bool minmea_parse_gga(struct minmea_sentence_gga *frame, const char *sentence);
bool minmea_parse_gsa(struct minmea_sentence_gsa *frame, const char *sentence);
bool minmea_parse_gll(struct minmea_sentence_gll *frame, const char *sentence);
bool minmea_parse_gst(struct minmea_sentence_gst *frame, const char *sentence);
bool minmea_parse_gsv(struct minmea_sentence_gsv *frame, const char *sentence);
bool minmea_parse_vtg(struct minmea_sentence_vtg *frame, const char *sentence);
bool minmea_parse_zda(struct minmea_sentence_zda *frame, const char *sentence);

/**
 * Serialize PMTK message from a Pmtk_message structure
 */
void minmea_serialize_pmtk(struct minmea_sentence_pmtk pmtk_message, char *message);

/**
 * Convert GPS UTC date/time representation to a UNIX timestamp.
 */
int minmea_gettime(
        struct timespec *ts, const struct minmea_date *date, const struct minmea_time *time_);

/**
 * Rescale a fixed-point value to a different scale. Rounds towards zero.
 */
static inline int_least32_t minmea_rescale(struct minmea_float *f, int_least32_t new_scale)
{
    if (f->scale == 0) {
        return 0;
    }
    if (f->scale == new_scale) {
        return f->value;
    }
    if (f->scale > new_scale) {
        return (f->value + ((f->value > 0) - (f->value < 0)) * f->scale / new_scale / 2)
                / (f->scale / new_scale);
    } else {
        return f->value * (new_scale / f->scale);
    }
}

/**
 * Convert a fixed-point value to a floating-point value.
 * Returns NaN for "unknown" values.
 */
static inline float minmea_tofloat(struct minmea_float *f)
{
    if (f->scale == 0) {
        return NAN;
    }
    return (float)f->value / (float)f->scale;
}

/**
 * Convert a raw coordinate to a floating point DD.DDD... value.
 * Returns NaN for "unknown" values.
 */
static inline float minmea_tocoord(struct minmea_float *f)
{
    if (f->scale == 0) {
        return NAN;
    }
    int_least32_t degrees = f->value / (f->scale * 100);
    int_least32_t minutes = f->value % (f->scale * 100);
    return (float)degrees + (float)minutes / (60 * f->scale);
}

#ifdef __cplusplus
}
#endif

#endif /* MINMEA_H */