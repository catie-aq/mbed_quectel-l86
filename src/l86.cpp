/*
 * Copyright (c) 2020-2021, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#include "l86.h"

namespace {
constexpr int LIMIT_SATELLITES = 4;                         //!< Max number of satellites in a view
}

L86::L86(BufferedSerial *uart)
{
    _registered_satellite_count = 0;
    _uart = uart;

    _position_informations.altitude = 0.0;
    _position_informations.latitude = 0.0;
    _position_informations.longitude = 0.0;

    _movement_informations.speed_kmh = 0.0;
    _movement_informations.speed_knots = 0.0;
    start_receive();
}

bool L86::set_satellite_system(SatelliteSystems satellite_systems)
{
    minmea_sentence_pmtk message = {
        MINMEA_PMTK_API_SET_GNSS_SEARCH_MODE, 0, true, false, false
    };
    sprintf(message.parameters, "%i,%i,%i,%i,%i",
            satellite_systems.test(static_cast<size_t>(SatelliteSystem::GPS)),
            satellite_systems.test(static_cast<size_t>(SatelliteSystem::GLONASS)),
            satellite_systems.test(static_cast<size_t>(SatelliteSystem::GALILEO)),
            satellite_systems.test(static_cast<size_t>(SatelliteSystem::GALILEO_FULL)),
            satellite_systems.test(static_cast<size_t>(SatelliteSystem::BEIDOU)));

    return generate_and_send_pmtk_message(message);
}

bool L86::set_nmea_output_frequency(NmeaCommands nmea_commands, NmeaFrequency frequency)
{
    minmea_sentence_pmtk message = {
        MINMEA_PMTK_API_SET_NMEA_OUTPUT, 0, true, false, false
    };
    sprintf(message.parameters, "%i,%i,%i,%i,%i,%i,0,0,0,0,0,0,0,0,0,0,0,0,0",
            nmea_commands.test(static_cast<size_t>(NmeaCommandType::GLL)) ? static_cast<int>(frequency) : 0,
            nmea_commands.test(static_cast<size_t>(NmeaCommandType::RMC)) ? static_cast<int>(frequency) : 0,
            nmea_commands.test(static_cast<size_t>(NmeaCommandType::VTG)) ? static_cast<int>(frequency) : 0,
            nmea_commands.test(static_cast<size_t>(NmeaCommandType::GGA)) ? static_cast<int>(frequency) : 0,
            nmea_commands.test(static_cast<size_t>(NmeaCommandType::GSA)) ? static_cast<int>(frequency) : 0,
            nmea_commands.test(static_cast<size_t>(NmeaCommandType::GSV)) ? static_cast<int>(frequency) : 0);

    return generate_and_send_pmtk_message(message);
}

bool L86::set_navigation_mode(NavigationMode navigation_mode)
{
    minmea_sentence_pmtk message = {
        MINMEA_PMTK_FR_MODE, 0, true, false, false
    };
    sprintf(message.parameters, "%i", static_cast<int>(navigation_mode));

    return generate_and_send_pmtk_message(message);
}

bool L86::set_position_fix_interval(uint16_t interval)
{
    minmea_sentence_pmtk message = {
        MINMEA_PMTK_API_SET_POS_FIX, 0, true, false, false
    };
    sprintf(message.parameters, "%u", interval);

    return generate_and_send_pmtk_message(message);
}

bool L86::start(StartMode start_mode)
{
    minmea_sentence_pmtk message = {
        MINMEA_PMTK_UNKNOWN, 0, false, false, false
    };
    switch (start_mode) {
        case StartMode::FULL_COLD_START: {
            message.type = MINMEA_PMTK_CMD_FULL_COLD_START;
            break;
        }
        case StartMode::COLD_START: {
            message.type = MINMEA_PMTK_CMD_COLD_START;
            break;
        }
        case StartMode::WARM_START: {
            message.type = MINMEA_PMTK_CMD_WARM_START;
            break;
        }
        case StartMode::HOT_START: {
            message.type = MINMEA_PMTK_CMD_HOT_START;
            break;
        }
    }

    return generate_and_send_pmtk_message(message);
}

bool L86::standby_mode(StandbyMode standby_mode)
{
    minmea_sentence_pmtk message = {
        MINMEA_PMTK_API_SET_PERIODIC_MODE, 0, true, false, false
    };
    sprintf(message.parameters, "%i", static_cast<int>(standby_mode));

    return generate_and_send_pmtk_message(message);
}

L86::Satellite *L86::satellites()
{
    return _satellites_informations.satellites;
}

double L86::latitude()
{
    return _position_informations.latitude;
}

double L86::longitude()
{
    return _position_informations.longitude;
}

double L86::altitude()
{
    return _position_informations.altitude;
}

double L86::speed(L86::SpeedUnit unit)
{
    if (unit == SpeedUnit::KMH) {
        return _movement_informations.speed_kmh;
    } else {
        return _movement_informations.speed_knots;
    }
}

double L86::speed()
{
    if (MBED_CONF_L86_SPEED_UNIT == SpeedUnit::KMH) {
        return _movement_informations.speed_kmh;
    } else {
        return _movement_informations.speed_knots;
    }
}

time_t L86::time()
{
    return mktime(&_global_informations.time);
}

L86::PositionningMode L86::positionning_mode()
{
    return _global_informations.positionning_mode;
}

L86::FixStatusGGA L86::fix_status()
{
    return _global_informations.fix_status;
}

L86::FixStatusGSA L86::fix_satellite_status()
{
    return _satellites_informations.status;
}

int L86::satellite_count()
{
    return _satellites_informations.satellite_count;
}

L86::Mode L86::mode()
{
    return _satellites_informations.mode;
}

L86::DilutionOfPrecision L86::dilution_of_precision()
{
    return _dilution_of_precision;
}

int L86::registered_satellite_count()
{
    return _registered_satellite_count;
}

bool L86::generate_and_send_pmtk_message(minmea_sentence_pmtk message)
{
    char buffer[MINMEA_PMTK_MAX_LENGTH];
    minmea_serialize_pmtk(message, (char *)buffer);
    _current_pmtk_message = message;

    for (int i = 0 ; i < 5 && _current_pmtk_message.result == false ; i++) {
        _uart->write((uint8_t *)buffer, strlen(buffer));
        if (!_current_pmtk_message.ack_expected) {
            return true;
        }
        for (int j = 0 ; j < 3 && _current_pmtk_message.ack_received == false; j++) {
            ThisThread::sleep_for(150ms);
        }
    }
    return _current_pmtk_message.result;
}

void L86::get_received_message()
{
    static int message_len = 0;
    while (_uart->readable()) {
        if (message_len >= MINMEA_MAX_LENGTH - 1) {
            message_len = 0;
        }
        _uart->read(&_received_message[message_len++], 1);
        if (_received_message[message_len - 1] == '\n') {
            // Completed message received
            _received_message[message_len] = '\0';
            parse_message(_received_message);
            message_len = 0;
        }
    }
}

void L86::parse_message(char *message)
{
    int limit = LIMIT_SATELLITES;
    switch (minmea_sentence_id(message, false)) {
        case MINMEA_SENTENCE_PMTK_ACK:
            struct minmea_sentence_pmtk_ack pmtk_ack_frame;
            if (minmea_parse_pmtk_ack(&pmtk_ack_frame, message)) {
                
                if (_current_pmtk_message.type == pmtk_ack_frame.command) {
                    _current_pmtk_message.ack_received = true;
                    if (pmtk_ack_frame.flag == MINMEA_PMTK_FLAG_SUCCESS) {
                        _current_pmtk_message.result = true;
                    } else {
                        _current_pmtk_message.result = false;
                    }
                }
            }
            break;

        case MINMEA_SENTENCE_RMC:
            struct minmea_sentence_rmc rmc_frame;
            if (minmea_parse_rmc(&rmc_frame, message)) {
                if (rmc_frame.valid) {
                    set_date(rmc_frame.date);
                    set_time(rmc_frame.time);
                    set_latitude(rmc_frame.latitude);
                    set_longitude(rmc_frame.longitude);
                    _movement_informations.speed_knots = minmea_tofloat(&rmc_frame.speed);
                    _movement_informations.course_over_ground = minmea_tofloat(&rmc_frame.course);
                    _position_informations.magnetic_variation = minmea_tofloat(&rmc_frame.variation);
                }
            }
            break;

        case MINMEA_SENTENCE_VTG:
            struct minmea_sentence_vtg vtg_frame;
            if (minmea_parse_vtg(&vtg_frame, message)) {
                _movement_informations.speed_knots = minmea_tofloat(&vtg_frame.speed_knots);
                _movement_informations.speed_kmh = minmea_tofloat(&vtg_frame.speed_kph);
                set_positionning_mode(vtg_frame.faa_mode);
            }
            break;

        case MINMEA_SENTENCE_GGA:
            struct minmea_sentence_gga gga_frame;
            if (minmea_parse_gga(&gga_frame, message)) {
                set_time(gga_frame.time);
                set_latitude(gga_frame.latitude);
                set_longitude(gga_frame.longitude);
                _position_informations.altitude = minmea_tofloat(&gga_frame.altitude);
                _satellites_informations.satellite_count = gga_frame.satellites_tracked;
                set_fix_status(gga_frame.fix_quality);
                _dilution_of_precision.horizontal = minmea_tofloat(&gga_frame.hdop);
            }
            break;

        case MINMEA_SENTENCE_GSA:
            struct minmea_sentence_gsa gsa_frame;
            if (minmea_parse_gsa(&gsa_frame, message)) {
                set_fix_satellite_status(gsa_frame.fix_type);
                set_mode(gsa_frame.mode);
                _dilution_of_precision.horizontal = minmea_tofloat(&gsa_frame.hdop);
                _dilution_of_precision.positional = minmea_tofloat(&gsa_frame.pdop);
                _dilution_of_precision.vertical = minmea_tofloat(&gsa_frame.vdop);
                for (int i = 0 ; i < MAX_SATELLITES ; i++) {
                    _satellites_informations.satellites[i].id = gsa_frame.sats[i];
                }
            }
            break;

        case MINMEA_SENTENCE_GSV:
            struct minmea_sentence_gsv gsv_frame;
            if (minmea_parse_gsv(&gsv_frame, message)) {
                // last sequence message
                if (gsv_frame.msg_nr == gsv_frame.total_msgs) {
                    limit = MAX_SATELLITES - gsv_frame.total_sats;
                }
                // reset satellites if first sequence number
                if (gsv_frame.msg_nr == 1) {
                    _registered_satellite_count = 0;
                }
                for (int i = 0 ; i <= limit ; i++) {
                    Satellite satellite;
                    satellite.id = (uint16_t)gsv_frame.sats[i].nr;
                    satellite.elevation = (uint16_t)gsv_frame.sats[i].elevation;
                    satellite.azimuth = (uint16_t)gsv_frame.sats[i].azimuth;
                    satellite.snr = (uint16_t)gsv_frame.sats[i].snr;
                    _satellites_informations.satellites[_registered_satellite_count++] = satellite;
                }
                _satellites_informations.satellite_count = gsv_frame.total_sats;
            }
            break;

        case MINMEA_SENTENCE_GLL:
            struct minmea_sentence_gll gll_frame;
            if (minmea_parse_gll(&gll_frame, message)) {
                if (gll_frame.status == 'A') {
                    set_time(gll_frame.time);
                    set_positionning_mode(gll_frame.mode);
                    set_latitude(gll_frame.latitude);
                    set_longitude(gll_frame.longitude);
                }
            }
            break;

        default: 
            // printf("\n -- Unknown command --> %s\n", message);
            break;
    }
}

void L86::start_receive()
{
    _uart->sigio(mbed_event_queue()->event(callback(this, &L86::get_received_message)));
}

void L86::stop_receive()
{
    _uart->sigio(NULL);
}

void L86::set_positionning_mode(char c_positionning_mode)
{
    switch (c_positionning_mode) {
        case 'N':
            _global_informations.positionning_mode = PositionningMode::NO_FIX;
            break;
        case 'A':
            _global_informations.positionning_mode = PositionningMode::AUTONOMOUS_GNSS_FIX;
            break;
        case 'D':
            _global_informations.positionning_mode = PositionningMode::DIFFERENTIAL_GNSS_FIX;
            break;
        default:
            _global_informations.positionning_mode = PositionningMode::UNKNOWN;
    }
}

void L86::set_fix_status(int c_fix_status)
{
    switch (c_fix_status) {
        case 0:
            _global_informations.fix_status = FixStatusGGA::INVALID;
            break;
        case 1:
            _global_informations.fix_status = FixStatusGGA::GNSS_FIX;
            break;
        case 2:
            _global_informations.fix_status = FixStatusGGA::DGPS_FIX;
            break;
        case 6:
            _global_informations.fix_status = FixStatusGGA::ESTIMATED_MODE;
            break;
        default:
            _global_informations.fix_status = FixStatusGGA::UNKNOWN;
    }
}

void L86::set_fix_satellite_status(int c_fix_satellite_status)
{
    switch (c_fix_satellite_status) {
        case 1:
            _satellites_informations.status = FixStatusGSA::NOFIX;
            break;
        case 2:
            _satellites_informations.status = FixStatusGSA::FIX2D;
            break;
        case 3:
            _satellites_informations.status = FixStatusGSA::FIX3D;
            break;
        default:
            _satellites_informations.status = FixStatusGSA::UNKNOWN;
    }
}

void L86::set_mode(char c_mode)
{
    switch (c_mode) {
        case 'M':
            _satellites_informations.mode = Mode::MANUAL_SWITCH;
            break;
        case 'A':
            _satellites_informations.mode = Mode::AUTOMATIC_SWITCH;
            break;
        default:
            _satellites_informations.mode = Mode::UNKNOWN;
    }
}

void L86::set_time(struct minmea_time time)
{
    _global_informations.time.tm_hour = time.hours;
    _global_informations.time.tm_min = time.minutes;
    _global_informations.time.tm_sec = time.seconds;
}

void L86::set_date(struct minmea_date date)
{
    _global_informations.time.tm_mday = date.day;
    _global_informations.time.tm_mon = date.month;
    _global_informations.time.tm_year = date.year;
}

void L86::set_longitude(minmea_float longitude)
{
    _position_informations.longitude = minmea_tocoord(&longitude);
}

void L86::set_latitude(minmea_float latitude)
{
    _position_informations.latitude = minmea_tocoord(&latitude);
}
