#include "l86.h"


namespace {

constexpr int PARAMETERS_COUNT_SATELLITE_SYSTEM = 5;        //!< Number of parameters to set satellite system
constexpr char SATELLITE_SYSTEM_CODE[] = "353";             //!< Satellite system command code
constexpr int GPS_FLAG = 0;                                 //!< GPS flag parameter index
constexpr int GLONASS_FLAG = 1;                             //!< GLONASS flag parameter index
constexpr int GALILEO_FLAG = 2;                             //!< GALILEO flag parameter index
constexpr int GALILEO_FULL_FLAG = 3;                        //!< GALILEO FULL flag parameter index
constexpr int BEIDOU_FLAG = 4;                              //!< BEIDOU flag parameter index

constexpr int PARAMETERS_COUNT_NMEA_OUTPUT_FREQUENCY = 5;   //!< Number of parameters to set nmea ouput frequency
constexpr char NMEA_OUTPUT_FREQUENCY_CODE[] = "314";        //!< Nmea output frequency command code
constexpr int GLL_FREQUENCY = 0;                            //!< GLL frequency parameter index
constexpr int RMC_FREQUENCY = 1;                            //!< RMC frequency parameter index
constexpr int VTG_FREQUENCY = 2;                            //!< VTG frequency parameter index
constexpr int GGA_FREQUENCY = 3;                            //!< GGA frequency parameter index
constexpr int GSA_FREQUENCY = 4;                            //!< GSA frequency parameter index
constexpr int GSV_FREQUENCY = 5;                            //!< GSV frequency parameter index

constexpr int PARAMETERS_COUNT_NAVIGATION_MODE = 1;         //!< Number of parameters to set navigation mode
constexpr char NAVIGATION_MODE_CODE[] = "886";              //!< Navigation mode command
constexpr int NAVIGATION_MODE = 0;                          //!< Navigation mode parameter index

constexpr int PARAMETERS_COUNT_POSITION_FIX_INTERVAL = 1;   //!< Number of parameters to set position fix interval
constexpr char POSITION_FIX_INTERVAL_CODE[] = "220";        //!< Position fix interval command code
constexpr int INTERVAL = 0;                                 //!< Position fix interval parameter index

constexpr char FULL_COLD_START_MODE_CODE[] = "104";         //!< Full cold start mode command code
constexpr char COLD_START_MODE_CODE[] = "103";              //!< Cold start mode command code
constexpr char WARM_START_MODE_CODE[] = "102";              //!< Warm start mode command code
constexpr char HOT_START_MODE_CODE[] = "101";               //!< Hot start mode command code

constexpr int PARAMETERS_COUNT_STANDBY_MODE = 1;            //!< Number of parameters to set standby mode
constexpr int STANDBY_MODE = 0;                             //!< Standby mode parameter index
constexpr int DEFAULT_PARAMETERS_COUNT_STANDBY_MODE = 6;    //!< Number of default parameters which will allways initialized to 0

constexpr int PARAMETERS_BEGIN = 7;                         //!< Parameters begin index in received messages
constexpr int LIMIT_SATELLITES = 4;                         //!< Max number of satellites in a view
constexpr char ACK_CODE[] = "001";                          //!< Ack command code

constexpr int PMTK_COMMAND_CODE_INDEX = 9;                  //!< Index of pmtk command code first character
constexpr int PMTK_COMMAND_RESULT = 13;                     //!< Pmtk command result index
constexpr int PMTK_ANSWER_SIZE = 50;                        //!< Pmtk received message size
constexpr int PMTK_PACKET_TYPE_INDEX = 5;                   //!< Pmtk received message command code index
constexpr char INVALID_PACKET = '0';                        //!< Invalid packet code
constexpr char UNSUPPORTED_PACKET_TYPE = '1';               //!< Unsupported packet code
constexpr char VALID_PACKET_AND_ACTION_FAILED = '2';        //!< Valid packet but action failed code
constexpr char VALID_PACKET_AND_COMMAND_SUCCEED = '3';      //!< Valid packet and command succed code
constexpr int CHECKSUM_LEN = 2;                             //!< Checksum length
constexpr int FRAME_END_LEN = 3;                            //!< Received message right shift to access to the checksum

constexpr int RMC_POSITIONNING_MODE = 11;                   //!< Positionning mode information index in RMC messages
constexpr int RMC_DATE = 8;                                 //!< Date information index in RMC messages
constexpr int RMC_TIME = 0;                                 //!< Time information index in RMC messages
constexpr int RMC_LATITUDE = 2;                             //!< Latitude information index in RMC messages
constexpr int RMC_LATITUDE_N_S = 3;                         //!< Latitude N/S information index in RMC messages
constexpr int RMC_LONGITUDE = 4;                            //!< Longitude information index in RMC messages
constexpr int RMC_LONGITUDE_E_W = 5;                        //!< Longitude E/W information index in RMC messages
constexpr int RMC_SPEED_KNOTS = 6;                          //!< Speed in knots information index in RMC messages

constexpr int VTG_POSITIONNING_MODE = 8;                    //!< Positionning mode information index in VTG messages
constexpr int VTG_SPEED_KNOTS = 4;                          //!< Speed in knots information index in VTG messages
constexpr int VTG_SPEED_KMH = 6;                            //!< Speed in km/h information index in VTG messages

constexpr int GGA_FIX_STATUS = 5;                           //!< Fixed status information index in GGA messages
constexpr int GGA_TIME = 0;                                 //!< Time information index in GGA messages
constexpr int GGA_LATITUDE = 1;                             //!< Latitude information index in GGA messages
constexpr int GGA_LATITUDE_N_S = 2;                         //!< Latitude N/S information index in GGA messages
constexpr int GGA_LONGITUDE = 3;                            //!< Longitude information index in GGA messages
constexpr int GGA_LONGITUDE_E_W = 4;                        //!< Longitude E/W information index in GGA messages
constexpr int GGA_ALTITUDE = 8;                             //!< Altitude information index in GGA messages
constexpr int GGA_SATELLITE_COUNT = 6;                      //!< Satellites count information index in GGA messages

constexpr int GSA_FIX_SATELLITE_STATUS = 1;                 //!< Fixed satellite status information index in GSA messages
constexpr int GSA_MODE = 0;                                 //!< Mode information index in GSA messages
constexpr int GSA_DILUTION_OF_PRECISION_HORIZONTAL = 15;    //!< Horizontal dilution of precision information index in GSA messages
constexpr int GSA_DILUTION_OF_PRECISION_POSITIONAL = 14;    //!< Positional dilution of precision information index in GSA messages
constexpr int GSA_DILUTION_OF_PRECISION_VERTICAL = 15;      //!< Vertical dilution of precision information index in GSA messages

constexpr int GSV_MESSAGES_COUNT = 0;                       //!< Messages count information index in GSV messages
constexpr int GSV_SEQUENCE_NUMBER = 1;                      //!< Sequence number information index in GSV messages
constexpr int GSV_SATELLITES_COUNT = 0;                     //!< Satellites count information index in GSV messages

constexpr int GLL_TIME = 4;                                 //!< Time information index in GLL messages
constexpr int GLL_POSITIONNING_MODE = 6;                    //!< Positionning mode information index in GLL messages
constexpr int GLL_LATITUDE = 0;                             //!< Latitude information index in GLL messages
constexpr int GLL_LATITUDE_N_S = 1;                         //!< Latitude N/S information index in GLL messages
constexpr int GLL_LONGITUDE = 2;                            //!< Longitude information index in GLL messages
constexpr int GLL_LONGITUDE_E_W = 3;                        //!< Longitude E/W information index in GLL messages

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
    minmea_sentence_pmtk message;
    message.type[0] = SATELLITE_SYSTEM_CODE[0];
    message.type[1] = SATELLITE_SYSTEM_CODE[1];
    message.type[2] = SATELLITE_SYSTEM_CODE[2];
    message.parameters_count = PARAMETERS_COUNT_SATELLITE_SYSTEM;
    message.parameters = (char **) malloc(sizeof(*message.parameters) * message.parameters_count);
    for (int i = 0 ; i < message.parameters_count ; i++) {
        *(message.parameters + i) = (char *) malloc(sizeof(**message.parameters) * 1);
    }
    if (satellite_systems.test(static_cast<size_t>(SatelliteSystem::GPS))) {
        message.parameters[GPS_FLAG] = (char *)"1";
    } else {
        message.parameters[GPS_FLAG] = (char *)"0";
    }

    if (satellite_systems.test(static_cast<size_t>(SatelliteSystem::GLONASS))) {
        message.parameters[GLONASS_FLAG] = (char *)"1";
    } else {
        message.parameters[GLONASS_FLAG] = (char *)"0";
    }

    if (satellite_systems.test(static_cast<size_t>(SatelliteSystem::GALILEO))) {
        message.parameters[GALILEO_FLAG] = (char *)"1";
    } else {
        message.parameters[GALILEO_FLAG] = (char *)"0";
    }

    if (satellite_systems.test(static_cast<size_t>(SatelliteSystem::GALILEO_FULL))) {
        message.parameters[GALILEO_FULL_FLAG] = (char *)"1";
    } else {
        message.parameters[GALILEO_FULL_FLAG] = (char *)"0";
    }

    if (satellite_systems.test(static_cast<size_t>(SatelliteSystem::BEIDOU))) {
        message.parameters[BEIDOU_FLAG] = (char *)"1";
    } else {
        message.parameters[BEIDOU_FLAG] = (char *)"0";
    }
    message.result = false;
    message.ack_received = false;
    message.ack_expected = true;
    return generate_and_send_pmtk_message(message);
}


bool L86::set_nmea_output_frequency(NmeaCommands nmea_commands, NmeaFrequency frequency)
{
    minmea_sentence_pmtk message;

    message.type[0] = NMEA_OUTPUT_FREQUENCY_CODE[0];
    message.type[1] = NMEA_OUTPUT_FREQUENCY_CODE[1];
    message.type[2] = NMEA_OUTPUT_FREQUENCY_CODE[2];
    message.parameters_count = PARAMETERS_COUNT_NMEA_OUTPUT_FREQUENCY;

    char c_frequency[10] = {0};
    switch (frequency) {
        case NmeaFrequency::ONE_POSITION_FIX:
            c_frequency[0] = '1';
            break;
        case NmeaFrequency::TWO_POSITION_FIXES:
            c_frequency[0] = '2';
            break;
        case NmeaFrequency::THREE_POSITION_FIXES:
            c_frequency[0] = '3';
            break;
        case NmeaFrequency::FOUR_POSITION_FIXES:
            c_frequency[0] = '4';
            break;
        case NmeaFrequency::FIVE_POSITION_FIXES:
            c_frequency[0] = '5';
            break;
    }
    message.parameters = (char **) malloc(sizeof(*message.parameters) * message.parameters_count);
    for (int i = 0 ; i < message.parameters_count ; i++) {
        *(message.parameters + i) = (char *) malloc(sizeof(**message.parameters) * 1);
    }

    if (nmea_commands.test(static_cast<size_t>(NmeaCommandType::GLL))) {
        message.parameters[GLL_FREQUENCY] = (char *)c_frequency;
    } else {
        message.parameters[GLL_FREQUENCY] = (char *)"0";
    }

    if (nmea_commands.test(static_cast<size_t>(NmeaCommandType::RMC))) {
        message.parameters[RMC_FREQUENCY] = (char *)c_frequency;
    } else {
        message.parameters[RMC_FREQUENCY] = (char *)"0";
    }

    if (nmea_commands.test(static_cast<size_t>(NmeaCommandType::VTG))) {
        message.parameters[VTG_FREQUENCY] = (char *)c_frequency;
    } else {
        message.parameters[VTG_FREQUENCY] = (char *)"0";
    }

    if (nmea_commands.test(static_cast<size_t>(NmeaCommandType::GGA))) {
        message.parameters[GGA_FREQUENCY] = (char *)c_frequency;
    } else {
        message.parameters[GGA_FREQUENCY] = (char *)"0";
    }

    if (nmea_commands.test(static_cast<size_t>(NmeaCommandType::GSA))) {
        message.parameters[GSA_FREQUENCY] = (char *)c_frequency;
    } else {
        message.parameters[GSA_FREQUENCY] = (char *)"0";
    }

    if (nmea_commands.test(static_cast<size_t>(NmeaCommandType::GSV))) {
        message.parameters[GSV_FREQUENCY] = (char *)c_frequency;
    } else {
        message.parameters[GSV_FREQUENCY] = (char *)"0";
    }

    for (uint8_t i = 6 ; i < message.parameters_count ; i++) {
        message.parameters[i] = (char *)"0";
    }

    message.result = false;
    message.ack_received = false;
    message.ack_expected = true;
    return generate_and_send_pmtk_message(message);
}


bool L86::set_navigation_mode(NavigationMode navigation_mode)
{
    minmea_sentence_pmtk message;
    message.type[0] = NAVIGATION_MODE_CODE[0];
    message.type[1] = NAVIGATION_MODE_CODE[1];
    message.type[2] = NAVIGATION_MODE_CODE[2];
    message.parameters_count = PARAMETERS_COUNT_NAVIGATION_MODE;

    message.parameters = (char **) malloc(sizeof(*message.parameters) * message.parameters_count);
    for (int i = 0 ; i < message.parameters_count ; i++) {
        *(message.parameters + i) = (char *) malloc(sizeof(**message.parameters) * 1);
    }

    switch (navigation_mode) {
        case NavigationMode::NORMAL_MODE:
            message.parameters[NAVIGATION_MODE] = (char *)"0";
            break;

        case NavigationMode::RUNNING_MODE:
            message.parameters[NAVIGATION_MODE] = (char *)"1";
            break;

        case NavigationMode::AVIATION_MODE:
            message.parameters[NAVIGATION_MODE] = (char *)"2";
            break;

        case NavigationMode::BALLOON_MODE:
            message.parameters[NAVIGATION_MODE] = (char *)"3";
            break;
    }

    message.result = false;
    message.ack_received = false;
    message.ack_expected = true;
    return generate_and_send_pmtk_message(message);
}

bool L86::set_position_fix_interval(uint16_t interval)
{
    minmea_sentence_pmtk message;
    message.type[0] = POSITION_FIX_INTERVAL_CODE[0];
    message.type[1] = POSITION_FIX_INTERVAL_CODE[1];
    message.type[2] = POSITION_FIX_INTERVAL_CODE[2];
    message.parameters_count = PARAMETERS_COUNT_POSITION_FIX_INTERVAL;

    unsigned char size = 0;
    if (interval >= 100 && interval < 1000) {
        size = 3;
    } else if (interval >= 1000 && interval < 10000) {
        size = 4;
    } else {
        size = 5;
    }

    message.parameters = (char **) malloc(sizeof(*message.parameters) * message.parameters_count);
    for (int i = 0 ; i < message.parameters_count ; i++) {
        *(message.parameters + i) = (char *) malloc(sizeof(**message.parameters) * size);
    }

    char s_interval[size] = {0};
    sprintf((char *)s_interval, "%d", interval);
    for (int i = 0 ; i < size ; i++) {
        message.parameters[INTERVAL][i] = s_interval[i];
    }
    message.parameters[INTERVAL][size] = '\0';

    message.ack_expected = true;
    message.result = false;
    message.ack_received = false;
    return generate_and_send_pmtk_message(message);
}

bool L86::start(StartMode start_mode)
{
    minmea_sentence_pmtk message;

    switch (start_mode) {
        case StartMode::FULL_COLD_START:
            message.type[0] = FULL_COLD_START_MODE_CODE[0];
            message.type[1] = FULL_COLD_START_MODE_CODE[1];
            message.type[2] = FULL_COLD_START_MODE_CODE[2];
            break;

        case StartMode::COLD_START:
            message.type[0] = COLD_START_MODE_CODE[0];
            message.type[1] = COLD_START_MODE_CODE[1];
            message.type[2] = COLD_START_MODE_CODE[2];
            break;

        case StartMode::WARM_START:
            message.type[0] = WARM_START_MODE_CODE[0];
            message.type[1] = WARM_START_MODE_CODE[1];
            message.type[2] = WARM_START_MODE_CODE[2];
            break;

        case StartMode::HOT_START:
            message.type[0] = HOT_START_MODE_CODE[0];
            message.type[1] = HOT_START_MODE_CODE[1];
            message.type[2] = HOT_START_MODE_CODE[2];
            break;
    }
    message.parameters_count = 0;
    message.ack_expected = false;
    message.result = false;
    message.ack_received = false;

    return generate_and_send_pmtk_message(message);
}

bool L86::standby_mode(StandbyMode standby_mode)
{
    minmea_sentence_pmtk message;

    message.type[0] = '2';
    message.type[1] = '2';
    message.type[2] = '5';
    message.parameters_count = PARAMETERS_COUNT_STANDBY_MODE;
    message.parameters = (char **) malloc(sizeof(*message.parameters) * message.parameters_count);
    for (int i = 0 ; i < message.parameters_count ; i++) {
        if (i != 0) {
            *(message.parameters + i) = (char *) malloc(sizeof(**message.parameters) * DEFAULT_PARAMETERS_COUNT_STANDBY_MODE);
        } else {
            *(message.parameters + i) = (char *) malloc(sizeof(**message.parameters) * 1);
        }
    }

    switch (standby_mode) {
        case StandbyMode::NORMAL_MODE:
            message.parameters[STANDBY_MODE] = (char *)"0";
            break;
        case StandbyMode::PERIODIC_BACKUP_MODE:
            message.parameters[STANDBY_MODE] = (char *)"1";
            break;
        case StandbyMode::PERIODIC_STANDBY_MODE:
            message.parameters[STANDBY_MODE] = (char *)"2";
            break;
        case StandbyMode::PERPETUAL_BACKUP_MODE:
            message.parameters[STANDBY_MODE] = (char *)"4";
            break;
        case StandbyMode::AL_STANDBY_MODE:
            message.parameters[STANDBY_MODE] = (char *)"8";
            break;
        case StandbyMode::AL_BACKUP_MODE:
            message.parameters[STANDBY_MODE] = (char *)"9";
            break;
    }

    message.result = false;
    message.ack_received = false;
    message.ack_expected = true;
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
            ThisThread::sleep_for(150);
        }
    }
    return _current_pmtk_message.result;
}

unsigned char L86::calculate_checksum(char *message)
{
    unsigned char sum = 0;
    bool is_message = false;
    uint8_t message_len = strlen(message);
    for (int index = 0 ; message[index] != '*' && index < message_len - 1  ; index++) {
        if (!is_message && message[index] == '$') {
            is_message = true;
        } else if (is_message) {
            sum ^= message[index];
        }
    }

    return sum;
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
                if (_current_pmtk_message.type[0] == pmtk_ack_frame.command[0] && _current_pmtk_message.type[1] == pmtk_ack_frame.command[1] && _current_pmtk_message.type[2] == pmtk_ack_frame.command[2]) {
                    _current_pmtk_message.ack_received = true;
                    if (pmtk_ack_frame.status == MINMEA_PMTK_ACK_CONFIG_STATUS_SUCCESS) {
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
            printf("\n -- Unknown command --> %s\n", message);
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

bool L86::verify_checksum(char *message)
{
    uint8_t checksum_initial_index = strlen(message) - FRAME_END_LEN - CHECKSUM_LEN;
    uint8_t checksum = uint8_t{strtol(&message[checksum_initial_index + 1], NULL, 16)};
    return (checksum == calculate_checksum(message));
}
