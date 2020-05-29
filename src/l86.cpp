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
constexpr int MAX_SATELLITES = 12;                          //!< Max number of satellites in a group of gsb message
constexpr char ACK_CODE[] = "001";                          //!< Ack command code

constexpr int PMTK_COMMAND_CODE_INDEX = 9;                  //!< Index of pmtk command code first character
constexpr int PMTK_COMMAND_RESULT = 13;                     //!< Pmtk command result index
constexpr int PMTK_PACKET_SIZE = 100;                       //!< Default packet size for pmtk command
constexpr int PMTK_ANSWER_SIZE = 50;                        //!< Pmtk received message size
constexpr int PMTK_PACKET_TYPE_INDEX = 5;                   //!< Pmtk received message command code index
constexpr char INVALID_PACKET = '0';                        //!< Invalid packet code
constexpr char UNSUPPORTED_PACKET_TYPE = '1';               //!< Unsupported packet code
constexpr char VALID_PACKET_AND_ACTION_FAILED = '2';        //!< Valid packet but action failed code
constexpr char VALID_PACKET_AND_COMMAND_SUCCEED = '3';      //!< Valid packet and command succed code
constexpr int CHECKSUM_LEN = 2;                             //!< Checksum length
constexpr int FRAME_END_LEN = 3;                            //!< Received message right shift to access to the checksum

constexpr int HALF_BYTE_SHIFT = 4;                          //!< Shift to operate to acceed to the first half of a byte
constexpr int ASCII_VALUE_1 = 48;                           //!< Ascii value for '1' character
constexpr int ASCII_VALUE_9 = 57;                           //!< Ascii value for '9' character
constexpr int ASCII_NUMBER_TO_INT_SHIFT = 48;               //!< Shift between Ascii number value and integer value
constexpr int ASCII_VALUE_A = 65;                           //!< Ascii value for 'A' character
constexpr int ASCII_VALUE_F = 70;                           //!< Ascii value for 'F' character
constexpr int ASCII_LETTER_TO_INT_SHIFT = 55;               //!< Shift between Ascii letter value and integer value

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


L86::L86(RawSerial *uart)
{
    this->_waiting_ack = false;
    this->_current_pmtk_command_code[0] = 0;
    this->_current_pmtk_command_code[1] = 0;
    this->_current_pmtk_command_code[2] = 0;
    this->_pmtk_command_result = false;
    _registered_satellite_count = 0;
    _uart = uart;

    _position_informations.altitude = 0.0;
    _position_informations.latitude = 0.0;
    _position_informations.longitude = 0.0;

    _movement_informations.speed_kmh = 0.0;
    _movement_informations.speed_knots = 0.0;
}

void L86::set_satellite_system(SatelliteSystems satellite_systems)
{
    Pmtk_message message;
    message.packet_type[0] = SATELLITE_SYSTEM_CODE[0];
    message.packet_type[1] = SATELLITE_SYSTEM_CODE[1];
    message.packet_type[2] = SATELLITE_SYSTEM_CODE[2];
    message.is_command = false;
    message.nb_param = PARAMETERS_COUNT_SATELLITE_SYSTEM;
    message.parameters = (char **) malloc(sizeof(*message.parameters) * message.nb_param);
    for (int i = 0 ; i < message.nb_param ; i++) {
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

    message.ack = true;
    this->write_pmtk_message(message);
}


void L86::set_nmea_output_frequency(NmeaCommands nmea_commands, NmeaFrequency frequency)
{
    Pmtk_message message;

    message.packet_type[0] = NMEA_OUTPUT_FREQUENCY_CODE[0];
    message.packet_type[1] = NMEA_OUTPUT_FREQUENCY_CODE[1];
    message.packet_type[2] = NMEA_OUTPUT_FREQUENCY_CODE[2];
    message.is_command = false;
    message.nb_param = PARAMETERS_COUNT_NMEA_OUTPUT_FREQUENCY;

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
    message.parameters = (char **) malloc(sizeof(*message.parameters) * message.nb_param);
    for (int i = 0 ; i < message.nb_param ; i++) {
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

    for (uint8_t i = 6 ; i < message.nb_param ; i++) {
        message.parameters[i] = (char *)"0";
    }

    message.ack = true;
    this->write_pmtk_message(message);
}


void L86::set_navigation_mode(NavigationMode navigation_mode)
{
    Pmtk_message message;
    message.packet_type[0] = NAVIGATION_MODE_CODE[0];
    message.packet_type[1] = NAVIGATION_MODE_CODE[1];
    message.packet_type[2] = NAVIGATION_MODE_CODE[2];
    message.is_command = false;
    message.nb_param = PARAMETERS_COUNT_NAVIGATION_MODE;

    message.parameters = (char **) malloc(sizeof(*message.parameters) * message.nb_param);
    for (int i = 0 ; i < message.nb_param ; i++) {
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

    message.ack = true;
    write_pmtk_message(message);
}

void L86::set_position_fix_interval(uint16_t interval)
{
    Pmtk_message message;
    message.packet_type[0] = POSITION_FIX_INTERVAL_CODE[0];
    message.packet_type[1] = POSITION_FIX_INTERVAL_CODE[1];
    message.packet_type[2] = POSITION_FIX_INTERVAL_CODE[2];
    message.is_command = false;
    message.nb_param = PARAMETERS_COUNT_POSITION_FIX_INTERVAL;

    unsigned char size = 0;
    if (interval >= 100 && interval < 1000) {
        size = 3;
    } else if (interval >= 1000 && interval < 10000) {
        size = 4;
    } else {
        size = 5;
    }

    message.parameters = (char **) malloc(sizeof(*message.parameters) * message.nb_param);
    for (int i = 0 ; i < message.nb_param ; i++) {
        *(message.parameters + i) = (char *) malloc(sizeof(**message.parameters) * size);
    }

    char s_interval[size] = {0};
    sprintf((char *)s_interval, "%d", interval);
    for (int i = 0 ; i < size ; i++) {
        message.parameters[INTERVAL][i] = s_interval[i];
    }
    message.parameters[INTERVAL][size] = '\0';

    message.ack = true;
    write_pmtk_message(message);
}

void L86::start(StartMode start_mode)
{
    Pmtk_message message;

    switch (start_mode) {
        case StartMode::FULL_COLD_START:
            message.packet_type[0] = FULL_COLD_START_MODE_CODE[0];
            message.packet_type[1] = FULL_COLD_START_MODE_CODE[1];
            message.packet_type[2] = FULL_COLD_START_MODE_CODE[2];
            break;

        case StartMode::COLD_START:
            message.packet_type[0] = COLD_START_MODE_CODE[0];
            message.packet_type[1] = COLD_START_MODE_CODE[1];
            message.packet_type[2] = COLD_START_MODE_CODE[2];
            break;

        case StartMode::WARM_START:
            message.packet_type[0] = WARM_START_MODE_CODE[0];
            message.packet_type[1] = WARM_START_MODE_CODE[1];
            message.packet_type[2] = WARM_START_MODE_CODE[2];
            break;

        case StartMode::HOT_START:
            message.packet_type[0] = HOT_START_MODE_CODE[0];
            message.packet_type[1] = HOT_START_MODE_CODE[1];
            message.packet_type[2] = HOT_START_MODE_CODE[2];
            break;
    }
    message.is_command = true;
    message.nb_param = 0;
    message.ack = false;

    this->write_pmtk_message(message);
    this->start_receive();
}

void L86::standby_mode(StandbyMode standby_mode)
{
    Pmtk_message message;

    message.packet_type[0] = '2';
    message.packet_type[1] = '2';
    message.packet_type[2] = '5';
    message.is_command = false;
    message.nb_param = PARAMETERS_COUNT_STANDBY_MODE;
    message.parameters = (char **) malloc(sizeof(*message.parameters) * message.nb_param);
    for (int i = 0 ; i < message.nb_param ; i++) {
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

    this->write_pmtk_message(message);

    for (int i = 0 ; i < message.nb_param ; i++) {
        free(message.parameters[i]);
        message.parameters[i] = NULL;
    }
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

void L86::callback_rx(void)
{
    static unsigned char index_car = 0;
    static char answer[MAX_ANSWER_SIZE];
    char parameters[MAX_PARAMETERS_COUNT][MAX_PARAMETER_SIZE] = {0};
    NmeaCommandType response_type;

    char cur_car = _uart->getc();
    if ((index_car == 0 && cur_car == '$') || (index_car != 0 && cur_car != '$')) {
        answer[index_car] = cur_car;
        index_car++;
    }
    if (answer[index_car - 1] == '\n') { /* Complete received */
        strcpy((char *)_last_received_command, (char *)answer);
        if (answer[1] == 'G') {                  /* Trames NMEA */
            if (answer[3] == 'R') {                                  /* RMC */
                response_type = NmeaCommandType::RMC;
            } else if (answer[3] == 'V') {                           /* VTG */
                response_type = NmeaCommandType::VTG;
            } else if (answer[3] == 'G' && answer[4] == 'G') { /* GGA */
                response_type = NmeaCommandType::GGA;
            } else if (answer[3] == 'G' && answer[5] == 'A') { /* GSA */
                response_type = NmeaCommandType::GSA;
            } else if (answer[3] == 'G' && answer[5] == 'V') { /* GSV */
                response_type = NmeaCommandType::GSV;
            } else if (answer[3] == 'G' && answer[5] == 'L') {  /* GLL */
                response_type = NmeaCommandType::GLL;
            }

            /* Parse arguments */
            int index_argument = 0;
            int index = 0;
            unsigned char i = 0;

            for (index = PARAMETERS_BEGIN ; answer[index] != '*' ; index++) {
                if (answer[index] == ',') {
                    index_argument++;
                    i = 0;
                } else {
                    parameters[index_argument][i] = answer[index];
                    i++;
                }
            }

            if (!verify_checksum(answer)) {
                memset(answer, 0, 120);
                memset(parameters, 0, (size_t)(sizeof(parameters[0][0]) * 19 * 10));
                index_car = 0;
                return;
            }

            /* Update informations */
            set_parameter(parameters, response_type);

        } else if (answer[1] == 'P' && _waiting_ack == true) {     /* Trames PMTK */
            if (answer[PMTK_COMMAND_CODE_INDEX] == _current_pmtk_command_code[0] && answer[PMTK_COMMAND_CODE_INDEX + 1] == _current_pmtk_command_code[1] && answer[PMTK_COMMAND_CODE_INDEX + 2] == _current_pmtk_command_code[2]) {
                char flag = answer[PMTK_COMMAND_RESULT];
                _waiting_ack = false;
                if (flag == VALID_PACKET_AND_COMMAND_SUCCEED) {
                    _pmtk_command_result = true;
                }
            }
        }
        memset(answer, 0, MAX_ANSWER_SIZE);
        memset(parameters, 0, (size_t)(sizeof(parameters[0][0]) * MAX_PARAMETERS_COUNT * MAX_PARAMETER_SIZE));
        index_car = 0;
    }
}

void L86::write_pmtk_message(Pmtk_message message)
{
    /* PMTK frame setting up*/
    char packet[PMTK_PACKET_SIZE];
    char packet_temp[PMTK_PACKET_SIZE];
    sprintf(packet, "$PMTK%c%c%c", message.packet_type[0], message.packet_type[1], message.packet_type[2]);

    for (int i = 0 ; i < message.nb_param ; i++) {
        sprintf(packet_temp, "%s", packet);
        sprintf(packet, "%s,%s", packet_temp, message.parameters[i]);
    }

    sprintf(packet_temp, "%s*", packet);

    unsigned char checksum = 0;
    checksum = this->calculate_checksum(packet_temp);

    sprintf(packet, "%s%02X\r\n", packet_temp, checksum);

    /* Send packet until received ack and get confirmation that command succeeds */
    do {
        _uart->write((uint8_t *)packet, strlen(packet), NULL);
        if (message.ack) {
            _waiting_ack = true;
            _pmtk_command_result = false;
            char ack_message[PMTK_ANSWER_SIZE];
            unsigned char index = 0;
            while (index < message.anwser_size) {
                char received_character = _uart->getc();
                if ((received_character == '$' && index == 0) || (received_character != '$' && index != 0)) {
                    ack_message[index] = received_character;
                    index++;
                }
            }
            if (ack_message[PMTK_PACKET_TYPE_INDEX] == ACK_CODE[0] && ack_message[PMTK_PACKET_TYPE_INDEX + 1] == ACK_CODE[1] && ack_message[PMTK_PACKET_TYPE_INDEX + 2] == ACK_CODE[2]) { /* ack frame */
                _waiting_ack = false;
                if (ack_message[PMTK_COMMAND_CODE_INDEX] == message.packet_type[0] && ack_message[PMTK_COMMAND_CODE_INDEX + 1] == message.packet_type[1] && ack_message[PMTK_COMMAND_CODE_INDEX + 2] == message.packet_type[2]) { /* Good command ack */
                    if (ack_message[PMTK_COMMAND_RESULT] == VALID_PACKET_AND_COMMAND_SUCCEED) { /* Command succeeds */
                        _pmtk_command_result = true;
                    }
                }
            }
        } else {
            this->_waiting_ack = false;
            this->_pmtk_command_result = true;
        }
        ThisThread::sleep_for(200);
    } while (_waiting_ack or !_pmtk_command_result);
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

void L86::start_receive()
{
    this->_uart->attach(callback(this, &L86::callback_rx), RawSerial::RxIrq);
}

void L86::stop_receive()
{
    this->_uart->attach(NULL);
}

void L86::set_parameter(char parameters[][10], NmeaCommandType command_type)
{
    bool flag = false;
    int limit = LIMIT_SATELLITES;
    switch (command_type) {
        case NmeaCommandType::RMC:
            set_positionning_mode(parameters[RMC_POSITIONNING_MODE][0]);
            if (_global_informations.positionning_mode != PositionningMode::NO_FIX && _global_informations.positionning_mode != PositionningMode::UNKNOWN) {
                set_date(parameters[RMC_DATE]);
                set_time(parameters[RMC_TIME]);

                set_latitude(parameters[RMC_LATITUDE], parameters[RMC_LATITUDE_N_S][0]);
                set_longitude(parameters[RMC_LONGITUDE], parameters[RMC_LONGITUDE_E_W][0]);
                _movement_informations.speed_knots = atof(parameters[RMC_SPEED_KNOTS]);
            }
            break;

        case NmeaCommandType::VTG:
            set_positionning_mode(parameters[VTG_POSITIONNING_MODE][0]);
            if (_global_informations.positionning_mode != PositionningMode::NO_FIX && _global_informations.positionning_mode != PositionningMode::UNKNOWN) {
                _movement_informations.speed_knots = atof(parameters[VTG_SPEED_KNOTS]);
                _movement_informations.speed_kmh = atof(parameters[VTG_SPEED_KMH]);
            }
            break;

        case NmeaCommandType::GGA:
            set_fix_status(parameters[GGA_FIX_STATUS][0]);
            if (_global_informations.fix_status != FixStatusGGA::INVALID && _global_informations.fix_status != FixStatusGGA::UNKNOWN) {
                set_time(parameters[GGA_TIME]);
                set_latitude(parameters[GGA_LATITUDE], parameters[GGA_LATITUDE_N_S][0]);
                set_longitude(parameters[GGA_LONGITUDE], parameters[GGA_LONGITUDE_E_W][0]);
                _position_informations.altitude = atof(parameters[GGA_ALTITUDE]);
                _satellites_informations.satellite_count = atoi(parameters[GGA_SATELLITE_COUNT]);
            }
            break;

        case NmeaCommandType::GSA:
            set_fix_satellite_status(parameters[GSA_FIX_SATELLITE_STATUS][0]);
            if (_satellites_informations.status != FixStatusGSA::NOFIX && _satellites_informations.status != FixStatusGSA::UNKNOWN) {
                set_mode(parameters[GSA_MODE][0]);

                _dilution_of_precision.horizontal = atof(parameters[GSA_DILUTION_OF_PRECISION_HORIZONTAL]);
                _dilution_of_precision.positional = atof(parameters[GSA_DILUTION_OF_PRECISION_POSITIONAL]);
                _dilution_of_precision.vertical = atof(parameters[GSA_DILUTION_OF_PRECISION_VERTICAL]);
            }

            break;

        case NmeaCommandType::GSV:
            /* last sequence message */
            if (strcmp(parameters[GSV_MESSAGES_COUNT], parameters[GSV_SEQUENCE_NUMBER]) == 0) {
                limit = MAX_SATELLITES - atoi(parameters[GSV_SATELLITES_COUNT]);
            }
            /* reset satellites if first sequence message */
            if (atoi(parameters[GSV_SEQUENCE_NUMBER]) == 1) {
                _registered_satellite_count = 0;
            }

            for (int i = 1 ; i <= limit ; i++) {
                Satellite sat;
                sat.id = atoi(parameters[i * 4 - 1]);
                sat.elevation = atoi(parameters[i * 4]);
                sat.azimuth = atoi(parameters[i * 4 + 1]);
                sat.snr = atoi(parameters[i * 4 + 2]);
                _satellites_informations.satellites[_registered_satellite_count] = sat;
                _registered_satellite_count++;
            }
            Satellite empty_sat;
            empty_sat.id = 0;
            empty_sat.elevation = 0;
            empty_sat.azimuth = 0;
            empty_sat.snr = 0;

            for (int i = _registered_satellite_count ; i <= NB_MAX_SATELLITES ; i++) {
                _satellites_informations.satellites[i] = empty_sat;
            }

            _satellites_informations.satellite_count = atoi(parameters[GSV_SATELLITES_COUNT]);
            break;

        case NmeaCommandType::GLL:
            set_time(parameters[GLL_TIME]);
            set_positionning_mode(parameters[GLL_POSITIONNING_MODE][0]);
            set_latitude(parameters[GLL_LATITUDE], parameters[GLL_LATITUDE_N_S][0]);
            set_longitude(parameters[GLL_LONGITUDE], parameters[GLL_LONGITUDE][0]);
    }
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

void L86::set_fix_status(char c_fix_status)
{
    switch (c_fix_status) {
        case '0':
            _global_informations.fix_status = FixStatusGGA::INVALID;
            break;
        case '1':
            _global_informations.fix_status = FixStatusGGA::GNSS_FIX;
            break;
        case '2':
            _global_informations.fix_status = FixStatusGGA::DGPS_FIX;
            break;
        case '6':
            _global_informations.fix_status = FixStatusGGA::ESTIMATED_MODE;
            break;
        default:
            _global_informations.fix_status = FixStatusGGA::UNKNOWN;
    }
}

void L86::set_fix_satellite_status(char c_fix_satellite_status)
{
    switch (c_fix_satellite_status) {
        case '1':
            _satellites_informations.status = FixStatusGSA::NOFIX;
            break;
        case '2':
            _satellites_informations.status = FixStatusGSA::FIX2D;
            break;
        case '3':
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

void L86::set_time(char *time)
{
    char buffer[2] = "";
    sprintf(buffer, "%c%c", time[0], time[1]);
    _global_informations.time.tm_hour = atoi(buffer);
    sprintf(buffer, "%c%c", time[2], time[3]);
    _global_informations.time.tm_min = atoi(buffer);
    sprintf(buffer, "%c%c", time[4], time[5]);
    _global_informations.time.tm_sec = atoi(buffer);
}

void L86::set_date(char *date)
{
    char buffer[2] = "";
    sprintf(buffer, "%c%c", date[0], date[1]);
    _global_informations.time.tm_mday = atoi(buffer);
    sprintf(buffer, "%c%c", date[2], date[3]);
    _global_informations.time.tm_mon = atoi(buffer) - 1;
    sprintf(buffer, "%c%c", date[4], date[5]);
    _global_informations.time.tm_year = 2000 + atoi(buffer) - 1900; /* basic calculs to get year */
}

void L86::set_longitude(char *longitude, char position)
{
    _position_informations.longitude = atof(longitude);
    if (position == 'E') {
        _position_informations.longitude *= -1;
    }
}

void L86::set_latitude(char *latitude, char direction)
{
    _position_informations.latitude = atof(latitude);
    if (direction == 'S') {
        _position_informations.latitude *= -1;
    }
}

bool L86::verify_checksum(char *message)
{
    uint8_t checksum = 0;
    uint8_t checksum_initial_index = strlen(message) - FRAME_END_LEN  - CHECKSUM_LEN;

    /* Convert ascii checksum contained in the received message to an integer checksum */
    for (int checksum_index = 1 ; checksum_index <= CHECKSUM_LEN ; checksum_index++) {
        char checksum_character = message[checksum_initial_index + checksum_index];
        uint8_t right_shift = 0;
        if (checksum_index == 1) {
            right_shift = HALF_BYTE_SHIFT;
        }
        if (checksum_character >= ASCII_VALUE_1 && checksum_character <= ASCII_VALUE_9) {   // Characters between 0 and 9
            checksum |= ((checksum_character - ASCII_NUMBER_TO_INT_SHIFT) << right_shift);
        } else if (checksum_character >= ASCII_VALUE_A && checksum_character <= ASCII_VALUE_F) { // Characters between A and F
            checksum |= ((checksum_character - ASCII_LETTER_TO_INT_SHIFT) << right_shift);
        } else {
            return false;
        }
    }
    return (checksum == calculate_checksum(message));
}
