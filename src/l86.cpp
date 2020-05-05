#include "l86.h"

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
    message.packet_type[0] = '3';
    message.packet_type[1] = '5';
    message.packet_type[2] = '3';
    message.is_command = false;
    message.nb_param = 5;
    message.parameters = (char **) malloc(sizeof(*message.parameters) * message.nb_param);
    for (int i = 0 ; i < message.nb_param ; i++) {
        *(message.parameters + i) = (char *) malloc(sizeof(**message.parameters) * 1);
    }
    if (satellite_systems.test(static_cast<size_t>(SatelliteSystem::GPS))) {
        message.parameters[0] = (char *)"1";
    } else {
        message.parameters[0] = (char *)"0";
    }

    if (satellite_systems.test(static_cast<size_t>(SatelliteSystem::GLONASS))) {
        message.parameters[1] = (char *)"1";
    } else {
        message.parameters[1] = (char *)"0";
    }

    if (satellite_systems.test(static_cast<size_t>(SatelliteSystem::GALILEO))) {
        message.parameters[2] = (char *)"1";
    } else {
        message.parameters[2] = (char *)"0";
    }

    if (satellite_systems.test(static_cast<size_t>(SatelliteSystem::GALILEO_FULL))) {
        message.parameters[3] = (char *)"1";
    } else {
        message.parameters[3] = (char *)"0";
    }

    if (satellite_systems.test(static_cast<size_t>(SatelliteSystem::BEIDOU))) {
        message.parameters[4] = (char *)"1";
    } else {
        message.parameters[4] = (char *)"0";
    }

    message.ack = true;
    this->write_pmtk_message(message);
}


void L86::set_nmea_output_frequency(NmeaCommands nmea_commands, NmeaFrequency frequency)
{
    Pmtk_message message;

    message.packet_type[0] = '3';
    message.packet_type[1] = '1';
    message.packet_type[2] = '4';
    message.is_command = false;
    message.nb_param = 19;

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
        message.parameters[0] = (char *)c_frequency;
    } else {
        message.parameters[0] = (char *)"0";
    }

    if (nmea_commands.test(static_cast<size_t>(NmeaCommandType::RMC))) {
        message.parameters[1] = (char *)c_frequency;
    } else {
        message.parameters[1] = (char *)"0";
    }

    if (nmea_commands.test(static_cast<size_t>(NmeaCommandType::VTG))) {
        message.parameters[2] = (char *)c_frequency;
    } else {
        message.parameters[2] = (char *)"0";
    }

    if (nmea_commands.test(static_cast<size_t>(NmeaCommandType::GGA))) {
        message.parameters[3] = (char *)c_frequency;
    } else {
        message.parameters[3] = (char *)"0";
    }

    if (nmea_commands.test(static_cast<size_t>(NmeaCommandType::GSA))) {
        message.parameters[4] = (char *)c_frequency;
    } else {
        message.parameters[4] = (char *)"0";
    }

    if (nmea_commands.test(static_cast<size_t>(NmeaCommandType::GSV))) {
        message.parameters[5] = (char *)c_frequency;
    } else {
        message.parameters[5] = (char *)"0";
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
    message.packet_type[0] = '8';
    message.packet_type[1] = '8';
    message.packet_type[2] = '6';
    message.is_command = false;
    message.nb_param = 1;

    message.parameters = (char **) malloc(sizeof(*message.parameters) * message.nb_param);
    for (int i = 0 ; i < message.nb_param ; i++) {
        *(message.parameters + i) = (char *) malloc(sizeof(**message.parameters) * 1);
    }

    switch (navigation_mode) {
        case NavigationMode::NORMAL_MODE:
            message.parameters[0] = (char *)"0";
            break;

        case NavigationMode::RUNNING_MODE:
            message.parameters[0] = (char *)"1";
            break;

        case NavigationMode::AVIATION_MODE:
            message.parameters[0] = (char *)"2";
            break;

        case NavigationMode::BALLOON_MODE:
            message.parameters[0] = (char *)"3";
            break;
    }

    message.ack = true;
    write_pmtk_message(message);
}

void L86::set_position_fix_interval(uint16_t interval)
{
    Pmtk_message message;
    message.packet_type[0] = '2';
    message.packet_type[1] = '2';
    message.packet_type[2] = '0';
    message.is_command = false;
    message.nb_param = 1;

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
        message.parameters[0][i] = s_interval[i];
    }
    message.parameters[0][size] = '\0';

    message.ack = true;
    write_pmtk_message(message);
}

void L86::start(StartMode start_mode)
{
    Pmtk_message message;

    switch (start_mode) {
        case StartMode::FULL_COLD_START:
            message.packet_type[0] = '1';
            message.packet_type[1] = '0';
            message.packet_type[2] = '4';
            break;

        case StartMode::COLD_START:
            message.packet_type[0] = '1';
            message.packet_type[1] = '0';
            message.packet_type[2] = '3';
            break;

        case StartMode::WARM_START:
            message.packet_type[0] = '1';
            message.packet_type[1] = '0';
            message.packet_type[2] = '2';
            break;

        case StartMode::HOT_START:
            message.packet_type[0] = '1';
            message.packet_type[1] = '0';
            message.packet_type[2] = '1';
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
    message.nb_param = 1;
    message.parameters = (char **) malloc(sizeof(*message.parameters) * message.nb_param);
    for (int i = 0 ; i < message.nb_param ; i++) {
        if (i != 0) {
            *(message.parameters + i) = (char *) malloc(sizeof(**message.parameters) * 6);
        } else {
            *(message.parameters + i) = (char *) malloc(sizeof(**message.parameters) * 1);
        }
    }

    switch (standby_mode) {
        case StandbyMode::NORMAL_MODE:
            message.parameters[0] = (char *)"0";
            break;
        case StandbyMode::PERIODIC_BACKUP_MODE:
            message.parameters[0] = (char *)"1";
            break;
        case StandbyMode::PERIODIC_STANDBY_MODE:
            message.parameters[0] = (char *)"2";
            break;
        case StandbyMode::PERPETUAL_BACKUP_MODE:
            message.parameters[0] = (char *)"4";
            break;
        case StandbyMode::AL_STANDBY_MODE:
            message.parameters[0] = (char *)"8";
            break;
        case StandbyMode::AL_BACKUP_MODE:
            message.parameters[0] = (char *)"9";
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

void L86::callback_rx(void)
{
    static unsigned char index_car = 0;
    static char answer[120];
    char parameters[19][10] = {0};
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
            unsigned char i = 0;
            for (int index = 7 ; answer[index] != '*' ; index++) {
                if (answer[index] == ',') {
                    index_argument++;
                    i = 0;
                } else {
                    parameters[index_argument][i] = answer[index];
                    i++;
                }
            }

            /* Update informations */

            set_parameter(parameters, response_type);
        } else if (answer[1] == 'P' && _waiting_ack == true) {     /* Trames PMTK */
            if (answer[9] == _current_pmtk_command_code[0] && answer[10] == _current_pmtk_command_code[1] && answer[11] == _current_pmtk_command_code[2]) {
                char flag = answer[12];
                _waiting_ack = false;
                if (flag == '3') {
                    _pmtk_command_result = true;
                }
            }
        }
        memset(answer, 0, 120);
        memset(parameters, 0, (size_t)(sizeof(parameters[0][0]) * 19 * 10));
        index_car = 0;
    }
}

void L86::write_pmtk_message(Pmtk_message message)
{
    /* PMTK frame setting up*/
    char packet[100];
    char packet_temp[100];
    sprintf(packet, "$PMTK%c%c%c", message.packet_type[0], message.packet_type[1], message.packet_type[2]);

    for (int i = 0 ; i < message.nb_param ; i++) {
        sprintf(packet_temp, "%s", packet);
        sprintf(packet, "%s,%s", packet_temp, message.parameters[i]);
    }

    sprintf(packet_temp, "%s*", packet);

    unsigned char checksum = 0;
    checksum = this->calculate_checksum(packet_temp);

    sprintf(packet, "%s%X\r\n", packet_temp, checksum);

    /* Send packet until received ack and get confirmation that command succeeds */
    do {
        _uart->write((uint8_t *)packet, strlen(packet), NULL);
        if (message.ack) {
            _waiting_ack = true;
            _pmtk_command_result = false;
            char ack_message[50];
            unsigned char index = 0;
            while (index < message.anwser_size) {
                char received_character = _uart->getc();
                if ((received_character == '$' && index == 0) || (received_character != '$' && index != 0)) {
                    ack_message[index] = received_character;
                    index++;
                }
            }
            if (ack_message[5] == '0' && ack_message[6] == '0' && ack_message[7] == '1') {  /* ack frame */
                _waiting_ack = false;
                if (ack_message[9] == message.packet_type[0] && ack_message[10] == message.packet_type[1] && ack_message[11] == message.packet_type[2]) { /* Good command ack */
                    if (ack_message[13] == '3') { /* Command succeeds */
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
    unsigned char i = 0;

    while (message[i] != '*') {
        if (is_message) {
            sum ^= message[i];
        }
        if (message[i] == '$') {
            is_message = true;
        }
        i++;
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
    int limit = 4;
    switch (command_type) {
        case NmeaCommandType::RMC:
            set_positionning_mode(parameters[11][0]);
            if (_global_informations.positionning_mode != PositionningMode::NO_FIX && _global_informations.positionning_mode != PositionningMode::UNKNOWN) {
                set_date(parameters[8]);
                set_time(parameters[0]);

                set_latitude(parameters[2], parameters[3][0]);
                set_longitude(parameters[4], parameters[5][0]);
                _movement_informations.speed_knots = atof(parameters[6]);
            }
            break;

        case NmeaCommandType::VTG:
            set_positionning_mode(parameters[8][0]);
            if (_global_informations.positionning_mode != PositionningMode::NO_FIX && _global_informations.positionning_mode != PositionningMode::UNKNOWN) {
                _movement_informations.speed_knots = atof(parameters[4]);
                _movement_informations.speed_kmh = atof(parameters[6]);
            }
            break;

        case NmeaCommandType::GGA:
            set_fix_status(parameters[5][0]);
            if (_global_informations.fix_status != FixStatusGGA::INVALID && _global_informations.fix_status != FixStatusGGA::UNKNOWN) {
                set_time(parameters[0]);
                set_latitude(parameters[1], parameters[2][0]);
                set_longitude(parameters[3], parameters[4][0]);
                _position_informations.altitude = atof(parameters[8]);
                _satellites_informations.satellite_count = atoi(parameters[6]);
            }
            break;

        case NmeaCommandType::GSA:
            set_fix_satellite_status(parameters[1][0]);
            if (_satellites_informations.status != FixStatusGSA::NOFIX && _satellites_informations.status != FixStatusGSA::UNKNOWN) {
                set_mode(parameters[0][0]);

                _dilution_of_precision.horizontal = atof(parameters[15]);
                _dilution_of_precision.positional = atof(parameters[14]);
                _dilution_of_precision.vertical = atof(parameters[16]);
            }

            break;

        case NmeaCommandType::GSV:
            if (strcmp(parameters[0], parameters[1]) == 0) {
                limit = 12 - atoi(parameters[2]);
            }

            for (int i = 1 ; i <= limit ; i++) {
                for (int j = 0 ; j < _registered_satellite_count ; j++) {
                    int sat_id = atoi(parameters[i * 4 - 1]);
                    if (_satellites_informations.satellites[j].id == sat_id) {
                        _satellites_informations.satellites[j].elevation = atoi(parameters[i * 4]);
                        _satellites_informations.satellites[j].azimuth = atoi(parameters[i * 4 + 1]);
                        _satellites_informations.satellites[j].snr = atoi(parameters[i * 4 + 2]);
                        flag = true;
                        break;
                    }
                }
                if (!flag) { /* Add to the table if not already stored */
                    Satellite sat;
                    sat.id = atoi(parameters[i * 4 - 1]);
                    sat.elevation = atoi(parameters[i * 4]);
                    sat.azimuth = atoi(parameters[i * 4 + 1]);
                    sat.snr = atoi(parameters[i * 4 + 2]);
                    _satellites_informations.satellites[_registered_satellite_count] = sat;
                    _registered_satellite_count++;
                    flag = false;
                }
            }
            _satellites_informations.satellite_count = atoi(parameters[2]);
            break;

        case NmeaCommandType::GLL:
            set_date(parameters[4]);
            set_positionning_mode(parameters[4][0]);
            set_latitude(parameters[0], parameters[1][0]);
            set_longitude(parameters[2], parameters[3][0]);
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
    _global_informations.time.tm_year = 2000 + atoi(buffer) - 1900;
}

void L86::set_latitude(char *latitude, char direction)
{
    _position_informations.latitude = atof(latitude);
    if (direction == 'S') {
        _position_informations.latitude *= -1;
    }
}

void L86::set_longitude(char *longitude, char position)
{
    _position_informations.longitude = atof(longitude);
    if (position == 'E') {
        _position_informations.longitude *= -1;
    }
}


