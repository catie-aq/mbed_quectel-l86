#include "l86.h"

L86::L86(RawSerial *uart)
{
    this->_waiting_ack = false;
    this->longitude = (char *) malloc(sizeof(char) * 11);
    this->latitude = (char *) malloc(sizeof(char) * 10);
    this->_current_pmtk_command_code[0] = 0;
    this->_current_pmtk_command_code[1] = 0;
    this->_current_pmtk_command_code[2] = 0;
    this->_pmtk_command_result = false;
    _uart = uart;

}

void L86::start_attach()
{
    this->_uart->attach(callback(this, &L86::callback_rx), RawSerial::RxIrq);
}

void L86::stop_attach()
{
    this->_uart->attach(NULL);
}

void L86::write_pmtk_message(Pmtk_message message)
{
    /* Formation de le trame PMTK */
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
            char ack[50];
            unsigned char index = 0;
            while (index < message.anwser_size) {
                char carac = _uart->getc();
                if ((carac == '$' && index == 0) || (carac != '$' && index != 0)) {
                    ack[index] = carac;
                    index++;
                }
            }
            if (ack[5] == '0' && ack[6] == '0' && ack[7] == '1') {  /* ack trame */
                _waiting_ack = false;
                if (ack[9] == message.packet_type[0] && ack[10] == message.packet_type[1] && ack[11] == message.packet_type[2]) {
                    if (ack[13] == '3') {
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

char *L86::get_longitude()
{
    return (char *) this->longitude;
}

char *L86::get_latitude()
{
    return (char *) this->latitude;
}

void L86::set_nmea_output_frequency(NmeaCommands nmea_command, NmeaFrequency frequency)
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

    if (nmea_command.test(static_cast<size_t>(NmeaCommandType::GLL))) {
        message.parameters[0] = (char *)c_frequency;
    } else {
        message.parameters[0] = (char *)"0";
    }

    if (nmea_command.test(static_cast<size_t>(NmeaCommandType::RMC))) {
        message.parameters[1] = (char *)c_frequency;
    } else {
        message.parameters[1] = (char *)"0";
    }

    if (nmea_command.test(static_cast<size_t>(NmeaCommandType::VTG))) {
        message.parameters[2] = (char *)c_frequency;
    } else {
        message.parameters[2] = (char *)"0";
    }

    if (nmea_command.test(static_cast<size_t>(NmeaCommandType::GGA))) {
        message.parameters[3] = (char *)c_frequency;
    } else {
        message.parameters[3] = (char *)"0";
    }

    if (nmea_command.test(static_cast<size_t>(NmeaCommandType::GSA))) {
        message.parameters[4] = (char *)c_frequency;
    } else {
        message.parameters[4] = (char *)"0";
    }

    if (nmea_command.test(static_cast<size_t>(NmeaCommandType::GSV))) {
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
    if(satellite_systems.test(static_cast<size_t>(SatelliteSystem::GPS))) {
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
    /* use default values */
    /*
    sprintf(message.parameters[1], "200000");
    sprintf(message.parameters[2], "200000");
    sprintf(message.parameters[3], "400000");
    sprintf(message.parameters[4], "400000");
     */

    this->write_pmtk_message(message);
}

void L86::callback_rx(void)
{
    static unsigned char index_carac = 0;
    static char reponse[120];
    static int cpt_trame_recu = 0;
    volatile int type = 0;
    char parameters[19][10] = {0};
    NmeaCommandType response_type;

    char carac = _uart->getc();
    if ((index_carac == 0 && carac == '$') || (index_carac != 0 && carac != '$')) {
        reponse[index_carac] = carac;
        index_carac++;
    }
    if (reponse[index_carac - 1] == '\n') { /* Complete received */
        strcpy((char *)_last_received_command, (char *)reponse);
        if (reponse[1] == 'G') {                  /* Trames NMEA */
            if (reponse[3] == 'R') {                                  /* RMC */
                response_type = NmeaCommandType::RMC;
            } else if (reponse[3] == 'V') {                           /* VTG */
                response_type = NmeaCommandType::VTG;
            } else if (reponse[3] == 'G' && reponse[4] == 'G') { /* GGA */
                response_type = NmeaCommandType::GGA;
            } else if (reponse[3] == 'G' && reponse[5] == 'A') { /* GSA */
                response_type = NmeaCommandType::GSA;
            } else if (reponse[3] == 'G' && reponse[5] == 'V') { /* GSV */
                response_type = NmeaCommandType::GSV;
            } else if (reponse[3] == 'G' && reponse[5] == 'L') {  /* GLL */
                response_type = NmeaCommandType::GLL;
            }

            /* Parse arguments */
            int index_argument = 0;
            unsigned char i = 0;
            for (int index = 7 ; reponse[index] != '*' ; index++) {
                if (reponse[index] == ',') {
                    index_argument++;
                    i = 0;
                } else {
                    parameters[index_argument][i] = reponse[index];
                    i++;
                }
            }

            /* Update informations */
            switch (response_type) {
                case NmeaCommandType::RMC:
                    sprintf(this->longitude, "%s%c", parameters[4], parameters[5][0]);
                    sprintf(this->latitude, "%s%c", parameters[2], parameters[3][0]);
                    break;

                case NmeaCommandType::GGA:
                    sprintf(this->longitude, "%s%c", parameters[3], parameters[4][0]);
                    sprintf(this->latitude, "%s%c", parameters[1], parameters[2][0]);
                    break;

                case NmeaCommandType::GLL:
                    sprintf(this->longitude, "%s%c", parameters[2], parameters[3][0]);
                    sprintf(this->latitude, "%s%c", parameters[0], parameters[1][0]);
                    break;
            }
        } else if (reponse[1] == 'P' && _waiting_ack == true) {     /* Trames PMTK */
            if (reponse[9] == _current_pmtk_command_code[0] && reponse[10] == _current_pmtk_command_code[1] && reponse[11] == _current_pmtk_command_code[2]) {
                char flag = reponse[12];
                _waiting_ack = false;
                if (flag == '3') {
                    _pmtk_command_result = true;
                }
            }
        }
        memset(reponse, 0, 120);
        memset(parameters, 0, (size_t)(sizeof(parameters[0][0]) * 19 * 10));
        index_carac = 0;
    }
}

void L86::update_informations(NmeaCommandType response_type, char **parameters)
{
    switch (response_type) {
        case NmeaCommandType::RMC:
            sprintf(this->longitude, "%s%c", parameters[4], parameters[5][0]);
            sprintf(this->latitude, "%s%c", parameters[2], parameters[3][0]);
            break;

        case NmeaCommandType::GGA:
            sprintf(this->longitude, "%s%c", parameters[3], parameters[4][0]);
            sprintf(this->latitude, "%s%c", parameters[1], parameters[2][0]);
            break;

        case NmeaCommandType::GLL:
            sprintf(this->longitude, "%s%c", parameters[2], parameters[3][0]);
            sprintf(this->latitude, "%s%c", parameters[0], parameters[1][0]);
            break;
    }
}

char *L86::get_last_received_command()
{
    return (char *)_last_received_command;
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

