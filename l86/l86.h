#ifndef CATIE_SIXTRON_L86_H_
#define CATIE_SIXTRON_L86_H_

#include "mbed.h"
#include <cstdio>
#include <cstdlib>
#include <cstdarg>
#include <bitset>

#define SATELLITE_SYSTEMS_COUNT 5
#define NMEA_COMMANDS_COUNT 6

typedef struct {
    char packet_type[3];
    bool is_command;
    uint8_t nb_param;
    uint8_t anwser_size = 19;
    char **parameters;
    bool ack;
} Pmtk_message;

typedef std::bitset<SATELLITE_SYSTEMS_COUNT> SatelliteSystems;
typedef std::bitset<NMEA_COMMANDS_COUNT> NmeaCommands;

class L86
{

private:
    RawSerial *_uart;
    bool _waiting_ack;
    char _current_pmtk_command_code[3];
    char _last_received_command[120];
    bool _pmtk_command_result;

    /* Position attributes */
    char *longitude;
    char *latitude;

protected:

    /*!
     * Callback triggered when a caracter is received by UART
     *
     */
    void callback_rx(void);

public:

    /* Start mode*/
    enum class StartMode {
        FULL_COLD_START,
        COLD_START,
        WARM_START,
        HOT_START
    };

    /* Satellite system */
    enum class SatelliteSystem : size_t {
        GPS,
        GLONASS,
        GALILEO,
        GALILEO_FULL,
        BEIDOU
    };

    /* Standby mode */
    enum class StandbyMode {
        NORMAL_MODE,
        PERIODIC_BACKUP_MODE,
        PERIODIC_STANDBY_MODE,
        PERPETUAL_BACKUP_MODE,
        AL_STANDBY_MODE,
        AL_BACKUP_MODE
    };

    /* NMEA commande types */
    enum class NmeaCommandType : size_t {
        RMC,
        VTG,
        GGA,
        GSA,
        GSV,
        GLL
    };

    /* Frequencies supported */
    enum class NmeaFrequency {
        ONE_POSITION_FIX,
        TWO_POSITION_FIXES,
        THREE_POSITION_FIXES,
        FOUR_POSITION_FIXES,
        FIVE_POSITION_FIXES
    };

    /* Navigation mode */
    enum class NavigationMode {
        NORMAL_MODE,
        RUNNING_MODE,
        AVIATION_MODE,
        BALLOON_MODE
    };


    /*!
    *  Default L86 constructor
    *
    *  \param uart
    */
    L86(RawSerial *uart);

    /*!
     *  Get the last received latitude
     *
     */
    char *get_latitude();

    /*!
     *  Get the last received longitude
     *
     */
    char *get_longitude();

    /*!
     *  Select a satellite system
     *
     *  \param satellite_system (GPS, GLONASS, GALILEO, BEIDOU)
     */
    void set_satellite_system(SatelliteSystems satellite_system);

    /*!
     *  Select NMEA output frequencies
     *
     *  \param nmea_trame (RMC, VTG, GGA, GSA, GSV, GLL)
     *  \param frequency
     */
    void set_nmea_output_frequency(NmeaCommands nmea_commands, NmeaFrequency frequency);

    /*!
     *  Select navigation mode
     *
     *  \param navigation_mode (normal, running, aviation, balloon)
     */
    void set_navigation_mode(NavigationMode navigation_mode);

    /*!
     *  Set position fix interval
     *
     *  \param interval
     */
    void set_position_fix_interval(uint16_t interval);

    /*!
     *  Write a PMTK message which permit to configure the L86 module
     *
     *  \param message : PMTK message object which contains all necessary informations to send to the L86 module
     *
     */
    void write_pmtk_message(Pmtk_message message);

    /*!
     *  Start the L86 module in the specified mode
     *
     *  \param start_mode (full cold, cold, warm, hot)
     */
    void start(StartMode start_mode);

    /*!
     *  Put the module in periodic standby mode
     *
     *  \param standby_mode (normal, periodic backup, periodic standby, periodic backup, AlwaysLocate standby, AlwaysLocate backup)
     */
    void standby_mode(StandbyMode standby_mode);

    /*!
     *  Update latitude and longitude values
     *
     *  \param response_type : NMEA command received type (RMC, VTG, GGA, GSA, GSV, GLL)
     *  \param parameters : every parameters received in the NMEA message
     */
    void update_informations(NmeaCommandType response_type, char **parameters);

    /*!
     *  Calculate the message checksum
     *
     *  \param message PMTK message from which we calculate the checksum
     *
     */
    unsigned char calculate_checksum(char *message);

    char *get_last_received_command();


    void start_attach();
    void stop_attach();

};

#endif /* CATIE_SIXTRON_L86_H_ */
