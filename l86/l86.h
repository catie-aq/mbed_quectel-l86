#ifndef CATIE_SIXTRON_L86_H_
#define CATIE_SIXTRON_L86_H_

#include "mbed.h"
#include <cstdio>
#include <cstdlib>
#include <bitset>

typedef struct {
    char id[4];
    char elevation[2];
    char azimuth[3];
    char snr[5];
} Satellite;

typedef struct {
    char latitude[10];
    char altitude[6];
    char longitude[11];
} Position;

typedef struct {
    char speed_kmh[6];
    char speed_knots[6];
} Movement;

typedef struct {
    char time[10];
    char date[6];
    char positionning_mode[1];
    char fix_status[1];
} Informations;

typedef struct {
    int satellites_count;
    Satellite *satellites;
} Satellites_info;



class L86
{

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
#define SATELLITE_SYSTEMS_COUNT 5

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
#define NMEA_COMMANDS_COUNT 6

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

    typedef std::bitset<SATELLITE_SYSTEMS_COUNT> SatelliteSystems;

    typedef std::bitset<NMEA_COMMANDS_COUNT> NmeaCommands;

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
     *  Get last received command from the L86 module
     *
     */
    char *get_last_received_command();

    /*!
     *  Get position informations
     */
    Position get_position_informations();

    /*!
     *  Get movement informations
     */
    Movement get_movement_informations();

    /*!
     *  Get global informations
     */
    Informations get_global_informations();

    /*!
     *  Get satellites informations
     */
    Satellites_info get_satellites_informations();

private:

    RawSerial *_uart;
    bool _waiting_ack;
    char _current_pmtk_command_code[3];
    char _last_received_command[120];
    bool _pmtk_command_result;
    Position _position_informations;
    Movement _movement_informations;
    Informations _global_informations;
    Satellites_info _satellites_informations;

    typedef struct {
        char packet_type[3];
        bool is_command;
        uint8_t nb_param;
        uint8_t anwser_size = 19;
        char **parameters;
        bool ack;
    } Pmtk_message;


    /*!
     * Callback triggered when a caracter is received by UART
     *
     */
    void callback_rx(void);

    /*!
     *  Write a PMTK message which permit to configure the L86 module
     *
     *  \param message : PMTK message object which contains all necessary informations to send to the L86 module
     *
     */
    void write_pmtk_message(Pmtk_message message);

    /*!
     *  Calculate the message checksum
     *
     *  \param message PMTK message from which we calculate the checksum
     *
     */
    unsigned char calculate_checksum(char *message);

    /*!
     *  Start receiving message from L86 module
     *
     */
    void start_receive();

    /*!
     *  Stop receiving message from L86 module
     */
    void stop_receive();

    void set_parameter(char parameters[][10], NmeaCommandType command_type);

};

#endif /* CATIE_SIXTRON_L86_H_ */
