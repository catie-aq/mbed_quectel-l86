#ifndef CATIE_SIXTRON_L86_H_
#define CATIE_SIXTRON_L86_H_

#include "mbed.h"
#include "UnbufferedSerial.h"
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <bitset>

#define MBED_CONF_L86_SPEED_UNIT SpeedUnit::KMH

#ifdef MBED_CONF_L86_SPEED_UNIT_KMH
#define MBED_CONF_L86_SPEED_UNIT SpeedUnit::KMH
#endif

#ifdef MBED_CONF_L86_SPEED_UNIT_KNOTS
#define MBED_CONF_L86_SPEED_UNIT SpeedUnit::KNOTS
#endif


class L86
{

public:

    enum class PositionningMode {
        NO_FIX,
        AUTONOMOUS_GNSS_FIX,
        DIFFERENTIAL_GNSS_FIX,
        UNKNOWN
    };

    enum class FixStatusGGA {
        INVALID,
        GNSS_FIX,
        DGPS_FIX,
        ESTIMATED_MODE,
        UNKNOWN
    };

    enum class FixStatusGSA {
        NOFIX,
        FIX2D,
        FIX3D,
        UNKNOWN
    };

    enum class Mode {
        MANUAL_SWITCH,
        AUTOMATIC_SWITCH,
        UNKNOWN
    };

    typedef struct {
        uint16_t id;
        uint16_t elevation;
        uint16_t azimuth;
        uint16_t snr;
    } Satellite;

    typedef struct {
        double latitude;
        double altitude;
        double longitude;
    } Position;

    typedef struct {
        double speed_kmh;
        double speed_knots;
    } Movement;

    typedef struct {
        tm time;
        PositionningMode positionning_mode;
        FixStatusGGA fix_status;
    } Informations;

    constexpr static int NB_MAX_SATELLITES = 20;       //!< Max number of satellites which are communating with L86 GNSS module
    typedef struct {
        int satellite_count;
        Mode mode;
        FixStatusGSA status;
        Satellite satellites[NB_MAX_SATELLITES];
    } Satellites_info;

    typedef struct {
        double positional;
        double horizontal;
        double vertical;
    } DilutionOfPrecision;

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

    enum class SpeedUnit {
        KMH,
        KNOTS
    };

    typedef std::bitset<SATELLITE_SYSTEMS_COUNT> SatelliteSystems;

    typedef std::bitset<NMEA_COMMANDS_COUNT> NmeaCommands;

    /*!
    *  Default L86 constructor
    *
    *  \param uart
    */
    L86(BufferedSerial *uart);

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

    Satellite *satellites();

    double latitude();

    double longitude();

    double altitude();

    double speed(SpeedUnit unit);

    double speed();

    time_t time();

    PositionningMode positionning_mode();

    FixStatusGGA fix_status();

    FixStatusGSA fix_satellite_status();

    int satellite_count();

    Mode mode();

    DilutionOfPrecision dilution_of_precision();

    int registered_satellite_count();


private:

    constexpr static int MAX_MESSAGE_SIZE = 200;        //!< Maximum received message size
    constexpr static int ID_PACKET_SIZE = 3;           //!< Command code size

    BufferedSerial *_uart;
    bool _waiting_ack;
    char _current_pmtk_command_code[ID_PACKET_SIZE];
    bool _pmtk_command_result;
    int _registered_satellite_count;
    char received_command[MAX_MESSAGE_SIZE];
    Position _position_informations;
    Movement _movement_informations;
    Informations _global_informations;
    Satellites_info _satellites_informations;
    DilutionOfPrecision _dilution_of_precision;

    constexpr static int MAX_PARAMETERS_COUNT = 19;    //!< Command parameters maximum number
    typedef struct {
        char packet_type[ID_PACKET_SIZE];
        bool is_command;
        uint8_t nb_param;
        uint8_t anwser_size = MAX_PARAMETERS_COUNT;
        char **parameters;
        bool ack;
    } Pmtk_message;

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

    /*!
     *  Callback called when the BufferedSerial RX file state changes
     *  Add received character to the received command buffer
     *  And parse the received message when it's completed
     */
    void analyze_receiving();

    constexpr static int MAX_PARAMETER_SIZE = 10;      //!< Command parameter maximum size
    void set_parameter(char parameters[][MAX_PARAMETER_SIZE], NmeaCommandType command_type);

    void set_positionning_mode(char c_positionning_mode);

    void set_fix_status(char c_fix_status);

    void set_fix_satellite_status(char c_fix_satellite_status);

    void set_mode(char c_mode);

    void set_time(char *time);

    void set_date(char *date);

    void set_longitude(char *longitude, char indicator);

    void set_latitude(char *latitude, char indicator);

    bool verify_checksum(char *message);

};

#endif /* CATIE_SIXTRON_L86_H_ */
