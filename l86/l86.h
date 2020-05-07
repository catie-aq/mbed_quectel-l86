#ifndef CATIE_SIXTRON_L86_H_
#define CATIE_SIXTRON_L86_H_

#include "mbed.h"
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



constexpr int PARAMETERS_COUNT_SATELLITE_SYSTEM = 5;
constexpr char SATELLITE_SYSTEM_CODE[] = "353";
constexpr int GPS_FLAG = 0;
constexpr int GLONASS_FLAG = 1;
constexpr int GALILEO_FLAG = 2;
constexpr int GALILEO_FULL_FLAG = 3;
constexpr int BEIDOU_FLAG = 4;

constexpr int PARAMETERS_COUNT_NMEA_OUTPUT_FREQUENCY = 5;
constexpr char NMEA_OUTPUT_FREQUENCY_CODE[] = "314";
constexpr int GLL_FREQUENCY = 0;
constexpr int RMC_FREQUENCY = 1;
constexpr int VTG_FREQUENCY = 2;
constexpr int GGA_FREQUENCY = 3;
constexpr int GSA_FREQUENCY = 4;
constexpr int GSV_FREQUENCY = 5;

constexpr int PARAMETERS_COUNT_NAVIGATION_MODE = 1;
constexpr char NAVIGATION_MODE_CODE[] = "886";
constexpr int NAVIGATION_MODE = 0;

constexpr int PARAMETERS_COUNT_POSITION_FIX_INTERVAL = 1;
constexpr char POSITION_FIX_INTERVAL_CODE[] = "220";
constexpr int INTERVAL = 0;

constexpr char FULL_COLD_START_MODE_CODE[] = "104";
constexpr char COLD_START_MODE_CODE[] = "103";
constexpr char WARM_START_MODE_CODE[] = "102";
constexpr char HOT_START_MODE_CODE[] = "101";

constexpr int PARAMETERS_COUNT_STANDBY_MODE = 1;
constexpr int STANDBY_MODE = 0;
constexpr int DEFAULT_PARAMETERS_COUNT_STANDBY_MODE = 6;

constexpr int NB_MAX_SATELLITES = 20;
constexpr int ID_PACKET_SIZE = 3;
constexpr int MAX_ANSWER_SIZE = 120;
constexpr int MAX_PARAMETERS_COUNT = 19;
constexpr int MAX_PARAMETER_SIZE = 10;
constexpr int PARAMETERS_BEGIN = 7;
constexpr int LIMIT_SATELLITES = 4;
constexpr int MAX_SATELLITES = 12;
constexpr char ACK_CODE[] = "001";

constexpr int PMTK_COMMAND_CODE_INDEX = 9;
constexpr int PMTK_COMMAND_RESULT = 13;
constexpr int PMTK_PACKET_SIZE = 100;
constexpr int PMTK_ANSWER_SIZE = 50;
constexpr int PMTK_PACKET_TYPE_INDEX = 5;
constexpr char INVALID_PACKET = '0';
constexpr char UNSUPPORTED_PACKET_TYPE = '1';
constexpr char VALID_PACKET_AND_ACTION_FAILED = '2';
constexpr char VALID_PACKET_AND_COMMAND_SUCCEED = '3';

constexpr int RMC_POSITIONNING_MODE = 11;
constexpr int RMC_DATE = 8;
constexpr int RMC_TIME = 0;
constexpr int RMC_LATITUDE = 2;
constexpr int RMC_LATITUDE_N_S = 3;
constexpr int RMC_LONGITUDE = 4;
constexpr int RMC_LONGITUDE_E_W = 5;
constexpr int RMC_SPEED_KNOTS = 6;

constexpr int VTG_POSITIONNING_MODE = 8;
constexpr int VTG_SPEED_KNOTS = 4;
constexpr int VTG_SPEED_KMH = 6;

constexpr int GGA_FIX_STATUS = 5;
constexpr int GGA_TIME = 0;
constexpr int GGA_LATITUDE = 1;
constexpr int GGA_LATITUDE_N_S = 2;
constexpr int GGA_LONGITUDE = 3;
constexpr int GGA_LONGITUDE_E_W = 4;
constexpr int GGA_ALTITUDE = 8;
constexpr int GGA_SATELLITE_COUNT = 6;

constexpr int GSA_FIX_SATELLITE_STATUS = 1;
constexpr int GSA_MODE = 0;
constexpr int GSA_DILUTION_OF_PRECISION_HORIZONTAL = 15;
constexpr int GSA_DILUTION_OF_PRECISION_POSITIONAL = 14;
constexpr int GSA_DILUTION_OF_PRECISION_VERTICAL = 15;

constexpr int GSV_MESSAGES_COUNT = 0;
constexpr int GSV_SEQUENCE_NUMBER = 1;
constexpr int GSV_SATELLITES_COUNT = 0;

constexpr int GLL_TIME = 4;
constexpr int GLL_POSITIONNING_MODE = 6;
constexpr int GLL_LATITUDE = 0;
constexpr int GLL_LATITUDE_N_S = 1;
constexpr int GLL_LONGITUDE = 2;
constexpr int GLL_LONGITUDE_E_W = 3;

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
    L86(RawSerial *uart);

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

    RawSerial *_uart;
    bool _waiting_ack;
    char _current_pmtk_command_code[ID_PACKET_SIZE];
    char _last_received_command[MAX_ANSWER_SIZE];
    bool _pmtk_command_result;
    int _registered_satellite_count;
    Position _position_informations;
    Movement _movement_informations;
    Informations _global_informations;
    Satellites_info _satellites_informations;
    DilutionOfPrecision _dilution_of_precision;

    typedef struct {
        char packet_type[ID_PACKET_SIZE];
        bool is_command;
        uint8_t nb_param;
        uint8_t anwser_size = MAX_PARAMETERS_COUNT;
        char **parameters;
        bool ack;
    } Pmtk_message;


    /*!
     *  Callback triggered when a caracter is received by UART
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

    void set_parameter(char parameters[][MAX_PARAMETER_SIZE], NmeaCommandType command_type);

    void set_positionning_mode(char c_positionning_mode);

    void set_fix_status(char c_fix_status);

    void set_fix_satellite_status(char c_fix_satellite_status);

    void set_mode(char c_mode);

    void set_time(char *time);

    void set_date(char *date);

    void set_longitude(char *longitude, char indicator);

    void set_latitude(char *latitude, char indicator);

};

#endif /* CATIE_SIXTRON_L86_H_ */
