/*
 * Copyright (c) 2020-2021, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CATIE_SIXTRON_L86_H_
#define CATIE_SIXTRON_L86_H_

#include "mbed.h"

#include <bitset>
#include <cstdio>
#include <cstdlib>
#include <ctime>

#include "minmea.h"

#define MBED_CONF_L86_SPEED_UNIT SpeedUnit::KMH

#ifdef MBED_CONF_L86_SPEED_UNIT_KMH
#define MBED_CONF_L86_SPEED_UNIT SpeedUnit::KMH
#endif

#ifdef MBED_CONF_L86_SPEED_UNIT_KNOTS
#define MBED_CONF_L86_SPEED_UNIT SpeedUnit::KNOTS
#endif

class L86 {

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
        float latitude;
        float altitude;
        float longitude;
        float magnetic_variation;
    } Position;

    typedef struct {
        float speed_kmh;
        float speed_knots;
        float course_over_ground;
    } Movement;

    typedef struct {
        tm time;
        PositionningMode positionning_mode;
        FixStatusGGA fix_status;
    } Informations;

    constexpr static int MAX_SATELLITES
            = 12; //!< Max number of satellites which are communating with L86 GNSS module

    typedef struct {
        int satellite_count;
        Mode mode;
        FixStatusGSA status;
        Satellite satellites[MAX_SATELLITES];
    } Satellites_info;

    typedef struct {
        float positional;
        float horizontal;
        float vertical;
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
        NORMAL_MODE = 0,
        PERIODIC_BACKUP_MODE = 1,
        PERIODIC_STANDBY_MODE = 2,
        PERPETUAL_BACKUP_MODE = 4,
        AL_STANDBY_MODE = 8,
        AL_BACKUP_MODE = 9
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
        ONE_POSITION_FIX = 1,
        TWO_POSITION_FIXES = 2,
        THREE_POSITION_FIXES = 3,
        FOUR_POSITION_FIXES = 4,
        FIVE_POSITION_FIXES = 5
    };

    /* Navigation mode */
    enum class NavigationMode {
        NORMAL_MODE = 0,
        RUNNING_MODE = 1,
        AVIATION_MODE = 2,
        BALLOON_MODE = 3
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
    L86(UnbufferedSerial *uart);

    /*!
     *  Select a satellite system
     *
     *  \param satellite_system (GPS, GLONASS, GALILEO, BEIDOU)
     */
    bool set_satellite_system(SatelliteSystems satellite_system);

    /*!
     *  Select NMEA output frequencies
     *
     *  \param nmea_trame (RMC, VTG, GGA, GSA, GSV, GLL)
     *  \param frequency
     */
    bool set_nmea_output_frequency(NmeaCommands nmea_commands, NmeaFrequency frequency);

    /*!
     *  Select navigation mode
     *
     *  \param navigation_mode (normal, running, aviation, balloon)
     */
    bool set_navigation_mode(NavigationMode navigation_mode);

    /*!
     *  Set position fix interval
     *
     *  \param interval
     */
    bool set_position_fix_interval(uint16_t interval);

    /*!
     *  Start the L86 module in the specified mode
     *
     *  \param start_mode (full cold, cold, warm, hot)
     */
    bool start(StartMode start_mode);

    /*!
     *  Put the module in periodic standby mode
     *
     *  \param standby_mode (normal, periodic backup, periodic standby, periodic backup,
     * AlwaysLocate standby, AlwaysLocate backup)
     */
    bool standby_mode(StandbyMode standby_mode);

    /*!
     *  Start receiving message from L86 module
     *
     */
    void start_receive();

    /*!
     *  Stop receiving message from L86 module
     */
    void stop_receive();

    Satellite *satellites();

    float latitude();

    float longitude();

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
    UnbufferedSerial *_uart;
    minmea_sentence_pmtk _current_pmtk_message;
    int _registered_satellite_count;
    char _received_message[MINMEA_MAX_LENGTH];
    Position _position_informations;
    Movement _movement_informations;
    Informations _global_informations;
    Satellites_info _satellites_informations;
    DilutionOfPrecision _dilution_of_precision;

    /*!
     *  Generate PMTK message and send it through serial communication
     *
     *  \param message : PMTK message object which contains all necessary informations to send to
     * the L86 module
     *
     *  \return true if pmtk message action is succesfully executed on the module else return false
     */
    bool generate_and_send_pmtk_message(minmea_sentence_pmtk message);

    /*!
     *  Callback called when the BufferedSerial RX file state changes
     *  Add received character to the received message buffer
     *  And parse the received message when it's completed
     */
    void get_received_message();

    void set_positionning_mode(char c_positionning_mode);

    void set_fix_status(int c_fix_status);

    void set_fix_satellite_status(int c_fix_satellite_status);

    void set_mode(char c_mode);

    void set_time(minmea_time time);

    void set_date(minmea_date date);

    void set_longitude(minmea_float longitude);

    void set_latitude(minmea_float latitude);

    void parse_message(char *message);
};

#endif /* CATIE_SIXTRON_L86_H_ */
