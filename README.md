# Quectel L86 GNSS receiver
Quectel L86 GNSS receiver mbed OS library.

Quectel L86 module is able to receive various kinds of information: position, satellite data, speed.

The module uses several NMEA messages to communicate this information:

- Recommend Minimum Position Data (RMC)
- Track Made Good and Ground Speed (VTG)
- Global Positioning Fix Data (GGA)
- GNSS DOP and Active Satellites (GSA)
- GNSS Satellites in View (GSV)

Refer to the sections below on how to use the driver.

## Load the driver
Select the UART to communicate with the module:
```cpp
RawSerial serial(UART1_TX, UART1_RX, 9600);
L86 l86(&serial);
```

## Setup the module
Configure the module depending on your needs:
```cpp
l86.set_satellite_system(2,
        L86::SatelliteSystem::GPS,
        L86::SatelliteSystem::GLONASS);
l86.set_nmea_output_frequency(1,
        L86::NmeaFrequency::FIVE_POSITION_FIXES, 
        L86::NmeaCommandType::RMC);
l86.set_navigation_mode(L86::NavigationMode::NORMAL_MODE);
l86.set_position_fix_interval(10000);
```

## Start the module
Start the module:
```cpp
l86.start(L86::StartMode::HOT_START);
```

And get messages from the module:
```cpp
l86.start_receive();
```

The module will then search satellites.

After satellites are found, received values may be retrieved with:
```cpp
printf("Latitude:  %s\n", l86.get_latitude());
printf("Longitude: %s\n", l86.get_longitude());
```
