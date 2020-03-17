# Quectel L86 Gnss receiver

Quectel L86 Gnss receiver mbed OS library.

Quectel L86 module could receive a lot of differents information (position, satellites informations, speed).
The module uses several NMEA trames to communicates those informations : 
- Recommend Minimum Position Data (RMC)
- Track Made Good and Ground Speed (VTG)
- Global Positionning Fix Data (GGA)
- GNSS DOP and Active Satellites (GSA)
- GNSS Satellites in View (GSV)

There are several steps to follow in order to use the driver :

1) Create a driver instance and add serial instance with the communication pins with the module :

```
RawSerial serial(UART1_TX, UART1_RX, 9600);   
L86 l86(&serial);
```

2) Configure the module with the dedicated methods depending on your needs :
```
l86.set_satellite_system(2, L86::SatelliteSystem::GPS, 
                        L86::SatelliteSystem::GLONASS);
l86.set_nmea_output_frequency(1, L86::NmeaFrequency::FIVE_POSITION_FIXES, 
                              L86::NmeaCommandType::RMC);
l86.set_navigation_mode(L86::NavigationMode::NORMAL_MODE);
l86.set_position_fix_interval(10000);
```

3) Start the L86 module :
```
l86.start(L86::StartMode::HOT_START);
```

4) Start to receive messages from the module :
```
l86.start_receive();
```

After those steps, the module will search satellites.
When satellites are found, it's possible to access to received values with dedicated methods :
```
swo.printf("Latitude : %s\n", l86.get_latitude());
swo.printf("Longitude : %s\n", l86.get_longitude());
```