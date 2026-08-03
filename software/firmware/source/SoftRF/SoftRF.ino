/*
 * SoftRF(.ino) firmware
 * Copyright (C) 2016-2026 Linar Yusupov
 *
 * Author: Linar Yusupov, linar.r.yusupov@gmail.com
 *
 * Web: http://github.com/lyusupov/SoftRF
 *
 * Credits:
 *   Arduino core for ESP8266 is developed/supported by ESP8266 Community (support-esp8266@esp8266.com)
 *   AVR/Arduino nRF905 Library/Driver is developed by Zak Kemble, contact@zakkemble.co.uk
 *   flarm_decode is developed by Stanislaw Pusep, http://github.com/creaktive
 *   Arduino Time Library is developed by Paul Stoffregen, http://github.com/PaulStoffregen
 *   "Aircraft" and MAVLink Libraries are developed by Andy Little
 *   TinyGPS++ and PString Libraries are developed by Mikal Hart
 *   Adafruit NeoPixel Library is developed by Phil Burgess, Michael Miller and others
 *   TrueRandom Library is developed by Peter Knight
 *   IBM LMIC and Semtech Basic MAC frameworks for Arduino are maintained by Matthijs Kooijman
 *   ESP8266FtpServer is developed by David Paiva
 *   Lib_crc is developed by Lammert Bies
 *   OGN/ADS-L library is developed by Pawel Jalocha
 *   NMEA library is developed by Timur Sinitsyn, Tobias Simon, Ferry Huberts
 *   ADS-B encoder C++ library is developed by yangbinbin (yangbinbin_ytu@163.com)
 *   Arduino Core for ESP32 is developed by Hristo Gochkov
 *   ESP32 BT SPP library is developed by Evandro Copercini
 *   Adafruit BMP085 library is developed by Limor Fried ( Ladyada )
 *   Adafruit BMP280 library is developed by Kevin Townsend
 *   Adafruit MPL3115A2 and LIS3DH libraries are developed by Limor Fried and Kevin Townsend
 *   U8g2 monochrome LCD, OLED and eInk library is developed by Oliver Kraus
 *   NeoPixelBus library is developed by Michael Miller
 *   jQuery library is developed by JS Foundation
 *   EGM96 data is developed by XCSoar team
 *   BCM2835 and RadioHead libraries are developed by Mike McCauley
 *   SimpleNetwork library is developed by Dario Longobardi
 *   ArduinoJson library is developed by Benoit Blanchon
 *   Flashrom library is part of the flashrom.org project
 *   Arduino Core for TI CC13X0 and CC13X2 is developed by Robert Wessels
 *   EasyLink library is developed by Robert Wessels and Tony Cave
 *   Dump978 library is developed by Oliver Jowett
 *   FEC library is developed by Phil Karn
 *   PCF8563, AXP20X, XPowersLib and SensorLib libraries are developed by Lewis He
 *   Arduino Core for STM32 is developed by Frederic Pillon
 *   TFT library is developed by Bodmer
 *   STM32duino Low Power and RTC libraries are developed by Wi6Labs
 *   Basic MAC library is developed by Michael Kuyper
 *   LowPowerLab SPIFlash library is maintained by Felix Rusu
 *   Arduino Core for ASR6x0x is developed by Aaron Lee (HelTec Automation)
 *   ADXL362 library is developed by Klaas-Jan Winkel
 *   Arduino Core for nRF52 and TinyUSB library are developed by Ha Thach
 *   Arduino-NVM library is developed by Frank Holtz
 *   AceButton library is developed by Brian Park
 *   GxEPD2 library is developed by Jean-Marc Zingg
 *   Adafruit GFX, BusIO, INA219 and BME680 libraries are developed by Adafruit Industries
 *   U8g2 fonts for Adafruit GFX are developed by Oliver Kraus
 *   Adafruit SPIFlash and SleepyDog libraries are developed by Adafruit Industries
 *   SdFat library is developed by Bill Greiman
 *   Arduino MIDI library is developed by Francois Best (Forty Seven Effects)
 *   Arduino uCDB library is developed by Ioulianos Kakoulidis
 *   Arduino Cores for Atmel SAMD and Renesas RA4 are developed by Arduino LLC
 *   FlashStorage library is developed by Arduino LLC
 *   USB host library 2.0 for Zero/M0/SAMD is developed by gdsports625@gmail.com
 *   Arduino Core for RP2XXX and ESP8266Audio library are developed by Earle Philhower
 *   MPU-9250 9 DoF sensor library is developed by Kris Winer and Hideaki Tai
 *   Pico PIO USB library is developed by sekigon-gonnoc
 *   IniFile library is developed by Steve Marple
 *   MD5 library is developed by Vasilis Georgitzikis
 *   LibAPRS-ESP32 library is developed by Evan Krall and Somkiat Nakhonthai
 *   SA818 library is developed by Gibbon Zen
 *   OpenDroneID library is developed by Steve Jack
 *   WiFiWebServer and Functional-Vlpp libraries are developed by Khoi Hoang
 *   RadioLib library is developed by Jan Gromes
 *   Arduino Core for EFR32 Series 2 is developed by Tamas Jozsi
 *   ArduinoBLE library is developed by Arduino LLC
 *   Arduino Core for CH32 is developed by Tianpei Lee
 *   QMA6100P library is developed by Tristan Alderson
 *   Arduino Core for nRF54L15 is developed by Loren Bufanu
 *   Arduino EEPROM library is developed by Jack Christensen
 *   Adafruit SPA06-003 library is developed by Limor Fried
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define MAVLINK_GCS

#include "src/system/OTA.h"
#include "src/system/Time.h"
#include "src/driver/LED.h"
#include "src/driver/GNSS.h"
#include "src/driver/RF.h"
#include "src/driver/Sound.h"
#include "src/driver/EEPROM.h"
#include "src/driver/Battery.h"
#include "src/protocol/data/MAVLink.h"
#include "src/protocol/data/GDL90.h"
#include "src/protocol/data/NMEA.h"
#include "src/protocol/data/D1090.h"
#include "src/system/SoC.h"
#include "src/driver/WiFi.h"
#include "src/ui/Web.h"
#include "src/driver/Baro.h"
#include "src/TTNHelper.h"
#include "src/TrafficHelper.h"
#include "src/system/Recorder.h"

#if defined(ENABLE_AHRS)
#include "src/driver/AHRS.h"
#endif /* ENABLE_AHRS */

#if !defined(SERIAL_FLUSH)
#define SERIAL_FLUSH() Serial.flush()
#endif

#define DEBUG 0
#define DEBUG_TIMING 0

#define isTimeToDisplay() (millis() - LEDTimeMarker     > 1000)
#define isTimeToExport()  (millis() - ExportTimeMarker  > 1000)

ufo_t ThisAircraft;

hardware_info_t hw_info = {
  .model    = DEFAULT_SOFTRF_MODEL,
  .revision = 0,
  .soc      = SOC_NONE,
  .rf       = RF_IC_NONE,
  .gnss     = GNSS_MODULE_NONE,
  .baro     = BARO_MODULE_NONE,
  .display  = DISPLAY_NONE,
  .storage  = STORAGE_NONE,
  .rtc      = RTC_NONE,
  .imu      = IMU_NONE,
  .mag      = MAG_NONE,
  .pmu      = PMU_NONE,
  .audio    = AUDIO_NONE,
  .touch    = TOUCH_NONE,
  .haptic   = HAPTIC_NONE,
  .camera   = CAMERA_NONE,
};

unsigned long LEDTimeMarker = 0;
unsigned long ExportTimeMarker = 0;

void setup()
{
  rst_info *resetInfo;

  hw_info.soc = SoC_setup(); // Has to be very first procedure in the execution order # STM32 platform setup

  resetInfo = (rst_info *) SoC->getResetInfoPtr();

  Serial.println();
  Serial.print(F(SOFTRF_IDENT "-"));
  Serial.print(SoC->name);
  Serial.print(F(" FW.REV: " SOFTRF_FIRMWARE_VERSION " DEV.ID: "));
  Serial.println(String(SoC->getChipId(), HEX));
  Serial.println(F("Copyright (C) 2015-2026 Linar Yusupov. All rights reserved."));

  SERIAL_FLUSH();

  if (resetInfo) {
    Serial.println(""); Serial.print(F("Reset reason: ")); Serial.println(resetInfo->reason);
  }
  Serial.println(SoC->getResetReason());
  Serial.print(F("Free heap size: ")); Serial.println(SoC->getFreeHeap());
  Serial.println(SoC->getResetInfo()); Serial.println("");

  SERIAL_FLUSH();

  EEPROM_setup(); // # Maybe need to change this if adding another mode? 

  SoC->Button_setup();

  ThisAircraft.addr = SoC->getChipId() & 0x00FFFFFF;

  hw_info.rf = RF_setup(); // # RF setup should be unaffected

  delay(100);

  hw_info.baro = Baro_setup(); // # Baro setup should be unaffected - no Baro detected
#if defined(ENABLE_AHRS)
  hw_info.imu = AHRS_setup();
#endif /* ENABLE_AHRS */
  hw_info.display = SoC->Display_setup(); // # Display not connected

#if !defined(EXCLUDE_MAVLINK)
  if (settings->mode == SOFTRF_MODE_UAV) { // # Use MODE UAV to test MAVlink ADS-B stream
    if (hw_info.model == SOFTRF_MODEL_STANDALONE) {
      Serial.begin(57600);
    }
    MAVLink_setup(); // # This just starts the SoC Software Serial port for MAVLink data stream
    ThisAircraft.aircraft_type = AIRCRAFT_TYPE_UAV;  
  }  else
#endif /* EXCLUDE_MAVLINK */
  {
    hw_info.gnss = GNSS_setup(); // # This should probe for the GNSS module and find that none is connected, so it should return GNSS_MODULE_NONE
    ThisAircraft.aircraft_type = settings->aircraft_type;
  }
  ThisAircraft.protocol = settings->rf_protocol;
  ThisAircraft.stealth  = settings->stealth;
  ThisAircraft.no_track = settings->no_track;

  Battery_setup(); // # Should be unaffected
  Traffic_setup(); // # Should be unaffected

  SoC->swSer_enableRx(false); // # Should be unaffected

  LED_setup(); // # Should be unaffected

  WiFi_setup(); // # Should be unaffected

  if (SoC->USB_ops) {
     SoC->USB_ops->setup(); // # Should be unaffected
  }

  if (SoC->Bluetooth_ops) {
     SoC->Bluetooth_ops->setup(); // # Should be unaffected
  }

  OTA_setup(); // # Should be unaffected
  Web_setup(); // # Should be unaffected
  NMEA_setup(); // # Should be unaffected

#if defined(ENABLE_TTN)
  TTN_setup();
#endif

  delay(1000);

  /* expedite restart on WDT reset */
  if (resetInfo->reason != REASON_WDT_RST) {
    LED_test();
  }

  Sound_setup();
  SoC->Sound_test(resetInfo->reason);

  switch (settings->mode)
  {
  case SOFTRF_MODE_TXRX_TEST:
  case SOFTRF_MODE_WATCHOUT:
    Time_setup();
    break;
  case SOFTRF_MODE_BRIDGE:
    break;
  case SOFTRF_MODE_NORMAL:
  case SOFTRF_MODE_UAV:
  default:
    SoC->swSer_enableRx(true); // # Not sure what this is doing as the protoype function in STM32.cpp is empty, maybe this is just a flag?
    break;
  }

  Recorder_setup(); // # Only records if GPS is installed

  SoC->post_init(); // # Looks like the init on line 789 of STM32.cpp shows either normal or UAV mode, so UAV mode may have other things happening that i need to be aware of. 

  SoC->WDT_setup(); // Init the watchdog timer with 5 seconds timeout
}

void loop()
{
  // Do common RF stuff first
  RF_loop();

  switch (settings->mode)
  {
#if !defined(EXCLUDE_TEST_MODE)
  case SOFTRF_MODE_TXRX_TEST:
    txrx_test();
    break;
#endif /* EXCLUDE_TEST_MODE */
#if !defined(EXCLUDE_MAVLINK)
  case SOFTRF_MODE_UAV: // We are going to be using this mode to test the MAVLink ADS-B stream
    uav();
    break;
#endif /* EXCLUDE_MAVLINK */
#if !defined(EXCLUDE_WIFI)
  case SOFTRF_MODE_BRIDGE:
    bridge();
    break;
#endif /* EXCLUDE_WIFI */
#if !defined(EXCLUDE_WATCHOUT_MODE)
  case SOFTRF_MODE_WATCHOUT:
    watchout();
    break;
#endif /* EXCLUDE_WATCHOUT_MODE */
  case SOFTRF_MODE_NORMAL:
  default:
    normal();
    break;
  }

  // Show status info on tiny OLED display
  SoC->Display_loop();

  // battery status LED
  LED_loop();

  // Handle DNS
  WiFi_loop();

  // Handle Web
  Web_loop();

  // Handle OTA update.
  OTA_loop();

  Recorder_loop();

  SoC->loop();

  if (SoC->Bluetooth_ops) {
    SoC->Bluetooth_ops->loop();
  }

  if (SoC->USB_ops) {
    SoC->USB_ops->loop();
  }

  if (SoC->UART_ops) {
     SoC->UART_ops->loop();
  }

  Battery_loop();

  SoC->Button_loop();

  Time_loop();

  yield();
}

void shutdown(int reason)
{
  SoC->WDT_fini();

  SoC->swSer_enableRx(false);

  Recorder_fini();

  Sound_fini();

  NMEA_fini();

  Web_fini();

  if (SoC->Bluetooth_ops) {
     SoC->Bluetooth_ops->fini();
  }

  if (SoC->USB_ops) {
     SoC->USB_ops->fini();
  }

  WiFi_fini();

  if (settings->mode != SOFTRF_MODE_UAV) {
    GNSS_fini();
  }

  SoC->Display_fini(reason);

  Baro_fini();

  RF_Shutdown();

  SoC->Button_fini();

  SoC_fini(reason);
}

void normal()
{
  bool success;

  Baro_loop();

#if defined(ENABLE_AHRS)
  AHRS_loop();
#endif /* ENABLE_AHRS */

  GNSS_loop();

  ThisAircraft.timestamp = now();
  if (isValidFix()) {
    ThisAircraft.latitude  = gnss.location.lat();
    ThisAircraft.longitude = gnss.location.lng();
    ThisAircraft.altitude  = gnss.altitude.meters();
    ThisAircraft.course    = gnss.course.deg();
    ThisAircraft.speed     = gnss.speed.knots();
    ThisAircraft.hdop      = (uint16_t) gnss.hdop.value();
    ThisAircraft.geoid_separation = gnss.separation.meters();

#if !defined(EXCLUDE_EGM96)
    /*
     * When geoidal separation is zero or not available - use approx. EGM96 value
     */
    if (ThisAircraft.geoid_separation == 0.0) {
      ThisAircraft.geoid_separation = (float) LookupSeparation(
                                                ThisAircraft.latitude,
                                                ThisAircraft.longitude
                                              );
      /* we can assume the GPS unit is giving ellipsoid height */
      ThisAircraft.altitude -= ThisAircraft.geoid_separation;
    }
#endif /* EXCLUDE_EGM96 */

    RF_Transmit(RF_Encode(&ThisAircraft), true);
  }

  success = RF_Receive();

#if DEBUG
  success = true;
#endif

  if (success && isValidFix()) ParseData();

#if defined(ENABLE_TTN)
  TTN_loop();
#endif

  if (isValidFix()) {
    Traffic_loop();
  }

  if (isTimeToDisplay()) {
    if (isValidFix()) {
      LED_DisplayTraffic();
    } else {
      LED_Clear();
    }
    LEDTimeMarker = millis();
  }

  Sound_loop();

  if (isTimeToExport()) {
    NMEA_Export();
    GDL90_Export();
    D1090_Export();

    ExportTimeMarker = millis();
  }

  // Handle Air Connect
  NMEA_loop();

  ClearExpired();
}

#if !defined(EXCLUDE_MAVLINK)
#if defined(MAVLINK_GCS)
void uav()
{
  bool success = false;

  /*
    Overall notes:
    It looks like we need gps to give a time sync to the UAV for the MAVlink ADS-B stream to work.
    We can try without, however the clearexpired function will not work. 
    I have setup a second uav function which we can try and use and see if it works. 
  
  */

  Baro_loop();
  GNSS_loop();  

  ThisAircraft.timestamp = now();

  success = RF_Receive();

  if (success && isValidFix()) ParseData();

  if (isValidFix()) {
    Traffic_loop();
  }

  if (isTimeToExport()) {
    MAVLinkShareTraffic();
    ExportTimeMarker = millis();
  }

  NMEA_loop();

  ClearExpired();
  /*
  We need thisaircraft.timestamp to be updated to make this work. However,
  we are not getting a valid fix, so thisaircraft.timestamp is not being updated. 
  We need to find another way to update thisaircraft.timestamp without a valid fix. 
  */  
}
#else
void uav()
{
  bool success = false;

  /*
    Overall notes:
    It looks like we need gps to give a time sync to the UAV for the MAVlink ADS-B stream to work.
    We can try without, however the clearexpired function will not work. 
    I have setup a second uav function which we can try and use and see if it works. 
  
  */

  PickMAVLinkFix(); 
  /*
  From MAVLink.cpp:  
    read_mavlink(); -> mavlink.cpp
    This reads the MAVlink data from Serial3 for STM32. If it reads a MAVlink message, it will show as 
    mavlink_active. 
    It is looking for a MAVlink heartbeat message. It will updated using do_mavlink_heartbeat.
    It also looks for system time and sys status messages, and will update the gps time stamp and stats like
    batt voltage and nave mode. 
    We dont need any of the above, so we need to do this another way.   
    *** Need to confirm if any of the above is needed for the MAVlink ADS-B stream to work. ***

  send_mavlink_heartbeat();
    This sends a MAVlink heartbeat message to the UAV. It is not needed for our purposes, so we will not call it.
  */

  MAVLinkTimeSync();
  /*
    This relies on gps.fix_type being set to something other than 0 for the MAVlink time sync to work.
    If no GPS is connected, then gps.fix_type will be 0, and the MAVlink time sync will not work.
    We need to find another way to get the MAVlink time sync to work without a GPS
  */
  MAVLinkSetWiFiPower();
  /*
    This relies on the heartbe at message being received from the UAV. If no heartbeat message is received, then the WiFi power will not be set.
    We need to find another way to set the WiFi power without a heartbeat message.
  */

  hw_info.gnss = get_num_heartbeats() > 0 ? GNSS_MODULE_MAV : GNSS_MODULE_NONE;
  /*
    Not 100% on what this is doing or if it is required. 
  */

  ThisAircraft.timestamp = now();

  if (isValidMAVFix()) {
    ThisAircraft.latitude  = the_aircraft.location.gps_lat / 1e7;
    ThisAircraft.longitude = the_aircraft.location.gps_lon / 1e7;
    ThisAircraft.altitude  = the_aircraft.location.gps_alt / 1000.0;
    ThisAircraft.course    = the_aircraft.location.gps_cog;
    ThisAircraft.speed     = (the_aircraft.location.gps_vog / 100.0) / _GPS_MPS_PER_KNOT;
    ThisAircraft.hdop      = the_aircraft.location.gps_hdop;
    ThisAircraft.pressure_altitude = the_aircraft.location.baro_alt;

#if !defined(EXCLUDE_EGM96)
    /*
     * When geoidal separation is not available - use approx. EGM96 value
     */
    ThisAircraft.geoid_separation = (float) LookupSeparation(
                                              ThisAircraft.latitude,
                                              ThisAircraft.longitude
                                              );
#endif /* EXCLUDE_EGM96 */

    RF_Transmit(RF_Encode(&ThisAircraft), true);
  }

  /*
  The above is for transmitting data, which we are not. 
  From radiolib.cpp, line 6061:
    static bool lr20xx_transmit()  
    ... 
    return false;  no transmit on 1090 or 978 MHz   
  This means no transmit will occur on 1090 or 978 MHz, which is what we want.
  */  


  success = RF_Receive();
  /*
  Okay - so this is what we want! This is the functiuon that reads ADS-B data. 
  It doesnt look like it needs anything else to do this, so we should be able to just call this function and get the ADS-B data.

  */

  if (success && isValidMAVFix()) ParseData();

  /*
  Should be able to change this to check for success and then call ParseData() to get the ADS-B data. 
  */

  if (isTimeToExport() && isValidMAVFix()) {
    MAVLinkShareTraffic();
    ExportTimeMarker = millis();
  }

   /*
  Should be able to change this to check for success and then call ParseData() to get the ADS-B data. 
  */

  ClearExpired();
  /*
  We need thisaircraft.timestamp to be updated to make this work. However,
  we are not getting a valid fix, so thisaircraft.timestamp is not being updated. 
  We need to find another way to update thisaircraft.timestamp without a valid fix. 
  */
  
}
#endif /* MAVLINK_GCS */
#endif /* EXCLUDE_MAVLINK */

#if !defined(EXCLUDE_WIFI)
void bridge()
{
  bool success;

  size_t tx_size = Raw_Receive_UDP(&TxBuffer[0]);

  if (tx_size > 0) {
    RF_Transmit(tx_size, true);
  }

  success = RF_Receive();

  if(success)
  {
    size_t rx_size = RF_Payload_Size(settings->rf_protocol);
    rx_size = rx_size > sizeof(fo.raw) ? sizeof(fo.raw) : rx_size;

    memset(fo.raw, 0, sizeof(fo.raw));
    memcpy(fo.raw, RxBuffer, rx_size);

    if (settings->nmea_p) {
      StdOut.print(F("$PSRFI,"));
      StdOut.print((unsigned long) now());    StdOut.print(F(","));
      StdOut.print(Bin2Hex(fo.raw, rx_size)); StdOut.print(F(","));
      StdOut.println(RF_last_rssi);
    }

    Raw_Transmit_UDP();
  }

  if (isTimeToDisplay()) {
    LED_Clear();
    LEDTimeMarker = millis();
  }
}
#endif /* EXCLUDE_WIFI */

#if !defined(EXCLUDE_WATCHOUT_MODE)
void watchout()
{
  bool success;

  success = RF_Receive();

  if (success) {
    size_t rx_size = RF_Payload_Size(settings->rf_protocol);
    rx_size = rx_size > sizeof(fo.raw) ? sizeof(fo.raw) : rx_size;

    memset(fo.raw, 0, sizeof(fo.raw));
    memcpy(fo.raw, RxBuffer, rx_size);

    if (settings->nmea_p) {
      StdOut.print(F("$PSRFI,"));
      StdOut.print((unsigned long) now());    StdOut.print(F(","));
      StdOut.print(Bin2Hex(fo.raw, rx_size)); StdOut.print(F(","));
      StdOut.println(RF_last_rssi);
    }
  }

  if (isTimeToDisplay()) {
    LED_Clear();
    LEDTimeMarker = millis();
  }
}
#endif /* EXCLUDE_WATCHOUT_MODE */

#if !defined(EXCLUDE_TEST_MODE)

unsigned int pos_ndx = 0;
unsigned long TxPosUpdMarker = 0;

void txrx_test()
{
  bool success = false;
#if DEBUG_TIMING
  unsigned long baro_start_ms, baro_end_ms;
  unsigned long tx_start_ms, tx_end_ms, rx_start_ms, rx_end_ms;
  unsigned long parse_start_ms, parse_end_ms, led_start_ms, led_end_ms;
  unsigned long export_start_ms, export_end_ms;
  unsigned long oled_start_ms, oled_end_ms;
#endif
  ThisAircraft.timestamp = now();

  if (TxPosUpdMarker == 0 || (millis() - TxPosUpdMarker) > 4000 ) {
    ThisAircraft.latitude  = pgm_read_float( &txrx_test_positions[pos_ndx][0]);
    ThisAircraft.longitude = pgm_read_float( &txrx_test_positions[pos_ndx][1]);
    pos_ndx = (pos_ndx + 1) % TXRX_TEST_NUM_POSITIONS;
    TxPosUpdMarker = millis();
  }
  ThisAircraft.altitude = TXRX_TEST_ALTITUDE;
  ThisAircraft.course   = TXRX_TEST_COURSE;
  ThisAircraft.speed    = TXRX_TEST_SPEED;
  ThisAircraft.vs       = TXRX_TEST_VS;

#if DEBUG_TIMING
  baro_start_ms = millis();
#endif
  Baro_loop();
#if DEBUG_TIMING
  baro_end_ms = millis();
#endif

#if defined(ENABLE_AHRS)
  AHRS_loop();
#endif /* ENABLE_AHRS */

#if DEBUG_TIMING
  tx_start_ms = millis();
#endif
  RF_Transmit(RF_Encode(&ThisAircraft), true);
#if DEBUG_TIMING
  tx_end_ms = millis();
  rx_start_ms = millis();
#endif
  success = RF_Receive();
#if DEBUG_TIMING
  rx_end_ms = millis();
#endif

#if DEBUG_TIMING
  parse_start_ms = millis();
#endif
  if (success) ParseData();
#if DEBUG_TIMING
  parse_end_ms = millis();
#endif

#if defined(ENABLE_TTN)
  TTN_loop();
#endif

  Traffic_loop();

#if DEBUG_TIMING
  led_start_ms = millis();
#endif
  if (isTimeToDisplay()) {
    LED_DisplayTraffic();
    LEDTimeMarker = millis();
  }
#if DEBUG_TIMING
  led_end_ms = millis();
#endif

  Sound_loop();

#if DEBUG_TIMING
  export_start_ms = millis();
#endif
  if (isTimeToExport()) {
#if defined(USE_NMEALIB)
    NMEA_Position();
#endif
    NMEA_Export();
    GDL90_Export();
    D1090_Export();
    ExportTimeMarker = millis();
  }
#if DEBUG_TIMING
  export_end_ms = millis();
#endif

#if DEBUG_TIMING
  oled_start_ms = millis();
#endif
//  SoC->Display_loop();
#if DEBUG_TIMING
  oled_end_ms = millis();
#endif

#if DEBUG_TIMING
  if (baro_start_ms - baro_end_ms) {
    Serial.print(F("Baro start: "));
    Serial.print(baro_start_ms);
    Serial.print(F(" Baro stop: "));
    Serial.println(baro_end_ms);
  }
  if (tx_end_ms - tx_start_ms) {
    Serial.print(F("TX start: "));
    Serial.print(tx_start_ms);
    Serial.print(F(" TX stop: "));
    Serial.println(tx_end_ms);
  }
  if (rx_end_ms - rx_start_ms) {
    Serial.print(F("RX start: "));
    Serial.print(rx_start_ms);
    Serial.print(F(" RX stop: "));
    Serial.println(rx_end_ms);
  }
  if (parse_end_ms - parse_start_ms) {
    Serial.print(F("Parse start: "));
    Serial.print(parse_start_ms);
    Serial.print(F(" Parse stop: "));
    Serial.println(parse_end_ms);
  }
  if (led_end_ms - led_start_ms) {
    Serial.print(F("LED start: "));
    Serial.print(led_start_ms);
    Serial.print(F(" LED stop: "));
    Serial.println(led_end_ms);
  }
  if (export_end_ms - export_start_ms) {
    Serial.print(F("Export start: "));
    Serial.print(export_start_ms);
    Serial.print(F(" Export stop: "));
    Serial.println(export_end_ms);
  }
  if (oled_end_ms - oled_start_ms) {
    Serial.print(F("OLED start: "));
    Serial.print(oled_start_ms);
    Serial.print(F(" OLED stop: "));
    Serial.println(oled_end_ms);
  }
#endif

  // Handle Air Connect
  NMEA_loop();

  ClearExpired();
}

#endif /* EXCLUDE_TEST_MODE */
