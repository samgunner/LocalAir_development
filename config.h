/*
 * The deinitions that go with the LocalAir Software
 */

/*
 * Below is the declaration of a string that will be converted into a dict
 * of the wifi networks that we're able to access.
 */
String wifi_networks = "{"
  "\"Juice4Jesus\" : \"J3w5forGingers\","
  "\"Harp Controller\" : \"harpaccess\","
  "\"LocalAir Project\" : \"BikesAreBetter\","
  "\"Scooter Demo\" : \"BikesAreBetter\","
  "\"Fire Flower\" : \"b57f23aa6047\""
"}";

// Upload Server details
// This uploads to Sam's hostgater server
#define HTTPS_SERVER "london.localair.uk"

//#define SERVER_ADDRESS "10.42.0.1" // James
#define SERVER_ADDRESS "10.0.69.1" // Mine

/*
 * System log file name:
 */
#define SYSTEM_LOG_FILE_NAME "LocalAirSystem.log"
#define ARCHIVE_FOLDER "archive"

// Configure the pins used for the ESP32 connection 
#if defined(TEENSYDUINO)
  #define SPIWIFI       SPI  // The SPI port
  #define SPIWIFI_SS     10   // Chip select pin
  #define ESP32_RESETN   5   // Reset pin
  #define SPIWIFI_ACK    7   // a.k.a BUSY or READY pin
  #define ESP32_GPIO0   -1
#endif

#include <WiFiNINA.h>
#include <ArduinoHttpClient.h>

#define STREAM false // deciding weather to stream the data or not...
#define ENCRYPT true // flag to weather to encrypt onto the SD card or not

// and the encryption key information
#define SPECKKEYSIZE 32

// below a variable that defines the number of bytes is a single line of the encrupted file.
int line_length = 960;

/* Pollution Sensors */
#define PMSerial Serial5

/* Debugging flags */
#define DEBUG false  // this turns on and off debug logging
#define CHECK_GPS_DATETIME true
#define AUTO_POWER_OFF true
#define INACTIVITY_TIMEOUT 5
#define WIFI_ATTEMPTS 5
//#define LOG_FILE_NAME_PREFIX "LocalAirData_"
#define LOG_FILE_NAME_PREFIX "LAD_"

/**************************** WiF Status Function ************************************/
