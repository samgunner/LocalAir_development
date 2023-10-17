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
//#define HTTPS_SERVER "192.168.1.222"
//#define FTP_USERNAME "localAir@sgunner.co.uk"
//#define FTP_PASSWORD "L0c4lA1rMunky!"

//#define SECRET_SSID "LocalAir Project" // James's
//#define SECRET_SSID "Scooter Demo" // Mine
//#define SECRET_SSID "Juice4Jesus" // Home
//define SECRET_SSID "Harp Controller" // Office

//#define SECRET_PASS "BikesAreBetter"
//#define SECRET_PASS "J3w5forGingers"
//define SECRET_PASS "harpaccess" // Office

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
byte specKey[32] = {0xad, 0x1c, 0x4b, 0x6, 0x5c, 0x85, 0x2a, 0x48, 0xe4, 0xed, 0x33, 0x23, 0x4c, 0x9f, 0xed, 0x56, 0x23, 0x46, 0x59, 0xfa, 0x3c, 0x70, 0x82, 0x97, 0x45, 0xbd, 0x2b, 0xf1, 0xdc, 0xf4, 0xb6, 0xce};

// below a variable that defines the number of bytes is a single line of the encrupted file.
int line_length = 960;

/* Pollution Sensors */
#define PMSerial Serial5

/* Debugging flags */
#define DEBUG true  // this turns on and off debug logging
#define CHECK_GPS_DATETIME false
#define AUTO_POWER_OFF true
#define INACTIVITY_TIMEOUT 5
#define WIFI_ATTEMPTS 5
//#define LOG_FILE_NAME_PREFIX "LocalAirData_"
#define LOG_FILE_NAME_PREFIX "LAD_"

/**************************** WiF Status Function ************************************/
