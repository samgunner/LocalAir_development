/*
 * The definitions that go with the LocalAir Software
 */

// Upload Server details
#define HTTPS_SERVER "london.localair.uk"

/*
 * System log file name:
 */
#define SYSTEM_LOG_FILE_NAME "LocalAirSystem.log"
#define ARCHIVE_FOLDER "archive"

// Configure the pins used for the ESP32 connection
#if defined(TEENSYDUINO)
#define SPIWIFI SPI    // The SPI port
#define SPIWIFI_SS 10  // Chip select pin
#define ESP32_RESETN 5 // Reset pin
#define SPIWIFI_ACK 7  // a.k.a BUSY or READY pin
#define ESP32_GPIO0 -1
#endif

#define ENCRYPT true // flag to weather to encrypt onto the SD card or not

// and the encryption key information
#define SPECK_KEY_SIZE 32

// below a variable that defines the number of bytes is a single line of the encrupted file.
#define LINE_LENGTH 960

/* the sensitivity of the accelerometer interupt */
#define ACCEL_TRIG_SENSE 0x50

/* Debugging flags */
#define DEBUG false     // this turns on and off debug logging
#define DEBUG_FFT false // Output live FFT data to the serial console
#define CHECK_GPS_DATETIME true
#define AUTO_POWER_OFF true
#define INACTIVITY_TIMEOUT 5
#define WIFI_ATTEMPTS 5
// #define LOG_FILE_NAME_PREFIX "LocalAirData_"
#define LOG_FILE_NAME_PREFIX DEVICE_ID

/**************************** WiF Status Function ************************************/
