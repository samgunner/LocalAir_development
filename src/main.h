// ****************************************************************************
// LocalAir pollution monitor
//
// Pollution monitoring software for Teensy-based hardware.
// ****************************************************************************

#include <Adafruit_ADXL343.h>
#include <Adafruit_GPS.h>
#include <Audio.h>
#include <SD.h>
#include <Speck.h>
#include <WiFiNINA.h>

// Prepare the Teensy for use
//
// Sets board pins to output mode.
void setup_teensy();

// Prepare the accelerometer for use
//
// Returns true on success.
bool setup_accelerometer(Adafruit_ADXL343 &accelerometer);

// Prepare the SD card for use
//
// The SD can currently get stuck in a loop where it just tries and fails
// forever, which sometimes it seems that a restart can bring things back. So
// give it a few tries and and then give up.
//
// Returns true on success.
bool setup_SD_card();
#define SD_CHIP_SELECT 9

// Prepare the GPS sensor for use
//
// Note that the GPS sensor is used as a trigger for the other data collection
// elements.
void setup_GPS(Adafruit_GPS &GPS);

// Prepare the audio sensor for use
//
// Takes the FFT data object to be used for audio analysis.
void setup_audio(AudioAnalyzeFFT1024 &FFT_data);

// Print messsage to the serial console and the syslog file
//
// A timestamp is appended and a newline appended. Then the syslog file is
// flushed.
template <typename Any>
void syslog(Any message);

// Print a formatted messsage to the serial console and the syslog file
//
// The `message` is formatted using the arguments that follow. A timestamp is
// appended and a newline appended. Then the syslog file is flushed.
template <typename... Args>
void syslog(const char *message, Args... arguments);

// Like builtin delay function, with the addition of a rainbow LED effect
void delay_with_rainbow_LED(unsigned int dur);

#define STATUS_LED_SUCCESS true
#define STATUS_LED_FAILURE false

// Flash the status LED
//
// The LED is flashed with a given colour, and then either green (for success)
// or red (for failure).
//
// Colours should be specified as integers between 0 and 255. Default is white
// then green (for success), flashed once.
void flash_status_LED(const int red = 255, const int green = 255, const int blue = 255, const bool status = STATUS_LED_SUCCESS, const int times = 1);

// TODO: doc
int wifiSetUp();

// TODO: doc
void power_off();

// TODO: doc
void printWifiStatus();

// TODO: doc
int uploadFile(File file, const bool sysFile = false);

// TODO: doc
int copyFile(File file);

// TODO: doc
String IpAddress2String(const IPAddress &ipAddress);

// Construct a logfile name using the current date and time from the GPS sensor
const char *get_logfile_name();

// Construct a formatted current date and time from the GPS sensor
const char *get_datetime();

// Encrypt a block of text
void speck_encrypt(BlockCipher *cipher, size_t key_size, byte *encrypted_output, byte *unencrypted_input);

// Round a number to 2 decimal places
double round2(double value);
