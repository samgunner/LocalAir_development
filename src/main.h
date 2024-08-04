#include <Arduino.h>

// ****************************************************************************
// Configuration
//
// We keep secrets in a separate file that is not in version control. See
// secrets.h.example for what is required.
// ****************************************************************************

#include "config.h"
#include "secrets.h"

// ****************************************************************************
// Sensors
// ****************************************************************************

// WiFi and HTTP
#include <ArduinoHttpClient.h>
#include <WiFiNINA.h>

// Accelerometer
#include <Adafruit_ADXL343.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define INPUT_PIN_INT1 (16) // The input pin to enable the interrupt on, connected to INT1 on the ADXL
                            // SAMD21/SAMD51 = 5 for interrupt pin
uint8_t int_resp;           // To hold the interrupt response
int activity_counter = INACTIVITY_TIMEOUT;

// Temperature and humidity sensor
#include <DHT.h>

// The DHT sensor will be on digital input pin 2. This is the forth pin down on
// the left hand side of the Teensy.
#define DHTPIN 2
#define DHTTYPE DHT22 // DHT 22 (AM2302), AM2321

// Multigas sensor
#include <Multichannel_Gas_GMXXX.h>

// PM sensor
#include <PMS.h>
#define PMSerial Serial5

// GPS sensor
#include <Adafruit_GPS.h>
#define GPSSerial Serial3

// Audio (including FFT analysis) and SD card
#include <Audio.h>
#include <SD.h>
#include <SPI.h>
#include <SerialFlash.h>

// Encryption
#include <Crypto.h>
#include <Speck.h>

// Data types
#include <ArduinoJson.h>
#include <Dictionary.h>

// ****************************************************************************
// Other pins on the Teensy
// ****************************************************************************

// Power off pin - drives the relay that will turn the power off. Set this to
// HIGH to turn the power off.
#define POWER_OFF_PIN (0)

// LED pins - the board now has an RGB LED, driven by these three pins. Use
// analogWrite, for example analogWrite(LED_PIN_G, 255) to turn the different
// colours on.
#define LED_PIN_R (3) // the Red LED pin
#define LED_PIN_G (4) // the Green LED pin
#define LED_PIN_B (6) // the Blue LED pin

// ****************************************************************************
// Functions defined in main.cpp
// ****************************************************************************

void rainbowLED(unsigned int dur);
void statusLED(int redLED, int greenLED, int blueLED, bool stat, int times);

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

int wifiSetUp();
void powerOff();
void printWifiStatus();
int uploadFile(File log_file, const bool sysFile = false);
int copyFile(File log_file);
String IpAddress2String(const IPAddress &ipAddress);

// Return a logfile name using the current date and time from the GPS sensor
String get_logfile_name();

// Return a formatted current date and time from the GPS sensor
String get_datetime();

// Encrypt a block of text
void speck_encrypt(BlockCipher *cipher, size_t key_size, byte *encrypted_output, byte *unencrypted_input);

// Round a number to 2 decimal places
double round2(double value);
