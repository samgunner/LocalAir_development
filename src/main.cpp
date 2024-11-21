// ****************************************************************************
// LocalAir pollution monitor
//
// Pollution monitoring software for Teensy-based hardware.
//
// Reads from the following sensors:
//   - DHT22 - temperature, humidiy
//   - Multigas - NO2, C2H5CH, VOC, CO
//   - PM - PM0.3, PM0.5, PM1, PM2.5, PM5, PM10
//   - Ambient noise - FFT bucket bins
//
// GPS provides for the date and location information. Everything is recorded to
// an SD card, using a Wi-Fi card for uploads.
// ****************************************************************************

#include "main.h"

// ****************************************************************************
// Configuration
//
// We keep secrets in a separate file that is not in version control. See
// secrets.h.example for what is required.
// ****************************************************************************

#include "config.h"
#include "secrets.h"

// ****************************************************************************
// Hardware setup
// ****************************************************************************

// WiFi and HTTP
#include <ArduinoHttpClient.h>
#include <WiFiNINA.h>
WiFiSSLClient client;
int wifi_status = WL_IDLE_STATUS;

// ****************************************************************************

// Accelerometer
#include <Adafruit_ADXL343.h>
#define INPUT_PIN_INT1 (16) // The input pin to enable the interrupt on, connected
                            // to INT1 on the ADXL SAMD21/SAMD51 = 5 for
                            // interrupt pin
uint8_t int_resp;           // To hold the interrupt response
int activity_counter = INACTIVITY_TIMEOUT;
Adafruit_ADXL343 accelerometer = Adafruit_ADXL343(12345); // Unique ID to the sensor

// ****************************************************************************

// Temperature and humidity sensor
#include <DHT.h>
DHT dht_sensor(2, DHT22); // The DHT sensor will be on digital input pin 2. This is
                          // the forth pin down on the left hand side of the
                          // Teensy. DHT22 = (AM2302), AM2321

// ****************************************************************************

// Multigas sensor
#include <Multichannel_Gas_GMXXX.h>
GAS_GMXXX<TwoWire> multigas_sensor;

// ****************************************************************************

// PM sensor
#include <PMS.h>
#define PM_SERIAL Serial5
PMS particulate_sensor(PM_SERIAL);
PMS::DATA particulate_data;

// ****************************************************************************

// GPS sensor (including date/time)
#include <Adafruit_GPS.h>
Adafruit_GPS GPS(&Serial3);
int last_seconds = 0; // For checking when a minute has elapsed

// ****************************************************************************

// Audio (including FFT analysis)
#include <Audio.h>

// Create the Audio components for the FFT. These should be created in the
// order data flows, inputs/sources -> processing -> outputs.
AudioInputAnalog adc1(A2); // using the adc

// Connect live input to the FFT
// Note: this must be changed for mic use
AudioAnalyzeFFT1024 FFT_data;
AudioConnection patchCord1(adc1, 0, FFT_data, 0);

float latest_FFT_data[40];

// ****************************************************************************

// SD card file storage
#include <SD.h>
File datalog_file;
File syslog_file;

// ****************************************************************************

// Other Arduino pins

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

// Encryption
#include <Speck.h>
Speck speck;

// ****************************************************************************

// Data types
#include <ArduinoJson.h>
#include <Dictionary.h>

// ****************************************************************************
// Setup - prepare sensors and log files
// ****************************************************************************

void setup() {
    syslog("Starting up");
    setup_teensy();
    delay_with_rainbow_LED(5000); // Let things settle

    // The accelerometer and the SD are the first things set up. At this early
    // stage, and syslog() messages are only logged to Serial, not the syslog
    // file (which is not open yet).

    syslog("Initialising ADXL343 accelerometer");
    if (setup_accelerometer(accelerometer)) {
        syslog("Accelerometer initialisation succeeded");
        flash_status_LED(255, 255, 0, STATUS_SUCCESS);
    } else {
        syslog("Accelerometer initialisation failed, powering off");
        flash_status_LED(255, 255, 0, STATUS_FAILURE);
        power_off();
    }

    syslog("Initialising SD card");
    if (setup_SD_card()) {
        syslog("SD initialisation succeeded");
        flash_status_LED(0, 0, 255, STATUS_SUCCESS, 3);
    } else {
        syslog("SD initialisation failed, powering off");
        flash_status_LED(0, 0, 255, STATUS_FAILURE, 10);
        power_off();
    }

    syslog("Creating syslog file");
    syslog_file = SD.open(SYSTEM_LOG_FILE_NAME, FILE_WRITE);
    if (syslog_file) {
        flash_status_LED(165, 42, 42, STATUS_SUCCESS, 3);
    } else {
        syslog("Syslog creation failed, powering off");
        flash_status_LED(165, 42, 42, STATUS_FAILURE, 10);
        power_off();
    }

    // Now the syslog file has been created, begin setting up the remaining
    // sensors.

    syslog("======================================");
    syslog("Starting LocalAir Pollution Monitoring");
    syslog("This is: " DEVICE_ID);
    syslog("Accelerometer and SD card already initialised");

    syslog("Initialising DHT sensor");
    dht_sensor.begin();

    syslog("Initialising multigas sensor");
    multigas_sensor.begin(Wire, 0x08);

    syslog("Initialising particulate sensor");
    PM_SERIAL.begin(9600);

    syslog("Initialising GPS sensor");
    setup_GPS(GPS);

    syslog("Initialising audio sensor");
    setup_audio(FFT_data);
    for (int i = 0; i < 40; i++) {
        latest_FFT_data[i] = -1;
    }

    syslog("Waiting for GPS time");
    wait_for_GPS_time(GPS);

    if (CHECK_GPS_DATETIME) {
        // Under normal operation we will only log data if the GPS clock is
        // correct. We can tell if it isn't correct becasue it will reporting a
        // year value of 80 (i.e. it thinks its 1980). If that is the case we
        // just shutdown.
        if (GPS.year == 80) {
            syslog("GPS clock not set, shutting down");
            flash_status_LED(0, 255, 125, STATUS_FAILURE, 5);
            power_off();
        }
    }

    syslog("GPS clock is set to: %s", get_datetime());

    const char *datalog_file_name = get_logfile_name();

    syslog("Creating datalog file %s", datalog_file_name);
    datalog_file = SD.open(datalog_file_name, FILE_WRITE);
    delay(100); // Let things settle
    if (datalog_file) {
        syslog("Created log file");
        flash_status_LED(75, 75, 125, STATUS_SUCCESS, 2);
    } else {
        syslog("Failed to create log file");
        flash_status_LED(75, 75, 125, STATUS_FAILURE, 10);
        power_off();
    }

    if (ENCRYPT) {
        syslog("ENCRYPT flag is set, log file will be encrypted");
    }
}

// ****************************************************************************
// Loop - read sensors and upload log files
// ****************************************************************************

void loop() {
    // ****************************************************************************
    // Collect new sensor data
    // ****************************************************************************

    particulate_sensor.read(particulate_data); // TODO: why is this here and not lower down?
    GPS.read();                                // TODO: why is this here and not lower down?

    // Get the FFT on every loop, but only use the data when the rest of the
    // sensors are read. It's not pretty, but I can't be sure that the data
    // won't back up if don't do this. The device seems to have more than enough
    // power to do this without worry.
    if (FFT_data.available()) {
        if (DEBUG_FFT) Serial.print("FFT data available ");
        for (int i = 0; i < 40; i++) {
            latest_FFT_data[i] = FFT_data.read(i);
            if (DEBUG_FFT) Serial.printf("%0.0f ", 1000 * latest_FFT_data[i]);
        }
        if (DEBUG_FFT) Serial.println();
    }

    // The rest of this loop is governed by the sending of GPS messages by the
    // GPS module. It will only run when a new valid message is received.
    // Getting lastNMEA() resets the newNMEAreceived() flag to false.
    if (!GPS.newNMEAreceived() || !GPS.parse(GPS.lastNMEA()) || last_seconds == GPS.seconds) {
        return;
    }

    // Data is collected in a JSON document, for later uploading.
    JsonDocument json_document;
    json_document["datetime"] = get_datetime();

    if (GPS.fix) {
        flash_status_LED(1, 50, 32, STATUS_SUCCESS, 1);
        json_document["GPS"]["fix"] = true;
        json_document["GPS"]["location"]["lat"] = GPS.latitudeDegrees;
        json_document["GPS"]["location"]["long"] = GPS.longitudeDegrees;
        json_document["GPS"]["location"]["hdop"] = GPS.HDOP;
        json_document["GPS"]["location"]["alt"] = GPS.altitude;
        json_document["GPS"]["speed"] = GPS.speed;
        json_document["GPS"]["angle"] = GPS.angle;
        json_document["GPS"]["satellites"] = GPS.satellites;
    } else {
        flash_status_LED(1, 50, 32, STATUS_FAILURE, 1);
        json_document["GPS"]["fix"] = false;
        json_document["GPS"]["location"]["lat"] = "-";
        json_document["GPS"]["location"]["long"] = "-";
        json_document["GPS"]["location"]["hdop"] = "-";
        json_document["GPS"]["location"]["alt"] = "-";
        json_document["GPS"]["speed"] = "-";
        json_document["GPS"]["angle"] = "-";
        json_document["GPS"]["satellites"] = "-";
    }

    // Reading temperature or humidity takes about 250 milliseconds! Sensor
    // readings may also be up to 2 seconds old (it's a very slow sensor)
    float temperature = dht_sensor.readTemperature();
    float humidity = dht_sensor.readHumidity();
    if (!isnan(temperature) && !isnan(humidity)) {
        json_document["DHT"]["temp"] = round2(temperature);
        json_document["DHT"]["humidity"] = round2(humidity);
        json_document["DHT"]["heat_index"] =
            round2(dht_sensor.computeHeatIndex(temperature, humidity, false)); // isFahreheit = false
    } else {
        json_document["DHT"]["temp"] = "-";
        json_document["DHT"]["humidity"] = "-";
        json_document["DHT"]["heat_index"] = "-";
    }

    json_document["MultiGas"]["no2"] = multigas_sensor.measure_NO2();
    json_document["MultiGas"]["c2h5ch"] = multigas_sensor.measure_C2H5OH();
    json_document["MultiGas"]["voc"] = multigas_sensor.measure_VOC();
    json_document["MultiGas"]["co"] = multigas_sensor.measure_CO();

    json_document["PM_Sensor"]["atmos_enviro"]["AE_1.0"] = particulate_data.PM_AE_UG_1_0;
    json_document["PM_Sensor"]["atmos_enviro"]["AE_2.5"] = particulate_data.PM_AE_UG_2_5;
    json_document["PM_Sensor"]["atmos_enviro"]["AE_10.0"] = particulate_data.PM_AE_UG_10_0;

    JsonArray fft_json = json_document["FFT"].to<JsonArray>();
    for (int i = 0; i < 40; i++) {
        fft_json.add(round2(latest_FFT_data[i] * 1000));
    }

    char serialized_json[LINE_LENGTH];
    serializeJson(json_document, serialized_json);
    Serial.println(serialized_json);

    // ****************************************************************************
    // Write data to the log file on the SD card
    // ****************************************************************************

    if (ENCRYPT) {
        byte plaintext_buffer[17]; // TODO: why is this 17 (16 + null terminator?)
        byte encrypted_buffer[16];
        byte encrypted_data[LINE_LENGTH];

        // Encrypt the data in 16-byte blocks
        for (int i = 0; i < (LINE_LENGTH / 16); i++) {
            memcpy(plaintext_buffer, &serialized_json[i * 16], 16);
            speck_encrypt(&speck, SPECK_KEY_SIZE, encrypted_buffer, plaintext_buffer);
            memcpy(&encrypted_data[i * 16], encrypted_buffer, 16);
        }

        // Save the file in ascii encoded hex, which of course doubles its
        // size... but that's ok
        for (int i = 0; i < (int)sizeof(encrypted_data); i++) {
            datalog_file.printf("%02X", encrypted_data[i]);
        }
        datalog_file.write("\n");
        datalog_file.flush();
    } else {
        datalog_file.write(serialized_json);
        datalog_file.write("\n");
        datalog_file.flush();
    }

    // ****************************************************************************
    // Upload log files (data + syslog) via Wi-Fi
    // ****************************************************************************

    // Only check for Wi-Fi at the start of each minute on the clock (new seconds < old seconds)
    if (GPS.seconds < last_seconds) {
        if (wifiSetUp() == WL_CONNECTED) {
            // Disable the FFT to see if this stops some of the funny behaviour
            AudioNoInterrupts();

            if (DEBUG) Serial.println("Debug - about to close log file");
            datalog_file.close();
            if (DEBUG) Serial.println("Debug - log file closed");

            // Count the number of datalog files there are on the SD card, before we try
            // to upload any of them. Assume they are all in the root directory.
            if (DEBUG) Serial.println("Debug - opening SD card root");
            File root_dir = SD.open("/");
            int num_files = 0;
            while (true) {
                if (DEBUG) Serial.println("Debug - about to open Next File");
                File file = root_dir.openNextFile();
                if (DEBUG) Serial.println("Debug - opened Next File");

                if (!file) {
                    if (DEBUG) Serial.println("Debug - No more files, breaking loop");
                    break;
                }

                // Don't count syslog file or directories
                if (strcmp(file.name(), SYSTEM_LOG_FILE_NAME) == 0) {
                    if (DEBUG) Serial.println("Debug - SysLog found, skipping");
                    continue;
                }
                if (file.isDirectory()) {
                    if (DEBUG) Serial.println("Debug - directory found, skipping");
                    continue;
                }

                num_files++;
            }
            root_dir.close();

            syslog("%s data log files to be uploaded", num_files);

            // Flash the LED to indicate the number of files to be uploaded
            flash_status_LED(123, 231, 78, STATUS_SUCCESS, num_files);

            // The second time round, upload the data files
            root_dir = SD.open("/");
            while (true) {
                File file = root_dir.openNextFile();

                if (!file) {
                    break;
                }
                if (strcmp(file.name(), SYSTEM_LOG_FILE_NAME) == 0) {
                    continue;
                }
                if (file.isDirectory()) {
                    continue;
                }

                if (upload_file(file)) {
                    // and since we don't yet have a good way of checking that the upload
                    // has worked we are going instead to try and copy to an archive folder
                    // on the SD card.
                    if (archive_file(file)) {
                        // this means the copy worked and so we can delete the file
                        Serial.println(file.name()); // <- for some reason it stops working when I remvoe this

                        // close and delete the log file
                        file.close();
                        SD.remove(file.name());

                        // check to see if it deleted ok
                        if (SD.exists(file.name())) {
                            syslog("Warning - could not delete %s", file.name());
                        } else {
                            syslog("Successfully deleted %s", file.name());
                        }
                    } else {
                        file.close();
                        syslog("Warning - could not copy %s", file.name());
                    }
                }

                file.close();
            }
            root_dir.close();

            // we can't, as yet, tell if the file upload was a success, and so we are
            // going to have to just shut down and hope for the best

            syslog("All Data files uploaded");

            // need to post to the sys log file, and then close and reopen it for reading
            syslog(SYSTEM_LOG_FILE_NAME " will now be uploaded, after which the system will shutdown");
            syslog_file.close(); // TODO: Just seek(0) ?
            syslog_file = SD.open(SYSTEM_LOG_FILE_NAME, FILE_READ);

            int upload_status = upload_file(syslog_file, true);

            if (upload_status == 0) {
                flash_status_LED(255, 215, 0, true, 5); // Log file uploaded
            } else {
                flash_status_LED(255, 215, 0, false, 5); // Log file failed to upload
            }

            /* just clear any interupts there might be on the acceleromter */
            int_resp = accelerometer.checkInterrupts();

            Serial.println("Shutting Down");

            power_off();
        }

        /* we are now going to check the interupts on the accelerometer
         *  to see if there has been any activity. The plan is that if there
         *  is no movement for 5 minutes then we will turn off.
         *  This will happen if we have wifi or not.
         *  looking at the output from the interups it looks like:
         *  131 -> means no interups
         *  147 -> means activity
         */
        int_resp = accelerometer.checkInterrupts();

        if (int_resp == 131) {
            /* no actvity since last minute, so decrement the 5 min counter */
            activity_counter = activity_counter - 1;
        } else {
            /* there has been activity so set it back to 5 */
            activity_counter = INACTIVITY_TIMEOUT;
        }

        // if the activity counter has got to zero then turn off
        if (activity_counter < 1) {
            syslog("No movement Detected for 5 minutes, switching off");

            syslog_file.close();
            datalog_file.close();

            flash_status_LED(192, 192, 192, true, 5); // Activity counter
            power_off();
        }
    }

    // saving this seconds so we can compair with the next to see if a minute has gone by
    last_seconds = GPS.seconds;
}

// ****************************************************************************
// Helper functions
// ****************************************************************************

void setup_teensy() {
    delay(1000); // Let things settle

    pinMode(POWER_OFF_PIN, OUTPUT);
    pinMode(LED_PIN_R, OUTPUT);
    pinMode(LED_PIN_G, OUTPUT);
    pinMode(LED_PIN_B, OUTPUT);
}

bool setup_accelerometer(Adafruit_ADXL343 &accelerometer) {
    bool accelerometer_detected = false;
    for (int tries = 5; tries > 0; tries--) {
        if (DEBUG) Serial.printf("Trying to initialise accelerometer, %d tries remaining\n", tries);
        if (accelerometer.begin()) {
            accelerometer_detected = true;
            break;
        }
        delay(1000);
    }
    if (!accelerometer_detected) {
        return false;
    }

    accelerometer.setRange(ADXL343_RANGE_2_G); // +/- 2g sensor range

    // Enable activity interrupts on the accelerometer and map them to the INT1
    // pin (see the Analog Devices ADXL343 Breakout Learning Guide)
    int_config g_int_config_enabled = {0};
    int_config g_int_config_map = {0};
    g_int_config_enabled.bits.activity = true;
    accelerometer.enableInterrupts(g_int_config_enabled);
    g_int_config_map.bits.activity = ADXL343_INT1;
    accelerometer.mapInterrupts(g_int_config_map);

    // Set the activity threshold for firing the interupt. I've tried a few
    // values for the activity threshold, 0x50 sort of worked, although putting
    // on the bike it looks like it timed out quite often, so am now trying
    // 0x30.
    accelerometer.writeRegister(0x24, ACCEL_TRIG_SENSE);
    accelerometer.writeRegister(0x27, 0xE0); // and trying to add all the of
    accelerometer.writeRegister(0x1D, 0xFF); // the axis to the interupt

    return true;
}

bool setup_SD_card() {
    for (int tries = 5; tries > 0; tries--) {
        if (SD.begin(SD_CHIP_SELECT)) {
            return true;
        }
        delay(1000);
    }
    return false;
}

void setup_GPS(Adafruit_GPS &GPS) {
    // 9600 NMEA is the default baud rate for Adafruit MTK GPS's - some use 4800
    GPS.begin(9600);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);         // Set up sentences
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_200_MILLIHERTZ); // 5 second update rate
}

void setup_audio(AudioAnalyzeFFT1024 &FFT_data) {
    // Setting up the analogue input pin. The internal pull down is too low a
    // value, which distorts the voltage being read (and presumably drains the
    // battery). Instead the internal pull down is turned off and an external
    // 1MOhm resistor is used.
    pinMode(A8, INPUT);

    // The FFT setup - audio connections require memory to work. For more
    // detailed information, see the MemoryAndCpuUsage example.
    AudioMemory(12);

    // Configure the window algorithm to use
    FFT_data.windowFunction(AudioWindowHanning1024);
}

void wait_for_GPS_time(Adafruit_GPS &GPS) {
    while (1) {
        GPS.read();

        if (GPS.newNMEAreceived()) {
            if (!GPS.parse(GPS.lastNMEA())) // this resets the newNMEAreceived() flag to false
                continue;                   // if the parse has failed then we skip and try again.
        }

        if (GPS.year > 0) {
            break;
        }
    }
}

#define RAINBOW_LED_DELAY 1
void delay_with_rainbow_LED(unsigned int duration) {
    unsigned start_time = millis();

    analogWrite(LED_PIN_R, 255);
    analogWrite(LED_PIN_G, 0);
    analogWrite(LED_PIN_B, 0);

    while (millis() < start_time + duration) {
        for (int i = 0; i <= 255; i++) {
            if (millis() > start_time + duration) {
                break;
            }
            analogWrite(LED_PIN_R, 255 - i);
            analogWrite(LED_PIN_G, i);
            delay(RAINBOW_LED_DELAY);
        }
        for (int i = 0; i <= 255; i++) {
            if (millis() > start_time + duration) {
                break;
            }
            analogWrite(LED_PIN_G, 255 - i);
            analogWrite(LED_PIN_B, i);
            delay(RAINBOW_LED_DELAY);
        }
        for (int i = 0; i <= 255; i++) {
            if (millis() > start_time + duration) {
                break;
            }
            analogWrite(LED_PIN_B, 255 - i);
            analogWrite(LED_PIN_R, i);
            delay(RAINBOW_LED_DELAY);
        }
    }

    analogWrite(LED_PIN_R, 0);
    analogWrite(LED_PIN_G, 0);
    analogWrite(LED_PIN_B, 0);
}

#define STATUS_LED_DELAY 500
void flash_status_LED(const int red, const int green, const int blue, const bool status, const int times) {
    delay(STATUS_LED_DELAY);

    for (int i = 0; i < times; i++) {
        analogWrite(LED_PIN_R, red);
        analogWrite(LED_PIN_G, green);
        analogWrite(LED_PIN_B, blue);

        delay(STATUS_LED_DELAY);

        if (status == STATUS_SUCCESS) {
            analogWrite(LED_PIN_R, 0);
            analogWrite(LED_PIN_G, 255);
            analogWrite(LED_PIN_B, 0);
        } else {
            analogWrite(LED_PIN_R, 255);
            analogWrite(LED_PIN_G, 0);
            analogWrite(LED_PIN_B, 0);
        }

        delay(STATUS_LED_DELAY);
    }

    analogWrite(LED_PIN_R, 0);
    analogWrite(LED_PIN_G, 0);
    analogWrite(LED_PIN_B, 0);
}

template <typename Any>
void syslog(Any message) {
    syslog("%s", message);
}

template <typename... Args>
void syslog(const char *message, Args... arguments) {
    const uint32_t timestamp = millis();
    Serial.printf("%010lu ", timestamp);
    syslog_file.printf("%010lu ", timestamp);

    Serial.printf(message, arguments...);
    syslog_file.printf(message, arguments...);

    Serial.println();
    syslog_file.println();
    syslog_file.flush();
}

// a function to set up the WiFi
int wifiSetUp() {
    // create the WiFi dict from the json string
    // if we add more than 10 networks then we will need update the number in the below
    // declaration
    Dictionary &wifiNetworks = *(new Dictionary(10));

    wifiNetworks.jload(wifi_networks);

    WiFi.setPins(SPIWIFI_SS, SPIWIFI_ACK, ESP32_RESETN, ESP32_GPIO0, &SPIWIFI);
    // checking for WiFi Networks
    int numSsid = WiFi.scanNetworks();
    // check through the list of WiFi networks to see if any are in our known list
    for (int thisNet = 0; thisNet < numSsid; thisNet++) {
        char ssid[20];
        char pass[20];

        String ssid_s = WiFi.SSID(thisNet);
        strcpy(ssid, WiFi.SSID(thisNet));

        String pass_s = wifiNetworks[WiFi.SSID(thisNet)];
        pass_s.toCharArray(pass, pass_s.length() + 1);

        if (wifiNetworks(ssid)) {
            syslog("%s networks found, including %s trying to connect.", numSsid, ssid);

            // try and connect to the matching network we have found
            // we are going to stop after a given number of attempts,
            // otherwise we might get stuck in this loop forever if the wifi networks
            // disappears.
            int i = WIFI_ATTEMPTS;
            do {
                i = i - 1;
                flash_status_LED(255, 255, 255, false, 1);
                // try to connect
                wifi_status = WiFi.begin(ssid, pass);
            } while ((wifi_status != WL_CONNECTED) && (i > 1));

            if (wifi_status == WL_CONNECTED) {
                syslog("WiFi Connected");
                flash_status_LED(255, 255, 255, true, 3);
                // print the status to the log
                if (DEBUG) Serial.println("Debug - about to print wifi status");
                printWifiStatus();
                if (DEBUG) Serial.println("Debug - printed wifi status");
                return WL_CONNECTED;
            } else {
                syslog("WiFi could not connect, aborting connection");
                flash_status_LED(255, 255, 255, false, 3);
                return WL_CONNECTED;
            }
        }
    }
    // if there was no network to connect to then we flash a fetching pink
    flash_status_LED(255, 150, 150, false, 1);
    return 0;
}

// Switch the Teensy off (or restart it when debugging)
void power_off() {
    // Clear any interupts there might be on the acceleromter
    int_resp = accelerometer.checkInterrupts();

    if (AUTO_POWER_OFF) {
        digitalWrite(POWER_OFF_PIN, HIGH);
    } else {
        SCB_AIRCR = 0x05FA0004; // Application Interrupt & Reset
        while (true) {
            asm("wfi");
        }
    }
}

/* A function for generating a loging the WiFi status information */
void printWifiStatus() {
    // print the SSID of the network you're attached to:
    IPAddress ip = WiFi.localIP();
    String ip_s = IpAddress2String(ip);

    // Length (with one extra character for the null terminator)
    int ip_sl = ip_s.length() + 1;

    // Prepare the character array (the buffer)
    char ip_ca[ip_sl];

    // Copy it over
    ip_s.toCharArray(ip_ca, ip_sl);

    long rssi_l = WiFi.RSSI();
    char rssi_ca[16];

    ltoa(rssi_l, rssi_ca, 10);

    syslog("SSID: %s; IP Address: %s; RSSI: %s dBm", WiFi.SSID(), ip_ca, rssi_ca);
}

// a function to convert the IP address to a char[]
String IpAddress2String(const IPAddress &ipAddress) {
    return String(ipAddress[0]) + String(".") +
           String(ipAddress[1]) + String(".") +
           String(ipAddress[2]) + String(".") +
           String(ipAddress[3]);
}

// a function for uploading the data file to the server
bool upload_file(File file, const bool is_syslog) {
    // we are now going to upload to James's server, using SSL of all things

    char file_size_string[7];
    itoa(file.size(), file_size_string, 10);

    Serial.print("NOT LOGGED - File size: ");
    Serial.print(file.size());
    Serial.println();

    syslog("Attempting to upload %s (%s Bytes)", file.name(), file_size_string);

    // we are going to check to see if connection was successfull
    // and then do something different it it wasn't.
    // SDG 240522 - seeing if we can switch the HTTP libary
    // bool server_connected = client.connect(server, 443);

    // check connection
    // if (server_connected) {
    if (true) {
        unsigned long fileSize = file.size();

        int red = 255;
        int green = 0;

        analogWrite(LED_PIN_R, red);
        analogWrite(LED_PIN_G, green);
        analogWrite(LED_PIN_B, 0);

        // this should be the number  of lines divided by 255
        int ledStep = (fileSize / LINE_LENGTH) / 255;

        if (ledStep == 0) {
            ledStep = 1;
        }

        // now recreating the HttpClient Object for each upload.
        HttpClient httpclient = HttpClient(client, HTTPS_SERVER, 443);

        httpclient.beginRequest();

        char post_address[50];
        strcpy(post_address, "/");

        if (is_syslog) {
            strcat(post_address, "la_syslog"); // this is where a new location to send the sysLog file wil go
        } else {
            strcat(post_address, "la_data");
        }

        strcat(post_address, "/");
        strcat(post_address, DEVICE_ID); // we are now including the device ID in the url, so that the
                                         // backend knows which key to use for decryption.
        strcat(post_address, "/");

        syslog("Uploading to: %s", post_address);

        httpclient.post(post_address);

        httpclient.sendHeader("Content-Length", file.size());
        // httpclient.sendHeader("Content-Length", 100);
        httpclient.sendHeader("Content-Type", "text/plain");
        // httpclient.sendHeader("Connection", "close");

        httpclient.beginBody();

        int i = ledStep;

        int fileSizeCount = 0;

        Serial.print("Uploading: ");
        while (file.available()) {
            /*
            char this_line[LINE_LENGTH*2+1];
            file.read(this_line, LINE_LENGTH*2+1);
            Serial.print("this_line: ");
            Serial.print(this_line);
            Serial.println();
            httpclient.print(this_line);
            //client.print('\n');
            */
            auto line = file.readStringUntil('\n');
            httpclient.println(line);

            // trying to count the amount of data that has been uploaded
            // fileSizeCount = fileSizeCount + LINE_LENGTH*2+1;
            fileSizeCount += line.length() + 1;

            i--;

            if (i <= 0) {
                Serial.print(".");
                analogWrite(LED_PIN_R, red--);
                analogWrite(LED_PIN_G, green++);

                i = ledStep;

                if (red > 255) red = 255;
                if (red < 0) red = 0;
                if (green > 255) green = 255;
                if (green < 0) green = 0;
            }
        }
        httpclient.endRequest();
        Serial.println();

        syslog("Number of Bytes Uploaded: %s", fileSizeCount);

        // printing the http status code.
        int statusCode = httpclient.responseStatusCode();

        String httpresponse = httpclient.responseBody();

        analogWrite(LED_PIN_R, 0);
        analogWrite(LED_PIN_G, 0);
        analogWrite(LED_PIN_B, 0);

        Serial.print("Responce: ");
        Serial.println(httpresponse);

        if (statusCode == 200) {
            syslog("Upload successful, status code: %s", statusCode);
            flash_status_LED(255, 0, 255, true, 3);
            return true;
        } else {
            // some status code was recieved that means it didn't work.
            syslog("ERROR, upload failed with status code: %s", statusCode);
            flash_status_LED(255, 0, 255, false, 3);
            return false;
        }
    }
}

// rather than deleting a file after up load we are going to move the file
// into an archive folder
bool archive_file(File file) {
    // check to see if the archive dir exisits.
    if (!SD.exists(ARCHIVE_FOLDER)) {
        SD.mkdir(ARCHIVE_FOLDER);
    }
    // first we are going to create the archive file in the archive dir
    char archiveFileName[40];
    strcpy(archiveFileName, ARCHIVE_FOLDER);
    strcat(archiveFileName, "/");
    strcat(archiveFileName, file.name());

    // check to see if this archive file already exists,
    while (SD.exists(archiveFileName)) {
        syslog("Warning - %s already exists, trying different name.", archiveFileName);

        // if it does exist then we are going to prepend a "random" number,
        // this is not actually random, but it doesn't matter.
        char rand_s[10];
        itoa(random(999), rand_s, 10);
        strcpy(archiveFileName, ARCHIVE_FOLDER);
        strcat(archiveFileName, "/id");
        strcat(archiveFileName, rand_s);
        strcat(archiveFileName, file.name());
    }

    // if we've managed to make a file that doesn't already exist then
    // we will open it, and start the upload process
    File archiveFile;
    String archiveFileName_s(archiveFileName);
    archiveFile = SD.open(archiveFileName_s.c_str(), FILE_WRITE);

    if (archiveFile) {
        syslog("Successfully opened archive file: %s", archiveFileName);
    } else {
        syslog("Warning - archive file did not open");
    }

    // more the begining of the original file
    file.seek(0);

    // we are going to do the same LED fading, but from blue to green
    unsigned long fileSize = file.size();

    int blue = 255;
    int green = 0;

    analogWrite(LED_PIN_R, 0);
    analogWrite(LED_PIN_G, green);
    analogWrite(LED_PIN_B, blue);

    int ledStep = (fileSize / ((LINE_LENGTH * 2) + 1)) / 255;

    if (ledStep == 0) {
        ledStep = 1;
    }

    int i = ledStep;

    while (file.available()) {
        // this is where we actually copy acoss.
        char this_line[((LINE_LENGTH * 2) + 1)];
        file.read(this_line, ((LINE_LENGTH * 2) + 1));
        archiveFile.write(this_line, ((LINE_LENGTH * 2) + 1));

        i--;

        if (i <= 0) {
            analogWrite(LED_PIN_B, blue--);
            analogWrite(LED_PIN_G, green++);
            i = ledStep;

            if (blue > 255) blue = 255;
            if (blue < 0) blue = 0;
            if (green > 255) green = 255;
            if (green < 0) green = 0;
        }
    }

    archiveFile.flush();

    // check to see if the file was copied across being seeing the input and outfile
    // are the same size
    if (file.size() == archiveFile.size()) {
        syslog("%s successfully copied to %s/%s", file.name(), ARCHIVE_FOLDER, archiveFile.name());
        archiveFile.close();
        return true;
    } else {
        syslog("Warning - %s and %s are not the same size. %d vs %d", file.name(), archiveFile.name(), file.size(), archiveFile.size());
        archiveFile.close();
        return false;
    }
}

// Construct a logfile name using the current date and time from the GPS sensor
const char *get_logfile_name() {
    static char buffer[50];
    snprintf(buffer,
             sizeof(buffer),
             "%s_20%02d%02d%02d-%02d%02d%02d.txt",
             LOG_FILE_NAME_PREFIX,
             GPS.year,
             GPS.month,
             GPS.day,
             GPS.hour,
             GPS.minute,
             GPS.seconds);
    return buffer;
}

// Construct a formatted current date and time from the GPS sensor
const char *get_datetime() {
    static char buffer[50];
    snprintf(buffer,
             sizeof(buffer),
             "20%02d-%02d-%02dT%02d:%02d:%02d.%03dZ",
             GPS.year,
             GPS.month,
             GPS.day,
             GPS.hour,
             GPS.minute,
             GPS.seconds,
             GPS.milliseconds);
    return buffer;
}

// Encrypt a block of text
void speck_encrypt(BlockCipher *cipher, size_t key_size, byte *encrypted_output, byte *unencrypted_input) {
    cipher->setKey(specKey, key_size);
    cipher->encryptBlock(encrypted_output, unencrypted_input);
    return;
}

// Round a number to 2 decimal places
double round2(double value) {
    return (int)(value * 100 + 0.5) / 100.0;
}
