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

#ifdef DEBUG
#define debug_syslog(x) syslog(x)
#else
#define debug_syslog(x) // No operation
#endif

/*********************** Audio Set up ************************/

// Create the Audio components for the FFT.  These should be created in the
// order data flows, inputs/sources -> processing -> outputs
AudioInputI2S audioInput;  // audio shield: mic or line-in
AudioInputAnalog adc1(A2); // using the adc
AudioAnalyzeFFT1024 myFFT;

// Connect live input to the FFT
AudioConnection patchCord1(adc1, 0, myFFT, 0); // NOTE: this must be changed for mic use.

// Initialise the sensors
DHT dht(DHTPIN, DHTTYPE);
GAS_GMXXX<TwoWire> gas;
PMS pms(PMSerial);
PMS::DATA psmdata;
Adafruit_GPS GPS(&GPSSerial);
Adafruit_ADXL343 accel = Adafruit_ADXL343(12345); // Unique ID to the sensor

/*********************** Other variable ************************/

// a holding variable that will store the last fft reading
float lastFFT[40];

// creating the vairable for the log file
File myFile;
// and the variable for the logfile name, that will be constructed later
String log_file_name;

// and creating the variable for the system log file
File sysLogFile;
// The variable that will store the system log file name
String system_log_file_name = SYSTEM_LOG_FILE_NAME;

// and another sd card related thing...
const int chipSelect = 9;

// a variable for checking when a minute has gone by
int last_seconds = 0;

/*********************** CONNECTION PARAMETERS ************************/
WiFiSSLClient client;

int status = WL_IDLE_STATUS;

/*********************** Encryption **************************/

Speck speck;

/*********************** SET FUNCTION ************************/

void setup() {
    delay(1000);

    // setting up flashy LEDS and Power off Pin
    pinMode(POWER_OFF_PIN, OUTPUT);
    pinMode(LED_PIN_R, OUTPUT);
    pinMode(LED_PIN_G, OUTPUT);
    pinMode(LED_PIN_B, OUTPUT);

    // seems risky, but if we are going to start logging quickly we need
    // to set up the SD card first

    // a 5s delay whiel things sort themselves out
    rainbowLED(5000);

    /* We are moving the accelerometer to the first thing we do as we are also
     * going put a depower command into the SD card checking bit.
     * This is because the SD can currently get stuck in a loop where it just tries
     * and failes for ever, which sometimes it seems that a restart can bring
     * things back.
     */
    /*************** Accelerometer Setup ****************/
    /* Initialise the sensor */
    while (!accel.begin()) {
        /* There was a problem detecting the ADXL343 ... check your connections */
        syslog("No ADXL343 accelerometer detected");
        statusLED(255, 255, 0, false, 1);
    }
    // if we get this far then the accelerometer has been detected.
    statusLED(255, 255, 0, true, 1);

    /* Set the range to whatever is appropriate for your project */
    accel.setRange(ADXL343_RANGE_2_G);

    int_config g_int_config_enabled = {0};
    int_config g_int_config_map = {0};

    /* Enable activity interrupts on the accelerometer. */
    g_int_config_enabled.bits.activity = true;
    accel.enableInterrupts(g_int_config_enabled);

    /* Map activity interrupts to INT1 pin. */
    g_int_config_map.bits.activity = ADXL343_INT1;
    accel.mapInterrupts(g_int_config_map);

    /* set the activty threshold for firing the interupt */
    /*
     *  I've tried a few values for hte activity threshold,
     *  0x50 sort of worked, although putting on the bike it looks like
     *  it timed out quite often, so am now trying 0x30
     */
    accel.writeRegister(0x24, ACCEL_TRIG_SENSE);
    /* and adding all the axis to the interupt */
    // and trying to add all the of the axis to the interupt
    accel.writeRegister(0x27, 0xE0);
    accel.writeRegister(0x1D, 0xFF);

    syslog("ADXL343 accelerometer init complete");
    statusLED(255, 255, 0, true, 1);
    /*********************** SD Card Setup ************************/

    Serial.print("Initializing SD card...");

    // we are going to have a number of goes at initialising the SD card and
    // and then give up and restart.
    int SD_retries = 5;

    while (true) {
        if (SD.begin(chipSelect)) {
            break;
        }
        Serial.println("initialization failed!");
        statusLED(0, 0, 255, false, 5); // SD initiasation failed

        // count down the number of retries
        SD_retries = SD_retries - 1;
        Serial.print(SD_retries);

        if (SD_retries < 1) {
            // do a restart and see if we have more luck with the SD card
            // next time.
            powerOff();
        }
    }
    Serial.println("initialization done.");
    statusLED(0, 0, 255, true, 3); // SD initiasation succeeded

    // create the system log file
    sysLogFile = SD.open(system_log_file_name.c_str(), FILE_WRITE);
    // check to see if the system log file now exists... not sure what we'll do if it doesn't...
    if (!sysLogFile) {
        statusLED(165, 42, 42, false, 10);
    } else {
        statusLED(165, 42, 42, true, 5);
    }

    // writing a start up message

    syslog("======================================");
    syslog("Starting LocalAir Polultion Monitoring");
    syslog(DEVICE_ID);

    /*********************** Setup sensors ************************/

    // set up DHT sensor
    dht.begin();

    // Set up MultiGas sensor
    gas.begin(Wire, 0x08);

    // The PMS sensor is on Serial5
    PMSerial.begin(9600);

    /*********************** Setup GPS ************************/

    // some configuration for the GPS, which is going to be trigger for the other data collection
    // elements.

    // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
    GPS.begin(9600);
    // set up sentences
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    // Set the update rate
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_200_MILLIHERTZ); // 5 second update

    /*********************** Audio Setup ************************/

    // Setting up the analouge input pin. The internal pull down is too low a value,
    // which distorts the voltage being read (and presumably drains the battery). Instead
    // the internal pull down is turned off and an external 1MOhm res is used.
    pinMode(A8, INPUT);

    // The FFT setup
    // Audio connections require memory to work.  For more
    // detailed information, see the MemoryAndCpuUsage example
    AudioMemory(12);

    // Configure the window algorithm to use
    myFFT.windowFunction(AudioWindowHanning1024);

    /*********************** Log File Creation ************************/

    // we are going to do a GPS read so that we can name the log file after the date and time
    // we're going to have to do this in a loop, until the date has come back from teh GPS
    while (1) {
        GPS.read();

        if (GPS.newNMEAreceived()) {
            if (!GPS.parse(GPS.lastNMEA())) // this resets the newNMEAreceived() flag to false
                continue;                   // if the parse has failed then we skip and try again.

            if (GPS.year > 0)
                break;
        }
    }

    /*  Under normal operation (i.e. CHECK_GPS_DATETIME == true) we will only log data if the
     *  GPS clock is correct. We can tell if it isn't correct becasue it will reporting a
     *  year value of 80 (i.e. it thinks its 1980...)
     *  If that is the case we just shutdown.
     */
    if (CHECK_GPS_DATETIME) {
        if (GPS.year == 80) {
            syslog("GPS clock not set, shutting down");
            statusLED(0, 255, 125, false, 5);
            powerOff();
        }
    }

    // save gps datetime to the syslog so that it can be compared with syslog times
    // although this means that it might be converterd into a char array first.
    String datetime_s = get_datetime();
    int datetime_l = datetime_s.length() + 1;
    char datetime_c[datetime_l];
    datetime_s.toCharArray(datetime_c, datetime_l);

    syslog(String("GPS clock is set to: ") + datetime_c);

    // set the log file name
    log_file_name = get_logfile_name();

    // also as a char array...
    char log_file_name_ca[log_file_name.length()];
    log_file_name.toCharArray(log_file_name_ca, log_file_name.length() + 1);

    // open the file.
    myFile = SD.open(log_file_name.c_str(), FILE_WRITE);

    delay(100);

    // check to see if the file opened ok!
    if (myFile) {
        syslog(String("Created log file: ") + log_file_name_ca);
        statusLED(75, 75, 125, true, 2);
    } else {
        syslog(String("Error - Count not create log file: ") + log_file_name_ca);
        statusLED(75, 75, 125, false, 10);
    }

    if (ENCRYPT) {
        syslog("ENCRYPT flag is set, log file will be encrypted");
    }
}

void loop() {
    /*********************** IO RUN ************************/

    // io.run(); is required for all sketches.
    // it should always be present at the top of your loop
    // function. it keeps the client connected to
    // io.adafruit.com, and processes any incoming data.
    // io.run();

    /*********************** Every Loop ********************/
    // this loop is now governed by the sending of GPS messages by the GPS module,
    // the bulk of the code will only run when a new message is recievd.

    // read the PSM data each loop through.
    pms.read(psmdata);

    // read data from the GPS in the 'main loop', to see if there is a new message
    GPS.read();

    // The FFT setup
    float fft_val;
    int fft_count;
    // we are going to grab the FFT on every loop, but only print the data when the rest of the
    // sensors are read. It's not pretty, but I can't be sure that the data won't back up if
    // don't do this. The device seems to have more than enough power to do this without worry.
    if (myFFT.available()) {
        for (fft_count = 0; fft_count < 40; fft_count++) {
            lastFFT[fft_count] = myFFT.read(fft_count);
        }
    }

    /*********************** Happens only when new GPS data exisits ********************/

    // The below will only trigger is the new message flag is set...
    if (GPS.newNMEAreceived()) {
        if (!GPS.parse(GPS.lastNMEA())) // this resets the newNMEAreceived() flag to false
            return;                     // if the parse has failed then we skip and try again.

        if (last_seconds == GPS.seconds) {
            return;
        }

        /*********************** JSON Object Creation ********************/

        // create the json object that we are going to store data to
        JsonDocument thisdata;
        JsonObject root = thisdata.to<JsonObject>();

        /*********************** Construct datetime ********************/

        // construct the GPS datetime... you feel there must be better way of doing this...
        String datetime = get_datetime();

        // adding datetime values to the json obj
        // moving the datetime to root, and off of the GPS.
        root["datetime"] = datetime;

        /*********************** Construct location ********************/

        // the root of GPS json object
        JsonObject json_gps = root["GPS"].to<JsonObject>();

        if (GPS.fix) {
            statusLED(1, 50, 32, true, 1);
            json_gps["fix"] = "True";
            json_gps["location"]["lat"] = GPS.latitudeDegrees;
            json_gps["location"]["long"] = GPS.longitudeDegrees;
            json_gps["location"]["hdop"] = GPS.HDOP;
            json_gps["location"]["alt"] = GPS.altitude;

            json_gps["speed"] = GPS.speed;
            json_gps["angle"] = GPS.angle;
            json_gps["satellites"] = GPS.satellites;
        } else {
            statusLED(1, 50, 32, false, 1);
            // If there is any hope if this being passable as a csv we have the fields the same
            json_gps["fix"] = "False";
            json_gps["location"]["lat"] = "-";
            json_gps["location"]["long"] = "-";
            json_gps["location"]["hdop"] = "-";
            json_gps["location"]["alt"] = "-";

            json_gps["speed"] = "-";
            json_gps["angle"] = "-";
            json_gps["satellites"] = "-";
        }

        /*********************** Get and store DHT Data ********************/

        // the root of DHT json object
        JsonObject json_dht = root["DHT"].to<JsonObject>();

        // Reading temperature or humidity takes about 250 milliseconds!
        // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
        float h = dht.readHumidity();
        // Read temperature as Celsius (the default)
        float t = dht.readTemperature();
        // Read temperature as Fahrenheit (isFahrenheit = true)
        float f = dht.readTemperature(true);

        // Check if any reads failed and exit early (to try again).
        if (isnan(h) || isnan(t) || isnan(f)) {
            json_dht["humidity"] = "-";
            json_dht["temp"] = "-";
            json_dht["heat_index"] = "-";
        } else {
            // Compute heat index in Celsius (isFahreheit = false)
            float hic = dht.computeHeatIndex(t, h, false);

            json_dht["humidity"] = round2(h);
            json_dht["temp"] = round2(t);
            json_dht["heat_index"] = round2(hic);
        }

        /*********************** Get and store MultiGas Data ********************/

        // the root of DHT json object
        // JsonObject json_multigas = root.createNestedObject("MultiGas");

        // Now reading in the MultiGas sensor readings

        root["MultiGas"]["no2"] = gas.measure_NO2();
        root["MultiGas"]["c2h5ch"] = gas.measure_C2H5OH();
        root["MultiGas"]["voc"] = gas.measure_VOC();
        root["MultiGas"]["co"] = gas.measure_CO();

        /*********************** Get and store PM Data ********************/

        // the root of DHT json object
        // JsonObject json_pms = root.createNestedObject("PM_Sensor");

        // JsonObject json_ae = json_pms.createNestedObject("atmos_enviro");

        root["PM_Sensor"]["atmos_enviro"]["AE_1.0"] = psmdata.PM_AE_UG_1_0;
        root["PM_Sensor"]["atmos_enviro"]["AE_2.5"] = psmdata.PM_AE_UG_2_5;
        root["PM_Sensor"]["atmos_enviro"]["AE_10.0"] = psmdata.PM_AE_UG_10_0;

        /*********************** Get and store FFT Data ********************/

        // the root of the FFT json object
        // JsonArray json_fft = root.createNestedArray("FFT");

        // we are just going to save the fft data into an array
        for (fft_count = 0; fft_count < 40; fft_count++) {
            fft_val = lastFFT[fft_count];
            root["FFT"].add(round2(fft_val * 1000));
        }

        /*********************** Printing to Serial ********************/

        String deserialized_for_post;
        char deserialized[960];

        serializeJson(root, deserialized);
        serializeJson(root, deserialized_for_post);

        // this is staying in as just a Serial print, as we do not want this going to the syslog file
        Serial.print(deserialized);
        Serial.println();

        /*********************** Printing to File ********************/
        if (ENCRYPT) {
            // a buffer for the plain text block
            byte bytePlanebuffer[17];

            // two duffers for the encrypted data
            byte encryptedText[line_length];
            byte encryptedbuffer[16];

            int specKeySize = SPECKKEYSIZE;

            // we now run through the line, encrupting it one 16 byte block at a time.
            int i;
            for (i = 0; i < (line_length / 16); i++) {
                memcpy(&bytePlanebuffer, &deserialized[i * 16], 16);

                speck_encrypt(&speck, specKeySize, encryptedbuffer, bytePlanebuffer);

                memcpy(&encryptedText[i * 16], &encryptedbuffer, 16);
            }
            // I've decided that I am going to save the file in ascii encoded hex,
            // which of course doubles its size... but that's ok
            for (i = 0; i < (int)sizeof(encryptedText); i++) {
                char hexChar[3];
                // cast the byte into hex
                sprintf(hexChar, "%02X", encryptedText[i]);
                // and write it to the file
                myFile.write(hexChar);
            }
            // write a new line and flush the buffer
            myFile.write("\n");
            myFile.flush();
        } else {
            // if we are not encrypting then just save to the file
            myFile.write(deserialized);
            myFile.write("\n");
            myFile.flush();
        }

        /*********************** CHECK TIME, THEN WIFI AND UPLOAD ********************/

        if (GPS.seconds < last_seconds) {
            // try to connect to the WiFi
            if (wifiSetUp() == WL_CONNECTED) {
                // This  means we have WiFi connection, and should try and upload our file

                // trying to diable the FFT to see if this stops some of the funny behaviour
                AudioNoInterrupts();

                debug_syslog("Debug - about to close log file");

                myFile.close();

                debug_syslog("Debug - log file closed");

                // we're going to try and upload all the files!... how exiting

                // first we are going to count the numebr of files there are on the SD card.
                // for the record, I am not worry about folders... hopefully there won't be any

                int numFiles = 0;

                File filesRoot = SD.open("/");

                debug_syslog("Debug - opening SD card root");

                while (true) {
                    debug_syslog("Debug - about to open Next File");

                    File log_file = filesRoot.openNextFile();

                    debug_syslog("Debug - opened Next File");

                    if (!log_file) {
                        debug_syslog("Debug - No more files, breaking loop");
                        break;
                    }

                    // don't upload the system log file stragiht away.
                    if (strcmp(log_file.name(), SYSTEM_LOG_FILE_NAME) == 0) {
                        debug_syslog("Debug - SysLog found, skipping");
                        continue;
                    }

                    // check to see if it is the archive directory, and if so move on.
                    if (log_file.isDirectory()) {
                        debug_syslog("Debug - directory found, skipping");
                        continue;
                    }

                    // and increment the counter
                    numFiles++;
                }

                analogWrite(LED_PIN_R, 0);
                analogWrite(LED_PIN_G, 0);
                analogWrite(LED_PIN_B, 0);

                int fileFlashDelay = 100;

                // now we flash the LED that number of times.
                for (int i = 0; i < numFiles; i++) {
                    analogWrite(LED_PIN_R, 123);
                    analogWrite(LED_PIN_G, 231);
                    analogWrite(LED_PIN_B, 78);

                    delay(fileFlashDelay);

                    analogWrite(LED_PIN_R, 0);
                    analogWrite(LED_PIN_G, 0);
                    analogWrite(LED_PIN_B, 0);

                    delay(fileFlashDelay);
                }

                syslog(String(numFiles) + " to be uploaded");

                filesRoot.close();

                // we are now going to do the same thing again,  but this time upload the files as we go.
                filesRoot = SD.open("/");

                while (true) {
                    File log_file = filesRoot.openNextFile();

                    if (!log_file) {
                        break;
                    }

                    // don't upload the system log file stragiht away.
                    if (strcmp(log_file.name(), SYSTEM_LOG_FILE_NAME) == 0) {
                        continue;
                    }

                    // check to see if it is the archive directory, and if so move on.
                    if (log_file.isDirectory()) {
                        continue;
                    }

                    // upload this log_file
                    int upload_return = uploadFile(log_file);

                    // we are going to check the return code of the uploadFile function
                    // to see if things actually uploaded.
                    if (upload_return == 0) {
                        // and since we don't yet have a good way of checking that the upload
                        // has worked we are going instead to try and copy to an archive folder
                        // on the SD card.
                        if (copyFile(log_file) == 0) {
                            // this means the copy worked and so we can delete the file
                            Serial.println(log_file.name()); // <- for some reason it stops working when I remvoe this

                            // close and delete the log file
                            log_file.close();
                            SD.remove(log_file.name());

                            // check to see if it deleted ok
                            if (SD.exists(log_file.name())) {
                                syslog(String("Warning - could not delete ") + log_file.name());
                            } else {
                                syslog(String("Successfully deleted ") + log_file.name());
                            }
                        } else {
                            log_file.close();
                            syslog(String("Warning - could not copy ") + log_file.name());
                        }
                    }

                    log_file.close();
                }

                // we can't, as yet, tell if the file upload was a success, and so we are
                // going to have to just shut down and hope for the best

                syslog("All Data files uploaded");

                // need to post to the sys log file, and then close and reopen it for reading
                syslog(String(SYSTEM_LOG_FILE_NAME) + " will now be uploaded, after which the system will shutdown");
                sysLogFile.close(); // TODO: Just seek(0) ?
                sysLogFile = SD.open(SYSTEM_LOG_FILE_NAME, FILE_READ);

                int upload_status = uploadFile(sysLogFile, true);

                if (upload_status == 0) {
                    statusLED(255, 215, 0, true, 5); // Log file uploaded
                } else {
                    statusLED(255, 215, 0, false, 5); // Log file failed to upload
                }

                /* just clear any interupts there might be on the acceleromter */
                int_resp = accel.checkInterrupts();

                Serial.println("Shutting Down");

                powerOff();
            }

            /* we are now going to check the interupts on the accelerometer
             *  to see if there has been any activity. The plan is that if there
             *  is no movement for 5 minutes then we will turn off.
             *  This will happen if we have wifi or not.
             *  looking at the output from the interups it looks like:
             *  131 -> means no interups
             *  147 -> means activity
             */
            int_resp = accel.checkInterrupts();

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

                sysLogFile.close();
                myFile.close();

                statusLED(192, 192, 192, true, 5); // Activity counter
                powerOff();
            }
        }

        // saving this seconds so we can compair with the next to see if a minute has gone by
        last_seconds = GPS.seconds;
    }
}

// This replaces the normal delay function, producing a rainbow on teh LED while
// for a given duration in miliseconds
void rainbowLED(unsigned int dur) {
    unsigned start_time = millis();
    int rainbow_delay = 1;

    analogWrite(LED_PIN_R, 255);
    analogWrite(LED_PIN_G, 0);
    analogWrite(LED_PIN_B, 0);

    while (millis() < start_time + dur) {
        for (int i = 0; i <= 255; i++) {
            if (millis() > start_time + dur) {
                break;
            }
            analogWrite(LED_PIN_R, 255 - i);
            analogWrite(LED_PIN_G, i);
            delay(rainbow_delay);
        }
        for (int i = 0; i <= 255; i++) {
            if (millis() > start_time + dur) {
                break;
            }
            analogWrite(LED_PIN_G, 255 - i);
            analogWrite(LED_PIN_B, i);
            delay(rainbow_delay);
        }
        for (int i = 0; i <= 255; i++) {
            if (millis() > start_time + dur) {
                break;
            }
            analogWrite(LED_PIN_B, 255 - i);
            analogWrite(LED_PIN_R, i);
            delay(rainbow_delay);
        }
    }
    analogWrite(LED_PIN_R, 0);
    analogWrite(LED_PIN_G, 0);
    analogWrite(LED_PIN_B, 0);
}

// we are going to have a function that will do the LED flashing,
// The plan is to have a different colour for each block of code,
// and then alternate between that colour and either Red or Green
// depending on whether it has been successfully or not.
// the three [colour]LED variables are the values for each colour
// the stat variable is the success/failure indicator (true is success)
// times is the number of times to flash.
// the frequency of the flashing is set by the statusLEDDelay variable, in defines.
void statusLED(int redLED, int greenLED, int blueLED, bool stat, int times) {
    int statusLEDDelay = 500;
    int i = 0;
    while (i < times) {
        analogWrite(LED_PIN_R, redLED);
        analogWrite(LED_PIN_G, greenLED);
        analogWrite(LED_PIN_B, blueLED);

        delay(statusLEDDelay);

        if (!stat) {
            analogWrite(LED_PIN_R, 255);
            analogWrite(LED_PIN_G, 0);
            analogWrite(LED_PIN_B, 0);
        } else {
            analogWrite(LED_PIN_R, 0);
            analogWrite(LED_PIN_G, 255);
            analogWrite(LED_PIN_B, 0);
        }

        delay(statusLEDDelay);
        i++;
    }
    analogWrite(LED_PIN_R, 0);
    analogWrite(LED_PIN_G, 0);
    analogWrite(LED_PIN_B, 0);
}

// Print messsage to the serial console and the syslog file
void syslog(const char message[]) {
    char millis_s[14];
    sprintf(millis_s, "%010lu - ", millis());
    Serial.print(millis_s);
    sysLogFile.write(millis_s);

    Serial.println(message);
    sysLogFile.write(message);
    sysLogFile.write('\n');
    sysLogFile.flush();
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
            syslog(String(numSsid) + " networks found, including " +
                   ssid + " trying to connect.");

            // try and connect to the matching network we have found
            // we are going to stop after a given number of attempts,
            // otherwise we might get stuck in this loop forever if the wifi networks
            // disappears.
            int i = WIFI_ATTEMPTS;
            do {
                i = i - 1;
                statusLED(255, 255, 255, false, 1);
                // try to connect
                status = WiFi.begin(ssid, pass);
            } while ((status != WL_CONNECTED) && (i > 1));

            if (status == WL_CONNECTED) {
                syslog("WiFi Connected");
                statusLED(255, 255, 255, true, 3);
                // print the status to the log
                debug_syslog("Debug - about to print wifi status");
                printWifiStatus();
                debug_syslog("Debug - printed wifi status");
                return WL_CONNECTED;
            } else {
                syslog("WiFi could not connect, aborting connection");
                statusLED(255, 255, 255, false, 3);
                return WL_CONNECTED;
            }
        }
    }
    // if there was no network to connect to then we flash a fetching pink
    statusLED(255, 150, 150, false, 1);
    return 0;
}

/* a function that will do the powering off */
void powerOff() {
    if (AUTO_POWER_OFF) {
        /* just clear any interupts there might be on the acceleromter */
        int_resp = accel.checkInterrupts();

        digitalWrite(POWER_OFF_PIN, HIGH);
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

    syslog(String("SSID: ") + WiFi.SSID() +
           "; IP Address: " + ip_ca +
           "; RSSI: " + rssi_ca + " dBm");
}

// a function to convert the IP address to a char[]
String IpAddress2String(const IPAddress &ipAddress) {
    return String(ipAddress[0]) + String(".") +
           String(ipAddress[1]) + String(".") +
           String(ipAddress[2]) + String(".") +
           String(ipAddress[3]);
}

// a function for uploading the data file to the server
int uploadFile(File log_file, const bool sysFile) {
    // we are now going to upload to James's server, using SSL of all things

    char log_file_size_string[7];
    itoa(log_file.size(), log_file_size_string, 10);

    Serial.print("NOT LOGGED - File size: ");
    Serial.print(log_file.size());
    Serial.println();

    syslog(String("Attempting to upload ") + log_file.name() +
           " (" + log_file_size_string + " Bytes)");

    // we are going to check to see if connection was successfull
    // and then do something different it it wasn't.
    // SDG 240522 - seeing if we can switch the HTTP libary
    // bool server_connected = client.connect(server, 443);

    // check connection
    // if (server_connected) {
    if (true) {
        unsigned long fileSize = log_file.size();

        int red = 255;
        int green = 0;

        analogWrite(LED_PIN_R, red);
        analogWrite(LED_PIN_G, green);
        analogWrite(LED_PIN_B, 0);

        // this should be the number  of lines divided by 255
        int ledStep = (fileSize / line_length) / 255;

        if (ledStep == 0) {
            ledStep = 1;
        }

        // now recreating the HttpClient Object for each upload.
        HttpClient httpclient = HttpClient(client, HTTPS_SERVER, 443);

        httpclient.beginRequest();

        char post_address[50];
        strcpy(post_address, "/");

        if (sysFile) {
            strcat(post_address, "la_syslog"); // this is where a new location to send the sysLog file wil go
        } else {
            strcat(post_address, "la_data");
        }

        strcat(post_address, "/");
        strcat(post_address, DEVICE_ID); // we are now including the device ID in the url, so that the
                                         // backend knows which key to use for decryption.
        strcat(post_address, "/");

        syslog(String("Uploading to: ") + post_address);

        httpclient.post(post_address);

        httpclient.sendHeader("Content-Length", log_file.size());
        // httpclient.sendHeader("Content-Length", 100);
        httpclient.sendHeader("Content-Type", "text/plain");
        // httpclient.sendHeader("Connection", "close");

        httpclient.beginBody();

        int i = ledStep;

        int fileSizeCount = 0;

        Serial.print("Uploading: ");
        while (log_file.available()) {
            /*
            char this_line[line_length*2+1];
            log_file.read(this_line, line_length*2+1);
            Serial.print("this_line: ");
            Serial.print(this_line);
            Serial.println();
            httpclient.print(this_line);
            //client.print('\n');
            */
            auto line = log_file.readStringUntil('\n');
            httpclient.println(line);

            // trying to count the amount of data that has been uploaded
            // fileSizeCount = fileSizeCount + line_length*2+1;
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

        syslog(String("Number of Bytes Uploaded: ") + fileSizeCount);

        // printing the http status code.
        int statusCode = httpclient.responseStatusCode();

        String httpresponse = httpclient.responseBody();

        analogWrite(LED_PIN_R, 0);
        analogWrite(LED_PIN_G, 0);
        analogWrite(LED_PIN_B, 0);

        Serial.print("Responce: ");
        Serial.println(httpresponse);

        if (statusCode == 200) {
            syslog(String("Upload successful, response code: ") + statusCode);

            statusLED(255, 0, 255, true, 3);

            // return a zero because things worked
            return 0;
        } else {
            // some status code was recieved that means it didn't work.
            syslog(String("ERROR, upload failed with status code: ") + statusCode);

            statusLED(255, 0, 255, false, 3);
            // return a non-zero because things didn't work
            return 1;
        }
    }
}

// rather than deleting a file after up load we are going to move the file
// into an archive folder
int copyFile(File log_file) {
    // check to see if the archive dir exisits.
    if (!SD.exists(ARCHIVE_FOLDER)) {
        SD.mkdir(ARCHIVE_FOLDER);
    }
    // first we are going to create the archive file in the archive dir
    char archiveFileName[40];
    strcpy(archiveFileName, ARCHIVE_FOLDER);
    strcat(archiveFileName, "/");
    strcat(archiveFileName, log_file.name());

    // check to see if this archive file already exists,
    while (SD.exists(archiveFileName)) {
        syslog(String("Warning - ") + archiveFileName +
               " already exisits, trying different name.");

        // if it does exist then we are going to prepend a "random" number,
        // this is not actually random, but it doesn't matter.
        char rand_s[10];
        itoa(random(999), rand_s, 10);
        strcpy(archiveFileName, ARCHIVE_FOLDER);
        strcat(archiveFileName, "/id");
        strcat(archiveFileName, rand_s);
        strcat(archiveFileName, log_file.name());
    }

    // if we've managed to make a file that doesn't already exist then
    // we will open it, and start the upload process
    File archiveFile;
    String archiveFileName_s(archiveFileName);
    archiveFile = SD.open(archiveFileName_s.c_str(), FILE_WRITE);

    if (archiveFile) {
        syslog(String("Successfully opened archive file: ") + archiveFileName);
    } else {
        syslog("Warning - archive file did not open");
    }

    // more the begining of the original file
    log_file.seek(0);

    // we are going to do the same LED fading, but from blue to green
    unsigned long fileSize = log_file.size();

    int blue = 255;
    int green = 0;

    analogWrite(LED_PIN_R, 0);
    analogWrite(LED_PIN_G, green);
    analogWrite(LED_PIN_B, blue);

    int ledStep = (fileSize / ((line_length * 2) + 1)) / 255;

    if (ledStep == 0) {
        ledStep = 1;
    }

    int i = ledStep;

    while (log_file.available()) {
        // this is where we actually copy acoss.
        char this_line[((line_length * 2) + 1)];
        log_file.read(this_line, ((line_length * 2) + 1));
        archiveFile.write(this_line, ((line_length * 2) + 1));

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
    if (log_file.size() == archiveFile.size()) {
        syslog(String(log_file.name()) + " successfully copied to " +
               ARCHIVE_FOLDER + "/" + archiveFile.name());

        archiveFile.close();
        return 0;
    } else {
        syslog(String("Warning - ") + log_file.name() +
               " and " + archiveFile.name() +
               " are not the same size. " +
               log_file.size() + " vs " + archiveFile.size());
        archiveFile.close();
        return 1;
    }
}

// Return a logfile name using the current date and time from the GPS sensor
String get_logfile_name() {
    char buffer[50];
    sprintf(buffer,
            "%s_20%02d%02d%02d-%02d%02d%02d.txt",
            LOG_FILE_NAME_PREFIX,
            GPS.year,
            GPS.month,
            GPS.day,
            GPS.hour,
            GPS.minute,
            GPS.seconds);
    return String(buffer); // TODO: Remove String() conversion when no longer required
}

// Return a formatted current date and time from the GPS sensor
String get_datetime() {
    char buffer[50];
    sprintf(buffer,
            "20%02d-%02d-%02dT%02d:%02d:%02d.%03dZ",
            GPS.year,
            GPS.month,
            GPS.day,
            GPS.hour,
            GPS.minute,
            GPS.seconds,
            GPS.milliseconds);
    return String(buffer); // TODO: Remove String() conversion when no longer required
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
