// PolMon - Log and Upload v0.1
//
// This is a first full verson of the poltion monitoring software
// at the moment it is able to read in the from the sensors:
//      DHT22 - temp, humidiy
//      Multigas - NO2, C2H5CH, VOC, CO
//      PM - PM0.3, PM0.5, PM1, PM2.5, PM5, PM10
//      Ambiant Noice - FFTs bucket bins
//
// The software also access a GPS for the date and location information.
// Everything is recorded to an SD card.
// The board also has an auxillary Wifi card for uploads.
// At the moment he plan is to use the adafruit message broker (Adafruit IO), 
// for which libraries exist have been implemented, although not here yet.

/************************** Configuration File Import ***********************************/

// edit the config.h tab and enter Adafruit IO credentials
// and any additional configuration needed for WiFi, cellular,
// or ethernet clients.
#include "config.h"

/************************ LED and Power Off pins definition  *******************************/

#define POWER_OFF_PIN (0) // This pin drives the relay that will turn the power off the the Teensy
                          // set this pin to HIGH to turn the power OFF
// The board now has an RGB LED, this is driven by the three pins given below.
// use analogWrite, for example "analogWrite(LED_PIN_G, 255);" to turn the different colours on.
#define LED_PIN_R (3) // the Red LED pin
#define LED_PIN_G (4) // the Green LED pin
#define LED_PIN_B (6) // the Blue LED pin

/************************ Accelerometer *******************************/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL343.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_ADXL343 accel = Adafruit_ADXL343(12345);

/** The input pin to enable the interrupt on, connected to INT1 on the ADXL. */
#define INPUT_PIN_INT1   (16) // SAMD21/SAMD51 = 5 for interrupt pin

/* variable for the interup responce */
uint8_t int_resp;
int activity_counter = INACTIVITY_TIMEOUT;

/*********************** Pollution Sensor Library Imports and setup ************************/

#include "DHT.h"
// the library for the multigas sensor
#include <Multichannel_Gas_GMXXX.h> // SDG 230314 - some messing about when moving to Ubuntu 22.04
// have refound the old libary and so putting things back how they were.
//#include <MultichannelGasSensor.h>
// presumbly the below is a typo... but it does work...
//#include "MutichannelGasSensor.h"
#include <Wire.h>

GAS_GMXXX<TwoWire> gas;

// adding the stuff for the PM Sensors
#include "PMS.h"

PMS pms(PMSerial);  // if you're changing this, then make sure you also change it futher down.
PMS::DATA psmdata;

/*********************** GPS Library Imports and setup ************************/

// Adding the stuff for the GPS
#include <Adafruit_GPS.h>

// what's the name of the hardware serial port?
#define GPSSerial Serial3

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

/*********************** Audio and SD card Library Imports and setup ************************/

// including the libraries required for the FFT analysis
// this is also necessary for the SD card 
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

/*********************** FTP Include ************************/

#include <FTPClient_Generic.h>

// This uploads to Sam's hostgater server
char ftp_server[] = FTP_SERVER;

char ftp_user[]   = FTP_USERNAME;
char ftp_pass[]   = FTP_PASSWORD;

// FTPClient_Generic(char* _serverAdress, char* _userName, char* _passWord, uint16_t _timeout = 10000);
FTPClient_Generic ftp (ftp_server, ftp_user, ftp_pass, 60000);

/*********************** JSON Imports and setup ************************/

// we are going to try storing the data to a json object, because it looks quite neet,
// this requires the below library
#include <ArduinoJson.h>

/*********************** Audio Set up ************************/

// set up the mic as th input
const int myInput = AUDIO_INPUT_MIC;

// Create the Audio components for the FFT.  These should be created in the
// order data flows, inputs/sources -> processing -> outputs
AudioInputI2S          audioInput;         // audio shield: mic or line-in
AudioInputAnalog       adc1(A2);         // using the adc
AudioAnalyzeFFT1024    myFFT;

// Connect live input to the FFT
AudioConnection patchCord1(adc1, 0, myFFT, 0);  // NOTE: this must be changed for mic use.

// this is alos copied from the library, but I don't think is needed.
AudioControlSGTL5000 audioShield;

/*********************** DHT Set up ************************/

// As part of the Scooter Polution Monitoring System, the DHT sensor will be on Digital Input Pin 2,
// this is the forth pin down on the left hand side of the Teensy
#define DHTPIN 2     // Digital pin connected to the DHT sensor

#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

// Initialize DHT sensor.
DHT dht(DHTPIN, DHTTYPE);

/*********************** Other variable ************************/

// we are going to use a dictionary to store WiFi information
 #include <Dictionary.h>

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
char serverAddress[] = SERVER_ADDRESS;  // server address
int port = 5000;

WiFiClient wifi;
HttpClient client = HttpClient(wifi, serverAddress, port);
//client.setHttpResponseTimeout(1);
int status = WL_IDLE_STATUS;

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

  /*********************** SD Card Setup ************************/

  Serial.print("Initializing SD card...");

  while (true) {
    if (SD.begin(chipSelect)) {
      break;
    }
    Serial.println("initialization failed!");
    statusLED(0, 0, 255, false, 5);
  }
  Serial.println("initialization done.");
  statusLED(0, 0, 255, true, 3);

  // create the system log file
  sysLogFile = SD.open(system_log_file_name.c_str(), FILE_WRITE);
  // check to see if the system log file now exists... not sure what we'll do if it doesn't...
  if (!sysLogFile) {
    statusLED(165,42,42,false,10);
  }
  else {
    statusLED(165,42,42,true,1);
  }

  // writing a start up message
  logAndPrint("Starting LocalAir Polultion Monitoring");

  // a 5s delay whiel things sort themselves out
  rainbowLED(5000);


  /*************** Accelerometer Setup ****************/
  /* Initialise the sensor */
  while(!accel.begin())
  {
    /* There was a problem detecting the ADXL343 ... check your connections */
    logAndPrint("No ADXL343 accelerometer detected");
    statusLED(255,255,0,false,1);
  }
  // if we get this far then the accelerometer has been detected.
  statusLED(255,255,0,true,1);

  /* Set the range to whatever is appropriate for your project */
  accel.setRange(ADXL343_RANGE_2_G);

  int_config g_int_config_enabled = { 0 };
  int_config g_int_config_map = { 0 };


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
  accel.writeRegister(0x24, 0x30);
  /* and adding all the axis to the interupt */
    // and trying to add all the of the axis to the interupt
  accel.writeRegister(0x27, 0xE0);
  accel.writeRegister(0x1D, 0xFF);

  logAndPrint("ADXL343 accelerometer init complete");
  statusLED(255,255,0,true,1);

  /*********************** WIFI CONNECTION  ************************/

  if (STREAM) {
      wifiSetUp();
  }
  else {
      Serial.println("Not streaming, skipping WiFi Setup");
  }

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
  pinMode(A8,INPUT);

  // The FFT setup
  // Audio connections require memory to work.  For more
  // detailed information, see the MemoryAndCpuUsage example
  AudioMemory(12);

  // I don't think the below is required, becasue are not outputting the audio
  // Enable the audio shield and set the output volume.
  //audioShield.enable();
  //audioShield.inputSelect(myInput);
  //audioShield.volume(0.5);

  // Configure the window algorithm to use
  myFFT.windowFunction(AudioWindowHanning1024);


/*********************** Log File Creation ************************/

  // we are going to do a GPS read so that we can name the log file after the date and time
  // we're going to have to do this in a loop, until the date has come back from teh GPS
  while(1) {
    GPS.read();
  
    if (GPS.newNMEAreceived()) {
      if (!GPS.parse(GPS.lastNMEA())) // this resets the newNMEAreceived() flag to false
        continue; // if the parse has failed then we skip and try again.

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
    if (GPS.year == 80 && !STREAM) {
      logAndPrint("GPS clock not set, shutting down");
      statusLED(0,255,125,false,5);
      powerOff();
    }
  }

  // save gps datetime to the syslog so that it can be compared with syslog times
  // although this means that it might be converterd into a char array first.
  String datetime_s = getDateTime();
  int datetime_l = datetime_s.length()+1; 
  char datetime_c[datetime_l];
  datetime_s.toCharArray(datetime_c,datetime_l);

  char to_log[50];
  strcat(to_log, "GPS clock is set to: ");
  strcat(to_log, datetime_c);
  logAndPrint(to_log);

  // set the log file name
  log_file_name = makeLogFileName();

  // also as a char array...
  char log_file_name_ca[log_file_name.length()];
  log_file_name.toCharArray(log_file_name_ca,log_file_name.length());

  // open the file. 
  myFile = SD.open(log_file_name.c_str(), FILE_WRITE);

  delay(100);

  // check to see if the file opened ok!
  if (myFile) {
     char to_log[50];
     strcat(to_log, "Created log file: ");
     strcat(to_log, log_file_name_ca);
     logAndPrint(to_log);
     statusLED(75,75,125, true, 2);
  }
  else {
    char to_log[50];
    strcat(to_log, "Error - Count not create log file: ");
    strcat(to_log, log_file_name_ca);
    statusLED(75,75,125, false, 10);
  }

}

void loop() {
  /*********************** IO RUN ************************/

  // io.run(); is required for all sketches.
  // it should always be present at the top of your loop
  // function. it keeps the client connected to
  // io.adafruit.com, and processes any incoming data.
  //io.run();

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
  bool fft_available;
  // we are going to grab the FFT on every loop, but only print the data when the rest of the
  // sensors are read. It's not pretty, but I can't be sure that the data won't back up if 
  // don't do this. The device seems to have more than enough power to do this without worry.
  if (myFFT.available()) {  
    for (fft_count=0; fft_count<40; fft_count++) {
        lastFFT[fft_count] = myFFT.read(fft_count);
    }
  }

  /*********************** Happens only when new GPS data exisits ********************/

  // The below will only trigger is the new message flag is set...
  if (GPS.newNMEAreceived()) {

    if (!GPS.parse(GPS.lastNMEA())) // this resets the newNMEAreceived() flag to false
      return; // if the parse has failed then we skip and try again.

  /*********************** JSON Object Creation ********************/

  // create the json object that we are going to store data to
  StaticJsonDocument<1024> thisdata;
  JsonObject root = thisdata.to<JsonObject>();

  /*********************** Construct datetime ********************/

  // construct the GPS datetime... you feel there must be better way of doing this...
  String datetime = getDateTime();

    
  // adding datetime values to the json obj
  // moving the datetime to root, and off of the GPS.
  root["datetime"] = datetime;


  /*********************** Construct location ********************/

  // the root of GPS json object
  JsonObject json_gps = root.createNestedObject("GPS");

  if (GPS.fix) {
      statusLED(1,50,32,true,1);
      json_gps["fix"] = "True";
      JsonObject json_location = json_gps.createNestedObject("location");
      json_location["lat"] = GPS.latitude;
      json_location["long"] = GPS.longitude;
      json_location["alt"] = GPS.altitude;

      json_gps["speed"] = GPS.speed;
      json_gps["angle"] = GPS.angle;
      json_gps["satellites"] = GPS.satellites;      
    }
    else {
      statusLED(1,50,32,false,1);
      // If there is any hope if this being passable as a csv we have the fields the same
      json_gps["fix"] = "False";
      JsonObject json_location = json_gps.createNestedObject("location");
      json_location["lat"] = "-";
      json_location["long"] = "-";
      json_location["alt"] = "-";

      json_gps["speed"] = "-";
      json_gps["angle"] = "-";
      json_gps["satellites"] = "-";      
    }

  /*********************** Get and store DHT Data ********************/

  // the root of DHT json object
  JsonObject json_dht = root.createNestedObject("DHT");
    
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
  }
  else {
    // Compute heat index in Celsius (isFahreheit = false)
    float hic = dht.computeHeatIndex(t, h, false);
    
    json_dht["humidity"] = h;
    json_dht["temp"] = t;
    json_dht["heat_index"] = hic;
  }
  
  /*********************** Get and store MultiGas Data ********************/

  // the root of DHT json object
  JsonObject json_multigas = root.createNestedObject("MultiGas");

  // Now reading in the MultiGas sensor readings

  json_multigas["no2"] = gas.measure_NO2();
  json_multigas["c2h5ch"] = gas.measure_C2H5OH();
  json_multigas["voc"] = gas.measure_VOC();
  json_multigas["co"] = gas.measure_CO();

  /*********************** Get and store PM Data ********************/

  // the root of DHT json object
  JsonObject json_pms = root.createNestedObject("PM_Sensor");

  JsonObject json_ae = json_pms.createNestedObject("atmos_enviro");

  json_ae["AE_1.0"] = psmdata.PM_AE_UG_1_0;
  json_ae["AE_2.5"] = psmdata.PM_AE_UG_2_5;
  json_ae["AE_10.0"] = psmdata.PM_AE_UG_10_0;

  /*********************** Get and store FFT Data ********************/ 

  // the root of the FFT json object
  JsonArray json_fft = root.createNestedArray("FFT");

  // we are just going to save the fft data into an array
  for (fft_count=0; fft_count<40; fft_count++) {
    fft_val = lastFFT[fft_count];
    json_fft.add(fft_val*1000);
  }

  /*********************** Printing to Serial ********************/

  String deserialized_for_post;
  char deserialized[1024];
  serializeJson(root, deserialized);
  serializeJson(root, deserialized_for_post);

  // this is staying in as just a Serial print, as we do not want this going to the syslog file
  Serial.print(deserialized);
  Serial.println();

  /*********************** Printing to File ********************/
  
  myFile.write(deserialized);
  myFile.write("\n");
  myFile.flush();
 
  /*********************** POSTing ********************/

  if (STREAM) {
 
    Serial.print("POSTing -> ");
    String contentType = "application/json";

    client.setHttpResponseTimeout(1000);

    digitalWrite(4, HIGH);
    client.post("/measurement_upload", contentType, deserialized_for_post);

    // read the status code and body of the response
    int statusCodePOST = client.responseStatusCode();
    String responsePOST = client.responseBody();

    Serial.print("Status code: ");
    Serial.println(statusCodePOST);
    Serial.print("Response: ");
    Serial.println(responsePOST);
  }

    /*********************** CHECK TIME, THEN WIFI AND UPLOAD ********************/

    if (GPS.seconds < last_seconds){
        // try to connect to the WiFi
        if (wifiSetUp() == WL_CONNECTED) {
          // This  means we have WiFi connection, and should try and upload our file

          myFile.close();

          uploadFile(log_file_name);

          // we can't, as yet, tell if the file upload was a success, and so we are 
          // going to have to just shut down and hope for the best
          
          logAndPrint("WiFi connect, Upload finished - shutting down!");

          statusLED(255,215,0, true, 5);

          /* just clear any interupts there might be on the acceleromter */
          int_resp = accel.checkInterrupts();
    
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
        }
        else {
          /* there has been activity so set it back to 5 */
          activity_counter = INACTIVITY_TIMEOUT;
        }

        // if the activity counter has got to zero then turn off
        if (activity_counter < 1) {

          myFile.close();

          statusLED(192,192,192,true,5);
          powerOff();
      }
    }
      
    // saving this seconds so we can compair with the next to see if a minute has gone by
    last_seconds = GPS.seconds;

  }
}

// This replaces the normal delay function, producing a rainbow on teh LED while 
// for a given duration in miliseconds
void rainbowLED(int dur) {
    long start_time = millis();
    int rainbow_delay = 1;
    
    analogWrite(LED_PIN_R, 255);
    analogWrite(LED_PIN_G, 0);
    analogWrite(LED_PIN_B, 0);

    while (millis() < start_time + dur) {
        for (int i = 0; i <= 255; i++) {
            if (millis() > start_time + dur) {
              break;
            }
            analogWrite(LED_PIN_R, 255-i);
            analogWrite(LED_PIN_G, i);
            delay(rainbow_delay);
        }
        for (int i = 0; i <= 255; i++) {
            if (millis() > start_time + dur) {
              break;
            }
            analogWrite(LED_PIN_G, 255-i);
            analogWrite(LED_PIN_B, i);
            delay(rainbow_delay);
        }
        for (int i = 0; i <= 255; i++) {
            if (millis() > start_time + dur) {
              break;
            }
            analogWrite(LED_PIN_B, 255-i);
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
  while ( i < times ) {
    analogWrite(LED_PIN_R, redLED);
    analogWrite(LED_PIN_G, greenLED);
    analogWrite(LED_PIN_B, blueLED);

    delay(statusLEDDelay);

    if (!stat) {
      analogWrite(LED_PIN_R, 255);
      analogWrite(LED_PIN_G, 0);
      analogWrite(LED_PIN_B, 0);
    }
    else {
      analogWrite(LED_PIN_R, 0);
      analogWrite(LED_PIN_G, 255);
      analogWrite(LED_PIN_B, 0);
    }

    delay(statusLEDDelay);
    i++;
  }
}

// A function that will both print messsages to serial and log them to
// the system log file
void logAndPrint(char *message) {
    char millis_s[12];
    itoa(millis(), millis_s, 10);
    char to_log[50];
    strcat(to_log, millis_s);
    strcat(to_log, " - ");
    strcat(to_log, message); 
    sysLogFile.write(to_log);
    Serial.println(to_log);
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
    if (wifiNetworks(WiFi.SSID(thisNet))) {
      char to_log[60];
      char numSsid_s[6];
      itoa(numSsid, numSsid_s, 10);
      strcat(to_log, numSsid);
      strcat(to_log, " networks found, including \"");
      strcat(to_log, thisNet);
      strcat(to_log, "\" trying to connect.");
      logAndPrint(to_log);

      // try and connect to the matching network we have found
      // we are going to stop after a given number of attempts,
      // otherwise we might get stuck in this loop forever if the wifi networks 
      // disappears.
      int i = WIFI_ATTEMPTS;
      do {
        i = i - 1;
        statusLED(255,255,255,false,1);
        // try to connect
        char ssid[20] = {WiFi.SSID(thisNet)};
        int pass_len = wifiNetworks[WiFi.SSID(thisNet)].length() + 1;
        char pass[pass_len];
        wifiNetworks[WiFi.SSID(thisNet)].toCharArray(pass, pass_len);
        status = WiFi.begin(ssid, pass);
        if ( i < 1 ) {
          break;
        }
      } while (status != WL_CONNECTED);
      if (status == WL_CONNECTED) {
        logAndPrint("WiFi Connected");
        statusLED(255,255,255,true,3);
        // print the status to the log
        printWifiStatus();
        return WL_CONNECTED;
      }
      else {
        logAndPrint("WiFi could not connect, aborting connection");
        statusLED(255,255,255,false,3);
        return WL_CONNECTED;
      }
    }
  }
  // if there was no network to connect to then we flash a fetching pink
  statusLED(255,150,150,false,1);
  return 0;
}

/* a function that will do the powering off */
void powerOff() {
  if (AUTO_POWER_OFF) {
    digitalWrite(POWER_OFF_PIN, HIGH); 
  }
}

/* A function for generating a loging the WiFi status information */
void printWifiStatus() {
  // print the SSID of the network you're attached to:

  IPAddress ip = WiFi.localIP();
  long rssi = WiFi.RSSI();

  char to_log[60];
  strcat(to_log, "SSID :");
  strcat(to_log, WiFi.SSID());
  strcat(to_log, "; IP Address: ");
  strcat(to_log, ip);
  strcat(to_log, "; RSSI: ");
  strcat(to_log, rssi);
  strcat(to_log, " dBm");
  //String message = "SSID: " + WiFi.SSID() + "; IP Address: " + ip + "; RSSI: " + rssi + " dBm";

  logAndPrint(to_log);
}

// a function for uploading the data file to the server
int uploadFile(String log_file_name) {
    char to_log[50];
    strcat(to_log, "Attempting to upload ");
    strcat(to_log, log_file_name.c_str());
    logAndPrint(to_log);
          
    myFile = SD.open(log_file_name.c_str());

    ftp.OpenConnection();

    ftp.InitFile(COMMAND_XFER_TYPE_BINARY);
    ftp.NewFile(log_file_name.c_str());

    int i = 0;

    // are going to do the LEDs manually here
    analogWrite(LED_PIN_R, 0);
    analogWrite(LED_PIN_G, 0);
    analogWrite(LED_PIN_B, 0);

    // while through the file uploading it line by line...
    while (myFile.available()) {
        byte data[256];
        myFile.read(data, 256);

        analogWrite(LED_PIN_R, 255);
        analogWrite(LED_PIN_G, 255);

        delay(50);
        
        ftp.Write(data);

        analogWrite(LED_PIN_R, 0);
        analogWrite(LED_PIN_G, 0);

        delay(50);

        i = i+1;
        
    }
    logAndPrint("Closing FTP Server File");
    ftp.CloseFile();

    logAndPrint("Closing FTP Connection");
    ftp.CloseConnection();

    //TODO, it would be nice to have some way of checking the uplaod had actually worked....
    // for now we are just going  to flash happily...

    statusLED(255,0,255, true, 3);

    myFile.close();
}



// a function for making the logfile name from the GPS derived time.
String makeLogFileName() {
  // Construct the file name... which is a bit of a polava!
  String tmp_log_file_name;
  
  tmp_log_file_name = LOG_FILE_NAME_PREFIX;

  if (GPS.year < 10 ) tmp_log_file_name = tmp_log_file_name + "0";
  tmp_log_file_name = tmp_log_file_name + GPS.year;

  if (GPS.month < 10 ) tmp_log_file_name = tmp_log_file_name + "0";
  tmp_log_file_name = tmp_log_file_name + GPS.month;

  if (GPS.day < 10 ) tmp_log_file_name = tmp_log_file_name + "0";
  tmp_log_file_name = tmp_log_file_name + GPS.day;

  tmp_log_file_name = tmp_log_file_name + "-";

  if (GPS.hour < 10 ) tmp_log_file_name = tmp_log_file_name + "0";
  tmp_log_file_name = tmp_log_file_name + GPS.hour;

  if (GPS.minute < 10 ) tmp_log_file_name = tmp_log_file_name + "0";
  tmp_log_file_name = tmp_log_file_name + GPS.minute;

  if (GPS.seconds < 10 ) tmp_log_file_name = tmp_log_file_name + "0";
  tmp_log_file_name = tmp_log_file_name + GPS.seconds;
  
  tmp_log_file_name = tmp_log_file_name + ".txt";

  return tmp_log_file_name;
}

String getDateTime() {
    String datetime;
    datetime = "20";
    if (GPS.year < 10 ) datetime = datetime + "0";
    datetime = datetime + String(GPS.year);
    datetime = datetime + "-";
    if (GPS.month < 10 ) datetime = datetime + "0";
    datetime = datetime + GPS.month;
    datetime = datetime + "-";
    if (GPS.day < 10 ) datetime = datetime + "0";
    datetime = datetime + GPS.day;
    datetime = datetime + "T";
    if (GPS.hour < 10 ) datetime = datetime + "0";
    datetime = datetime + GPS.hour;
    datetime = datetime + ":";
    if (GPS.minute < 10 ) datetime = datetime + "0";
    datetime = datetime + GPS.minute;
    datetime = datetime + ":";
    if (GPS.seconds < 10 ) datetime = datetime + "0";
    datetime = datetime + GPS.seconds;
    datetime = datetime + ".";
    if (GPS.milliseconds < 100) datetime = datetime + "0";
    if (GPS.milliseconds < 10) datetime = datetime + "0";
    datetime = datetime + GPS.milliseconds;
    datetime = datetime + "Z";

    return datetime;
}
