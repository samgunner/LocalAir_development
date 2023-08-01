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
int activity_counter = 5;

/************************ Adafruit IO Feed Set Up  *******************************/

// this int will hold the current count for our sketch
//int count = 0;

// set up the 'Pollution Data' feed
//AdafruitIO_Feed *polmon_feed = io.feed("Pollution Data");

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

PMS pms(Serial5);  // if you're changing this, then make sure you also change it futher down.
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
char ftp_server[] = "sgunner.co.uk";

char ftp_user[]   = "localAir@sgunner.co.uk";
char ftp_pass[]   = "L0c4lA1rMunky!";

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

// Variables will change
int ledState = LOW;             // ledState used to set the LED

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;        // will store last time LED was updated

const long interval = 2000;           // interval at which to blink (milliseconds)

// When viewing the DHT22 board fromm above with the conenctor at the bottom,
//   ---------
//   | o o o |
//     A A A
//     | | +-- GND  - connect to pin G of the Teensy
//     | +---- V+   - this is , and claims to be 3.3 or 5V, seems to work on Teensy 3V output pin
//     +------ Data - Teensy pin 2 (forth down in LHS)

// Initialize DHT sensor.
// Note that older versions of this library took an optional third parameter to
// tweak the timings for faster processors.  This parameter is no longer needed
// as the current DHT reading algorithm adjusts itself to work on faster procs.
DHT dht(DHTPIN, DHTTYPE);

/*********************** Other variable ************************/

// MultiGas Sensor variable
static uint8_t recv_cmd[8] = {};

// Set the reference value for the ADC inputs, used to calculate the measured voltage
#define ADC_VOLT_REF 3.28

// the two variables used to measure and calculate the GPS batter voltage
int gps_bat_val; // the raw value form the ADC read
float gps_bat_volt; // the calculated 

// a holding variable that will store the last fft reading
float lastFFT[40];

// creating the vairable for the log file
File myFile;
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

String log_file_name;

/*********************** SET FUNCTION ************************/

void setup() {

  delay(5000);

  // setting up flashy LEDS and Power off Pin
  pinMode(POWER_OFF_PIN, OUTPUT);
  pinMode(LED_PIN_R, OUTPUT);
  pinMode(LED_PIN_G, OUTPUT);
  pinMode(LED_PIN_B, OUTPUT); 


  // start the serial connection
   

  // wait for serial monitor to open
  //while(! Serial);
  // SDG 230516 - bypassing the serial wait so that the thing will launch when 
  // not connected to a computer.
  delay(1000);

  Serial.println(F("Voi Polultion Monitoring"));

  /*************** Accelerometer Setup ****************/
  /* Initialise the sensor */
  while(!accel.begin())
  {
    /* There was a problem detecting the ADXL343 ... check your connections */
    Serial.println("Ooops, no ADXL343 detected ... `  Check your wiring!");
  }

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
  accel.writeRegister(0x24, 0x50);
  /* and adding all the axis to the interupt */
    // and trying to add all the of the axis to the interupt
  accel.writeRegister(0x27, 0xE0);
  accel.writeRegister(0x1D, 0xFF);

  Serial.println("ADXL343 init complete.");

  /*********************** WIFI CONNECTION  ************************/

  char ssid[] = SECRET_SSID;
  char pass[] = SECRET_PASS;

  if (STREAM) {
    WiFi.setPins(SPIWIFI_SS, SPIWIFI_ACK, ESP32_RESETN, ESP32_GPIO0, &SPIWIFI);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    do {
      analogWrite(LED_PIN_R, 0);
      analogWrite(LED_PIN_G, 128);
      analogWrite(LED_PIN_B, 128);
      status = WiFi.begin(ssid, pass);
      delay(50); // wait until connected
      analogWrite(LED_PIN_R, 0);
      analogWrite(LED_PIN_G, 0);
      analogWrite(LED_PIN_B, 0);
      delay(50); // wait until connected
    } while (status != WL_CONNECTED);
    Serial.println("Connected to wifi");

    analogWrite(LED_PIN_R, 0);
    analogWrite(LED_PIN_G, 255);
    analogWrite(LED_PIN_B, 255);
    delay(100); // wait until connected
    analogWrite(LED_PIN_R, 0);
    analogWrite(LED_PIN_G, 0);
    analogWrite(LED_PIN_B, 0);

    printWifiStatus();;
  }
  else {
    Serial.println("Not streaming, skipping WiFi Setup");
  }

  /*********************** Setup sensors ************************/
  
  // set up DHT sensor
  dht.begin();

  // If you have changed the I2C address of gas sensor, you must to be specify the address of I2C.
  //The default addrss is 0x08;
  gas.begin(Wire, 0x08); // use the hardware I2C - SDG 230314 - Getting rid of the Wire call in this.
  //gas.begin(0x08); // use the hardware I2C - SDG 230314 - reinstating having found the old library
  // SDG 230314 everything below can therefore be ignored for now.
  // The MultiGas setup, which seems to have changed since the first version of the software
  // and now it seems to need this... maybe, or at least that is what this website suggests:
  // https://github.com/Seeed-Studio/Mutichannel_Gas_Sensor/blob/master/examples/ReadSensorValue_Grove/ReadSensorValue_Grove.ino
  //gas.powerOn();

  // The PMS sensor is on Serial5
  Serial5.begin(9600);

  /*********************** Setup GPS ************************/

  // some configuration for the GPS, which is going to be trigger for the other data collection
  // elements.

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // Set the update rate
  // GPS.sendCommand(PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ); // 10 second update time
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_200_MILLIHERTZ); // 5 second update
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate

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
  audioShield.enable();
  audioShield.inputSelect(myInput);
  audioShield.volume(0.5);

  // Configure the window algorithm to use
  myFFT.windowFunction(AudioWindowHanning1024);
  //myFFT.windowFunction(NULL);

  /*********************** SD Card Setup ************************/

  ///// SD Card and Log File setup
  // the SPI for the SD Card
  //SPI.setMOSI(7);  // Audio shield has MOSI on pin 7
  //SPI.setSCK(14);  // Audio shield has SCK on pin 14

  analogWrite(LED_PIN_R, 128);
  analogWrite(LED_PIN_G, 128);
  analogWrite(LED_PIN_B, 0);

  delay(500);

  Serial.print("Initializing SD card...");

  while (true) {
    if (SD.begin(chipSelect)) {
      break;
    }
    Serial.println("initialization failed!");
    analogWrite(LED_PIN_R, 0);
    analogWrite(LED_PIN_G, 0);
    analogWrite(LED_PIN_B, 0);
    delay(200);
    analogWrite(LED_PIN_R, 255);
    delay(200);
    analogWrite(LED_PIN_R, 0);
    delay(200);
    analogWrite(LED_PIN_B, 255);
    delay(200);
    analogWrite(LED_PIN_B, 0);
    delay(200);
    analogWrite(LED_PIN_R, 255);
    delay(200);
    analogWrite(LED_PIN_R, 0);
    delay(200);
    analogWrite(LED_PIN_B, 255);
    delay(200);
    analogWrite(LED_PIN_B, 0);
    delay(200);
    analogWrite(LED_PIN_R, 255);
    delay(200);
    analogWrite(LED_PIN_R, 0);
    delay(200);
    analogWrite(LED_PIN_B, 255);
    delay(200);
    analogWrite(LED_PIN_B, 0);
    //digitalWrite(POWER_OFF_PIN, HIGH);
  }
  Serial.println("initialization done.");
  analogWrite(LED_PIN_R, 0);
  analogWrite(LED_PIN_G, 0);
  analogWrite(LED_PIN_B, 0);


/*********************** Log File Creation ************************/

  analogWrite(LED_PIN_R, 128);
  analogWrite(LED_PIN_G, 0);
  analogWrite(LED_PIN_B, 128);

    // The GPS function suggests a delay of 1s, so we're putting it in here.
  Serial.println("GPS delay");
  delay(1000);
  Serial.println("GPS delay end");
  
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

  /* Check time */
 /* if there is no time on the GPS (i.e. it thinks it 1980) then we
 *  just shutdown, becsause the wrest is useless...
 */
  Serial.println("Checking GPS clock");

  // for testing SDG
  if (GPS.year == 80) {
    Serial.println("GPS clock not correct, shutting down");
    analogWrite(LED_PIN_R, 0);
    analogWrite(LED_PIN_G, 0);
    analogWrite(LED_PIN_B, 0);
    delay(200);
    analogWrite(LED_PIN_R, 255);
    delay(200);
    analogWrite(LED_PIN_R, 0);
    delay(200);
    analogWrite(LED_PIN_R, 255);
    delay(200);
    analogWrite(LED_PIN_R, 0);
    delay(200);
    analogWrite(LED_PIN_R, 255);
    delay(200);
    analogWrite(LED_PIN_R, 0);
    delay(200);
    analogWrite(LED_PIN_R, 255);
    delay(200);
    analogWrite(LED_PIN_R, 0);
    delay(200);
    
    digitalWrite(POWER_OFF_PIN, HIGH);
    
  }

  analogWrite(LED_PIN_R, 0);
  analogWrite(LED_PIN_G, 0);
  analogWrite(LED_PIN_B, 0);

  // Construct the file name... which is a bit of a polava!
  log_file_name = "Scoopolog_";

  if (GPS.year < 10 ) log_file_name = log_file_name + "0";
  log_file_name = log_file_name + GPS.year;

  if (GPS.month < 10 ) log_file_name = log_file_name + "0";
  log_file_name = log_file_name + GPS.month;

  if (GPS.day < 10 ) log_file_name = log_file_name + "0";
  log_file_name = log_file_name + GPS.day;

  log_file_name = log_file_name + "-";

  if (GPS.hour < 10 ) log_file_name = log_file_name + "0";
  log_file_name = log_file_name + GPS.hour;

  if (GPS.minute < 10 ) log_file_name = log_file_name + "0";
  log_file_name = log_file_name + GPS.minute;

  if (GPS.seconds < 10 ) log_file_name = log_file_name + "0";
  log_file_name = log_file_name + GPS.seconds;
  
  log_file_name = log_file_name + ".txt";

  //const char filename[] = log_file_name.c_str();

  analogWrite(LED_PIN_R, 128);
  analogWrite(LED_PIN_G, 0);
  analogWrite(LED_PIN_B, 128);

  // open the file. 
  myFile = SD.open(log_file_name.c_str(), FILE_WRITE);
  //myFile = SD.open("test.txt", FILE_WRITE);

  delay(100);

  // check to see if the file opened ok!
  if (myFile) {
     Serial.print("Created Log file: ");
     Serial.print(log_file_name);
     analogWrite(LED_PIN_R, 255);
     analogWrite(LED_PIN_G, 0);
     analogWrite(LED_PIN_B, 255);
  }

  Serial.println();
  analogWrite(LED_PIN_R, 0);
  analogWrite(LED_PIN_G, 0);
  analogWrite(LED_PIN_B, 0);

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

  // Do something with the LEDs...
  analogWrite(LED_PIN_B, 255);

  /*********************** JSON Object Creation ********************/

  // create the json object that we are going to store data to
  StaticJsonDocument<1024> thisdata;
  JsonObject root = thisdata.to<JsonObject>();

  /*********************** Construct datetime ********************/

  // construct the GPS datetime... you feel there must be better way of doing this...
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

    // adding datetime values to the json obj
    // moving the datetime to root, and off of the GPS.
    root["datetime"] = datetime;


  /*********************** Construct location ********************/

  // the root of GPS json object
    JsonObject json_gps = root.createNestedObject("GPS");

    analogWrite(LED_PIN_B, 255);

    // LED will go Green if there is a fix and Red otherwise
    if (GPS.fix) {
      analogWrite(LED_PIN_G, 255);
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
      analogWrite(LED_PIN_R, 255);
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
  
    // measure and calcuate the gps backup battery voltage
    gps_bat_val = analogRead(8);
    gps_bat_volt = gps_bat_val * ADC_VOLT_REF / 1023.0;

    // adding values to the json obj
    json_gps["batt_volt"] = gps_bat_volt;

    delay(100);

    analogWrite(LED_PIN_R, 0);
    analogWrite(LED_PIN_G, 0);
    analogWrite(LED_PIN_G, 0);

    delay(100);

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

    // commented out much of this, because the new libary doesn't seem to include some parts of the
    // data structure.
    // The PM Sensor gives out the readings in a numebr of different formats:
    //JsonObject json_nop = json_pms.createNestedObject("num_of_part");

    //json_nop["UG_0.3"] = psmdata.PM_NP_UG_0_3;
    //json_nop["UG_0.5"] = psmdata.PM_NP_UG_0_5;
    //json_nop["UG_1.0"] = psmdata.PM_NP_UG_1_0;
    //json_nop["UG_2.5"] = psmdata.PM_NP_UG_2_5;
    //json_nop["UG_5.0"] = psmdata.PM_NP_UG_5_0;
    //json_nop["UG_10.0"] = psmdata.PM_NP_UG_10_0;
    
    //JsonObject json_sp = json_pms.createNestedObject("stand_part");

    //json_sp["SP_1.0"] = psmdata.PM_SP_UG_1_0;
    //json_sp["SP_2.5"] = psmdata.PM_SP_UG_2_5;
    //json_sp["SP_10.0"] = psmdata.PM_SP_UG_10_0;

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

    Serial.print(deserialized);
    Serial.println();

  /*********************** Printing to File ********************/
  // turn both Red and green on fully for file writing
  analogWrite(LED_PIN_G, 255);
  analogWrite(LED_PIN_R, 255);
    myFile.write(deserialized);
    myFile.write("\n");
    myFile.flush();
    // close
    //myFile.close();
  // and off again
  analogWrite(LED_PIN_G, 0);
  analogWrite(LED_PIN_R, 0);

  /*********************** POSTing ********************/

  if (STREAM) {
    // save data to the feed variable on Adafruit IO
    // turn both Red and green on half for file writing
    analogWrite(LED_PIN_G, 125);
    analogWrite(LED_PIN_R, 125);
    Serial.print("POSTing -> ");
    String contentType = "application/json";

    // SDG 230316 - trying to get a timeout to work.
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

    analogWrite(LED_PIN_G, 0);
    analogWrite(LED_PIN_R, 0);

    // Now turn off blue
    analogWrite(LED_PIN_B, 0);

    // Adafruit IO is rate limited for publishing, so a delay is required in
    // between feed->save events. In this example, we will wait three seconds
    // (1000 milliseconds == 1 second) during each loop.
    //delay(3000);

   /*********************** CHECK TIME, THEN WIFI AND UPLOAD ********************/

    char ssid[] = SECRET_SSID;
    char pass[] = SECRET_PASS;

    if (GPS.seconds < last_seconds){
      Serial.println("New Minutes, checking WiFi");

      int numSsid = WiFi.scanNetworks();
      if (numSsid == -1) {
        Serial.println("Couldn't get a wifi connection");
        while (true);
      }

      // print the list of networks seen:
      Serial.print("number of available networks:");
      Serial.println(numSsid);

      // print the network number and name for each network found:
      for (int thisNet = 0; thisNet < numSsid; thisNet++) {

        if (strcmp(WiFi.SSID(thisNet),ssid) == 0){
          Serial.print(ssid);
          Serial.println(" Found, trying to connect");

            do {
              analogWrite(LED_PIN_R, 0);
              analogWrite(LED_PIN_G, 128);
              analogWrite(LED_PIN_B, 128);
              status = WiFi.begin(ssid, pass);
              delay(50); // wait until connected
              analogWrite(LED_PIN_R, 0);
              analogWrite(LED_PIN_G, 0);
              analogWrite(LED_PIN_B, 0);
              delay(50); // wait until connected
              Serial.print(".");
              } while (status != WL_CONNECTED);
            Serial.println("");
            Serial.println("Connected to wifi");
            printWifiStatus();;

            myFile.close();

            // Create the file new and write a string into it
            Serial.print("Attempting to upload ");
            Serial.println(log_file_name.c_str());

            myFile = SD.open(log_file_name.c_str());

            ftp.OpenConnection();

            ftp.InitFile(COMMAND_XFER_TYPE_BINARY);
            ftp.NewFile(log_file_name.c_str());

            int i = 0;

            // while through the file uploading it line by line...
            while (myFile.available()) {
              byte data[256];
              myFile.read(data, 256);
    
              ftp.Write(data);

              Serial.print("Uploading: ");
              Serial.println(i);
              i = i+1;
              delay(100);
             }
             Serial.println("Closing File");
             ftp.CloseFile();

             Serial.println("Closing Connection");
             ftp.CloseConnection();

             myFile.close();

             Serial.print("Upload finished - shutting down");
             delay(500);

             analogWrite(LED_PIN_R, 0);
             analogWrite(LED_PIN_G, 0);
             analogWrite(LED_PIN_B, 0);
             delay(200);
             analogWrite(LED_PIN_G, 255);
             delay(200);
             analogWrite(LED_PIN_G, 0);
             delay(200);
             analogWrite(LED_PIN_G, 255);
             delay(200);
             analogWrite(LED_PIN_G, 0);
             delay(200);
             analogWrite(LED_PIN_G, 255);
             delay(200);
             analogWrite(LED_PIN_G, 0);
             delay(200);
             analogWrite(LED_PIN_G, 255);
             delay(200);
             analogWrite(LED_PIN_G, 0);
             delay(200);

             /* just clear any interupts there might be on the acceleromter */
             int_resp = accel.checkInterrupts();

             delay(5000);
    
             digitalWrite(POWER_OFF_PIN, HIGH);
         }      
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
        activity_counter = 5;
      }

      // if the activity counter has got to zero then turn off
      if (activity_counter < 1) {

        myFile.close();
        
        analogWrite(LED_PIN_R, 0);
        analogWrite(LED_PIN_G, 0);
        analogWrite(LED_PIN_B, 0);
        delay(200);
        analogWrite(LED_PIN_R, 255);
        analogWrite(LED_PIN_G, 255);
        delay(200);
        analogWrite(LED_PIN_R, 0);
        analogWrite(LED_PIN_G, 0);
        delay(200);
        analogWrite(LED_PIN_R, 255);
        analogWrite(LED_PIN_G, 255);
        delay(200);
        analogWrite(LED_PIN_R, 0);
        analogWrite(LED_PIN_G, 0);
        delay(200);
        analogWrite(LED_PIN_R, 255);
        analogWrite(LED_PIN_G, 255);
        delay(200);
        analogWrite(LED_PIN_R, 0);
        analogWrite(LED_PIN_G, 0);
        delay(200);
        analogWrite(LED_PIN_R, 255);
        analogWrite(LED_PIN_G, 255);
        delay(200);
        analogWrite(LED_PIN_R, 0);
        analogWrite(LED_PIN_G, 0);
        delay(200);
        digitalWrite(POWER_OFF_PIN, HIGH);
      }
    }
      
    // saving this seconds so we can compair with the next to see if a minute has gone by
    last_seconds = GPS.seconds;

  }
}
