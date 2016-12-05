
#include <Arduino.h>
#include <SPI.h>

#include <Adafruit_ATParser.h>
#include <Adafruit_BLE.h>
#include <Adafruit_BLEBattery.h>
#include <Adafruit_BLEEddystone.h>
#include <Adafruit_BLEGatt.h>
#include <Adafruit_BLEMIDI.h>
#include <Adafruit_BluefruitLE_SPI.h>
#include <Adafruit_BluefruitLE_UART.h>

#include <SD.h>
#include <Wire.h>
#include "RTClib.h"

#include "BluefruitConfig.h"

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/*
   Logger for the Billion Oyster Project, Michael Sloan Warren 2016
   using Adafruit Datalogger shield based on https://github.com/adafruit/Light-and-Temp-logger/blob/master/lighttemplogger.pde
   and with i2c components from https://www.whiteboxes.ch/tentacle/docs/

   We'll be using the Tentacle Shield by Whitebox on i2c mode so that the pins we need for the datashield are free
   Use the Whitebox utility program to ensure i2c address match up and to calibrate sensors.

   This code is in the public domain and is not intended for commerical use.
   V2.2 April 17, 2016
*/




// define the sensor addresses
const int phsensoraddress = 100;
const int dosensoraddress = 101;
const int consensoraddress = 102;
const int tempsensoraddress = 103;
//need to define these using the utility, I use 100-103 (0x64-0x67)
//Using a quick i2c scanner isn't a bad idea: http://playground.arduino.cc/Main/I2cScanner

//indicate if we want to use the serial monitor to mirror readings
const boolean ECHO_TO_SERIAL = true;
const boolean ECHO_TO_BLE = true;

//datalogger variables
const int redLEDpin = 2; // digital pins that connect to the LEDs on the logger shield
const int greenLEDpin = 3; // digital pins that connect to the LEDs on the logger shield
const int chipSelect = 10; // for the data logging shield, we use digital pin 10 for the SD cs line
uint32_t syncTime = 0; // time of last sync()

//i2c variables
char sensordata[30]; //A 30 byte character array to hold incoming data from the sensors
byte i2c_response_code = 0;              //used to hold the I2C response code.
byte in_char = 0;                    //used as a 1 byte buffer to store in bound bytes from an I2C stamp.
byte sensor_bytes_received = 0;


//timing variables
long currentMillis = 0;
long previousMillis = 0;
long interval = 600000;  //this is the interval between readings in milliseconds; default is 600000 (10 minutes)

File logfile; // the logging file
RTC_DS1307 RTC; // define the Real Time Clock object


void setup(void)
{
  Serial.begin(9600);
  Serial.println();

  // use debugging LEDs
  pinMode(redLEDpin, OUTPUT);
  pinMode(greenLEDpin, OUTPUT);

  // initialize the SD card
  Serial.print("Initializing SD card...");
  pinMode(10, OUTPUT);

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    error("Card failed, or not present");
  }
  Serial.println("card initialized.");

  // create a new file on the SD card
  char filename[] = "LOGGER00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i / 10 + '0';
    filename[7] = i % 10 + '0';
    if (! SD.exists(filename)) {
      logfile = SD.open(filename, FILE_WRITE);
      break;
    }
  }
  if (! logfile) {
    error("couldnt create file");
  }
  else {
    Serial.print("Logging to: ");
    Serial.println(filename);
  }

  //BLUEFRUIT SETUP:
  if(ECHO_TO_BLE) {
    if ( !ble.begin(VERBOSE_MODE) )
    {
      error("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?");
    }
    /* Wait for connection */
    while (! ble.isConnected()) {
      delay(500);
      Serial.println("connecting to BLE");
    }
    /* Set module to DATA mode */
    ble.setMode(BLUEFRUIT_MODE_DATA);
    delay(3000);
  }
  
  // connect to RTC
  Wire.begin();
  if (!RTC.begin()) {
    writeData("RTC failed");
  }

  //WRITE HEADER:
  writeData("msec,timestamp,datetime,pH,DO,temp, conductivity\n");
  ble.print("specifig");
  ble.flush();
}

void loop(void)
{
  DateTime now;
  digitalWrite(greenLEDpin, HIGH); //green LED indicates that we're between readings
  // delay for the amount of time we want between readings

  //timer to pause the code between intervals
  currentMillis = millis();
  if ((currentMillis - previousMillis) > interval) {
    previousMillis = currentMillis - (currentMillis % interval);
  }
  else {
    return;
  }

  digitalWrite(greenLEDpin, LOW);

  // log milliseconds since starting the session
  uint32_t m = millis();
  writeData(m);


  // fetch the time from the shield
  now = RTC.now();
  // log time
  writeData(now.unixtime()); // seconds since 1/1/1970 - makes it easier to crunch later
  writeData(", ");
  writeData('"');
  writeData(now.year());
  writeData("/");
  writeData(now.month());
  writeData("/");
  writeData(now.day());
  writeData(" ");
  writeData(now.hour());
  writeData(":");
  writeData(now.minute());
  writeData(":");
  writeData(now.second());
  writeData("\"");


  //now, take readings from the sensors
  //check next tab for more details, just need to change i2c address each time
  readsensor(phsensoraddress); //ph sensor
  writeData(", ");
  writeData(sensordata);
 
  readsensor(dosensoraddress); //do sensor
  writeData(", ");
  writeData(sensordata);
  
  readsensor(tempsensoraddress); //temp sensor
  writeData(", ");
  writeData(sensordata);
 
  readsensor(consensoraddress); //conductivity sensor
  writeData(", ");
  writeData(sensordata);

  //end the line for formatting
  logfile.print("\n");

  digitalWrite(redLEDpin, HIGH);  // blink red LED to show we are syncing data to the card
  logfile.flush();
  digitalWrite(redLEDpin, LOW);
}

//handle three modes of writing data:
void writeData(int n) {
  writeData(String(n, DEC));
}
void writeData(String s) {
  if (ECHO_TO_SERIAL) Serial.println(s);
  if (ECHO_TO_BLE ) ble.print(s);
  logfile.print(s);
}

void readsensor(int address) {
  i2c_response_code = 254;
  sensor_bytes_received = 0;                          // reset data counter
  memset(sensordata, 0, sizeof(sensordata));          // clear sensordata array;

  Wire.beginTransmission(address);                  //open line to i2c address
  Wire.write("r");                                  //transmit the read command
  Wire.endTransmission();                           //end the I2C data transmission.

  while (i2c_response_code == 254) {                 // in case the cammand takes longer to process, we keep looping here until we get a success or an error
    delay(1000);                                  // delay to allow the stamp time to read
    Wire.requestFrom(address, 48, 1);     //call the circuit and request 48 bytes (this is more then we need).
    i2c_response_code = Wire.read();              //the first byte is the response code, we read this separately.
    //Serial.println(i2c_response_code);          //debugging
    while (Wire.available()) {                    //as long are there bytes to receive...
      in_char = Wire.read();                      //receive a byte.

      if (in_char == 0) {                         //once we are sent a null command.
        Wire.endTransmission();                   //end the I2C data transmission.
        break;                                    //exit the while loop.
      }
      else {
        sensordata[sensor_bytes_received] = in_char;        //load this byte into our array.
        sensor_bytes_received++;                //keep track of how many bytes it is
      }
    }
  }
}

void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);

  // red LED indicates error
  digitalWrite(redLEDpin, HIGH);
  while (1);
}
