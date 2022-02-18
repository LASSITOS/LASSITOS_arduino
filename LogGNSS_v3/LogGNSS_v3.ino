/*
  Getting time and date using u-blox commands an saving them to SD card.
  By: AcCapellis
  Date: December 19th, 2021
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  Based on Examples from used libraries and hardware.

  Hardware Connections:
  
  Plug a Qwiic cable into the GNSS and a BlackBoard
  Open the serial monitor at 115200 baud to see the output
  
  Connect RXI of OpenLog to pin 27 on ESP32 and TXI of OpenLog to pin 12 on ESP32
  
  
  BLE:
  Set up BLE connections with UART communication
  Start new data files if "START" is received and stop current data files if "STOP" is received
  
*/

#include <Wire.h> //Needed for I2C to GNSS

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> 
SFE_UBLOX_GNSS myGNSS;


//HardwareSerial
// #include <SoftwareSerial.h> //Needed for Software Serial to OpenLog
// SoftwareSerial OpenLog;

//HardwareSerial
#include <HardwareSerial.h>
HardwareSerial OpenLog(2);

#define version "v3.0"

long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.
long lastTime2 = 0; //Second simple local timer. 
long lastTime_logstatus = 0; //Second simple local timer. 
int statLED = 13;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
int resetOpenLog = 25; //This pin resets OpenLog. Connect pin 25 to pin GRN on OpenLog.
//PINs can be changed to any pin. // For Hardware Serial use Pin 16 an 17. SoftwareSerial worked on pins 12 and 27
int PIN_Rx = 16; // 12 = Hardware RX pin,
int PIN_Tx = 17; // 27 = Hardware TX pin,
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
int baudrateOpenLog= 115200 ;

char headerFileName[24]; //Max file name length is 23 characters)
char dataFileName[24]; //Max file name length is 23 characters)
char hFN2[12] ; 
char dataFileName2[12];


// Setting for u-blox 
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
bool logging = false;

bool IMUcalibration=false;
int NavigationFrequency = 10;
dynModel DynamicModel = (dynModel)4;
bool log_RMX = false;
bool log_ESFRAW  = false;
bool log_ESFMEAS = false;
bool log_ESFALG = false;
bool log_NAVPVAT = false;
bool log_NAVPVT = true;
bool log_NAVATT = true;
bool log_ESFINS = true;
bool log_STATUS = true;
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-



// #define BufferSize 256
// #define packetLength 100 // NAV PVT is 92 + 8 bytes in length (including the sync chars, class, id, length and checksum bytes)

#define sdWriteSize 256 // Write data to the SD card in blocks of 512 bytes
#define fileBufferSize 8192 // Allocate 32KBytes of RAM for UBX message storage

unsigned long lastPrint; // Record when the last Serial print took place
unsigned long bytesWritten = 0; // Record how many bytes have been written to SD card


long Year ;
long Month ;
long Day ;
long Hour ;
long Minute ;
long Second;


// Setting for BLE connection
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
char txString[500];   // String containing messages to be send to BLE terminal
char subString[20];
bool BLE_message=false;
bool BLE_stop=false;   // Stop datalogger over BLE
bool BLE_start=false;  // Start datalogger over BLE

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"






void LED_blink(int len, int times ) { // Used for blinking LED  times at interval len in milliseconds
  int a;
  for( a = 0; a < times; a = a + 1 ){
      digitalWrite(statLED, HIGH);
      delay(len);
      digitalWrite(statLED, LOW);
      if  (times > 1) delay(len); // Wait after blinking next time
   }   
}





// /////////////////////////////////////////////
// ---------------------------------------------
//GNSS code
// ---------------------------------------------
// /////////////////////////////////////////////


// Callback: printPVTdata will be called when new NAV PVT data arrives
// See u-blox_structs.h for the full definition of UBX_NAV_PVT_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setAutoPVTcallback
//        /                  _____  This _must_ be UBX_NAV_PVT_data_t
//        |                 /               _____ You can use any name you like for the struct
//        |                 |              /
//        |                 |              |
void printPVTdata(UBX_NAV_PVT_data_t ubxDataStruct)
{
    Serial.println();

    Serial.print(F("Time: ")); // Print the time
    uint8_t hms = ubxDataStruct.hour; // Print the hours
    if (hms < 10) Serial.print(F("0")); // Print a leading zero if required
    Serial.print(hms);
    Serial.print(F(":"));
    hms = ubxDataStruct.min; // Print the minutes
    if (hms < 10) Serial.print(F("0")); // Print a leading zero if required
    Serial.print(hms);
    Serial.print(F(":"));
    hms = ubxDataStruct.sec; // Print the seconds
    if (hms < 10) Serial.print(F("0")); // Print a leading zero if required
    Serial.print(hms);
    Serial.print(F("."));
    unsigned long millisecs = ubxDataStruct.iTOW % 1000; // Print the milliseconds
    if (millisecs < 100) Serial.print(F("0")); // Print the trailing zeros correctly
    if (millisecs < 10) Serial.print(F("0"));
    Serial.print(millisecs);

    long latitude = ubxDataStruct.lat; // Print the latitude
    Serial.print(F(" Lat: "));
    Serial.print(latitude);

    long longitude = ubxDataStruct.lon; // Print the longitude
    Serial.print(F(" Long: "));
    Serial.print(longitude);
    Serial.print(F(" (degrees * 10^-7)"));

    long altitude = ubxDataStruct.hMSL; // Print the height above mean sea level
    Serial.print(F(" Height above MSL: "));
    Serial.print(altitude);
    Serial.println(F(" (mm)"));
  
  BLE_message=true;
    sprintf(txString, "Time:  %02d:%02d:%02d.%04d,  Lat: %d, Long: %d, Elev: %.1d (m) /n",
         ubxDataStruct.hour,ubxDataStruct.min,ubxDataStruct.sec,
         latitude * 0.0000001,longitude * 0.0000001,altitude/1000);
}


void checkIMUcalibration(){
    BLE_message=true;
    // ESF data is produced at the navigation rate, so by default we'll get fresh data once per second
    if (myGNSS.getEsfInfo()) // Poll new ESF STATUS data
    {
      strcat(txString,"Fusion Mode: ");  
      sprintf(subString,"%d",myGNSS.packetUBXESFSTATUS->data.fusionMode);  
      strcat(txString,subString); 
      if (myGNSS.packetUBXESFSTATUS->data.fusionMode == 0){
      strcat(txString,"  Sensor is initializing..."); 
      strcat(txString,"\n");
      // logging=false;
      }
      else if (myGNSS.packetUBXESFSTATUS->data.fusionMode == 1){
      strcat(txString,"  Sensor is calibrated!");  
      strcat(txString,"\n");
      IMUcalibration=false;
      }
      else if (myGNSS.packetUBXESFSTATUS->data.fusionMode == 2){
      strcat(txString,"  Sensor fusion is suspended!"); 
      strcat(txString,"\n");
      IMUcalibration=false;
      }
      else if (myGNSS.packetUBXESFSTATUS->data.fusionMode == 3){
      strcat(txString,"  Sensor fusion is disabled!"); 
      strcat(txString,"\n");
      IMUcalibration=false;
      }
    
    }
}




// /////////////////////////////////////////////
// ---------------------------------------------
// SD card funtions und code
// ---------------------------------------------
// /////////////////////////////////////////////

//This function pushes OpenLog into command mode
void gotoCommandMode(void) {
  bool cmdMod =false;
  int count = 0;
  if (OpenLog.available()) OpenLog.read(); 
  
  //Send three control z to enter OpenLog command mode
  //Works with Arduino v1.0
  OpenLog.write(26);
  OpenLog.write(26);
  OpenLog.write(26);

  //Wait for OpenLog to respond with '>' to indicate we are in command mode
// / while (1) {
  while(!cmdMod) {
    for (int timeOut = 0 ; timeOut < 500 ; timeOut++) {
      if (OpenLog.available()){
        if (OpenLog.read() == '>') {
          Serial.println("In command mode");
          cmdMod = true;
          break; 
          
        }
      }
      delay(1);
    }
    
    if (!cmdMod) {
      Serial.print("Can't go in command mode. Reseting SD");
      ++count;
      if (count>2) break;
      digitalWrite(resetOpenLog, LOW);
      delay(1000);
      digitalWrite(resetOpenLog, HIGH);
      delay(1000);
      if (OpenLog.available()) OpenLog.read();
      OpenLog.write(26);
      OpenLog.write(26);
      OpenLog.write(26);
    }
  }
}

//This function pushes OpenLog into command mode if it is not there jet. 
//Use with caution. It will push garbage data to Openlog that will be saved to Logfile if it is not in Command mode. 
// void checkandgotoCommandMode(void) {
  // bool resp = false;
  // int count = 0;
  // Serial.print("Going to command mode.");
  // while (!resp){
    // ++count;
    // //Send three control z to enter OpenLog command mode
    // //Works with Arduino v1.0
    // OpenLog.write(26);
    // OpenLog.write(26);
    // OpenLog.write(26);
  
    // //Wait for OpenLog to respond with '>' to indicate we are in command mode
    // for (int timeOut = 0 ; timeOut < 500 ; timeOut++) {
      // if (OpenLog.available()){
        // if (OpenLog.read() == '>') {
          // resp = true;
          // Serial.println(F("OpenLog is in Command Mode"));
          // break;  
        // }
      // }
      // delay(1);
    // }
    // if (count > 2) { 
      // Serial.println(F("Can't get in command mode. Freezing."));
      // while (1){
      // }
    // } else if (!resp) {
      // Serial.println(F("Problem getting in command mode. Retry in case OpenLog was already in command mode"));
    // }
  // }
// }


//Setups up the software serial, resets OpenLog so we know what state it's in, and waits
//for OpenLog to come online and report '<' that it is ready to receive characters to record
void setupOpenLog(void) {
  pinMode(resetOpenLog, OUTPUT);
  
  Serial.println(F("Connecting to openLog")); 
//    OpenLog.begin(baudrateOpenLog, SWSERIAL_8N1, PIN_Rx, PIN_Tx, false, 256); // Use this for SoftwareSerial
  OpenLog.begin(baudrateOpenLog,SERIAL_8N1, PIN_Rx, PIN_Tx);  // Use this for HardwareSerial
  
  if (!OpenLog) { // If the object did not initialize, then its configuration is invalid
    Serial.println(F("Invalid SoftwareSerial pin configuration, check config")); 
    while (1) { // Don't continue with invalid configuration. Blink LED 3 times every 2 seconds
      LED_blink(200, 3);
      delay(1000);
        //Reset OpenLog
      digitalWrite(resetOpenLog, LOW);
      delay(5);
      digitalWrite(resetOpenLog, HIGH);
      delay(1000);
    }
  }

  //Reset OpenLog
  digitalWrite(resetOpenLog, LOW);
  delay(5);
  digitalWrite(resetOpenLog, HIGH);

  //Wait for OpenLog to respond with '<' to indicate it is alive and recording to a file
  while (1) {
    if (OpenLog.available())
      if (OpenLog.read() == '<') break;
  }
  Serial.println("OpenLog online");
}


//This function creates a given file and then opens it in append mode (ready to record characters to the file)
//Then returns to listening mode
//This function assumes the OpenLog is in command mode
void createFile(char *fileName) {
  while(OpenLog.available()) OpenLog.read(); //Clear incoming buffer
  //New way
  OpenLog.print("new ");
  OpenLog.print(fileName);
  OpenLog.write(13); //This is \r
//  OpenLog.println(fileName); //regular println works with OpenLog v2.51 and above

  //Wait for OpenLog to return to waiting for a command
    //Wait for OpenLog to respond with '>' to indicate we are in command mode
  while (1) {
    if (OpenLog.available())
      if (OpenLog.read() == '>') break;
  }
//  Serial.print("Made new file");

  OpenLog.print("append ");
//  OpenLog.println(fileName); //regular println works with OpenLog v2.51 and above
  OpenLog.print(fileName);
  OpenLog.write(13); //This is \r

  //Wait for OpenLog to indicate file is open and ready for writing
    //Wait for OpenLog to respond with '>' to indicate we are in command mode
//  while (1) {
//    if (OpenLog.available())
//      if (OpenLog.read() == '>') break;
//  }
  Serial.println("OpenLog is now waiting for characters and will record them to the new file");
  //OpenLog is now waiting for characters and will record them to the new file
}


//Reads the contents of a given file and dumps it to the serial terminal
//This function assumes the OpenLog is in command mode
void readFile(char *fileName) {
  
  while(OpenLog.available()) OpenLog.read(); //Clear incoming buffer

  OpenLog.print("read ");
  OpenLog.print(fileName);
  OpenLog.write(13); //This is \r

  //The OpenLog echos the commands we send it by default so we have 'read log823.txt\r' sitting
  //in the RX buffer. Let's try to not print this.
  while (1) {
    if (OpenLog.available())
      if (OpenLog.read() == '\r') break;
  }

  Serial.print("Reading from file: ");
  Serial.println(fileName);
  //This will listen for characters coming from OpenLog and print them to the terminal
  //This relies heavily on the SoftSerial buffer not overrunning. This will probably not work
  //above 38400bps.
  //This loop will stop listening after 1 second of no characters received
  for (int timeOut = 0 ; timeOut < 1000 ; timeOut++) {
    while (OpenLog.available()) {
      char tempString[100];

      int spot = 0;
      while (OpenLog.available()) {
        tempString[spot++] = OpenLog.read();
        if (spot > 98) break;
      }
      tempString[spot] = '\0';
      Serial.write(tempString); //Take the string from OpenLog and push it to the Arduino terminal
      timeOut = 0;
    }

    delay(1);
  }

  //This is not perfect. The above loop will print the '.'s from the log file. These are the two escape characters
  //recorded before the third escape character is seen.
  //It will also print the '>' character. This is the OpenLog telling us it is done reading the file.

  //This function leaves OpenLog in command mode
}


//Check the stats of the SD card via 'disk' command
//This function assumes the OpenLog is in command mode
void readDisk() {
  OpenLog.println("disk");
  
  //The OpenLog echos the commands we send it by default so we have 'disk\r' sitting 
  //in the RX buffer. Let's try to not print this.
  while(1) {
    if(OpenLog.available())
      if(OpenLog.read() == '\r') break;
  }  

  //This will listen for characters coming from OpenLog and print them to the terminal
  //This relies heavily on the SoftSerial buffer not overrunning. This will probably not work
  //above 38400bps.
  //This loop will stop listening after 1 second of no characters received
  for(int timeOut = 0 ; timeOut < 1000 ; timeOut++) {
    while(OpenLog.available()) {
      char tempString[100];
      
      int spot = 0;
      while(OpenLog.available()) {
        tempString[spot++] = OpenLog.read();
        if(spot > 98) break;
      }
      tempString[spot] = '\0';
      Serial.write(tempString); //Take the string from OpenLog and push it to the Arduino terminal
      timeOut = 0;
    }

    delay(1);
  }

  //This is not perfect. The above loop will print the '.'s from the log file. These are the two escape characters
  //recorded before the third escape character is seen.
  //It will also print the '>' character. This is the OpenLog telling us it is done reading the file.  

  //This function leaves OpenLog in command mode
}



void readDateTime(){
  Year = myGNSS.getYear();
  Month = myGNSS.getMonth();
  Day = myGNSS.getDay();
  Hour = myGNSS.getHour();
  Minute = myGNSS.getMinute();
  Second = myGNSS.getSecond();
}

void  printDateTime() {  
  Serial.print(Year);
  Serial.print("-");
  Serial.print(Month);
  Serial.print("-");
  Serial.print(Day);
  Serial.print("  ");
  Serial.print(Hour);
  Serial.print(":");
  Serial.print(Minute);
  Serial.print(":");
  Serial.print(Second);
}

void  Write_header() {  
  OpenLog.println(F("# GNSS log file"));
  OpenLog.print("# Date: ");
  OpenLog.print(Year);
  OpenLog.print("-");
  OpenLog.print(Month);
  OpenLog.print("-");
  OpenLog.println(Day);
  OpenLog.print("# Time: ");
  OpenLog.print(Hour);
  OpenLog.print(":");
  OpenLog.print(Minute);
  OpenLog.print(":");
  OpenLog.println(Second);
  OpenLog.println(F("# --------------------------------------"));
  OpenLog.print(F("# UBX binary data are in  file: "));
  OpenLog.print(dataFileName);
  OpenLog.println(F(" "));
}

char nth_letter(int n)
{
    return "abcdefghijklmnopqrstuvwxyz"[n];
}


void  log_settings() {  
  OpenLog.println(F("# --------------------------------------"));
  OpenLog.print(F("Script version: "));
  OpenLog.println(version);
  OpenLog.println(F("Measurements settings: "));
  OpenLog.print(F("IMU calibration: "));
  OpenLog.println(IMUcalibration);
  OpenLog.print(F("NavigationFrequency: "));
  OpenLog.println(NavigationFrequency);
  OpenLog.print(F("DynamicModel: "));
  OpenLog.println(DynamicModel);
  OpenLog.print(F("log_NAVATT: "));
  OpenLog.println(log_ESFALG);
  OpenLog.print(F("log_NAVPVT: "));
  OpenLog.println(log_ESFALG);
  OpenLog.print(F("log_NAVPVAT: "));
  OpenLog.println(log_ESFALG);
  OpenLog.print(F("log_ESFINS: "));
  OpenLog.println(log_ESFALG);
  OpenLog.print(F("log_ESFRAW: "));
  OpenLog.println(log_ESFRAW);
  OpenLog.print(F("log_ESFMEAS: "));
  OpenLog.println(log_ESFMEAS);
  OpenLog.print(F("log_ESFALG: "));
  OpenLog.println(log_ESFALG);
  OpenLog.print(F("log_RMX : "));
  OpenLog.println(log_RMX );
  OpenLog.println(F("# --------------------------------------"));
}


// Make new files in SD card. file names are derived from GNSS time if available. 
void  makeFiles() {  
  Serial.println(F("Making new files"));

  // Check for GPS to have good time and date
  int count = 0;
  while (1) {  // Check for GPS to have good time and date
    ++count;
    if ((myGNSS.getTimeValid() == true) && (myGNSS.getDateValid() == true)) {
        Serial.print(F("Date and time are valid. It is: "));
        readDateTime();
        printDateTime();
        Serial.println();
        break;
    }else if (count > 5) {
        Year = 2000 ;
        Month = 01;
        Day = 01;
        Hour = random(23) ;
        Minute = random(60);
        Second = random(60);
        break;
        Serial.println(F("GPS is not good. Making random filename with date:"));
        printDateTime();
        Serial.println(); 
    }
    
    Serial.println(F("Date or time are not valid. Waiting for better GPS connection."));
    readDateTime();
    Serial.print(F("Date and time: "));
    printDateTime();
    Serial.println();
    LED_blink(1000, 5);
  }
  
  sprintf(headerFileName, "%02d%02d%02d_%02d%02d.txt", Year%1000 ,Month ,Day ,Hour,Minute);  // create name of header file
  sprintf(dataFileName, "%02d%02d%02d_%02d%02d.ubx", Year%1000 ,Month ,Day ,Hour,Minute);   // create name of data file
  Serial.println(headerFileName);
  Serial.println(dataFileName);
  
  
  gotoCommandMode(); //Puts OpenLog in command mode.
//  readDisk();
//  checkandgotoCommandMode();
  Serial.print("Making header file.");
  createFile(headerFileName); //Creates a new file called Date_Time.txt
  Serial.print(F("created file: "));
  Serial.println(headerFileName);
  Write_header();
  delay(100);
  log_settings();
  delay(100);
  
  gotoCommandMode();  //Puts OpenLog in command mode.
//  readFile(headerFileName);

  Serial.print("Making data file:");
  Serial.println(dataFileName);
  createFile(dataFileName); //Creates a new file called Date_Time.ubx
  Serial.print("Created file: ");
  Serial.println(dataFileName);
  LED_blink(100,10);
}


void  Write_stop() {  
  OpenLog.print(F("Measurements stopped on: "));
  OpenLog.print(Year);
  OpenLog.print("-");
  OpenLog.print(Month);
  OpenLog.print("-");
  OpenLog.print(Day);
  OpenLog.print(", ");
  OpenLog.print(Hour);
  OpenLog.print(":");
  OpenLog.print(Minute);
  OpenLog.print(":");
  OpenLog.println(Second);
  OpenLog.println(F("# --------------------------------------"));
}



// setting up GPS 
void setupGNSS(){ 
    Serial.println(F("setting up GPS for automatic messages"));
    myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
      
    myGNSS.setDynamicModel(DynamicModel);
    myGNSS.setNavigationFrequency(NavigationFrequency); //Produce  navigation solution at given frequency
      
    if (log_NAVPVT) {
      // myGNSS.setAutoPVTcallback(&printPVTdata); // Enable automatic NAV PVT messages with callback to printPVTdata
      myGNSS.setAutoPVT(true, false); // Enable automatic NAV PVT messages without callback to printPVTdata
      myGNSS.logNAVPVT(); // Enable NAV PVT data logging
      }
    if (log_NAVPVAT) {
      // myGNSS.setAutoPVTcallback(&printPVTdata); // Enable automatic NAV PVT messages with callback to printPVTdata
      myGNSS.setAutoNAVPVAT(true, false); // Enable automatic NAV PVT messages without callback to printPVTdata
      myGNSS.logNAVPVAT(); // Enable NAV PVT data logging
    }
    if (log_NAVATT) {
      myGNSS.setAutoNAVATT(true, false); 
      myGNSS.logNAVATT(); 
    }
    if (log_ESFINS ) {
       myGNSS.setAutoESFINS(true, false); 
       myGNSS.logESFINS();
    }
    if (log_ESFRAW ) {     
      myGNSS.setAutoESFRAW(true, false);    
      myGNSS.logESFRAW(); 
    }
    if (log_ESFMEAS) {
      myGNSS.setAutoESFMEAS(true, false); 
      myGNSS.logESFMEAS();
    }
    if (log_ESFALG ) {
      myGNSS.setAutoESFALG(true, false); 
      myGNSS.logESFALG(); 
    }  
    if (log_RMX ) {
      myGNSS.disableUBX7Fcheck(); // RAWX data can legitimately contain 0x7F, so we need to disable the "7F" check in checkUbloxI2C
      myGNSS.setAutoRXMSFRBX(true, false); // Enable automatic RXM SFRBX messages: without callback; without implicit update
      myGNSS.logRXMSFRBX(); // Enable RXM SFRBX data logging
      myGNSS.setAutoRXMRAWX(true, false); // Enable automatic RXM RAWX messages: without callback; without implicit update
      myGNSS.logRXMRAWX(); // Enable RXM RAWX data logging
    }
    
	if (log_STATUS) {
      myGNSS.logESFSTATUS();
	  myGNSS.logESFALG();
    }
	
    Serial.println(F("GPS setting updated"));
    strcat(txString,"GPS setting updated");
}



// Stop data logging process
void stop_logging() {
	logging = false; // Set flag to false
	BLE_message=true;
	strcpy(txString,"Turning datalogging OFF!");

	if (log_RMX ) {
	  myGNSS.setAutoRXMSFRBX(false, false); // Disable the automatic RXM SFRBX messages
	  myGNSS.setAutoRXMRAWX(false, false); // Disable the automatic RXM RAWX messages
	}
	if (log_NAVPVT) {
	  myGNSS.setAutoPVT(false, false); // Enable automatic NAV PVT messages without callback to printPVTdata
	  }
	if (log_NAVPVAT) {
	  myGNSS.setAutoNAVPVAT(false, false); // Enable automatic NAV PVT messages without callback to printPVTdata
	}
	if (log_NAVATT) {
	  myGNSS.setAutoNAVATT(false, false); 
	}
	if (log_ESFINS ) {
	   myGNSS.setAutoESFINS(false, false); 
	}
	if (log_ESFRAW ) {     
	  myGNSS.setAutoESFRAW(false, false);    
	}
	if (log_ESFMEAS) {
	  myGNSS.setAutoESFMEAS(false, false); 
	}
	if (log_ESFALG ) {
	  myGNSS.setAutoESFALG(false, false); 
	}
    delay(1000); // Allow time for any remaining messages to arrive
    myGNSS.checkUblox(); // Process any remaining data
  
    uint16_t remainingBytes = myGNSS.fileBufferAvailable(); // Check if there are any bytes remaining in the file buffer
    
    while (remainingBytes > 0) // While there is still data in the file buffer
    {
      digitalWrite(LED_BUILTIN, HIGH); // Flash LED_BUILTIN while we write to the SD card
      uint8_t myBuffer[sdWriteSize]; // Create our own buffer to hold the data while we write it to SD card
      uint16_t bytesToWrite = remainingBytes; // Write the remaining bytes to SD card sdWriteSize bytes at a time
      if (bytesToWrite > sdWriteSize)
      {
        bytesToWrite = sdWriteSize;
      }
  
      myGNSS.extractFileBufferData((uint8_t *)&myBuffer, bytesToWrite); // Extract bytesToWrite bytes from the UBX file buffer and put them into myBuffer
      OpenLog.write(myBuffer, bytesToWrite); // Write bytesToWrite bytes from myBuffer to the ubxDataFile on the SD card
      
      bytesWritten += bytesToWrite; // Update bytesWritten
      remainingBytes -= bytesToWrite; // Decrement remainingBytes
    }
    digitalWrite(LED_BUILTIN, LOW); // Turn LED_BUILTIN off

    Serial.print(F("The total number of bytes written to SD card is: ")); // Print how many bytes have been written to SD card
    Serial.println(bytesWritten);

    uint16_t maxBufferBytes = myGNSS.getMaxFileBufferAvail(); // Show how full the file buffer has been (not how full it is now)
    Serial.print(F("The maximum number of bytes which the file buffer has contained is: "));
    Serial.println(maxBufferBytes);
  
  
	// output file string
	char filesstring[50];
	sprintf(filesstring, " Files: %s, %s",headerFileName, dataFileName);
	strcat(txString,filesstring);
	Serial.println("Turning datalogging OFF!");
	Serial.println(filesstring);
	LED_blink(25, 4);


	gotoCommandMode(); //Puts OpenLog in command mode.
	OpenLog.print("append ");
	OpenLog.print(headerFileName);
	OpenLog.write(13); //This is 

	// while (1) {
	// if (OpenLog.available())
	  // if (OpenLog.read() == '>') break;
	// }
	Serial.println("Ready to append");  
	LED_blink(25, 4);
	readDateTime();
	Write_stop();
	OpenLog.print(F("The total number of bytes written to SD card is: ")); // Print how many bytes have been written to SD card
	OpenLog.println(bytesWritten);
	OpenLog.print(F("The maximum number of bytes which the file buffer has contained is: "));
	OpenLog.println(maxBufferBytes);
	Serial.println("Measurement stopped successfully");  
}



// Restart data logging process
void restart_logging() {
  logging = false;
  BLE_message=true;
  strcpy(txString,"Turning ON datalogging with new files!");
  Serial.println(txString);
  LED_blink(100, 5);

  // //Reset OpenLog
  // digitalWrite(resetOpenLog, LOW);
  // delay(5);
  // digitalWrite(resetOpenLog, HIGH);

  // //Wait for OpenLog to respond with '<' to indicate it is alive and recording to a file
  // while (1) {
    // if (OpenLog.available())
      // if (OpenLog.read() == '<') break;
  // }
  // Serial.println("OpenLog resete and back online");
  // LED_blink(100, 5);
  makeFiles();
  delay(500);
  logging = true;
  setupGNSS();
  myGNSS.clearFileBuffer();
  
}


// set new GNSS NavigationRate
void setRate( String rxValue){  
   if (!logging) {
    int index = rxValue.indexOf(":");\
    int index2 = rxValue.indexOf(":",index+1);
    if (index !=-1 and index2 !=-1){
//      Serial.println(rxValue);
//      sprintf(txString,"index1: %d, index2: %d",index,index2);
//      Serial.println(txString);
//      Serial.println(rxValue.substring(index+1,index2));
//      Serial.println(rxValue.substring(index+1,index2).toInt());
      NavigationFrequency = rxValue.substring(index+1,index2).toInt();
      BLE_message=true;
      sprintf(txString,"New navigation frequency: %d",NavigationFrequency);
      Serial.println(txString);
//      myGNSS.setNavigationFrequency(NavigationFrequency); //Produce  navigation solution at given frequency
    } else {
      BLE_message=true;
      sprintf(txString,"New frequency can not be parsed form string '%s'. Valid format is 'RATE:2:'",txString);
      Serial.println(txString);
    }
  } else {
    BLE_message=true;
    strcpy(txString,"Datalogging running. Can't change data frequency now. First stop measurment!");
    Serial.println(txString);
  }
}



// change Dynamic Model 
void setDynamicModel( String rxValue){  
   if (!logging) {
    int index = rxValue.indexOf(":");\
    int index2 = rxValue.indexOf(":",index+1);
    if (index !=-1 and index2 !=-1){

      DynamicModel = (dynModel)rxValue.substring(index+1,index2).toInt();
      BLE_message=true;
      sprintf(txString,"New dynamic navigation model: %d",DynamicModel);
      Serial.println(txString);
      IMUcalibration=true;      //New calibration is necessary after changing dynamic model
//      myGNSS.setDynamicModel(DynamicModel); //Set new Dynamic Model for GNSS solution.
    } else {
      BLE_message=true;
      sprintf(txString,"Dynamic model can not be parsed form string '%s'. Valid format is 'DYNMODEL:4:'",txString);
      Serial.println(txString);
    }
  } else {
    BLE_message=true;
    strcpy(txString,"Datalogging running. Can't change dynamic navigation plattform now. First stop measurment!");
    Serial.println(txString);
  }
}


// change RMX log setting
void setLogRMX( String rxValue){  
   if (!logging) {
    int index = rxValue.indexOf(":");\
    int index2 = rxValue.indexOf(":",index+1);
    if (index !=-1 and index2 !=-1){
      log_RMX = (rxValue.substring(index+1,index2).toInt() > 0);
      BLE_message=true;
      sprintf(txString,"New RMX log setting is: %d",log_RMX );
      Serial.println(txString);
      
    } else {
      BLE_message=true;
      sprintf(txString,"RMX log setting can not be parsed form string '%s'. Valid format is 'LOGRMX:0:'",txString);
      Serial.println(txString);
    }
  } else {
    BLE_message=true;
    strcpy(txString,"Datalogging running. Can't change RMX log setting now. First stop measurment!");
    Serial.println(txString);
  }
}


// change bool setting
void setLogFlag( String rxValue, bool &flag, String flagname){  
   if (!logging) {
    int index = rxValue.indexOf(":");\
    int index2 = rxValue.indexOf(":",index+1);
    if (index !=-1 and index2 !=-1){
      flag = (rxValue.substring(index+1,index2).toInt() > 0);
      BLE_message=true;
      sprintf(txString,"New %s log setting is: %d",flagname,flag );
      Serial.println(txString);
      
    } else {
      BLE_message=true;
      sprintf(txString,"%s log setting can not be parsed from string '%s'. Valid format is 'FLAGNAME:0:'",flagname,txString);
      Serial.println(txString);
    }
  } else {
    BLE_message=true;
    strcpy(txString,"Datalogging running. Can't change flag log setting now. First stop measurment!");
    Serial.println(txString);
  }
}


// change setting for regular check for IMU calibration
void  setIMUcal( String rxValue){  
   if (!logging) {
    int index = rxValue.indexOf(":");\
    int index2 = rxValue.indexOf(":",index+1);
    if (index !=-1 and index2 !=-1){
      IMUcalibration = (rxValue.substring(index+1,index2).toInt() > 0);
      BLE_message=true;
      sprintf(txString,"New IMUcalibration setting is: %d",IMUcalibration );
      Serial.println(txString);
      
    } else {
      BLE_message=true;
      sprintf(txString,"IMUcalibration setting can not be parsed form string '%s'. Valid format is 'IMUCAL:0:'",txString);
      Serial.println(txString);
    }
  } else {
    BLE_message=true;
    strcpy(txString,"Datalogging running. Can't change IMUcalibration setting now. First stop measurment!");
    Serial.println(txString);
  }
}


// Get IMU data and send them over BLE+serial for manual check
void  check_IMU(){  
   Send_tx_String(txString);
   strcpy(txString,"");
   if (!logging) {
		myGNSS.getESFSTATUS();  // call for ESF-STATUS message 
		strcat(txString,"IMU check \n# --------------------------------------\n");  
		strcat(txString,"Fusion Mode: ");  
		sprintf(subString,"%d",myGNSS.packetUBXESFSTATUS->data.fusionMode);  
		strcat(txString,subString); 
		
		strcat(txString,"\nIMU sensors status: "); // See interface description for explanation of values.
		for (uint16_t i = 0; i <7 ; i++)
		{
		sprintf(subString,"\nSensor %d:",i );  
        strcat(txString,subString); 
		int st1 = myGNSS.packetUBXESFSTATUS->data.status[i].sensStatus1.all;
        int st2 = myGNSS.packetUBXESFSTATUS->data.status[i].sensStatus2.all;
        sprintf(subString,"type %d, ",st1/4);
        strcat(txString,subString); 
        sprintf(subString,"used %d,",st1%4/2);
        strcat(txString,subString); 
        sprintf(subString,"ready %d,",st1%2);
        strcat(txString,subString); 
        sprintf(subString,"calsts %d,",st2/4);
        strcat(txString,subString); 
        sprintf(subString,"timests %d,",st2%4);
        strcat(txString,subString); 
			  //sprintf(subString,"\nSensor %d type %d, used %d ready %d calsts %d timests %d",i, st1/4, st1%4/2, st1%2, st2/4, st2%4);   
        //        Serial.print(st1);
        //        Serial.println(st2); 
			}
  } else {
    strcpy(txString,"Datalogging running. Can't run IMU _check now!");
  }
  Send_tx_String(txString);
}



// /////////////////////////////////////////////
// ---------------------------------------------
// BLE funtions und code
// ---------------------------------------------
// /////////////////////////////////////////////

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("Received Value: ");
        for (int i = 0; i < rxValue.length(); i++)
          Serial.print(rxValue[i]);
        Serial.println();

        //Start new data files if START is received and stop current data files if STOP is received. RATE and NAVMODE ar other options
        if (rxValue.find("START") != -1) { 
           BLE_start=true;
        } else if (rxValue.find("STOP") != -1) {
           BLE_stop=true;
		} else if (rxValue.find("CHECKIMU") != -1) {
		    check_IMU();
        } else if (rxValue.find("RATE") != -1) {
           char passValue[40];
           for (int i = 0; i < rxValue.length(); i++){
               passValue[i] = rxValue[i];}
               setRate(passValue);
        } else if (rxValue.find("DYNMODEL") != -1) {
           char passValue[40];
           for (int i = 0; i < rxValue.length(); i++){
               passValue[i] = rxValue[i];}
           setDynamicModel(passValue);
		} else if (rxValue.find("LOGRMX") != -1) {
           char passValue[40];
		   for (int i = 0; i < rxValue.length(); i++){
               passValue[i] = rxValue[i];}
           setLogRMX(passValue);
        } else if (rxValue.find("IMUCAL") != -1) {
           char passValue[40];
		   for (int i = 0; i < rxValue.length(); i++){
               passValue[i] = rxValue[i];}
           setIMUcal(passValue);
		} else if (rxValue.find("NAVPVT") != -1) {
           char passValue[40];
		   for (int i = 0; i < rxValue.length(); i++){
               passValue[i] = rxValue[i];}
           setLogFlag( passValue, log_NAVPVT, "NAVPVT");
		} else if (rxValue.find("NAVATT") != -1) {
           char passValue[40];
		   for (int i = 0; i < rxValue.length(); i++){
               passValue[i] = rxValue[i];}
           setLogFlag( passValue, log_NAVATT, "NAVATT");
		} else if (rxValue.find("NAVPVAT") != -1) {
           char passValue[40];
		   for (int i = 0; i < rxValue.length(); i++){
               passValue[i] = rxValue[i];}
           setLogFlag( passValue, log_NAVPVAT, "NAVPVAT");
		} else if (rxValue.find("ESFINS") != -1) {
           char passValue[40];
		   for (int i = 0; i < rxValue.length(); i++){
               passValue[i] = rxValue[i];}
           setLogFlag( passValue, log_ESFINS, "ESFINS");
		} else if (rxValue.find("ESFRAW") != -1) {
           char passValue[40];
		   for (int i = 0; i < rxValue.length(); i++){
               passValue[i] = rxValue[i];}
           setLogFlag( passValue, log_ESFRAW, "ESFRAW");
		} else if (rxValue.find("ESFMEAS") != -1) {
           char passValue[40];
		   for (int i = 0; i < rxValue.length(); i++){
               passValue[i] = rxValue[i];}
           setLogFlag( passValue, log_ESFMEAS, "ESFMEAS");
		} else if (rxValue.find("ESFALG") != -1) {
           char passValue[40];
		   for (int i = 0; i < rxValue.length(); i++){
               passValue[i] = rxValue[i];}
           setLogFlag( passValue, log_ESFALG, "ESFALG");
		}else{
          BLE_message=true;
          strcpy(txString,"Input can not be parsed retry!");
          Serial.println(txString);
        }
        Serial.println("*********");
      }
    }
};



void setup_BLE() {
  Serial.println("bluetooth connection: 'GNSS BLE UART' ");
  // Create the BLE Device
  BLEDevice::init("GNSS BLE UART");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
                    CHARACTERISTIC_UUID_TX,
                    BLECharacteristic::PROPERTY_NOTIFY
                  );
                      
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
                       CHARACTERISTIC_UUID_RX,
                      BLECharacteristic::PROPERTY_WRITE
                    );

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}


// Send txSTring to BLE and Serial
void Send_tx_String(char *txString){
    // strcat(txString,"\n");
    Serial.print(txString);

    if (deviceConnected) {
      pTxCharacteristic->setValue(txString);
      pTxCharacteristic->notify();
      BLE_message=false;
    }

    strcpy(txString,"");
  }






// /////////////////////////////////////////////
// -%--------------------------------------------
// Setup and loop
// %%------------------------------------------------------------------------------------------------------------------------------
void setup(){
    pinMode(statLED, OUTPUT);
    digitalWrite(statLED, HIGH);
    delay(1000);
    digitalWrite(statLED, LOW);

    pinMode(resetOpenLog, OUTPUT);
    
    Serial.begin(115200);
    while (!Serial) {
      ; //Wait for user to open terminal
      LED_blink(100, 4);
      delay(1000);
    }  
    Serial.println(F("Loggin GPS data to OpenLog"));
    
    // Setup GPS connection
    Serial.println("Connecting to GPS");
    Wire.begin();
  
  
    myGNSS.setFileBufferSize(fileBufferSize); // setFileBufferSize must be called _before_ .begin
   
    if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
    {
      Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
      while (1){
        LED_blink(200, 2);
        delay(2000);
        if (myGNSS.begin() == true) { //Connect to the u-blox module using Wire port
          Serial.println(F("u-blox GNSS detected I2C address. Reconnection was successfull."));
          break;
        }
      }
    }
    
    myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
	
  //myGNSS.enableDebugging(); // Uncomment this line to enable lots of helpful GNSS debug messages on Serial
  //myGNSS.enableDebugging(Serial, true); // Or, uncomment this line to enable only the important GNSS debug messages on Serial
	
    Serial.println(F("Connection to GPS succesful"));

    setup_BLE();

    //Setup OpenLog for saving data on SD
    setupOpenLog(); //Resets logger and waits for the '<' I'm alive character
    Serial.println(F("Connection to OpenLog succesful!"));
    //Write header file and create data file ready for writing binary data
    if (logging) {  
      makeFiles();
      delay(1000);
    } else {
      BLE_message=true;
      strcpy(txString,"Waiting for command 'START' over Serial of BLE for starting logging data!");
      Serial.println(txString);
    }

    // setting up GPS for automatic messages
  	setupGNSS();
  	delay(200);
	
	// scall for GPS status messages
	if (log_STATUS ){   
		myGNSS.getESFSTATUS();
		myGNSS.getESFALG();
	}
	
   //Check fusion mode
   if (IMUcalibration){   
		checkIMUcalibration();
		Serial.print(txString);
   }
	
   Serial.println(F("Setup completeded."));
   lastPrint = millis(); // Initialize lastPrint
}


void loop(){
  //################################# Data logging ############################################# 
  if (logging){    // if logging is avctive check for GNSS data and save them to SD card in binary file
    myGNSS.checkUblox(); // Check for the arrival of new data and process it.
    // myGNSS.checkCallbacks(); // Check if any callbacks are waiting to be processed.

    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

    while (myGNSS.fileBufferAvailable() >= sdWriteSize) // Check to see if we have at least sdWriteSize waiting in the buffer
    {
      digitalWrite(LED_BUILTIN, HIGH); // Flash LED_BUILTIN each time we write to the SD card

      uint8_t myBuffer[sdWriteSize]; // Create our own buffer to hold the data while we write it to SD card

      myGNSS.extractFileBufferData((uint8_t *)&myBuffer, sdWriteSize); // Extract exactly sdWriteSize bytes from the UBX file buffer and put them into myBuffer

      OpenLog.write(myBuffer, sdWriteSize); // Write exactly sdWriteSize bytes from myBuffer to the ubxDataFile on the SD card

      bytesWritten += sdWriteSize; // Update bytesWritten

      // In case the SD writing is slow or there is a lot of data to write, keep checking for the arrival of new data
      myGNSS.checkUblox(); // Check for the arrival of new data and process it.

      digitalWrite(LED_BUILTIN, LOW); // Turn LED_BUILTIN off again
      Serial.print(".");
	    delay(20);
    }

    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

    if (millis() > (lastPrint + 1000)) // Print bytesWritten once per second
    {
//      Serial.print(F("The number of bytes written to SD card is: ")); // Print how many bytes have been written to SD card
//      Serial.println(bytesWritten);
//
//      uint16_t maxBufferBytes = myGNSS.getMaxFileBufferAvail(); // Get how full the file buffer has been (not how full it is now)
//
//      //Serial.print(F("The maximum number of bytes which the file buffer has contained is: ")); // It is a fun thing to watch how full the buffer gets
//      //Serial.println(maxBufferBytes);
//
//      if (maxBufferBytes > ((fileBufferSize / 5) * 4)) // Warn the user if fileBufferSize was more than 80% full
//      {
//         Serial.println(F("Warning: the file buffer has been over 80% full. Some data may have been lost."));
//      }

      Serial.println(" ");
      BLE_message=true;
      strcpy(txString,".");
      lastPrint = millis(); // Update lastPrint
    }

    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  }
  

  
  
  //################################# Do other stuff #############################################
  
    // Check BLE connection
  if (deviceConnected && BLE_message ) {
    pTxCharacteristic->setValue(txString);
    pTxCharacteristic->notify();
    BLE_message=false;
    //delay(10); // bluetooth stack will go into congestion, if too many packets are sent
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    //delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }
  
  
  if (BLE_stop){
    stop_logging();
    BLE_stop= false;
  } else if (BLE_start){
    restart_logging();
    BLE_start= false;
  }
    
  
  if (Serial.available()){ // Check Serial inputs
    String rxValue = Serial.readString();
    if (rxValue.length() > 0) {
      Serial.println("*********");
      Serial.print("Received Value: ");
      for (int i = 0; i < rxValue.length(); i++)
        Serial.print(rxValue[i]);
      Serial.println();

      //Start new data files if START is received and stop current data files if STOP is received
      if (rxValue.indexOf("START") != -1) { 
        restart_logging();
      } else if (rxValue.indexOf("STOP") != -1) {
        stop_logging();
      } else if (rxValue.indexOf("RATE") != -1) {
        setRate(rxValue);
      }else{
        BLE_message=true;
        strcpy(txString,"Input can not be parsed retry!");
        Serial.println(txString);
      }
      Serial.println("*********");
      }
    
  }  
  
    //Check fusion mode
  if (IMUcalibration  and (millis() - lastTime2 > 10000)){   
    checkIMUcalibration();
    Send_tx_String(txString);
    lastTime2 = millis(); //Update the timer
  }
	
  if (log_STATUS  and (millis() - lastTime_logstatus > 60000)){   
    myGNSS.getESFSTATUS();
	myGNSS.getESFALG();
    lastTime_logstatus = millis(); //Update the timer
  }


  delay(5);
}
