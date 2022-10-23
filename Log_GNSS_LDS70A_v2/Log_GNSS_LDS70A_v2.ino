/*
  Getting data from u-blox GNSS module and from LDS70A Laser altimeter and saving them to SD card.
  By: AcCapelli
  Date: February 25th, 2022
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  Based on Examples from used libraries and hardware.

  Hardware Connections:
  
  Plug a Qwiic cable into the GNSS and a BlackBoard
  Open the serial monitor at 115200 baud to see the output
  
  Using Hardwareserial for altimeter. Connect pins  16 to R1_out and 17 to T1_in of ADM3222 (transreceiver TTL to RS232)

  Connect the SD card to the following pins (using V_SPI):
   * SD Card | ESP32
   *    CS       CS0   GPIO5
   *    CMD      MOSI  GPIO32  ! GPIO 23 is used by I2C port to GNSS! Use other PIN
   *    VDD      3.3V
   *    CLK      SCK   GPIO18
   *    GND      GND
   *    D0       MISO  GPIO19
  
  BLE:
  Set up BLE connections with UART communication
  Start new data files if "START" is received and stop current data files if "STOP" is received. See below for more BLE commands.
  
*/

#include <Wire.h> //Needed for I2C to GNSS

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> 
SFE_UBLOX_GNSS myGNSS;

#include "FS.h"
#include "SD.h"
#include "SPI.h"


//HardwareSerial
#include <HardwareSerial.h>
HardwareSerial RS232(2);

#define version "GNSS+Altimeter v1.1"

long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.
long lastTime2 = 0; //Second simple local timer. 
long lastTime_logstatus = 0; //Second simple local timer. 
long logTime_laser;
long logTime_laser2;

int statLED = 13;

// settings SD
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
#define SCK  18
#define MISO  19
#define MOSI  32
#define CS  5
#define SPI_rate 80000000
#define CD_pin 27         // chip detect pin is shorted to GND if a card is inserted. (Otherwise pulled up by 10kOhm)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

#define sdWriteSize 512 // Write data to the SD card in blocks of 512 bytes
#define sdWriteSize_Laser 500 // Write data to the SD card in blocks of almost 512 bytes
uint8_t *myBuffer; // A buffer to hold the GNSS data while we write it to SD car
uint8_t *myBuffer_laser; // A buffer to hold the Laser data while we write it to SD car

SPIClass spi = SPIClass(VSPI);

char headerFileName[24]; //Max file name length is 23 characters)
char dataFileName[24]; //Max file name length is 23 characters)
File dataFile; //File that all data is written to
File headerFile; //File containing a header with settings description



// settings altimeter LSD70A
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
//PINs can be changed to any pin. // For Hardware Serial use Pin 16 an 17. SoftwareSerial worked on pins 12 and 27
int PIN_Rx = 16; // 16 = Hardware RX pin,
int PIN_Tx = 17; // 17 = Hardware TX pin,
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#define baudrateRS232 115200 

#define Laser_BufferSize 1024 // Allocate 1024 Bytes of RAM for UART serial storage
#define flush_intervall 30000
#define Laser_log_intervall 2000
int bitesToWrite;


// Setting for u-blox 
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
bool logging = false;
bool log_GPS = true;

bool IMUcalibration=false;
int NavigationFrequency = 5;
dynModel DynamicModel = (dynModel)4;
bool log_RMX = false;
bool log_ESFRAW  = false;
bool log_ESFMEAS = false;
bool log_ESFALG = false;
bool log_NAVPVAT = true;
bool log_NAVPVT = false;
bool log_NAVATT = false;
bool log_ESFINS = false;
bool log_STATUS = true;
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

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
// ---------------------------------------------
//GNSS code
// ---------------------------------------------
// /////////////////////////////////////////////



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

void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.path(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

void createDir(fs::FS &fs, const char * path){
    Serial.printf("Creating Dir: %s\n", path);
    if(fs.mkdir(path)){
        Serial.println("Dir created");
    } else {
        Serial.println("mkdir failed");
    }
}

void removeDir(fs::FS &fs, const char * path){
    Serial.printf("Removing Dir: %s\n", path);
    if(fs.rmdir(path)){
        Serial.println("Dir removed");
    } else {
        Serial.println("rmdir failed");
    }
}

void readFile(fs::FS &fs, const char * path){
    Serial.printf("## Reading file: %s\n", path);

    File file = fs.open(path);
    if(!file){
        Serial.println("Failed to open file for reading");
        return;
    }

//    Serial.println("Read from file: ");
    while(file.available()){
        Serial.write(file.read());
    }
   file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("File written");
    } else {
        Serial.println("Write failed");
    }
    file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("Failed to open file for appending");
        return;
    }
    if(file.print(message)){
        Serial.println("Message appended");
    } else {
        Serial.println("Append failed");
    }
    file.close();
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
    Serial.printf("Renaming file %s to %s\n", path1, path2);
    if (fs.rename(path1, path2)) {
        Serial.println("File renamed");
    } else {
        Serial.println("Rename failed");
    }
}

void deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s\n", path);
    if(fs.remove(path)){
        Serial.println("File deleted");
    } else {
        Serial.println("Delete failed");
    }
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
  headerFile.println(F("# GNSS and Laser altimeter log file"));
  headerFile.print("# Date: ");
  headerFile.print(Year);
  headerFile.print("-");
  headerFile.print(Month);
  headerFile.print("-");
  headerFile.println(Day);
  headerFile.print("# Time: ");
  headerFile.print(Hour);
  headerFile.print(":");
  headerFile.print(Minute);
  headerFile.print(":");
  headerFile.println(Second);
  headerFile.println(F("# --------------------------------------"));
  headerFile.print(F("# UBX binary data are in  file: "));
  headerFile.print(dataFileName);
  headerFile.println(F(" "));
}



void  log_settings() {  
  headerFile.println(F("# --------------------------------------"));
  headerFile.print(F("Script version: "));
  headerFile.println(version);
  headerFile.println(F("Measurements settings: "));
  headerFile.print(F("IMU calibration: "));
  headerFile.println(IMUcalibration);
  headerFile.print(F("NavigationFrequency: "));
  headerFile.println(NavigationFrequency);
  headerFile.print(F("DynamicModel: "));
  headerFile.println(DynamicModel);
  headerFile.print(F("log_NAVATT: "));
  headerFile.println(log_NAVATT);
  headerFile.print(F("log_NAVPVT: "));
  headerFile.println(log_NAVPVT);
  headerFile.print(F("log_NAVPVAT: "));
  headerFile.println(log_NAVPVAT);
  headerFile.print(F("log_ESFINS: "));
  headerFile.println(log_ESFINS);
  headerFile.print(F("log_ESFRAW: "));
  headerFile.println(log_ESFRAW);
  headerFile.print(F("log_ESFMEAS: "));
  headerFile.println(log_ESFMEAS);
  headerFile.print(F("log_ESFALG: "));
  headerFile.println(log_ESFALG);
  headerFile.print(F("log_RMX : "));
  headerFile.println(log_RMX );
  headerFile.println(F("# --------------------------------------"));
}


void  log_Laser_settings() {  
  headerFile.println(F(" "));
  headerFile.println(F("# --------------------------------------"));
  headerFile.println(F("Laser settings: "));
  headerFile.println(F("# --------------------------------------"));
  RS232.write(0x1B);
  delay(100);
  while(RS232.read() >= 0) ; // flush the receive buffer.
  RS232.write("PA");   
  RS232.write(0x0D);
  uint8_t *Buffer_lasersetting = new uint8_t[3000];
  unsigned long startTime  = millis();
  while(millis() < (startTime + 500)){
    bitesToWrite=RS232.available();
    RS232.readBytes(Buffer_lasersetting, bitesToWrite);
    headerFile.write( Buffer_lasersetting, bitesToWrite);
//    Serial.write( Buffer_lasersetting, bitesToWrite);
    delay(1);
  }
  delete[] Buffer_lasersetting;
//  while ( RS232.available()){ 
//	headerFile.print(RS232.read());
//  } 
  headerFile.println(F("# --------------------------------------"));
  headerFile.println(F(" "));
}

// Make new files in SD card. file names are derived from GNSS time if available. 
void  makeFiles(fs::FS &fs) {  
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
    }else if (count > 3) {
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
    LED_blink(1000, 3);
  }
  
  sprintf(headerFileName, "/a%02d%02d%02d_%02d%02d.txt", Year%1000 ,Month ,Day ,Hour,Minute);  // create name of header file
  sprintf(dataFileName, "/a%02d%02d%02d_%02d%02d.ubx", Year%1000 ,Month ,Day ,Hour,Minute);   // create name of data file
//  strcpy(headerFileName, "/data.txt");  // create name of header file
//  strcpy(dataFileName, "/data.ubx");   // create name of data file
  Serial.println(headerFileName);
  Serial.println(dataFileName);
  
  

  Serial.print("Making header file.");
  headerFile = fs.open(headerFileName, FILE_WRITE);
  if(!headerFile){
    Serial.println(F("Failed to create header file! Freezing..."));
    Serial.println(headerFileName);
    while (1);
  }
  
  Serial.print(F("created file: "));
  Serial.println(headerFileName);
  Write_header();
  delay(100);
  log_settings();
  delay(100);
  log_Laser_settings();
  delay(100);
  headerFile.close(); // Close the data file

  Serial.print("Making data file:");
  Serial.println(dataFileName);
  
  dataFile=fs.open(dataFileName, FILE_WRITE);
  // dataFile = SD.open("/data2.ubx", FILE_WRITE);
  if(!dataFile){
    Serial.println(F("Failed to create header file! Freezing..."));
    Serial.println(dataFileName);
    while (1);
  }
  strcpy(txString,"");
  strcat(txString,"Created file: ");
  strcat(txString,dataFileName);  
  Send_tx_String(txString);
  LED_blink(100,10);
}


void  Write_stop() {  
  headerFile.print(F("Measurements stopped on: "));
  headerFile.print(Year);
  headerFile.print("-");
  headerFile.print(Month);
  headerFile.print("-");
  headerFile.print(Day);
  headerFile.print(", ");
  headerFile.print(Hour);
  headerFile.print(":");
  headerFile.print(Minute);
  headerFile.print(":");
  headerFile.println(Second);
  headerFile.println(F("# --------------------------------------"));
}



// setting up GPS 
void setupGNSS(){ 
          
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
    else {
      // myGNSS.setAutoPVTcallback(&printPVTdata); // Enable automatic NAV PVT messages with callback to printPVTdata
      myGNSS.setAutoNAVPVAT(false, false); // Enable automatic NAV PVT messages without callback to printPVTdata
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
    
	  myGNSS.setNavigationFrequency(NavigationFrequency); //Produce  navigation solution at given frequency
   
    Serial.println(F("GPS setting updated"));
    strcat(txString,"GPS setting updated");
}





// /////////////////////////////////////////////
// ---------------------------------------------
// Input parsing functions
// ---------------------------------------------
// /////////////////////////////////////////////

// Stop data logging process
void stop_logging(fs::FS &fs) {
	logging = false; // Set flag to false
	BLE_message=true;
	strcpy(txString,"Turning datalogging OFF!");

	if (log_RMX ) {
	  myGNSS.setAutoRXMSFRBX(false, false); // Disable the automatic RXM SFRBX messages
	  myGNSS.setAutoRXMRAWX(false, false); // Disable the automatic RXM RAWX messages
	}
	if (log_NAVPVT) {
	  myGNSS.setAutoPVT(false, false); // Disable  automatic NAV PVT messages without callback to printPVTdata
	  }
	if (log_NAVPVAT) {
	  myGNSS.setAutoNAVPVAT(false, false); // Disable  automatic NAV PVAT messages without callback to printPVTdata
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
  
  while (remainingBytes > 0 and log_GPS) // While there is still data in the file buffer
  {
    digitalWrite(LED_BUILTIN, HIGH); // Flash LED_BUILTIN while we write to the SD card
    uint8_t myBuffer[sdWriteSize]; // Create our own buffer to hold the data while we write it to SD card
    uint16_t bytesToWrite = remainingBytes; // Write the remaining bytes to SD card sdWriteSize bytes at a time
    if (bytesToWrite > sdWriteSize)
    {
      bytesToWrite = sdWriteSize;
    }

    myGNSS.extractFileBufferData((uint8_t *)&myBuffer, bytesToWrite); // Extract bytesToWrite bytes from the UBX file buffer and put them into myBuffer
    dataFile.write(myBuffer, bytesToWrite); // Write bytesToWrite bytes from myBuffer to the ubxDataFile on the SD card
    
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
    dataFile.close(); // Close the data file
	Serial.println("Datafile closed.");
	
	headerFile = fs.open(headerFileName, FILE_APPEND);
	Serial.println("Ready to append");  
	LED_blink(25, 4);
	readDateTime();
	Write_stop();
	headerFile.print(F("The total number of bytes written to SD card is: ")); // Print how many bytes have been written to SD card
	headerFile.println(bytesWritten);
	headerFile.print(F("The maximum number of bytes which the file buffer has contained is: "));
	headerFile.println(maxBufferBytes);
    headerFile.close(); 
    Serial.println("Header file closed.");
//    listDir(SD, "/", 0);
	Serial.println("Measurement stopped successfully"); 
  
}



// Restart data logging process
void restart_logging() {
  logging = false;
  BLE_message=true;
  strcpy(txString,"Turning ON datalogging with new files!");
  Serial.println(txString);
  LED_blink(100, 5);

  makeFiles(SD);
  delay(500);
  logging = true;
  setupGNSS();
  myGNSS.clearFileBuffer();  // flush the receive buffer.
  while(RS232.read() >= 0) ; // flush the receive buffer.
  RS232.write("DT");
  RS232.write(0x0D);
  
  lastPrint = millis(); // Initialize lastPrint
  logTime_laser  = millis(); // logTime_laser
  
  //call for GPS status messages
  if (log_STATUS ){   
    myGNSS.getESFSTATUS();
    myGNSS.getESFALG();
  }

  
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
      myGNSS.setNavigationFrequency(NavigationFrequency); //Produce  navigation solution at given frequency
    } else {
      BLE_message=true;
      sprintf(txString,"New frequency can not be parsed form string '%s'. Valid format is 'RATE:2:'",rxValue);
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
      sprintf(txString,"Dynamic model can not be parsed form string '%s'. Valid format is 'DYNMODEL:4:'",rxValue);
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
      sprintf(txString,"RMX log setting can not be parsed form string '%s'. Valid format is 'LOGRMX:0:'",rxValue);
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
      sprintf(txString,"%s log setting can not be parsed from string '%s'. Valid format is 'FLAGNAME:0:'",flagname,rxValue);
      Serial.println(txString);
    }
  } else {
    BLE_message=true;
    strcpy(txString,"Datalogging running. Can't change flag log setting now. First stop measurment!");
    Serial.println(txString);
  }
  Send_tx_String(txString);
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
      sprintf(txString,"IMUcalibration setting can not be parsed form string '%s'. Valid format is 'IMUCAL:0:'",rxValue);
      Serial.println(txString);
    }
  } else {
    BLE_message=true;
    strcpy(txString,"Datalogging running. Can't change IMUcalibration setting now. First stop measurment!");
    Serial.println(txString);
  }
   Send_tx_String(txString);
}


void readFiles( String rxValue){  
    /*
    Read last files (haderfile and datafile) if no argument is passed  ("READ" ) otherwise read passed files (READ:<<filepath>>:).
    */
   if (!logging) {
//    Serial.println(rxValue);
    int index = rxValue.indexOf(":");\
    int index2 = rxValue.indexOf(":",index+1);
//    Serial.println(index );
//    Serial.println(index2);
    
    if (index !=-1 and index2 !=-1){
      char path[32];
      rxValue.substring(index+1,index2).toCharArray(path,32);
      Serial.println("");
//      Serial.print("%% Reading file:");
//      Serial.println(path);
      readFile(SD,path);
      Serial.println(" \n## End of file");
      
    } else if (index =-1){
      Serial.print("Reading file:");
      Serial.println(headerFileName);
      readFile(SD,headerFileName);
      Serial.print("Reading file:");
      Serial.println(dataFileName);
      readFile(SD,dataFileName);
      
    } else {
      BLE_message=true;
      sprintf(txString,"File can not be parsed form string '%s'. Valid format is 'READ' or 'READ:<<filepath>>:' !",rxValue);
      Serial.println(txString);
    }
  } else {
    BLE_message=true;
    strcpy(txString,"Datalogging running. Can't files now. First stop measurment!");
    Serial.println(txString);
  }
   Send_tx_String(txString);
}

void getFileList(){  
    /*
    Return list of files in SD card
    */
   if (!logging) {
    Serial.println("");
    Serial.println("%% File list:");
    listDir(SD, "/", 0);
    Serial.println("%% end"); 
      
  } else {
    Serial.println("Datalogging running. Can't read file list now. First stop measurment!");
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


// Get attitude angles from IMU anf GPS coordinates and send them over BLE+serial for manual check
void  check_attitude(){  
   Send_tx_String(txString);
   strcpy(txString,"");
   if (!logging) {
      myGNSS.getNAVPVAT();  // call for NAVPVAT message 
      delay(500);
//      Serial.println("got PVAT");
      strcat(txString,"GNSS and IMU check \n# --------------------------------------\n");  
      strcat(txString,"Roll angle (deg): "); 
      sprintf(subString,"%.3f",(float)(myGNSS.packetUBXNAVPVAT->data.vehRoll)/100000);  
      strcat(txString,subString); 
      strcat(txString,"\nPitch angle (deg): ");  
      sprintf(subString,"%.3f",(float)(myGNSS.packetUBXNAVPVAT->data.vehPitch)/100000);  
      strcat(txString,subString); 
      strcat(txString,"\nHeading angle (deg): ");  
      sprintf(subString,"%.3f",(float)(myGNSS.packetUBXNAVPVAT->data.vehHeading)/100000);  
//      Serial.println("ATT saved");
      strcat(txString,subString); 
      strcat(txString,"\nLatitude (deg): ");  
      sprintf(subString,"%.3f",(float)(myGNSS.packetUBXNAVPVAT->data.lat)/10000000);  
      strcat(txString,subString); 
      strcat(txString,"\nLongitude (deg): ");
      sprintf(subString,"%.3f",(float)(myGNSS.packetUBXNAVPVAT->data.lon)/10000000);  
      strcat(txString,subString); 
      strcat(txString,"\nElevation (m a.s.l.): ");  
      sprintf(subString,"%.2f",(float)(myGNSS.packetUBXNAVPVAT->data.hMSL)/1000);  
      strcat(txString,subString); 
      strcat(txString," \n# --------------------------------------\n"); 
//      Serial.println("GNSS saved");
  } else {
    strcpy(txString,"Datalogging running. Can't run IMU _check now!");
  }
  Send_tx_String(txString);
}


// Get Laser data for 1 second and send it over BLE and serial
void  check_laser(){  
   Send_tx_String(txString);
   strcpy(txString,"");
   if (!logging) {
      strcat(txString,"\nLaser data \n# --------------------------------------\n");  
      Send_tx_String(txString);
      lastTime=millis();
      while(RS232.read() >= 0) ; // flush the receive buffer.
      while(millis()-lastTime < 1000);
      int availableBytes = RS232.available();
      RS232.readBytes(myBuffer_laser, availableBytes);
      Serial.write( myBuffer_laser, availableBytes);
      if (deviceConnected) {
        pTxCharacteristic->setValue(myBuffer_laser, availableBytes);
        pTxCharacteristic->notify();
        BLE_message=false;
      }
      strcpy(txString,"");
      strcat(txString,"# --------------------------------------\n");  
      Send_tx_String(txString);
  } else {
    strcpy(txString,"Datalogging running. Can't run IMU _check now!");
  }
  Send_tx_String(txString);
}

// Print commands
void  commands(){  
   Send_tx_String(txString);
   strcpy(txString,"");
   strcat(txString,"\nCommands \n# --------------------------------------\n");
   strcat(txString,"\n Serial only\n# .................\n");
   strcat(txString,"READ Read last file of last measurement. If argument is passed with READ:<< nfile>>: read file nfile \nLIST  Get list of files in SD card \n ");

   strcat(txString,"\n Serial and BLE:\n# .................\n");
   strcat(txString," STOP  Stops measurement \nSTART Starts new measurement \nRATE:<<N>>: Set sampling rate to N \nCHECKATT  Get attitude angles from IMU anf GPS coordinates \n CHECKLASER  Get Laser data for 1 second and send it over BLE and serial \nCOMS  List commands \nLOGGPS:<<b>>: Log GPS data if b=1  (Default) ");
   Send_tx_String(txString);
   delay(100);
   strcpy(txString,"");
   strcat(txString,"\n BLE only:\n# .................\n");
   strcat(txString,"CHECKIMU  Get IMU data and send them over BLE+serial for manual check \nDYNMODEL:<<n>>: Set dynamic model to n.  Sensor need to be recalibrated after changing the setting.\n        n=4: Automotive (default) \n        n=0: Applications with low acceleration. \n        n=6: Airborne. Greater vertical acceleration  \n");
   strcat(txString,"IMUCAL:<<b>>: Change setting for regular check for IMU calibration. b=0,1 \n LOGRMX:<<b>>: Log RMX messages or not. b=0,1 \nNAVPVT:<<b>>: same \nNAVPVAT:<<b>>: same  \n");
   strcat(txString,"ESFINS:<<b>>: same\nESFRAW:<<b>>:  same \nESFMEAS:<<b>>:  same \nESFALG:<<b>>: same \n");
   strcat(txString,"# --------------------------------------\n");   
   Send_tx_String(txString);
   delay(100);
   strcpy(txString,"");
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
        } else if (rxValue.find("CHECKATT") != -1) {
           check_attitude( );
           check_laser();
        } else if (rxValue.find("CHECKLASER") != -1) {
           check_laser();
        } else if (rxValue.find("COMS") != -1) {
           commands();
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
		} else if (rxValue.find("LOGGPS") != -1) {
           char passValue[40];
       for (int i = 0; i < rxValue.length(); i++){
               passValue[i] = rxValue[i];}
           setLogFlag( passValue, log_GPS, "LOGGPS");
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




// /////////////////////////////////////////////
// -%--------------------------------------------
// Setup and loop
// %%------------------------------------------------------------------------------------------------------------------------------
void setup(){
    pinMode(statLED, OUTPUT);
    digitalWrite(statLED, HIGH);
    delay(1000);
    digitalWrite(statLED, LOW);

    Serial.begin(115200);

    // Setup RS232 connection to Laser
    //--------------------
    RS232.begin(baudrateRS232,SERIAL_8N1, PIN_Rx, PIN_Tx);  // Use this for HardwareSerial
    RS232.setRxBufferSize(Laser_BufferSize);
    delay(500);
  	RS232.write(0x1B);  // stop sending data
    delay(500);
  	RS232.write("SD 0 3");  // set data format
  	RS232.write(0x0D);  
    delay(100);


    // Setup GPS connection
    //--------------------
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
    myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
    myGNSS.setDynamicModel(DynamicModel);  
    myGNSS.setNavigationFrequency(NavigationFrequency); //Produce  navigation solution at given frequency
    Serial.println(F("Connection to GPS succesful"));


    // Setup BLE connection
    //--------------------
    setup_BLE();


    // Setup SD connection
    //--------------------
    myBuffer_laser = new uint8_t[sdWriteSize_Laser*3]; // Create our own buffer to hold the data while we write it to SD card
    
    Serial.print("CD pin value:");
    Serial.println(digitalRead(CD_pin));
    if (!digitalRead(CD_pin)){
      Serial.println("No SD card inserted. Waiting for it.");
      while (1){
        LED_blink(200, 3);
        delay(2000);
        if (digitalRead(CD_pin)) { 
          Serial.println(F("SD card inset=rted continuing."));
          break;
        }
      }
    }

	  spi.begin(SCK, MISO, MOSI, CS);							   
    if(!SD.begin(CS,spi,SPI_rate)){
        Serial.println("Card Mount Failed");
        while (1);
    }
    
    uint8_t cardType = SD.cardType();
    Serial.print("SD Card Type: ");
    if(cardType == CARD_MMC){
        Serial.println("MMC");
    } else if(cardType == CARD_SD){
        Serial.println("SDSC");
    } else if(cardType == CARD_SDHC){
        Serial.println("SDHC");
    } else if(cardType == CARD_NONE){
        Serial.println("No SD card attached. ");
    } else {
        Serial.println("UNKNOWN");
    }
//    listDir(SD, "/", 0);
    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);
    Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
    Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
    
    
    if (logging) {  
      makeFiles(SD);
    } else {
      BLE_message=true;
      strcpy(txString,"Waiting for command 'START' over Serial of BLE for starting logging data!");
      Send_tx_String(txString);
    }

    // setting up GPS for automatic messages
  	setupGNSS();
  	delay(200);
	
    //call for GPS status messages
    if (log_STATUS ){   
    	myGNSS.getESFSTATUS();
    	myGNSS.getESFALG();
    }
    
    //Check fusion mode
    if (IMUcalibration){   
      checkIMUcalibration();
      Serial.print(txString);
    }
    
	//start Laser measuremens
	RS232.write("DT");
    RS232.write(0x0D);
	
    Serial.println(F("Setup completeded."));
    lastPrint = millis(); // Initialize lastPrint
    logTime_laser  = millis(); // logTime_laser  
}


void loop(){
  
  //################################# Data logging ############################################# 
  
  if (logging){    // if logging is avctive check for GNSS data and save them to SD card in binary file
    
    //  GNSS data
    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    myGNSS.checkUblox(); // Check for the arrival of new data and process it.
    while (myGNSS.fileBufferAvailable() >= sdWriteSize and ((millis()-logTime_laser) < Laser_log_intervall) and log_GPS) // Check to see if we have at least sdWriteSize waiting in the buffer
    {
//      digitalWrite(LED_BUILTIN, HIGH); // Flash LED_BUILTIN each time we write to the SD card

      uint8_t myBuffer[sdWriteSize]; // Create our own buffer to hold the data while we write it to SD card

      myGNSS.extractFileBufferData((uint8_t *)&myBuffer, sdWriteSize); // Extract exactly sdWriteSize bytes from the UBX file buffer and put them into myBuffer

      dataFile.write(myBuffer, sdWriteSize); // Write exactly sdWriteSize bytes from myBuffer to the ubxDataFile on the SD card

      bytesWritten += sdWriteSize; // Update bytesWritten

      // In case the SD writing is slow or there is a lot of data to write, keep checking for the arrival of new data
//      myGNSS.checkUblox(); // Check for the arrival of new data and process it.

//      digitalWrite(LED_BUILTIN, LOW); // Turn LED_BUILTIN off again
      Serial.print(".");
	    delay(20);
    }

    //  Laser data
    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    if ( RS232.available() >= sdWriteSize_Laser) {   
      bitesToWrite=RS232.available();
      if (bitesToWrite>sdWriteSize_Laser*2) {
        bitesToWrite=sdWriteSize_Laser*2;
        Serial.println("More bits than Writesize");
      }
      Serial.println(",");
      Serial.println(bitesToWrite);
      dataFile.println(" ");
      dataFile.print("# iTOW ");
      dataFile.println(myGNSS.packetUBXNAVPVAT->data.iTOW);
//      Serial.println("Got time");
//      while(RS232.available()){
//        dataFile.write(RS232.read());   
//      }
      RS232.readBytes(myBuffer_laser, bitesToWrite);
//      Serial.println("Got data");
      dataFile.write( myBuffer_laser, bitesToWrite);
//      Serial.println("wrote data");
      dataFile.print("# end ");
      dataFile.println(myGNSS.packetUBXNAVPVAT->data.iTOW);
//      Serial.println("end");
      logTime_laser2  = millis();
    }
    logTime_laser  = millis();



    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    if (millis() > (lastPrint + 5000)) // Print bytesWritten once per 5 seconds
    {
      Serial.println(" ");
      Serial.println(RS232.available());
      if ((millis()-logTime_laser2) > 5*Laser_log_intervall) {  //Flush RS232 buffer if data were are not read for too long.
         while(RS232.read() >= 0);
      }
      BLE_message=true;
      strcpy(txString,".");
      lastPrint = millis(); // Update lastPrint
    }

    if (lastTime_logstatus + flush_intervall < millis()){
      dataFile.flush();
      lastTime_logstatus  = millis();
      Serial.println("flushed");
    }
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
    stop_logging(SD);
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
        logging = false;
        stop_logging(SD);
      } else if (rxValue.indexOf("RATE") != -1) {
        setRate(rxValue);
      } else if (rxValue.indexOf("READ") != -1) {
        readFiles(rxValue);
      } else if (rxValue.indexOf("LIST") != -1) {
        getFileList();
      } else if (rxValue.indexOf("CHECKATT") != -1) {
        check_attitude( );
      } else if (rxValue.indexOf("CHECKLASER") != -1) {
        check_laser();
      } else if (rxValue.indexOf("COMS") != -1) {
        commands(); 
      }else if (rxValue.indexOf("LOGGPS") != -1) {
           setLogFlag( rxValue, log_GPS, "LOGGPS");
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
