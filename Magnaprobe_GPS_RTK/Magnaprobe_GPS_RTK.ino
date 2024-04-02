/*
  Getting NMEA from u-blox GNSS module and saving them to SD card.
  By: AcCapelli
  Date: March, 2024
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  Based on Examples from used libraries and hardware.

  Hardware Connections:
  
  Plug a Qwiic cable into the GNSS 
  Open the serial monitor at 115200 baud to see the output
  
  
  BLE:
  Set up BLE connections with UART communication
  Start new data files if "START" is received and stop current data files if "STOP" is received. See below for more BLE commands.
  
*/

#include <Wire.h> //Needed for I2C to GNSS

#include <SparkFun_u-blox_GNSS_v3.h> //Click here to get the library: http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS myGNSS;

#include "FS.h"
#include "SD.h"
#include "SPI.h"


#define version "GNSS Magnaprobe v1.0"

long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.
long lastTime2 = 0; //Second simple local timer. 
long lastTime_logstatus = 0; //Second simple local timer. 


int statLED = 13;

// settings SD
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
#define SCK  18
#define MISO  19
#define MOSI  23
#define CS  5
#define SPI_rate 20000000
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

#define sdWriteSize 1024 // Write data to the SD card in blocks of 512 bytes
uint8_t *myBuffer; // A buffer to hold the GNSS data while we write it to SD car
uint8_t *myBuffer_laser; // A buffer to hold the Laser data while we write it to SD car
#define flushSD_intervall 120000  // Flush SD every ** milliseconds to avoid losing data if ESP32 crashes

SPIClass spi = SPIClass(VSPI);

char myFileName[24]; //Max file name length is 23 characters)
File myFile; //File that all data is written to



int bitesToWrite;


// Setting for u-blox 
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
bool logging = false;


bool IMUcalibration=false;
int NavigationFrequency = 5;
dynModel DynamicModel = (dynModel)1;
bool log_RMX = false;
bool log_ESFRAW  = false;
bool log_ESFMEAS = false;
bool log_ESFALG = false;
bool log_NAVPVAT = false;
bool log_NAVPVT = false;
bool log_NAVATT = false;
bool log_ESFINS = false;
bool log_STATUS = false;
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

#define fileBufferSize 16384 // Allocate 32KBytes of RAM for UBX message storage

unsigned long lastPrint; // Record when the last Serial print took place
unsigned long bytesWritten = 0; // Record how many bytes have been written to SD card


long Year ;
long Month ;
long Day ;
long Hour ;
long Minute ;
long Second;


// Setting for BLE connection
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
#include "BluetoothSerial.h"

//#define USE_PIN // Uncomment this to use PIN during pairing. The pin is specified on the line below
const char *pin = "1234"; // Change this to more secure PIN.

String device_name = "MagnaprobeGNSS";

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;

char txString[512];  // String containing messages to be send to BLE terminal
char subString[64];
char buffStr[512];
bool BLE_message=false;


// Send txSTring to BLE and Serial
void Send_tx_String(char *txString) {

  Serial.print(txString);
  SerialBT.print(txString);

  strcpy(txString, "");

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
  myFile.println(F("# GNSS and Laser altimeter log file"));
  myFile.print("# Date: ");
  myFile.print(Year);
  myFile.print("-");
  myFile.print(Month);
  myFile.print("-");
  myFile.println(Day);
  myFile.print("# Time: ");
  myFile.print(Hour);
  myFile.print(":");
  myFile.print(Minute);
  myFile.print(":");
  myFile.println(Second);
  myFile.println(F("# --------------------------------------"));
  myFile.print(F("# UBX binary data are in  file: "));
  myFile.print(myFileName);
  myFile.println(F(" "));
}



void  log_settings() {  
  myFile.println(F("# --------------------------------------"));
  myFile.print(F("Script version: "));
  myFile.println(version);
  myFile.println(F("Measurements settings: "));
  myFile.print(F("IMU calibration: "));
  myFile.println(IMUcalibration);
  myFile.print(F("NavigationFrequency: "));
  myFile.println(NavigationFrequency);
  myFile.print(F("DynamicModel: "));
  myFile.println(DynamicModel);
  myFile.print(F("log_NAVATT: "));
  myFile.println(log_NAVATT);
  myFile.print(F("log_NAVPVT: "));
  myFile.println(log_NAVPVT);
  myFile.print(F("log_NAVPVAT: "));
  myFile.println(log_NAVPVAT);
  myFile.print(F("log_ESFINS: "));
  myFile.println(log_ESFINS);
  myFile.print(F("log_ESFRAW: "));
  myFile.println(log_ESFRAW);
  myFile.print(F("log_ESFMEAS: "));
  myFile.println(log_ESFMEAS);
  myFile.print(F("log_ESFALG: "));
  myFile.println(log_ESFALG);
  myFile.print(F("log_RMX : "));
  myFile.println(log_RMX );
  myFile.println(F("# --------------------------------------"));
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
    }else if (count > 5) {
        Year = 2000 ;
        Month = 01;
        Day = 01;
        Hour = random(24) ;
        Minute = random(60);
        Second = random(60);
        Serial.println(F("GPS is not good. Making random filename with date:"));
        printDateTime();
        Serial.println(); 
		break;
    }
    
    Serial.println(F("Date or time are not valid. Waiting for better GPS connection."));
    readDateTime();
    Serial.print(F("Date and time: "));
    printDateTime();
    Serial.println();
	delay(1000);
  }
  
  sprintf(myFileName, "/MagGNSS_%02d%02d%02d_%02d%02d%02d.txt", Year%1000 ,Month ,Day ,Hour,Minute,Second);  // create name of header file
  Serial.println(myFileName);
  Serial.println(myFileName);
  
  

  Serial.print("Making header file.");
  myFile = fs.open(myFileName, FILE_WRITE);
  if(!myFile){
    Serial.println(F("Failed to create header file! Freezing..."));
    Serial.println(myFileName);
    while (1);
  }
  
  Serial.print(F("created file: "));
  Serial.println(myFileName);
  Write_header();
  delay(100);
  log_settings();
  delay(100);

  
}


void  Write_stop() {  
  myFile.print(F("Measurements stopped on: "));
  myFile.print(Year);
  myFile.print("-");
  myFile.print(Month);
  myFile.print("-");
  myFile.print(Day);
  myFile.print(", ");
  myFile.print(Hour);
  myFile.print(":");
  myFile.print(Minute);
  myFile.print(":");
  myFile.println(Second);
  myFile.println(F("# --------------------------------------"));
}



// setting up GPS 
void setupGNSS(){ 
          
    // if (log_NAVPVT) {
      // // myGNSS.setAutoPVTcallback(&printPVTdata); // Enable automatic NAV PVT messages with callback to printPVTdata
      // myGNSS.setAutoPVT(true, false); // Enable automatic NAV PVT messages without callback to printPVTdata
      // myGNSS.logNAVPVT(); // Enable NAV PVT data logging
      // }
    // if (log_NAVPVAT) {
      // // myGNSS.setAutoPVTcallback(&printPVTdata); // Enable automatic NAV PVT messages with callback to printPVTdata
      // myGNSS.setAutoNAVPVAT(true, false); // Enable automatic NAV PVT messages without callback to printPVTdata
      // myGNSS.logNAVPVAT(); // Enable NAV PVT data logging
    // }
    // else {
      // // myGNSS.setAutoPVTcallback(&printPVTdata); // Enable automatic NAV PVT messages with callback to printPVTdata
      // myGNSS.setAutoNAVPVAT(false, false); // Enable automatic NAV PVT messages without callback to printPVTdata
      // myGNSS.logNAVPVAT(); // Enable NAV PVT data logging
    // }
    // if (log_NAVATT) {
      // myGNSS.setAutoNAVATT(true, false); 
      // myGNSS.logNAVATT(); 
    // }
    // if (log_ESFINS ) {
       // myGNSS.setAutoESFINS(true, false); 
       // myGNSS.logESFINS();
    // }
    // if (log_ESFRAW ) {     
      // myGNSS.setAutoESFRAW(true, false);    
      // myGNSS.logESFRAW(); 
    // }
    // if (log_ESFMEAS) {
      // myGNSS.setAutoESFMEAS(true, false); 
      // myGNSS.logESFMEAS();
    // }
    // if (log_ESFALG ) {
      // myGNSS.setAutoESFALG(true, false); 
      // myGNSS.logESFALG(); 
    // }  
    // if (log_RMX ) {
      // myGNSS.disableUBX7Fcheck(); // RAWX data can legitimately contain 0x7F, so we need to disable the "7F" check in checkUbloxI2C
      // myGNSS.setAutoRXMSFRBX(true, false); // Enable automatic RXM SFRBX messages: without callback; without implicit update
      // myGNSS.logRXMSFRBX(); // Enable RXM SFRBX data logging
      // myGNSS.setAutoRXMRAWX(true, false); // Enable automatic RXM RAWX messages: without callback; without implicit update
      // myGNSS.logRXMRAWX(); // Enable RXM RAWX data logging
    // }
    
	  // if (log_STATUS) {
      // myGNSS.logESFSTATUS();
	  // myGNSS.logESFALG();
    // }
    
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
  
  while (remainingBytes > 0 ) // While there is still data in the file buffer
  {
    digitalWrite(LED_BUILTIN, HIGH); // Flash LED_BUILTIN while we write to the SD card
    uint8_t myBuffer[sdWriteSize]; // Create our own buffer to hold the data while we write it to SD card
    uint16_t bytesToWrite = remainingBytes; // Write the remaining bytes to SD card sdWriteSize bytes at a time
    if (bytesToWrite > sdWriteSize)
    {
      bytesToWrite = sdWriteSize;
    }

    myGNSS.extractFileBufferData((uint8_t *)&myBuffer, bytesToWrite); // Extract bytesToWrite bytes from the UBX file buffer and put them into myBuffer
    myFile.write(myBuffer, bytesToWrite); // Write bytesToWrite bytes from myBuffer to the ubxmyFile on the SD card
    
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
	sprintf(filesstring, " Files: %s, %s",myFileName, myFileName);
	strcat(txString,filesstring);
	Serial.println("Turning datalogging OFF!");
	Serial.println(filesstring);

	readDateTime();
	Write_stop();
	myFile.print(F("The total number of bytes written to SD card is: ")); // Print how many bytes have been written to SD card
	myFile.println(bytesWritten);
	myFile.print(F("The maximum number of bytes which the file buffer has contained is: "));
	myFile.println(maxBufferBytes);
    myFile.close(); 
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


  makeFiles(SD);
  delay(500);
  logging = true;
  setupGNSS();
  myGNSS.clearFileBuffer();  // flush the receive buffer.

  
  lastPrint = millis(); // Initialize lastPrint
  
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
      myGNSS.setDynamicModel(DynamicModel); //Set new Dynamic Model for GNSS solution.
    } else {
      BLE_message=true;
      uint8_t currentDynmodel= myGNSS.getDynamicModel(DynamicModel); //Set new Dynamic Model for GNSS solution.
      sprintf(txString,"Current dynamic model: %d. Format for changing model:DYNMODEL:4:",currentDynmodel);
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
    Read last files (haderfile and myFile) if no argument is passed  ("READ" ) otherwise read passed files (READ:<<filepath>>:).
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
      Serial.println(myFileName);
      readFile(SD,myFileName);
      Serial.print("Reading file:");
      Serial.println(myFileName);
      readFile(SD,myFileName);
      
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




    // Setup GPS connection
    //--------------------
    Serial.println("Connecting to GPS");
    Wire.begin();

    myGNSS.setFileBufferSize(fileBufferSize); // setFileBufferSize must be called _before_ .begin
   
    if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
    {
      Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
      while (1){
        delay(2000);
        if (myGNSS.begin() == true) { //Connect to the u-blox module using Wire port
          Serial.println(F("u-blox GNSS detected I2C address. Reconnection was successfull."));
          break;
        }
      }
    }
    
    myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA); //Set the I2C port to output both UBX and NMEA messages
    // myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
//    myGNSS.setDynamicModel(DynamicModel);  
    myGNSS.setNavigationFrequency(NavigationFrequency); //Produce  navigation solution at given frequency
  myGNSS.newCfgValset(VAL_LAYER_RAM_BBR); // Use cfgValset to disable / enable individual NMEA messages
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GNS_I2C, 1); // Ensure the GxGGA (Global positioning system fix data) message is enabled. Send every measurement.
  // myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GGA_I2C, 1); // Ensure the GxGGA (Global positioning system fix data) message is enabled. Send every measurement.
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GSA_I2C, 10); // Ensure the GxGSA (GNSS DOP and Active satellites) message is enabled. Send every measurement.
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GSV_I2C, 30); // Ensure the GxGSV (GNSS satellites in view) message is enabled. Send every measurement.
  myGNSS.sendCfgValset(); // Send the configuration VALSET

  myGNSS.setNMEALoggingMask(SFE_UBLOX_FILTER_NMEA_ALL); // Enable logging of all enabled NMEA messages
  //myGNSS.setNMEALoggingMask(SFE_UBLOX_FILTER_NMEA_GGA | SFE_UBLOX_FILTER_NMEA_GSA); // Or we can, for example, log only GxGGA & GxGSA and ignore GxGSV
	
    Serial.println(F("Connection to GPS succesful"));


    // Bluetooth Serial setup
	//------------------------
	SerialBT.begin(device_name); //Bluetooth device name
	Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());
	#ifdef USE_PIN
		SerialBT.setPin(pin);
		Serial.println("Using PIN");
	#endif

    // Setup SD connection
    //--------------------   
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
	 
	myBuffer = new uint8_t[sdWriteSize]; // Create our own buffer to hold the data while we write it to SD card

    // setting up GPS for automatic messages
  	setupGNSS();
  	delay(200);
	
    // //call for GPS status messages
    // if (log_STATUS ){   
    	// myGNSS.getESFSTATUS();
    	// myGNSS.getESFALG();
    // }
    
    // //Check fusion mode
    // if (IMUcalibration){   
      // checkIMUcalibration();
      // Serial.print(txString);
    // }
    

    Serial.println(F("Setup completeded."));
    lastPrint = millis(); // Initialize lastPrint

}


void loop(){
  
  //################################# Data logging ############################################# 
  
  if (logging){    // if logging is avctive check for GNSS data and save them to SD card in binary file
    
    //  GNSS data
    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    myGNSS.checkUblox(); // Check for the arrival of new data and process it.
    while (myGNSS.fileBufferAvailable() >= sdWriteSize ) // Check to see if we have at least sdWriteSize waiting in the buffer
    {
//      digitalWrite(LED_BUILTIN, HIGH); // Flash LED_BUILTIN each time we write to the SD card

      // uint8_t myBuffer[sdWriteSize]; // Create our own buffer to hold the data while we write it to SD card

      myGNSS.extractFileBufferData((uint8_t *)&myBuffer, sdWriteSize); // Extract exactly sdWriteSize bytes from the UBX file buffer and put them into myBuffer

      myFile.write(myBuffer, sdWriteSize); // Write exactly sdWriteSize bytes from myBuffer to the ubxmyFile on the SD card

      bytesWritten += sdWriteSize; // Update bytesWritten

      myGNSS.checkUblox(); // Check for the arrival of new data and process it.
    }

    



    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    if (lastTime_logstatus + flushSD_intervall < millis()){
      myFile.flush();
      lastTime_logstatus  = millis();
      Serial.println("flushed");
    }
  }


  
  
  //################################# Do other stuff #############################################
  
    // Check BLE connection
  if ( BLE_message ) {
    Send_tx_String(txString);
    BLE_message=false;
  }
  
  if (SerialBT.available()) {  // Check SerialBT inputs
    String rxValue = SerialBT.readString();
    if (rxValue.length() > 0) {
      SerialBT.println("*********");
      SerialBT.print("Received Value: ");
      for (int i = 0; i < rxValue.length(); i++)
        SerialBT.print(rxValue[i]);
      SerialBT.println("*********");
      parse(rxValue);
    }
  }
  
  if (Serial.available()) {  // Check Serial inputs
    String rxValue = Serial.readString();
    if (rxValue.length() > 0) {
      Serial.println("*********");
      Serial.print("Received Value: ");
      for (int i = 0; i < rxValue.length(); i++)
        Serial.print(rxValue[i]);
		Serial.println("*********");
    parse(rxValue);
      
    }
  }
 
  
    // //Check fusion mode
  // if (IMUcalibration  and (millis() - lastTime2 > 10000)){   
    // checkIMUcalibration();
    // Send_tx_String(txString);
    // lastTime2 = millis(); //Update the timer
  // }
	
  // if (log_STATUS  and (millis() - lastTime_logstatus > 60000)){   
    // myGNSS.getESFSTATUS();
	  // myGNSS.getESFALG();
    // lastTime_logstatus = millis(); //Update the timer
  // }

  delay(10);
}
