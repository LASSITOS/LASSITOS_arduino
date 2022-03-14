/*
 * Connect the SD card to the following pins:
 *
 * SD Card | ESP32
 *    D2       -
 *    D3       SS
 *    CMD      MOSI
 *    VSS      GND
 *    VDD      3.3V
 *    CLK      SCK
 *    VSS      GND
 *    D0       MISO         ! GPIO 23 is used by I2C port to GNSS! Use other PIN
 *    D1       -
 */
#include "FS.h"
#include "SD.h"
#include "SPI.h"


//HardwareSerial
#include <HardwareSerial.h>
HardwareSerial RS232(2);


long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.
long lastTime2 = 0; //Second simple local timer. 
long lastTime_logstatus = 0; //Second simple local timer. 
long logTime_laser;
long start;

#define statLED  13
bool logging = true;

// settings SD
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
#define SCK  18
#define MISO  19
#define MOSI  32  //! GPIO 23 is used by I2C port to GNSS! Use other PIN
#define CS  5
#define SPI_rate 80000000
#define CD_pin 27         // chip detect pin is shorted to GND if a card is inserted. (Otherwise pulled up by 10kOhm)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

#define sdWriteSize 256 // Write data to the SD card in blocks of 512 bytes
uint8_t *myBuffer; // A buffer to hold the data while we write it to SD car
SPIClass spi = SPIClass(VSPI);


char headerFileName[24]; //Max file name length is 23 characters)
char dataFileName[24]; //Max file name length is 23 characters)
File dataFile; //File that all data is written to
File headerFile; //File containing a header with settings dscription
//File testFile;



// settings altimeter LSD70A
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
//PINs can be changed to any pin. // For Hardware Serial use Pin 16 an 17. SoftwareSerial worked on pins 12 and 27
int PIN_Rx = 16; // 16 = Hardware RX pin,
int PIN_Tx = 17; // 17 = Hardware TX pin,
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

#define baudrateRS232 115200 
#define Laser_fileBufferSize 512 // Allocate 512Bytes of RAM for UART serial storage
#define logIntervall_laser 1000
int bitsToWrite;


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
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if(!file){
        Serial.println("Failed to open file for reading");
        return;
    }

    Serial.print("Read from file: ");
    while(file.available()){
        Serial.write(file.read());
    }
//    file.close();
    while(RS232.read() >= 0);
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

void testFileIO(fs::FS &fs, const char * path){
    File file = fs.open(path);
    static uint8_t buf[512];
    size_t len = 0;
    uint32_t start = millis();
    uint32_t end = start;
    if(file){
        len = file.size();
        size_t flen = len;
        start = millis();
        while(len){
            size_t toRead = len;
            if(toRead > 512){
                toRead = 512;
            }
            file.read(buf, toRead);
            len -= toRead;
        }
        end = millis() - start;
        Serial.printf("%u bytes read for %u ms\n", flen, end);
        file.close();
    } else {
        Serial.println("Failed to open file for reading");
    }


    file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }

    size_t i;
    start = millis();
    for(i=0; i<2048; i++){
        file.write(buf, 512);
    }
    end = millis() - start;
    Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
    file.close();
}



File openFile(fs::FS &fs, const char * path){
    Serial.printf("Creating file: %s\n", path);
    return fs.open(path, FILE_WRITE);
}



void setup(){
    Serial.begin(115200);


    // Setup RS232 connection to Laser
    //--------------------
    RS232.begin(baudrateRS232,SERIAL_8N1, PIN_Rx, PIN_Tx);  // Use this for HardwareSerial
    RS232.setRxBufferSize(Laser_fileBufferSize);

    myBuffer = new uint8_t[sdWriteSize]; // Create our own buffer to hold the data while we write it to SD card
    
    Serial.print("CD pin value:");
    Serial.println(digitalRead(CD_pin));
    if (!digitalRead(CD_pin)){
      Serial.println("No SD card inserted. Freezing.");
      while (1);
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
    listDir(SD, "/", 0);
    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);
    Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
    Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
    
    
    
    // Make data file
    strcpy(dataFileName,  "/data.ubx");   // create name of data file
    Serial.print("Making data file:");
    Serial.println(dataFileName);
    
    dataFile=SD.open( dataFileName, FILE_WRITE);
//    dataFile.print("Test,");
//    dataFile.print("second line");
//    dataFile.close();
//    Serial.print("Created file: ");
//    Serial.println(dataFileName);
//    delay(500);
//    dataFile=SD.open( dataFileName, FILE_APPEND);
    if(!dataFile){
              Serial.print("Freezing! Could no open file for appending: ");
              Serial.println(dataFileName);
              while (1);
    } else {
    Serial.print("Opened file for appending: ");
    Serial.println(dataFileName);
    }
    while(RS232.read() >= 0) ; // flush the receive buffer.
    logTime_laser  = millis(); // logTime_laser  
    start  = millis(); // logTime_laser 
}

void loop(){
  if (logging){
    
    while(RS232.available()){
        Serial.write(RS232.read());   
    }
    if (millis()-logTime_laser > 2000){
      Serial.print("Time: ");
      Serial.println(millis());
      logTime_laser=millis();
    }

//    if ( RS232.available() >= sdWriteSize) {   
//      Serial.println(".");
//      Serial.println(RS232.available());
//      
//      while(RS232.available()){
//        dataFile.write(RS232.read());   
//      }
////    }else if (RS232.available()){
////      Serial.println(RS232.available());
//    }
//    if ( RS232.available() >= sdWriteSize) {   
//      bites=RS232.available()
//      Serial.print(", ");
//      Serial.println(RS232.readBytes(myBuffer, sdWriteSize));
//      Serial.print(".");
//      Serial.println(dataFile.write( myBuffer, sdWriteSize))
//    }



//  Laser data
    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
//    if ( RS232.available() >= sdWriteSize and (millis()-logTime_laser) > 1000) {   
//      bitsToWrite=RS232.available();
//      Serial.println(",");
//      Serial.println(bitsToWrite);
//      Serial.println(millis()-start);
//      dataFile.println(" ");
//      dataFile.print("# iTOW ");
//      dataFile.println(millis()-start);
////      while(RS232.available()){
////        dataFile.write(RS232.read());   
////      }
//      Serial.print(", ");
//      Serial.println(RS232.readBytes(myBuffer, bitsToWrite));
//      Serial.print(".");
//      Serial.println(dataFile.write( myBuffer, bitsToWrite));
//
//      Serial.println(millis()-start);
//      dataFile.print("# end ");
//      dataFile.println(millis()-start);
////      delay(20);
//      logTime_laser  = millis();
//    }

    
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
      if  (rxValue.indexOf("STOP") != -1)  {
         
          if(dataFile){
            Serial.println("Closing datafile:");
            Serial.println(dataFileName);
            delay(500);
            dataFile.close(); // Close the data file
            Serial.println("Datafile closed.");
            listDir(SD, "/", 0);
            readFile(SD,dataFileName);
          }
          logging = false;
      }
      Serial.println("*********");
    }
  }

  if (lastTime_logstatus  +10000 < millis()){
    dataFile.flush();
    lastTime_logstatus  = millis();
    Serial.println("Flushed");
  }
  delay(10);

}
