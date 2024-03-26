#include <HardwareSerial.h>
#include <NMEAParser.h>
#include "Support.h"

HardwareSerial GPS(1);
NMEAParser<3> parser;



// settings ESP now
//------------------------------------
unsigned long  ESP_inbytes=0;
int  ESP_inbytes_buff=0;




long lastTime = 0;  //Simple local timer
long timer = 0;  //Simple local timer

long lastTime1 = 0;  //Simple local timer
long lastTime2 = 0;  //Simple local timer
long lastTime3 = 0;  //Simple local timer
long lastTime_flushSD = 0;
long logTime_GPS;
long lastTime_ASCB=0; 
long startTime=0;     //keep track of ESP32 measuing time


TaskHandle_t TaskStart;
TaskHandle_t TaskEnd;



#include "FS.h"
#include "SD.h"
#include "SPI.h"


// settings PIN SPI 
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
#define SCK 18
#define MISO 19
#define MOSI 23
#define SPI_rate_SD 20000000  
#define CS_SD 5   // cip select  SD card
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

SPIClass spi = SPIClass(VSPI);

uint16_t addr;
char addrStr[5];
uint16_t dat;
char datStr[19];
uint32_t msg;
uint16_t out;


// settings SD card
//------------------

#define sdWriteSize 16384     // Write data to the SD card in blocks of 512 bytes
#define WriteSize_Laser 23  // Write data to buffer in blocks (should be shorter than expected message)
#define WriteSize_GPS 30  // Write data to buffer in blocks (should be shorter than expected message)
#define myBufferSize 49152
#define tempBufferSize 16384      // must be bigger than sdWriteSize,WriteSize_Laser and WriteSize_GPS
#define LaserBufferSize 2048 
#define flushSD_intervall 120000  // Flush SD every ** milliseconds to avoid losing data if ESP32 crashes

uint8_t myBuffer[myBufferSize];       // Create our own buffer to hold the data while we write it to SD card
uint8_t tempBuffer[tempBufferSize];   // Create temporay buffer
uint8_t myBuffer_laser[LaserBufferSize];  // Create buffer for laser data

int BufferTail = 0;
int BufferHead = 0;
int bitesToWrite;


char dataFileName[24];  //Max file name length is 23 characters)
File dataFile;          //File where data is written to


char txString[500];  // String containing messages to be send to BLE terminal
char txString2[50];
char subString[64];


bool measuring = false;
bool StartFlag = false;
bool StopFlag = false;
bool StopMultiFlag = false;
bool  flagcalMulti = false;
bool closefileFlag = false;
bool SD_mounted =0;
bool SD_filecreated =0;
unsigned long lastPrint;  // Record when the last Serial print took place


// settings UART GPS
//-------------------
const unsigned int GPS_RX = 33; //  Hardware RX pin,
const unsigned int GPS_TX = 27; // Hardware TX pin,

const unsigned int BAUDGPS= 57600 ;

char RxVal;
#define baudrateGPS 234375 //57600 //230400  //115200
#define GPS_BufferSize 8192  // Allocate 1024 Bytes of RAM for UART serial storage
#define GPS_log_intervall 2000

long Year =1999;
long Month =88;
long Day =77;
long Hour =1;
long Minute = 2;
long Second = 3;
bool validDate=0;  // flag indicating when a valid date was received



// /////////////////////////////////////////////
// --------------------------------
// other functions
//---------------------------------
// /////////////////////////////////////////////








void writeToBuffer(uint8_t *tempBuf, int &bitesToWrite) {
  // move BufferHead if circular buffer is full
  int nDump;
  if (BufferTail >= BufferHead) {
    nDump = bitesToWrite - (myBufferSize + BufferHead - BufferTail);
  } else {
    nDump = bitesToWrite - (BufferHead - BufferTail);
  }
  if (nDump >= 0) {
    Serial.print("Data buffer is getting full. Dumping N data:");
    Serial.println(nDump + 1);
    BufferHead = (BufferHead + nDump + 1) % myBufferSize;
  }

  // write data to buffer
  if ((BufferTail + bitesToWrite) < myBufferSize) {
    for (int i = 0; i < bitesToWrite; i++) {
      myBuffer[BufferTail] = tempBuf[i];
      BufferTail++;
    }
  } else {
    int a = myBufferSize - BufferTail;
    for (int i = 0; i < a; i++) {
      myBuffer[BufferTail] = tempBuf[i];
      BufferTail++;
    }
    BufferTail = 0;
    for (int i = 0; i < (bitesToWrite - a); i++) {
      myBuffer[BufferTail] = tempBuf[i + a];
      BufferTail++;
    }
  }
}






void startMeasuring() {

  
  makeFiles(SD);
  delay(200);

  lastPrint = millis();      // Initialize lastPrint
  logTime_GPS = millis();
  BufferTail = 0;
  BufferHead = 0;
  closefileFlag = false;

  
  // Write start of data to datafile
  dataFile.println(F("###Data###"));  
  

  while (GPS.available() > 0){
    char k = GPS.read();
  }



  if (SD_filecreated){
    strcat(txString, "\r\nFile succesfully created on ESP32 SD card");
    Serial.println(txString);

  } else{
    strcat(txString, "\r\n ESP32 SD not working. Abort.");
    Serial.println(txString);
	  
	  
	measuring = false;  // Set flag to false
	Serial.println("abort mesurment!");


	delay(25);
	dataFile.println(F("# unsuccesful measurement"));

	closefileFlag = true;

	sendToBase(txString);
	  
  }
  
  strcat(txString, "\r\nStarted measurement succesfully!");
  Serial.println(txString);
  sendToBase(txString);
  
  
}




// Stop data measuring process
void stopTask(void * pvParameters) {
  while(true){
  Serial.println("Turning dataMeasuring OFF!");
  

  measuring = false;  // Set flag to false
  
  delay(100);
  emptyBuffer();
  delay(100);
  closefileFlag = true;

  // Serial.println("Datafile closed.");                     
  strcat(txString, "Measurement stopped successfully");
  sendToBase(txString);
  Serial.println("Measurement stopped successfully");

  delay(100);
  vTaskDelete(TaskEnd);
  }
}


void closeFile(fs::FS &fs){
  // output file string
  char filesstring[50];
  strcpy(txString, "Closed file:");
  sprintf(filesstring, " File: %s", dataFileName);
  strcat(txString, filesstring);
  Serial.print("Closing file:");
  Serial.println(filesstring);
  delay(25);
  Write_stop();
  dataFile.close();  // Close the data file
  Serial.println("Datafile closed."); 

}

void emptyBuffer(){
  //  Empty data buffer
  // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  Serial.print("Total bytes in buffer:");
  Serial.print((myBufferSize + BufferTail - BufferHead) % myBufferSize);
  Serial.print(", buffer tail:");
  Serial.print(BufferTail);
  Serial.print(", buffer head:");
  Serial.println(BufferHead);
  int bytesTot = 0;
  int remainingBytes = (myBufferSize + BufferTail - BufferHead) % myBufferSize;  // Check if there are any bytes remaining in the file buffer

  while (remainingBytes > 0)  // While there is still data in the file buffer
  {
    int bytesToWrite = remainingBytes;  // Write the remaining bytes to SD card sdWriteSize bytes at a time
    if (bytesToWrite > sdWriteSize) {
      bytesToWrite = sdWriteSize;
    }
    // write data to buffer
    if ((BufferHead + bytesToWrite) < myBufferSize) {  // case head + bytesToWrite smaller than buffer size
      for (int i = 0; i < bytesToWrite; i++) {
        tempBuffer[i] = myBuffer[BufferHead];
        BufferHead++;
        bytesTot++;
      }
    } else {
      int a = myBufferSize - BufferHead;
      for (int i = 0; i < a; i++) {
        myBuffer[i] = tempBuffer[BufferHead];
        BufferHead++;
        bytesTot++;
      }
      BufferHead = 0;
      for (int i = a; i < (bytesToWrite); i++) {
        tempBuffer[i] = myBuffer[BufferHead];
        BufferHead++;
        bytesTot++;
      }
    }
    dataFile.write(tempBuffer, bytesToWrite);
    remainingBytes -= bytesToWrite;
  }
  Serial.print("Buffer emptied! Data written:");
  Serial.println(bytesTot);
  
}


// Stop data measuring process
void stop_Measuring(fs::FS &fs) {
  measuring = false;  // Set flag to false
  Serial.println("Turning dataMeasuring OFF!");

   
  //  INS data
  int times=0;
  while (GPS.available() ) {
    bitesToWrite = GPS.available();
    if (tempBufferSize < bitesToWrite) {
      bitesToWrite = tempBufferSize;
    }

    GPS.readBytes(tempBuffer, bitesToWrite);
    writeToBuffer(tempBuffer, bitesToWrite);
    times++;
    delay(1);
  }
  Serial.print("Wrote INS data to buffer N times. N=");
  Serial.println(times);


  emptyBuffer();

  // output file string
  char filesstring[50];
  strcpy(txString, "Closed file:");
  sprintf(filesstring, " File: %s", dataFileName);
  strcat(txString, filesstring);
  Serial.print("Closing file:");
  Serial.println(filesstring);
  delay(25);
  Write_stop();
  dataFile.close();  // Close the data file
  // Serial.println("Datafile closed.");                     
  strcat(txString, "Measurement stopped successfully");
  sendToBase(txString);
}











void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);  
  Serial.println("Starting ");
  
  
  // Setup ESPNOW connection //
  InitESPNow();
    while(!ScanForSlave()){
    Serial.println("REP ESP not found. trying again in 5 seconds.");
    delay(5000);
  }
  Serial.println("REP ESP found.");
  xLemToBaseQueue = xQueueCreate( QUEUE_LENGTH, sizeof(ESP_Msg_t) );   // create a queue of 10 messages
  xTaskCreate(espNOWsendTask, "espNOWsendTask", 1024*6, NULL, 4, NULL);
  //xTaskCreate(serialEventRadioTask, "serialEventRadioTask", 1024*6, NULL, 5, NULL);
  
  
  
    // Setup GPS connection
  //--------------------
  
  GPS.begin(BAUDGPS,SERIAL_8N1, GPS_RX, GPS_TX);  
  GPS.setRxBufferSize(GPS_BufferSize);
  Serial.println("Started serial to GPS");




  // Setup parser NMEA messages //
  parser.setErrorHandler(errorHandler);
  parser.addHandler("LEMMS", LEM_Handler);
  parser.setDefaultHandler(unknownCommand);

  lastTime=millis(); 
  
  
  
  
  
   // Setup SPI+SD 
  //--------------------
  spi.begin(SCK, MISO, MOSI, CS_SD);  
  Serial.println("Started SPI");
  pinMode(CS_SD, OUTPUT); //set up slave select pins as outputs as the Arduino API doesn't handle automatically pulling SS low
  digitalWrite(CS_SD, HIGH);

  
  // Setup 
  //--------------------
  if (!SD.begin(CS_SD, spi, SPI_rate_SD)) {
    Serial.println("Card Mount Failed");
    // while (1)
      ;
  } else{
    Serial.println("SD mounted");
	SD_mounted=1;
  }

  uint8_t cardType = SD.cardType();
  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else if (cardType == CARD_NONE) {
    Serial.println("No SD card attached. ");
  } else {
    Serial.println("UNKNOWN");
  }
  //    listDir(SD, "/", 0);
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
  Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
  Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));


  strcpy(txString, "Setup completeded. Waiting for command 'START' over Serial of BLE for starting measuring data!");
  sendToBase(txString);

}





void loop() {

  //################################# Data logging #############################################

  if (measuring) {  // if measuring is active check for INS and Laser data and save them to SD card

    //  GPS data
    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    if (GPS.available() >= WriteSize_GPS) {
      bitesToWrite = GPS.available();
      if (tempBufferSize < bitesToWrite) {
        bitesToWrite = tempBufferSize;
      }

      GPS.readBytes(tempBuffer, bitesToWrite);
      writeToBuffer(tempBuffer, bitesToWrite);
      logTime_GPS = millis();

      // check if an entire message was written
      if (tempBuffer[bitesToWrite-5]!='*' and GPS.available()>0 ){
        bitesToWrite = GPS.available();
        if (tempBufferSize < bitesToWrite) {
          bitesToWrite = tempBufferSize;
        }
        GPS.readBytes(tempBuffer, bitesToWrite);
        writeToBuffer(tempBuffer, bitesToWrite);
      }
      delay(1);
    }







    //  Write data to SD
    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    if (((myBufferSize + BufferTail - BufferHead) % myBufferSize) > sdWriteSize) {
      lastTime = millis();
	  
      // write data to buffer
      if ((BufferHead + sdWriteSize) < myBufferSize) {  // case head + sdWritesize smaller than buffer size
        for (int i = 0; i < sdWriteSize; i++) {
          tempBuffer[i] = myBuffer[BufferHead];
          BufferHead++;
        }
      } else {
        int a = myBufferSize - BufferHead;
        for (int i = 0; i < a; i++) {
          tempBuffer[i] = myBuffer[BufferHead];
          BufferHead++;
        }
        BufferHead = 0;
        for (int i = a; i < sdWriteSize; i++) {
          tempBuffer[i] = myBuffer[BufferHead];
          BufferHead++;
        }
      }
      lastTime2 = millis();


      dataFile.write(tempBuffer, sdWriteSize);
      lastTime3 = millis();
      Serial.print(",Data written:");
      Serial.print(sdWriteSize);
      // Serial.print(" ,write to tempBuffer (ms):");
      // Serial.print(lastTime2-lastTime);
      Serial.print(" , in time (ms):");
      Serial.println(lastTime3-lastTime);
      delay(1);

      // include stamp for writing to SD
      strcpy(subString,"stamp SDwrite\n");
      int msglen=strlen(subString);
      for (int i = 0; i < msglen; i++){
          tempBuffer[i] = uint8_t(subString[i]);
        } 
      writeToBuffer(tempBuffer, msglen);
      // Serial.println("stamp SDwrite" );

    }


    //==========================================================


    // flush GPS buffer if stuck
    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-
    if ((millis() - logTime_GPS) > GPS_log_intervall and GPS.available()) {  //Flush RS232 buffer if data were are not read for too long.
      while (GPS.read() >= 0)
        ;
      Serial.println("GPS buffer flushed");
    }
    


    // flush SDCard every "flushSD_intervall"
    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-
    if (lastTime_flushSD + flushSD_intervall < millis()) {
      dataFile.flush();
      dataFile.flush();
      lastTime_flushSD = millis();
      Serial.println("flushed");
    }


  } else {
    delay(50);  // wait 50 ms if not 
	//  Serial.print("loop() running on core ");
	// Serial.println(xPortGetCoreID());
  }


// Moving STOP and START in loop should make them independent of BLE
  if (StopFlag) {
    delay(100);
    StopFlag = false;
    // measuring = false;
	  // stop_Measuring(SD);
	xTaskCreatePinnedToCore(stopTask,"TaskEnd",10000, NULL, 1,  &TaskEnd,0); 
  }

  if (closefileFlag) {
    closefileFlag = false;
	  closeFile(SD);
    Serial.println("End of measurement");  
  }

  if (StartFlag) {
    StartFlag = false;
    measuring = true;
    startMeasuring();
  }

  
  if (Serial.available()) {      // If anything comes in Serial is passed to NMEA parser
      // while(Serial.available())  {   
      //       RxVal=Serial.read();
      //       Serial.write(RxVal);  
      //       parser << RxVal; 
      // }
    String rxValue = Serial.readString();
    if (rxValue.length() > 0) {
      Serial.println("*********");
      Serial.print("Received Value: ");
      for (int i = 0; i < rxValue.length(); i++)
        Serial.print(rxValue[i]);
      Serial.println();
		  parse(rxValue);
      Serial.println("*********");
    }
    }
  



  if (ESP_inbytes_buff>1000) {      // If anything comes in Serial is passed to NMEA parser
      ESP_inbytes+=ESP_inbytes_buff/1000;
      ESP_inbytes_buff=0;
      Serial.print("Total kb received:");
      Serial.println(ESP_inbytes);
    }

  if (millis()-lastTime >5000) {      // Send periodic mesages to base
      sendToBase("hey");
      lastTime=millis();
    }
  delay(5);

}








void sendToBase(char * msg){
  ESP_Msg_t espMsg = {0};
  memcpy(&espMsg.msg, msg, strlen(msg));
  espMsg.length = strlen(msg);
  xQueueSend( xLemToBaseQueue, ( void * ) &espMsg, ( TickType_t ) 0 );
  Serial.print(msg);
}
