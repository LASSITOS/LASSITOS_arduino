/*
  Code for controlling the LASSITOS sea ice EM esp32 microprocessor.
  By: AcCapelli
  Date: April 7th, 2023
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  Based on Examples from multiple libraries and hardware.
  
  BLE:
  Set up BLE connections with UART communication
  Start new data files if "START" is received and stop current data files if "STOP" is received. See below for more BLE commands.
  
*/

#include "FS.h"
#include "SD.h"
#include "SPI.h"


//HardwareSerial
#include <HardwareSerial.h>
HardwareSerial RS232(2);
HardwareSerial IMX5(1);
// HardwareSerial Serial(0);

#define baudrateSerial 115200 
String LEM_version= "LEM sounder ESP32 v1.2";

String LEM_ID="LEM00";

long lastTime = 0;  //Simple local timer
long lastTime1 = 0;  //Simple local timer
long lastTime2 = 0;  //Simple local timer
long lastTime3 = 0;  //Simple local timer
long lastTime_STROBE =0;
long lastTime_flushSD = 0;
long logTime_laser;
long logTime_IMX5;
long lastTime_VBat=0; 
long lastTime_Temp=0; 
long lastTime_ASCB=0; 
long lastTime_CAL =0;
long startTime=0;     //keep track of ESP32 measuing time


TaskHandle_t TaskStart;
TaskHandle_t TaskEnd;
TaskHandle_t TaskSwitchMulti;
TaskHandle_t TaskCalMulti;
bool CalMulti_on = false;
TaskHandle_t TaskCal;

// settings PIN SPI 
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
#define SCK 18
#define MISO 19
#define MOSI 23
#define SPI_rate    400000  // DAC up to 80 MHz 
#define SPI_rate_SD 20000000  
#define CS_SD 5   // cip select  SD card
#define CS_DAC 14
#define CS_MSP430  15  // Chip select MSP430
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

SPIClass spi = SPIClass(VSPI);

uint16_t addr;
char addrStr[5];
uint16_t dat;
char datStr[16];
uint32_t msg;
uint16_t out;


//Other PINs
//-=-=-=-=-=-=-=-=-=-=-=-=-=
#define CD_pin 36  // chip detect pin is shorted to GND if a card is inserted. (Otherwise pulled up by 10kOhm)
#define triggerDAC 32     
#define PIN_VBat 34    

float VBat=0;
long VBat_intervall = 10000;



// settings SD card
//------------------

#define sdWriteSize 16384     // Write data to the SD card in blocks of 512 bytes
#define WriteSize_Laser 23  // Write data to buffer in blocks (should be shorter than expected message)
#define WriteSize_IMX5 30  // Write data to buffer in blocks (should be shorter than expected message)
#define myBufferSize 49152
#define tempBufferSize 16384      // must be bigger than sdWriteSize,WriteSize_Laser and WriteSize_IMX5
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


bool measuring = false;
bool StartFlag = false;
bool StopFlag = false;
bool StopMultiFlag = false;
bool  flagcalMulti = false;
bool closefileFlag = false;




String BLEname="LEM BLE";
uint16_t gainDAT = 0x1000;   // Should not be higher than maxGAIN 



//resonant frequencies:
int F1= 656 ;//684
int F2 =1062; //1079
int F3= 1906 ;//1908
int F4= 5766; //5754
int F5= 6867 ;//6848
int F6= 8233 ;  //8207

int F46= 4737; //4725
int F456= 3909 ;//3900


uint64_t freqs[] ={F1,F2,F3,F456,F46,F4,F5,F6};
uint16_t Nfreq=8;  //Number of frequencies to use.  Not larger than length of 'freqs'.


int ifreq =2;
uint64_t freq=freqs[ifreq];
int Nmulti =0;
int i_multi =0;
uint64_t Multifreqs[]={1,3,6,0,0,0,0,0,0,0}; //Must be same or larger than Nfreq
int MultiPeriod=300;
long MultiTime=0;
int Nmulti_reg=3;




int CAL_states[]={0,1,2,4,8,12,14};
int CAL_state=0;
int N_cal=7;
bool calibrating=0;
bool cal_on=0;
int  CAL_intervall_on=3000;
int  CAL_intervall_off=500;
int  N_CalMulti_on=5;
int  N_CalMulti_off=2;
int  CALMulti_intervall_on=Nmulti_reg*MultiPeriod*N_CalMulti_on;
int  CALMulti_intervall_off=Nmulti_reg*MultiPeriod*N_CalMulti_off;


// Non operational variables for passing LEM values to datafile 
//-=-=-=-=-=-=-=-=-=-=-=- 
int N_CalCoil=320; 
int N_RxCoil=1014; 
int N_BuckCoil=32;
float Rs[]={77.5,143.5,400, 619,277,125,};
float d_Rx=1.92;
float d_Bx=0.56;
float d_Cx=1.695;
float L_CalCoil=0.011365 ;  
float A_CalCoil= 0.00502 ; 
float A_RxCoil= 0.00502;
float A_BuckCoil= 0.00502;
float distCenter=0.244;







// /////////////////////////////////////////////
// -%--------------------------------------------
// Setup and loop
// %%------------------------------------------------------------------------------------------------------------------------------
void setup() {

  Serial.begin(baudrateSerial);





  // Setup SPI connection
  //--------------------
  spi.begin(SCK, MISO, MOSI, CS_SD);  
  Serial.println("Started SPI");
  pinMode(CS_SD, OUTPUT); //set up slave select pins as outputs as the Arduino API doesn't handle automatically pulling SS low
  digitalWrite(CS_SD, HIGH);
 
  // Setup SD connection
  //--------------------
  if (!SD.begin(CS_SD, spi, SPI_rate_SD)) {
    Serial.println("Card Mount Failed");
    // while (1)
      ;
  } else{
    Serial.println("SD mounted");
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




  readSettingsFile(SD);


}


void loop() {


}
