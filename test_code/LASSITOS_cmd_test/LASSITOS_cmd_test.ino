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

#include "SPI.h"



#define Version "LASSITOS EM sounder ESP32 v1.0"

long lastTime = 0;  //Simple local timer
long lastTime1 = 0;  //Simple local timer
long lastTime2 = 0;  //Simple local timer
long lastTime3 = 0;  //Simple local timer
long lastTime_STROBE =0;
long lastTime_flushSD = 0;
long logTime_laser;
long logTime_IMX5;
long lastTime_VBat; 
long lastTime_Temp; 


// settings PIN SPI 
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
#define SCK 18
#define MISO 19
#define MOSI 23
#define SPI_rate    400000  // DAC up to 80 MHz  %toCheck
#define SPI_rate_SD 40000000  
#define CS_SD 5   // cip select  SD card
#define CS_DAC 14
#define CS_MSP430  15  // Chip select MSP430
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

SPIClass spi = SPIClass(VSPI);

uint16_t addr;
char addrStr[5];
uint16_t dat;
char datStr[19];
uint32_t msg;
uint16_t out;




bool measuring = false;
unsigned long lastPrint;  // Record when the last Serial print took place





long Year=2023;
long Month=4;
long Day=23;
long Hour=22;
long Minute=45;
long Second=00;





// Setting for BLE connection
//---------------------------------
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>


char txString[500];  // String containing messages to be send to BLE terminal
char txString2[50];
char subString[32];
bool BLE_message = false;














//-=-=-=-=-=-=-=-=-=-=-=-
// Microcontroller communication settings
//-=-=-=-=-=-=-=-=-=-=-=-
# define CDM_start 0x81
# define CDM_stop 0x82
# define CDM_status 0xFF
# define CDM_reset 0x84

// status flags 
bool MSP430_measuring =0;



// /////////////////////////////////////////////
// ---------------------------------------------
// Communication to Microcontroller
// ---------------------------------------------
// /////////////////////////////////////////////


void startMicro(){
//	uint8_t startMSG[13];
  /*char timestr[13];
	sprintf(timestr, "%02d%02d%02d%02d%02d", Year % 100, Month, Day, Hour, Minute); 
	startMSG[0]=CDM_start;
	for (int i=0;i<10;i++){
    startMSG[1+i] = uint8_t(timestr[i]);
  }
  startMSG[11]=0;
  startMSG[12]=0;*/
  
  
  // Serial.print("Timestring: ");
  // Serial.println(timestr);
  // Serial.print("Start message: ");
  // for (int i=0;i<12;i++){
    // Serial.printf("0x%02X.",startMSG[i]);
  // }
  // Serial.println("");
  uint8_t startMSG[13] = {0x81, 0x32, 0x33, 0x30, 0x34, 0x32, 0x35, 0x31, 0x38, 0x31, 0x15, 0x00, 0x00};
	spiTransfer2( &startMSG[0], 12, CS_MSP430 );

  
  delay(500)   ;  // Wait until microcontroller started the measurement
	uint8_t out2=spiCommand8( CDM_status   , CS_MSP430 );
	Serial.print("Microcontroller status: ");
  Serial.printf("%02X",out2);
  Serial.println(" ");
  // Parsing  response from microcontroller
	MSP430_measuring = out & 0b00000001;
  
}

void stopMicro(){ 
    uint8_t out=spiCommand8( CDM_stop  , CS_MSP430 );
	  delay(1000)   ;  // Check that this is enought time fot the microcontroller stopping the measurements
	  uint8_t out2=spiCommand8( CDM_status , CS_MSP430 );
	  
    Serial.printf("Microcontroller status: %b",out2);
    Serial.println(" ");
    
	  // Parsing  response from microcontroller
	  
	  Serial.print("Send stop to Microcontroller!");
}

void statusMicro(){   
	  uint8_t out=spiCommand8( CDM_status   , CS_MSP430 );
	  
	  // Parsing  response from microcontroller
	  MSP430_measuring = out & 0b00000001 ;
	  
	  Serial.printf("Microcontroller status: %b",out);
}

void resetMicro(){
  uint8_t out=spiCommand8( CDM_reset  , CS_MSP430 );
	  delay(1000)   ;  // Check that this is enought time fot the microcontroller stopping the measurements
	  uint8_t out2=spiCommand8( CDM_status , CS_MSP430 );
	  
    Serial.printf("Microcontroller status: %b",out2);
    Serial.println(" ");
    
	  // Parsing  response from microcontroller
	  
	  Serial.print("Send reset to Microcontroller!");
}




// /////////////////////////////////////////////
// ---------------------------------------------
// SPI functions
// ---------------------------------------------
// /////////////////////////////////////////////

uint32_t spiCommand32( uint32_t msg , int CS ) {  
  spi.beginTransaction(SPISettings(SPI_rate, MSBFIRST, SPI_MODE0));
  digitalWrite(CS, LOW); //pull SS slow to prep other end for transfer
  uint32_t out= spi.transfer32(msg);
  digitalWrite(CS, HIGH); //pull ss high to signify end of data transfer
  spi.endTransaction();
  // Serial.printf("Command to SPI send with CS: %d",CS);
  return out;
}


uint8_t spiCommand8( uint8_t msg , int CS ) {  
  spi.beginTransaction(SPISettings(SPI_rate, MSBFIRST, SPI_MODE0));
  digitalWrite(CS, LOW); //pull SS slow to prep other end for transfer
  uint8_t out= spi.transfer(msg);
  digitalWrite(CS, HIGH); //pull ss high to signify end of data transfer
  spi.endTransaction();
  // Serial.printf("Command to SPI send with CS: %d",CS);
  return out;
}


void spiTransfer( uint8_t msg[], uint32_t size, int CS ) {  
  spi.beginTransaction(SPISettings(SPI_rate, MSBFIRST, SPI_MODE0));
  digitalWrite(CS, LOW); //pull SS slow to prep other end for transfer
  spi.transfer(msg,size);
  digitalWrite(CS, HIGH); //pull ss high to signify end of data transfer
  spi.endTransaction();
}


void spiTransfer2( uint8_t msg[], uint32_t size, int CS ) {  
  // Serial.print("start transfer");
  spi.beginTransaction(SPISettings(SPI_rate, MSBFIRST, SPI_MODE0));
  digitalWrite(CS, LOW); //pull SS slow to prep other end for transfer
  for (int i=0; i<size; ++i){
    spi.transfer(msg[i]);    
    // Serial.print(".");
  }
  digitalWrite(CS, HIGH); //pull ss high to signify end of data transfer
  spi.endTransaction();
  // Serial.print("stop transfer");
}

void writeMSG (uint16_t addr,uint16_t dat ) {
  uint32_t msg = (0x00 << 24) + (addr << 16) + dat;
  spiCommand32(  msg ,CS_DAC);
}

void writeReg (uint16_t addr,uint16_t dat) {
  writeMSG(  addr,dat);
  delay(1);
  writeMSG( 0x1D , 0x01 );
}


uint32_t readReg(uint16_t addr){
//   Serial.println("Reading register");
  //  Serial.printf("Reading register: CS_DAC: %d",CS_DAC);
   uint32_t msg = (0x80 << 24) + (addr << 16) + 0x0000;
   uint16_t out=spiCommand32(  msg ,CS_DAC);
   
   return out;
}






void startMeasuring() {
  BLE_message = true;
  
  startMicro();	



  strcpy(txString, "Started measurement!");
  Serial.println(txString);
}


// /////////////////////////////////////////////
// -%--------------------------------------------
// Setup and loop
// %%------------------------------------------------------------------------------------------------------------------------------
void setup() {

  Serial.begin(115200);



  // Setup SPI connection
  //--------------------
  spi.begin(SCK, MISO, MOSI, CS_SD);  
  Serial.println("Started SPI");
  pinMode(CS_SD, OUTPUT); //set up slave select pins as outputs as the Arduino API doesn't handle automatically pulling SS low
  digitalWrite(CS_SD, HIGH);
  pinMode(CS_DAC, OUTPUT); //set up slave select pins as outputs as the Arduino API doesn't handle automatically pulling SS low
  digitalWrite(CS_DAC, HIGH);
  pinMode(CS_MSP430 , OUTPUT); //set up slave select pins as outputs as the Arduino API doesn't handle automatically pulling SS low
  digitalWrite(CS_MSP430 , HIGH);

 
  BLE_message = true;
  strcpy(txString, "Setup completeded. Waiting for command 'START' over Serial of BLE for starting measuring data!");
 // Send_tx_String(txString);

}


void loop() {

  if (Serial.available()) {  // Check Serial inputs
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


  delay(50);
}

void parse( String rxValue){     //%toCheck
  //Start new data files if START is received and stop current data files if STOP is received
  BLE_message = true;
  if (rxValue.indexOf("START") != -1) {
	  if (measuring) {
    strcpy(txString, "Already measuring.");
    Serial.println(txString);
    return;
  }
  measuring = true;
  startMeasuring();
  
  } else if (rxValue.indexOf("STOP") != -1) {
	measuring = false;
	stop_Measuring();
  } else if (rxValue.indexOf("STATUS") != -1) {
	statusMicro();
	
  } else if(rxValue.indexOf("RESET") != -1){
    resetMicro();
  } else {
	BLE_message = true;
	strcpy(txString, "Input can not be parsed retry!");
	Serial.println(txString);
  }
}


void stop_Measuring() {
  measuring = false;  // Set flag to false


  stopMicro();
                                                                                    
  Serial.println("Measurement stopped successfully");
}