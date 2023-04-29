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
unsigned long lastPrint;  // Record when the last Serial print took place



// settings altimeter LSD70A
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
//PINs can be changed to any pin. // For Hardware Serial use Pin 16 an 17. SoftwareSerial worked on pins 12 and 27
int PIN_Rx = 17;  // 16 = Hardware RX pin,
int PIN_Tx = 16;  // 17 = Hardware TX pin,
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#define baudrateRS232  230400  //115200
#define Laser_BufferSize 2048  // Allocate 1024 Bytes of RAM for UART serial storage
#define Laser_log_intervall 2000


// settings INS IMX5
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
//PINs can be changed to any pin. // For Hardware Serial use Pin 16 an 17. SoftwareSerial worked on pins 12 and 27
int  IMX5_Rx = 26;  //  Hardware RX pin, to PIN10 on IMX5
int  IMX5_Tx =25;  //  Hardware TX pin, to PIN8 on IMX5
int  IMX5_strobe = 4; //  GPIO for STROBE input (first), to  PIN2 on IMX5 !!!!!!!!Reserved for radio but can be used meanwhile!!!!!!!!
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#define baudrateIMX5 230400  //115200
#define IMX5_BufferSize 2048  // Allocate 1024 Bytes of RAM for UART serial storage
#define IMX5_log_intervall 2000
#define STROBE_intervall 5000
// char asciiMessage[] = "$ASCB,512,,,200,,,,30000,,30000,,,";  // // Get PINS1 @ 100ms, 30s  on the connected serial port, leave all other broadcasts the same, and save persistent messages.
char asciiMessage[] = "$ASCB,512,,,6,,,,2000,,2000,,,";     // new IMX5 has different IMU data rate  (16 ms). I dont know why!! But it is ok.
char asciiMessageformatted[128];
int IMXrate=100;
int IMUdataRate=16;

long Year;
long Month;
long Day;
long Hour;
long Minute;
long Second;


// --------------------------------
// Settings I2C, Amp. Cswitches, Temp.
//---------------------------------
#include <Wire.h>
#include "Adafruit_MCP9808.h"

Adafruit_MCP9808 sensor = Adafruit_MCP9808();
Adafruit_MCP9808 tempsens1 = Adafruit_MCP9808();
Adafruit_MCP9808 tempsens2 = Adafruit_MCP9808();
Adafruit_MCP9808 tempsens3 = Adafruit_MCP9808();
Adafruit_MCP9808 tempsens4 = Adafruit_MCP9808();


// I2C addresses
#define  MA12070P_address 0x20
#define  CSwitchTx_address 0x24
#define  CSwitchCal_address 0x26
#define  Temp1_address 0x18  // 000 ; Tx, Tail
#define  Temp2_address 0x19  // 001 ;  Rx, Tip
#define  Temp3_address 0x1A  // 010 ;  Battery
#define  Temp4_address 0x1B  // 011


#define N_TempSens 2
int TempSens_addr[] ={Temp1_address,Temp2_address};
long Temp_intervall = 10000;  // logging rate for temperature in ms


uint8_t CSwitch =0 ;  // Controlling witch switch is open. 0 all a close.  1 for first, 2 for second,   4  for third. Sum for combinations. 
uint8_t  CSwitch_code =0xFF;

#define PIN_EN 27    // must be =1 at startup
#define PIN_MUTE 12  // must be =0 at startup




// Setting for BLE connection
//---------------------------------
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
char txString[500];  // String containing messages to be send to BLE terminal
char txString2[50];
char subString[32];
bool BLE_message = false;


// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"  // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLEname "LASSITOS BLE"






//-=-=-=-=-=-=-=-=-=-=-=-
// Function generation settings
//-=-=-=-=-=-=-=-=-=-=-=-
uint64_t clock_divider=1;
uint64_t AWG_clock_freq = 7372800; // 125000000; 
uint16_t freqAdd;
uint16_t freqDat;
float gain=0;
uint16_t gainDAT = 0x1000;


//resonant frequencies:
#define F1 720
#define F2 1155
#define F3 1920
#define F4 6110
#define F5 7037
#define F6 8448

#define F46 5130
#define F456 4049


uint64_t freqs[] ={F1,F2,F3,F456,F46,F4,F5,F6};
int Nfreq=8;  //Number of frequencies to use
int ifreq =2;
uint64_t freq=freqs[ifreq];


// states of CSwitch for each frequencz
#define stateF1 0x80
#define stateF2 0x40
#define stateF3 0x02
#define stateF4 0x01
#define stateF5 0x10
#define stateF6 0x08

uint8_t CSw_states[] ={stateF1,stateF2,stateF3,stateF4+stateF5+stateF6,stateF4+stateF6,stateF4,stateF5,stateF6};
uint8_t CSw_state=CSw_states[ifreq];
#define CALF1 1055
#define CALF2 4744
#define CALF3 8805
int CAL_states[]={0,1,2,4};
int CAL_state=0;

#define MAXGAIN 0x1800

//-=-=-=-=-=-=-=-=-=-=-=-





//-=-=-=-=-=-=-=-=-=-=-=-
// Microcontroller communication settings
//-=-=-=-=-=-=-=-=-=-=-=-
# define CDM_start 0x81
# define CDM_stop 0x82
# define CDM_status 0xFF
# define CDM_status_read 0x00
# define CDM_status_read32 0X00
# define CDM_reset 0x84							 

// status commands
#define TSTAT_CMD_OK        0x80
#define TSTAT_CALIBRATING   0x08
#define TSTAT_STARTING      0x04
#define TSTAT_SD_ERR        0x02
#define TSTAT_RUNNING       0x01


// status flags Micro
bool MSP430_CMD_OK =0;
bool MSP430_CALIBRATING =0;
bool MSP430_STARTING =0;
bool MSP430_SD_ERR =0;
bool MSP430_measuring =0;

// other flags
bool ADCstarted=0;  //flag checking if ADC responded to start command
bool SD_mounted =0;
bool SD_filecreated =0;



// /////////////////////////////////////////////
// ---------------------------------------------
// Communication to Microcontroller
// ---------------------------------------------
// /////////////////////////////////////////////


void startMicro(){
	uint8_t startMSG[13];
  char timestr[13];
	sprintf(timestr, "%02d%02d%02d%02d%02d", Year % 100, Month, Day, Hour, Minute); 
	startMSG[0]=CDM_start;
	for (int i=0;i<11;i++){
		startMSG[1+i] = uint8_t(timestr[i]);
		}

	startMSG[12]=0;
	spiTransfer2( startMSG, 12, CS_MSP430 );
}

void stopMicro(){ 
    spiCommand8( CDM_stop  , CS_MSP430 );
	  delay(500)   ;  // Check that this is enought time fot the microcontroller stopping the measurements
	  // spiCommand8( CDM_status   , CS_MSP430 );
    // Serial.print("Send stop to Microcontroller!");
    // delay(2);
    // statusMicro();
}

uint32_t statusMicro(){   
	spiCommand8( CDM_status   , CS_MSP430 );
  delay(10);
  spiCommand32( CDM_status_read32,  CS_MSP430);
	delay(10);
	spiCommand32( CDM_status_read32,  CS_MSP430);
	delay(10);
	return spiCommand32( CDM_status_read32,  CS_MSP430);
}

void statusMicro8(){   
	  spiCommand8( CDM_status   , CS_MSP430 );
    delay(1);
    uint8_t out=spiCommand8( CDM_status_read   , CS_MSP430 );
	  ParseStatus(out);
}

void statusMicroLong(){   
	  spiCommand8( CDM_status   , CS_MSP430 );
    delay(10);
    Serial.print("Send long STATUS request");
    uint32_t out_long =spiCommand32( CDM_status_read32,  CS_MSP430);
	  Serial.printf("%04X",out_long);
    Serial.println(" ");
	  Serial.print("BIN: ");
    Serial.println(out_long,BIN);
	  
}



void ParseStatus(uint8_t out){   	
	// Parsing  response from microcontroller
	if ((out& TSTAT_CMD_OK )==TSTAT_CMD_OK ){
		MSP430_CMD_OK =1;
		
		if((out& TSTAT_RUNNING)==TSTAT_RUNNING){
			MSP430_measuring = 1;
		}else if((out& TSTAT_SD_ERR)==TSTAT_SD_ERR){
			MSP430_SD_ERR = 1;
			Serial.print("Microcontroller SD not running!");
		}else if((out& TSTAT_CALIBRATING)==TSTAT_CALIBRATING){
			MSP430_CALIBRATING = 1;
			Serial.print("Microcontroller is calibrating!");
		}else if((out& TSTAT_STARTING)==TSTAT_STARTING){
			MSP430_STARTING = 1;
			Serial.print("Microcontroller is starting!");
		}

	} else {
		MSP430_CMD_OK =0;
		MSP430_measuring  =0;
    Serial.print("Microcontroller got wrong message");
	}

}


void resetMicro(){
  spiCommand8( CDM_reset  , CS_MSP430 );
	delay(500)   ;  // Check that this is enought time fot the microcontroller stopping the measurements
	// Serial.print("Send reset to Microcontroller!");
	// spiCommand8( CDM_status , CS_MSP430 );
	// delay(2);
	// uint8_t out=spiCommand8( CDM_status_read   , CS_MSP430 );
  //   Serial.printf("Micro stat: %02X",out);
  //   Serial.println(" ");
}



// /////////////////////////////////////////////
// ---------------------------------------------
// Signal generation: running and test functions
// ---------------------------------------------
// /////////////////////////////////////////////






// /////////////////////////////////////////////
// ---------------------------------------------
// SignalGen functions
// ---------------------------------------------
// /////////////////////////////////////////////
void configureResFreq(int ifreq){   //set resonant frequency from set or possible frequencies.
  freq=freqs[ifreq];
  CSw_state=CSw_states[ifreq];
  configureSineWave();
  setCswitchTx ( CSw_state );
}


void configureSineWave(){

  // Prepare the parameters:
  uint64_t temp=freq*0x1000000*clock_divider;
  uint64_t freqTW=temp/AWG_clock_freq;  // get frequency tuning word 0x1000000=2**24
  uint16_t freqMSB=freqTW>>8;
  uint16_t freqLSB=(freqTW&0xFF)<<8;

  
  writeReg(0x27,0x0031);  //Sinewave Mode channel 1
  // writeReg(0x27,0x3131);  //Sinewave Mode, channel 1 and 2
  writeReg(0x26,0x0031);  //Sinewave Mode channel 3
//  
  writeReg(0x45,0x0000); //Static phase/freq
  writeReg(0x3E,freqMSB); //Freq MSB
  writeReg(0x3F,freqLSB); //Freq LSB

//  gainDAT=int((gain+2)*4095/4) << 4;
  writeReg(0x35,gainDAT); //digital gain Ch1
  writeReg(0x34,gainDAT); //digital gain Ch2
  writeReg(0x33,gainDAT); //digital gain Ch3
}

void run(){
  // out=readReg(0x1E);
  // Serial.print("Current run mode:");
  // Serial.println(out,BIN);
  writeReg(0x1E,0x0001);
  delay(1);
  trigger();
  out=readReg(0x1E);
  Serial.print("New run mode:");
  Serial.println(out,BIN);
  if ((out & 0x3)!=0x03){
	ADCstarted=0;
	Serial.print("DAC didn't start correctly!");
  }else{
	ADCstarted=1;  
	Serial.print("DAC started correctly!");
  }
}

void run2(){
  writeReg(0x1E,0x0001);
  trigger();
  out=readReg(0x1E);
  if ((out & 0x3)!=0x03){
	ADCstarted=0;
  }else{
	ADCstarted=1;  
  }
}


void trigger(){
  // Serial.println("[AWG] Triggerring");
  digitalWrite(triggerDAC,HIGH);
  delay(1);
  digitalWrite(triggerDAC,LOW);
}

void stop_trigger(){
  // Serial.println("Stop triggerring");
  digitalWrite(triggerDAC,LOW);
  delay(1);
  digitalWrite(triggerDAC,HIGH);
  writeReg(0x1E,0x0000);
}

void setGain2(int value){
//  gainDAT=int((gain+2)*4095/4) << 4;
  gainDAT=value;
  writeReg(0x35,gainDAT); //digital gain ch1
  writeReg(0x34,gainDAT); //digital gain Ch2
  writeReg(0x33,gainDAT); //digital gain Ch3  
}



// /////////////////////////////////////////////
// --------------------------------
// other functions
//---------------------------------
// /////////////////////////////////////////////
float readVBat(){
	int VBat_int= analogRead(PIN_VBat);
	VBat =float(VBat_int)/4095*3.2*10;
	// Serial.printf("Voltage battery GPIO value: %d \n", VBat_int);
  // Serial.printf("Voltage battery: %05.2f V\n", VBat);
	return VBat;
}
void logVBat(){
    readVBat();
	  sprintf(subString,"VBat %05.2f\n",VBat );
	  int msglen=11;
    for (int i = 0; i < msglen; i++){
      tempBuffer[i] = uint8_t(subString[i]);
    } 
    writeToBuffer(tempBuffer, msglen);
}


void logTemp(Adafruit_MCP9808 sensor, int  sensorNumber){
  float c = sensor.readTempC();
  // Serial.printf("Temp%d: %.4f* C\n",sensorNumber,c); 
  sprintf(subString,"Temp%d %07.4f\n",sensorNumber,c );
  int msglen=strlen(subString);
  for (int i = 0; i < msglen; i++){
      tempBuffer[i] = uint8_t(subString[i]);
    } 
    writeToBuffer(tempBuffer, msglen);
}

void readTemp(Adafruit_MCP9808 sensor, int  sensorNumber){
  float c = sensor.readTempC();
  sprintf(txString,"Temperature: %.4f* C\n",c);
  Serial.println(txString);
  
}

// void logTemp0(){
//   float c = tempsens1.readTempC();
//   Serial.printf("Temp1: %.4f* C\n",c); 

//   sprintf(subString,"Temp1 %07.4f\n",-c );
//   int msglen=strlen(subString);
//   for (int i = 0; i < msglen; i++){
//       tempBuffer[i] = uint8_t(subString[i]);
//     } 
//     writeToBuffer(tempBuffer, msglen);
// }


// void logTemp2(int  sensorNumber){
//   int sensorAdderess = TempSens_addr[sensorNumber];
//   sensor.begin(sensorAdderess);
//   float c = sensor.readTempC();
//   Serial.printf("Temp%d: %.4f* C\n",sensorNumber,c); 
//   sprintf(subString,"Temp%d %07.4f\n",sensorNumber,c );
//   int msglen=strlen(subString);
//   for (int i = 0; i < msglen; i++){
//       tempBuffer[i] = uint8_t(subString[i]);
//     } 
//     writeToBuffer(tempBuffer, msglen);
// }

// /////////////////////////////////////////////
// --------------------------------
// I2C communication
//---------------------------------
// /////////////////////////////////////////////
void setCswitchTx ( uint8_t state ){

   CSwitch_code = 0xFF-state;
	 //Write message to preiferal
	 Serial.printf("New state is: %d\n", state);
	 Serial.printf("Setting Cswitch #: 0X%x to: 0X%x\n", CSwitchTx_address,CSwitch_code);
	 Wire.beginTransmission(CSwitchTx_address); // For writing last bit is set to 0 (set to 1 for reading)
	 Wire.write(CSwitch_code );
	 uint8_t error = Wire.endTransmission(true);
	 Serial.printf("endTransmission: %u\n", error);
}


void setCswitchCal ( uint8_t state ){

	CSwitch_code = 0xFF-state;

	 //Write message to the slave
	 Serial.printf("New state is: %d\n", state);
	 Serial.printf("Setting Cswitch Cal #: 0X%x to: 0X%x\n", CSwitchCal_address,CSwitch_code);
	 Wire.beginTransmission(CSwitchCal_address); // For writing last bit is set to 0 (set to 1 for reading)
	 Wire.write(CSwitch_code );
	 uint8_t error = Wire.endTransmission(true);
	 Serial.printf("endTransmission: %u\n", error);
}


void write_I2C (uint8_t device, uint8_t address, uint8_t msg ){
  //Write message to the slave
  Serial.printf("Writing to reg: %x, msg: %x\n", address,msg);
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(msg);
  uint8_t error = Wire.endTransmission(true);
  Serial.printf("endTransmission: %u\n", error);
}


uint8_t readRegI2C (uint8_t device, uint8_t address){
  uint8_t bytesReceived=0;
  int temp=0;
  Serial.printf("Reading reg: %#02X\n", address);
  Wire.beginTransmission(device);
  Wire.write(address);
  uint8_t error = Wire.endTransmission(false);
  Serial.printf("endTransmission error: %u\n", error);
  //Read 16 bytes from the slave
  bytesReceived = Wire.requestFrom(device, 1);
  temp = Wire.read();
  Serial.printf("bytes received: %d\n", bytesReceived);
  Serial.printf("value: %x\n", temp);
  Serial.printf("value in decimal: %d\n", temp);
  return  temp;
}

void scanI2C (){
  byte error, address;
  int nDevices = 0;

  delay(5000);

  Serial.println("Scanning for I2C devices ...");
  for(address = 0x01; address < 0x7f; address++){
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0){
      Serial.printf("I2C device found at address 0x%02X\n", address);
      nDevices++;
    } else if(error != 2){
      Serial.printf("Error %d at address 0x%02X\n", error, address);
    }
  }
  if (nDevices == 0){
    Serial.println("No I2C devices found");
  }
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


void spiTransfer( uint8_t msg[], uint32_t size, int CS, uint8_t out[] ) {  
  spi.beginTransaction(SPISettings(SPI_rate, MSBFIRST, SPI_MODE0));
  digitalWrite(CS, LOW); //pull SS slow to prep other end for transfer
  for (int i=0; i<size; ++i){
    out[i]=spi.transfer(msg[i]);    
    // Serial.print(".");
  }
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



// /////////////////////////////////////////////
// ---------------------------------------------
//INS code
// ---------------------------------------------
// /////////////////////////////////////////////


// add checksum to asciiMessage
int FormatAsciiMessage(char *Message, int messageLength, char *outMessage) {
  int checkSum = 0;
  unsigned int ptr = 0;
  char buf[16];
  outMessage[0] = '\0';

  if (Message[0] == '$') {
    ptr++;
  } else {
    strcpy(outMessage, "$");
  }

  // concatenate Message to outMessage
  strcat(outMessage, Message);

  // compute checksum
  while (ptr < messageLength) {
    checkSum ^= Message[ptr];
    ptr++;
  }
  // Serial.print("Size of message: ");
  // Serial.println(messageLength);
  sprintf(buf, "*%.2x\r\n", checkSum);
  strcat(outMessage, buf);
  return sizeof(outMessage);
}


// setting up INS
void setupINS() {
  if (!FormatAsciiMessage(asciiMessage, sizeof(asciiMessage), asciiMessageformatted)) {
    Serial.println("Failed to encode ASCII get INS message\r\n");
  } else {
    Serial.println(asciiMessageformatted);
    IMX5.write(asciiMessageformatted);  //send instruction for sending ASCII messages
    Serial.println("Send instruction for sending ASCII messages to IMX5");
  }
  while (IMX5.read() >= 0)
    ;
  Serial.println(F("INS setting updated"));
  strcat(txString, "INS setting updated");

  pulseStrobeIMX5();  
  lastTime_STROBE = millis();
}




void getDateTime() {
  char msgOut[256];
  char msgStart[] = "$ASCB,0,,,,,,,,,,,200,";
  FormatAsciiMessage(msgStart, sizeof(msgStart), msgOut);
  int out = 0;
  String msg = "NotValidMessage";
  IMX5.write("$STPC*14\r\n");
  Serial.println(msgOut);
  IMX5.write(msgOut);
  lastTime = millis();
  while (!out) {
    if (IMX5.available()) {  // Check Serial inputs
      // String msg = Serial.readString();
      bitesToWrite = IMX5.available();
      if (tempBufferSize < bitesToWrite) {
        bitesToWrite = tempBufferSize;
      }
      IMX5.readBytes(tempBuffer, bitesToWrite);
      String msg = (char*)tempBuffer;
      // Serial.println(msg);
      out = parseDateTime(msg);
    }
    if (millis() - lastTime > 5000) {
      Serial.print(F("No GPS. Setting date to default and random time!"));
      // out = parseDateTime("$GPZDA,001924,06,01,1980,00,00*41");
      out = parseDateTime("NotAValidMessage");
      out = 1;
    }
    delay(50);
  }
  IMX5.write("$STPC*14\r\n");
  char msgStop[] = "$ASCB,0,,,,,,,,,,,0,";
  FormatAsciiMessage(msgStop, sizeof(msgStop), msgOut);
  Serial.println(msgOut);
  IMX5.write(msgOut);

  Serial.print(F("Current date:"));
  strcpy(txString, printDateTime().c_str());
  Serial.println(txString);
}

int parseDateTime(String GPDZA) {
  //$GPZDA,001924,06,01,1980,00,00*41\r\n
  //$GPZDA,032521,08,02,2023,00,00*46   
  int a=GPDZA.indexOf("$GPZDA");
  int b=GPDZA.indexOf("*");
  if (a != -1 and b!= -1 and a<b ) {
    Serial.print("Valid msg:");
    Serial.println(GPDZA.substring(a, b+3));
    int index[6];
    index[0] = a + 7;
    // Serial.print(index[0]);
    for (int i = 0; i < 5; i++) {
      index[i + 1] = GPDZA.indexOf(',', index[i]) + 1;
      // Serial.print(i+1);
      // Serial.println(index[i+1]);
    }
    // Serial.println(GPDZA.substring(index[1],index[2]-1));
    Day = GPDZA.substring(index[1], index[2] - 1).toInt();
    // Serial.print(Year);
    // Serial.println(GPDZA.substring(index[2],index[3]-1));
    Month = GPDZA.substring(index[2], index[3] - 1).toInt();
    // Serial.print(Month);
    // Serial.println(GPDZA.substring(index[3],index[4]-1));
    Year = GPDZA.substring(index[3], index[4] - 1).toInt();
    // Serial.print(Day);
    // Serial.println(GPDZA.substring(index[0],index[0]+2));
    Hour = GPDZA.substring(index[0], index[0] + 2).toInt();
    // Serial.print(Hour);
    // Serial.println(GPDZA.substring(index[0]+2,index[0]+4));
    Minute = GPDZA.substring(index[0] + 2, index[0] + 4).toInt();
    // Serial.print(Minute);
    // Serial.println(GPDZA.substring(index[0]+4,index[0]+6));
    Second = GPDZA.substring(index[0] + 4, index[0] + 6).toInt();
    // Serial.print(Second);
    // Serial.println("Date parsed");
    return 1;
  } else {
    Year = 2000;
    Month = 01;
    Day = 01;
    Hour = random(23);
    Minute = random(60);
    Second = random(60);
    return 0;
  }
}


String printDateTime() {
  sprintf(subString, "%d-%d-%d %d:%d:%d", Year, Month, Day, Hour, Minute, Second);
  Serial.println(subString);
  return subString;
}


void setIMX5message(){
	int IMXrate2=int( round(IMXrate/IMUdataRate));
	sprintf(asciiMessage, "$ASCB,512,,,%d,,,,,,,,,",IMXrate2);
}


void pulseStrobeIMX5(){
 digitalWrite(IMX5_strobe, HIGH);
 delay(1);
 digitalWrite(IMX5_strobe, LOW);
}



// /////////////////////////////////////////////
// ---------------------------------------------
// SD card funtions und code
// ---------------------------------------------
// /////////////////////////////////////////////

void listDir(fs::FS &fs, const char *dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels) {
        listDir(fs, file.path(), levels - 1);
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

void createDir(fs::FS &fs, const char *path) {
  Serial.printf("Creating Dir: %s\n", path);
  if (fs.mkdir(path)) {
    Serial.println("Dir created");
  } else {
    Serial.println("mkdir failed");
  }
}

void removeDir(fs::FS &fs, const char *path) {
  Serial.printf("Removing Dir: %s\n", path);
  if (fs.rmdir(path)) {
    Serial.println("Dir removed");
  } else {
    Serial.println("rmdir failed");
  }
}

void readFile(fs::FS &fs, const char *path) {
  Serial.printf("## Reading file: %s\n", path);
  int nmax=50000;
  File file = fs.open(path);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  //    Serial.println("Read from file: ");
  while (file.available()) {
    Serial.write(file.read());
	nmax--;
	if (nmax==0){
		Serial.println("####### maximum number of characters has been written!");
		break;
	}
  }
  file.close();
}

void writeFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

void renameFile(fs::FS &fs, const char *path1, const char *path2) {
  Serial.printf("Renaming file %s to %s\n", path1, path2);
  if (fs.rename(path1, path2)) {
    Serial.println("File renamed");
  } else {
    Serial.println("Rename failed");
  }
}

void deleteFile(fs::FS &fs, const char *path) {
  Serial.printf("Deleting file: %s\n", path);
  if (fs.remove(path)) {
    Serial.println("File deleted");
  } else {
    Serial.println("Delete failed");
  }
}




void Write_header() {
  dataFile.println(F("#LASSITOS INS and Laser altimeter log file"));
  dataFile.print("# Date: ");
  dataFile.print(Year);
  dataFile.print("-");
  dataFile.print(Month);
  dataFile.print("-");
  dataFile.println(Day);
  dataFile.print("# Time: ");
  dataFile.print(Hour);
  dataFile.print(":");
  dataFile.print(Minute);
  dataFile.print(":");
  dataFile.println(Second);
  dataFile.print(F("# Script version: "));
  dataFile.println(Version);
  dataFile.print(F("# IMX5 NMEA settings: "));
  dataFile.println(asciiMessage);
  dataFile.println(F("#Signal generation settings: "));
  dataFile.println(F("# --------------------------"));
  dataFile.printf("# Frequency: %d",freq);
  dataFile.println("#");
}


void log_Laser_settings() {
  dataFile.println(F("#Laser settings: "));
  dataFile.println(F("# --------------"));
  // for (int i = 0; i < 3; i++) {
  //   RS232.write(0x1B);
  //   delay(50);
  // }
  Serial.println("Reading Laser setting:");
  RS232.write(0x1B);
  delay(250);
  RS232.write(0x0D);
  delay(100);
  while (RS232.read() >= 0)
    ;  // flush the receive buffer.
  uint8_t *Buffer_lasersetting = new uint8_t[4096];
  unsigned long startTime = millis();
  bitesToWrite=0;
  RS232.write("ID");
  RS232.write(0x0D);
  delay(100);
  RS232.write("PA");
  RS232.write(0x0D);
  delay(100);
  while ((millis() -startTime) < 500) {
    while (RS232.available()){
      Buffer_lasersetting[bitesToWrite]=RS232.read();
      bitesToWrite++;
    }
    delay(5);
  }
  if( Buffer_lasersetting>0){
    dataFile.write(Buffer_lasersetting, bitesToWrite);
    Serial.write(Buffer_lasersetting, bitesToWrite);
  } else{
    Serial.println("No data from Laser");
  }
  delete[] Buffer_lasersetting;
  dataFile.println(F("# --------------------------------------"));
  dataFile.println(F(" "));
}



// Make new files in SD card. file names are derived from GNSS time if available.
void makeFiles(fs::FS &fs) {
  Serial.println(F("Making new files"));

  // Check for GPS to have good time and date
  getDateTime();
  sprintf(dataFileName, "/INS%02d%02d%02d_%02d%02d.csv", Year % 100, Month, Day, Hour, Minute);  // create name of data file
                                                                                                  //sprintf(dataFileName, "/testfile.csv");   // create name of data file
  Serial.println(dataFileName);
  dataFile = fs.open(dataFileName, FILE_WRITE);
  if (!dataFile) {
    Serial.println(F("Failed to create data file! "));
    Serial.println(dataFileName);
    SD_filecreated=0;
  } else{
	SD_filecreated=1;
  }
  
  delay(100);
  Write_header();
  delay(100);
  log_Laser_settings();
  Serial.println("Logged Laser settings");
  delay(100);

  strcpy(txString, "");
  strcat(txString, "Created file: ");
  strcat(txString, dataFileName);
  Send_tx_String(txString);
}




void Write_stop() {
  dataFile.println(F("#STOP"));
  dataFile.println(F("#--------------------------------------"));
}




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

  // Serial.print("Data in Buffer:");
  // Serial.println((myBufferSize + BufferTail - BufferHead) % myBufferSize);
}






void startMeasuring() {
  BLE_message = true;
  
  // Configure DAC
  //-----------
  stop_trigger();
  delay(10);
  configureResFreq(ifreq);
  setCswitchCal(0);
  delay(10);
  digitalWrite(PIN_MUTE , HIGH);  // unmute
  
  
  makeFiles(SD);
  delay(500);

  lastPrint = millis();      // Initialize lastPrint
  logTime_laser = millis();  // logTime_laser
  logTime_IMX5 = logTime_laser;
  BufferTail = 0;
  BufferHead = 0;

  // setting up GPS for automatic messages
  setupINS();
  delay(50);
  
  // Write start of data to datafile
  dataFile.println(F("###Data###"));  
  
  //start Laser measuremens
  RS232.write("DT");
  RS232.write(0x0D);
  delay(200);
	
	
  while (RS232.read() >= 0)
    ;  // flush the receive buffer.
  // while (IMX5.read() >= 0)
    ;  // flush the receive buffer.
  while (IMX5.available() > 0){
    char k = IMX5.read();
  }
  
  char infoMsg[] = "$INFO";
  FormatAsciiMessage(infoMsg, sizeof(infoMsg), asciiMessageformatted);
  IMX5.write(asciiMessageformatted);  //send instruction for sending ASCII messages

  
  
  //start Microcontroller data logger
  //----------------------------------
  resetMicro();
  delay(200) ;
  startMicro();	  
  delay(4000)   ;  // Wait until microcontroller started the measurement
  uint32_t out = statusMicro();
  Serial.print("Microcontroller status: 0x");
  Serial.printf("%02X, ",out);
	Serial.print("BIN: ");
  Serial.println(out,BIN);
  ParseStatus( (out & 0xFF000000)>>24);

  if(!MSP430_measuring){
        strcpy(txString, "Measurement on Microcontroller not started!");
    if (MSP430_SD_ERR){
      strcat(txString, " SD not working");
	  }
    Serial.println(txString);  
    // measuring = false;  
    // return;
  }
  

  // Start DAC
  //-----------
  run();

  strcpy(txString, "Started measurement!");
  Serial.println(txString);
}




// Stop data measuring process
void stop_Measuring(fs::FS &fs) {
  measuring = false;  // Set flag to false
  BLE_message = true;
  Serial.println("Turning dataMeasuring OFF!");
  
  //Stop DAC
  stop_trigger();
  digitalWrite(PIN_MUTE , LOW);  // mute
  
  //Stop Micro
  delay(1000);
  stopMicro();
  
  
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
  Serial.println("Measurement stopped successfully");
}



// /////////////////////////////////////////////
// ---------------------------------------------
// Input parsing functions    %toCheck
// ---------------------------------------------
// /////////////////////////////////////////////
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
	  stop_Measuring(SD);
	  
  } else if (rxValue.indexOf("STATUS") != -1) {
	  statusMicro();

	} else if (rxValue.indexOf("STATLONG") != -1) {
	  statusMicroLong();	

	} else if (rxValue.indexOf("STRMICRO") != -1) {
	  startMicro();	

  } else if (rxValue.indexOf("STPMICRO") != -1) {
	  stopMicro();	
  		   
  } else if(rxValue.indexOf("RESET") != -1){
   ESP.restart();
	
  } else if(rxValue.indexOf("RESMICRO") != -1){
    resetMicro();
  
	
  } else if (rxValue.indexOf("READ") != -1) {
	readFiles(rxValue);
  } else if (rxValue.indexOf("LIST") != -1) {
	getFileList();
  } else if (rxValue.indexOf("COMS") != -1 or rxValue.indexOf("?") != -1) {
	commands();
  } else if (rxValue.indexOf("CHECKLASER") != -1) {
	check_laser();
  } else if (rxValue.indexOf("CHECKINS") != -1) {
	check_INS();
  } else if (rxValue.indexOf("INSRATE") != -1) {
	  Serial.println("Setting new frequency value! ");
    int index = rxValue.indexOf(":");
    int index2 = rxValue.indexOf(":",index+1);
    if (index !=-1 and index2 !=-1){
		if (!measuring) {
		  IMXrate=rxValue.substring(index+1,index2).toInt();
		  sprintf(txString,"New rate is: %d ms",IMXrate );
		  Serial.println(txString);
		  setIMX5message(); //update ASCII setting message
		  setupINS(); //send setting message to IMX5
		  } else {
			sprintf(txString,"Data measuring is running. Can't change IMX rate now. First stop measurment!");
			Serial.println(txString);
		}
    } else {
      sprintf(txString,"INS frequency (ms) can not be parsed from string '%s''",rxValue);
      Serial.println(txString);
    }
	
	
  } else if (rxValue.indexOf("SETRESFREQ") != -1) {
	  Serial.println("Setting new frequency value! ");
    int index = rxValue.indexOf(":");\
    int index2 = rxValue.indexOf(":",index+1);
    if (index !=-1 and index2 !=-1){
      ifreq=rxValue.substring(index+1,index2).toInt();
      if (ifreq<Nfreq and ifreq>=0 ){
		  stop_trigger();
		  configureResFreq(ifreq);
		  sprintf(txString,"New resonant frequency is: %d",freqs[ifreq]);
		  Serial.println(txString);
	  }else {
		  sprintf(txString,"Error! Pass a valid index for resonant frequency between 0 and '%d''", Nfreq-1);
	  }
    } else {
      sprintf(txString,"Resonant frequency can not be parsed from string '%s''",rxValue);
      Serial.println(txString);
    }
	
	
  } else if (rxValue.indexOf("SETFREQ") != -1) {
	  Serial.println("Setting new frequency value! ");
    int index = rxValue.indexOf(":");\
    int index2 = rxValue.indexOf(":",index+1);
    if (index !=-1 and index2 !=-1){
      freq=rxValue.substring(index+1,index2).toInt();
      configureSineWave();
      sprintf(txString,"New frequency is: %d",freq );
      Serial.println(txString);
    } else {
      sprintf(txString,"Frequency can not be parsed from string '%s''",rxValue);
      Serial.println(txString);
    }
  } else if (rxValue.indexOf("SETGAIN2") != -1) {
      Serial.println("Setting new digital gain value! ");
      int index = rxValue.indexOf(":");\
      int index2 = rxValue.indexOf(":",index+1);
      if (index !=-1 and index2 !=-1){
        rxValue.substring(index+1,index2).toCharArray(datStr ,index2-index+1);
        setGain2(strtoul (datStr, NULL, 16));
        Serial.println(datStr);
        sprintf(txString,"New gain tuning word is: %04X",gainDAT );
        Serial.println(txString);
        out=readReg(0x35);
        Serial.print("New gain:");
        Serial.println(out,BIN);
      } else {
        sprintf(txString,"Gain can not be parsed from string '%s''",rxValue);
        Serial.println(txString);
    }  
	
  } else if (rxValue.indexOf("SCANI2C") != -1) {
      Serial.println("Scanning I2C devices");
      scanI2C ();
	
  } else if (rxValue.indexOf("VBAT") != -1) {
      BLE_message = true;
      VBat=readVBat();
      sprintf(txString,"Voltage battery: %05.2f V\n", VBat);
      Serial.println(txString);
    
  } else if (rxValue.indexOf("TEMP") != -1) {
      BLE_message = true;
      float c = tempsens1.readTempC();
      sprintf(txString,"Temperature: %.4f* C\n",c);
      Serial.println(txString);

  } else if (rxValue.indexOf("STROBE") != -1) {
      Serial.println("strobe pulse");
      pulseStrobeIMX5(); 
	
  } else if (rxValue.substring(0,4)== "SPIR"  and rxValue.charAt(8)== 'R') {
      rxValue.substring(4,8).toCharArray(addrStr,8);
      addr=strtoul (addrStr, NULL, 16);
      Serial.print("Reading register: ");
  //          Serial.println(addr, HEX);
      out=readReg(addr);
      sprintf(txString2,"Addr:%#02X Data:",addr);
      Serial.print(txString2);
      Serial.println(out,BIN);

	  
  // Write to SPI register    
  } else if (rxValue.indexOf("SPIW") != -1  and (rxValue.charAt(8) == 'X' or rxValue.charAt(8) == 'B')) {
	  if(rxValue.charAt(8) == 'X'){
		  rxValue.substring(9,15).toCharArray(datStr,7);
		  Serial.print(datStr);
		  dat=strtoul (datStr, NULL, 16);
		
	  }else if(rxValue.charAt(8)== 'B'){
		   rxValue.substring(9,25).toCharArray(datStr,17);
//               Serial.print(datStr);
		   dat=strtoul (datStr, NULL, 2);
	  }
	  rxValue.substring(4,8).toCharArray(addrStr,5);
	  addr=strtoul (addrStr, NULL, 16);
	  
//          Serial.print("Writing register: ");
//          Serial.print(addr, HEX);
//          Serial.print(", data: ");
//          Serial.println(dat, HEX);
	  sprintf(txString2,"Writing register:%#02X Data:%#04X",addr);
	  Serial.print(txString2);
	  Serial.println(dat,BIN);
	  writeReg(addr,dat);
	
	
  } else {
	BLE_message = true;
	strcpy(txString, "Input can not be parsed retry!");
	Serial.println(txString);
  }
}


	



void readFiles(String rxValue) {
  /*
    Read last files (haderfile and datafile) if no argument is passed  ("READ" ) otherwise read passed files (READ:<<filepath>>:).
    */
  if (!measuring) {
    //    Serial.println(rxValue);
    int index = rxValue.indexOf(":");
    int index2 = rxValue.indexOf(":", index + 1);
    //    Serial.println(index );
    //    Serial.println(index2);

    if (index != -1 and index2 != -1) {
      char path[32];
      rxValue.substring(index + 1, index2).toCharArray(path, 32);
      Serial.println("");
      //      Serial.print("%% Reading file:");
      //      Serial.println(path);
      readFile(SD, path);
      Serial.println(" \n## End of file");

    } else if (index = -1) {
      Serial.print("Reading file:");
      Serial.println(dataFileName);
      readFile(SD, dataFileName);

    } else {
      BLE_message = true;
      sprintf(txString, "File can not be parsed form string '%s'. Valid format is 'READ' or 'READ:<<filepath>>:' !", rxValue);
      Serial.println(txString);
    }
  } else {
    BLE_message = true;
    strcpy(txString, "Data Measuring running. Can't read files now. First stop measurment!");
    Serial.println(txString);
  }
  Send_tx_String(txString);
}

void getFileList() {
  /*
    Return list of files in SD card
    */
  if (!measuring) {
    Serial.println("");
    Serial.println("%% File list:");
    listDir(SD, "/", 0);
    Serial.println("%% end");

  } else {
    Serial.println("Data Measuring running. Can't read file list now. First stop measurment!");
  }
}




// Get Laser data for 1 second and send it over BLE and serial
void check_laser() {
  Send_tx_String(txString);
  strcpy(txString, "");
  if (!measuring) {
    strcat(txString, "\nLaser data \n# --------------------------------------\n");
    Send_tx_String(txString);
	strcpy(txString, "");
    lastTime = millis();
    while (RS232.read() >= 0)
      ;  // flush the receive buffer.
    while (millis() - lastTime < 1000)
      ;
    int availableBytes = RS232.available();
    if (LaserBufferSize < availableBytes) {
        availableBytes = LaserBufferSize;
		    strcat(txString, "# Buffer full. Data were cut.\n");
    }
	
	
	RS232.readBytes(myBuffer_laser, availableBytes);
    Serial.write(myBuffer_laser, availableBytes);
    if (deviceConnected) {
      pTxCharacteristic->setValue(myBuffer_laser, availableBytes);
      pTxCharacteristic->notify();
      BLE_message = false;
    }
  strcat(txString, "# --------------------------------------\n");
  Send_tx_String(txString);
  } else {
    strcpy(txString, "Datalogging running. Can't run LASER_check now!");
  }
  Send_tx_String(txString);
}

// Get Laser data for 1 second and send it over BLE and serial
void check_INS() {
  Send_tx_String(txString);
  strcpy(txString, "");
  if (!measuring) {
    strcat(txString, "\nIMX5 data \n# --------------------------------------\n");
    Send_tx_String(txString);
	strcpy(txString, "");
    while (IMX5.read() >= 0)
      ;  // flush the receive buffer.
    
    delay(1000);
    int availableBytes = IMX5.available();
	
	if (LaserBufferSize < availableBytes) {
        availableBytes = LaserBufferSize;
		strcat(txString, "# Buffer full. Data were cut.\n");
    }
	
    IMX5.readBytes(myBuffer_laser, availableBytes);
    Serial.write(myBuffer_laser, availableBytes);
    if (deviceConnected) {
      pTxCharacteristic->setValue(myBuffer_laser, availableBytes);
      pTxCharacteristic->notify();
      BLE_message = false;
    }
    
    strcat(txString, "# --------------------------------------\n");
    Send_tx_String(txString);
  } else {
    strcpy(txString, "Datalogging running. Can't run INS_check now!");
  }
  Send_tx_String(txString);
}


// Print commands  %toCheck:  Need to make new list of commands
void commands() {
  Send_tx_String(txString);
  strcpy(txString, "");
  strcat(txString, "\nCommands \n# --------------------------------------\n");
  strcat(txString, "\n Serial only\n# .................\n");
  strcat(txString, "READ Read last file of last measurement. If argument is passed with READ:<< nfile>>: read file nfile \nLIST  Get list of files in SD card \n ");
  Send_tx_String(txString);
  delay(100);
  strcat(txString, "\n Serial and BLE:\n# .................\n");
  strcat(txString, "STOP  Stops measurement \nSTART Starts new measurement  \n" );
  strcat(txString, "CHECKLASER  Get Laser data for 1 second and send it over BLE and serial \n" );
  strcat(txString, "COMS  List commands \n");
  strcat(txString, "CHECKIMU  Get IMU data and send them over BLE+serial for manual check \n");
  
  strcat(txString, "# --------------------------------------\n");
  Send_tx_String(txString);
  delay(100);
  strcpy(txString, "");
}



// /////////////////////////////////////////////
// ---------------------------------------------
// BLE funtions und code
// ---------------------------------------------
// /////////////////////////////////////////////

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
  }
};

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();

    if (rxValue.length() > 0) {
		String rxValue2=rxValue.c_str();
		// for (int i = 0; i < rxValue.length(); i++)
		// rxValue2 += rxValue[i];

		Serial.println("*********");
		Serial.println("Received Value: ");
		Serial.println("*********");
		BLE_message=true;
		sprintf(subString,"BLE input: %s\n",rxValue2);
		strcat(txString,subString);
		parse(rxValue2);
    }
  }
};



void setup_BLE() {
  Serial.printf("bluetooth connection: %s \n",BLEname);
  // Create the BLE Device
  BLEDevice::init(BLEname);

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    BLECharacteristic::PROPERTY_NOTIFY);

  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    BLECharacteristic::PROPERTY_WRITE);

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}

// Send txSTring to BLE and Serial
void Send_tx_String(char *txString) {
  // strcat(txString,"\n");
  Serial.print(txString);

  if (deviceConnected) {
    pTxCharacteristic->setValue(txString);
    pTxCharacteristic->notify();
    BLE_message = false;
  }

  strcpy(txString, "");
}



// /////////////////////////////////////////////
// -%--------------------------------------------
// Setup and loop
// %%------------------------------------------------------------------------------------------------------------------------------
void setup() {

  Serial.begin(115200);

	//Mute and Enable PINs Amplifier MA12070 
	pinMode(PIN_EN , OUTPUT); 
	digitalWrite(PIN_EN , HIGH);   // must be =1 at startup
	pinMode(PIN_MUTE , OUTPUT); 
	digitalWrite(PIN_MUTE , LOW);  // must be =0 at startup
	

	// Initialize the I2C transmitter.	
	Wire.begin();	
	Serial.println("Wire set up");
  
	// Set the initial state of the pins on the PCF8574 devices
	Wire.beginTransmission(CSwitchTx_address); // device 1
    Wire.write(0xff); // all ports off
    uint8_t error = Wire.endTransmission();
    Serial.printf("endTransmission on CSwitch Tx: %u\n", error);
    Wire.begin();
    Wire.beginTransmission(CSwitchCal_address); // device 2
    Wire.write(0xff); // all ports off
    error = Wire.endTransmission();
    Serial.printf("endTransmission on CSwitch CalibCoil: %u\n", error);
	

if (!tempsens1.begin(Temp1_address)) {
    Serial.println("Couldn't Temperature sensor 1! Check your connections and verify the address is correct.");
  }	
if (!tempsens2.begin(Temp2_address)) {
    Serial.println("Couldn't Temperature sensor 2! Check your connections and verify the address is correct.");
  }
if (!tempsens3.begin(Temp2_address)) {
    Serial.println("Couldn't Temperature sensor 2! Check your connections and verify the address is correct.");
  }
  if (!tempsens4.begin(Temp2_address)) {
    Serial.println("Couldn't Temperature sensor 2! Check your connections and verify the address is correct.");
  }
  
  
	//enable MA12070 to be allow to acces registers
	digitalWrite(PIN_EN , LOW);

  // Setup RS232 connection to Laser
  //--------------------
  Serial.println("Connecting to LASER");
  RS232.begin(baudrateRS232, SERIAL_8N1, PIN_Rx, PIN_Tx);  // Use this for HardwareSerial
  RS232.setRxBufferSize(Laser_BufferSize);
  Serial.println("Connected to LASER");
  //    delay(500);
  //  	RS232.write(0x1B);  // stop sending data
  //    delay(500);
  //  	RS232.write("SD 0 3");  // set data format
  //  	RS232.write(0x0D);
  //    delay(100);


  // Setup IMX5 connection
  //--------------------
  Serial.println("Connecting to IMX5");
  IMX5.begin(baudrateIMX5, SERIAL_8N1, IMX5_Rx, IMX5_Tx);  // Use this for HardwareSerial
  IMX5.setRxBufferSize(IMX5_BufferSize);
  Serial.println("Started serial to IMX5");
  pinMode(IMX5_strobe, OUTPUT);
  digitalWrite(IMX5_strobe, LOW);


  // Setup BLE connection
  //--------------------
  setup_BLE();


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
  
  pinMode(triggerDAC , OUTPUT); 
  digitalWrite(triggerDAC, HIGH);
  
  
  // Setup SD connection
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


  BLE_message = true;
  strcpy(txString, "Setup completeded. Waiting for command 'START' over Serial of BLE for starting measuring data!");
  Send_tx_String(txString);

}


void loop() {

  //################################# Data logging #############################################

  if (measuring) {  // if measuring is active check for INS and Laser data and save them to SD card

    //  INS data
    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    if (IMX5.available() >= WriteSize_IMX5) {
      bitesToWrite = IMX5.available();
      if (tempBufferSize < bitesToWrite) {
        bitesToWrite = tempBufferSize;
      }

      IMX5.readBytes(tempBuffer, bitesToWrite);
      writeToBuffer(tempBuffer, bitesToWrite);
      logTime_IMX5 = millis();

      if (tempBuffer[bitesToWrite-5]!='*' and IMX5.available()>0 ){
        bitesToWrite = IMX5.available();
        if (tempBufferSize < bitesToWrite) {
          bitesToWrite = tempBufferSize;
        }
        IMX5.readBytes(tempBuffer, bitesToWrite);
        writeToBuffer(tempBuffer, bitesToWrite);
      }
      delay(1);
    }



    //  Laser data
    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    if (RS232.available() >= WriteSize_Laser) {
      bitesToWrite = RS232.available();
      if (tempBufferSize < bitesToWrite) {
        bitesToWrite = tempBufferSize;
      }
      RS232.readBytes(tempBuffer, bitesToWrite);
      writeToBuffer(tempBuffer, bitesToWrite);
      logTime_laser = millis();
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
    }


    //==========================================================


    // flush Laser buffer if stuck
    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-
    if ((millis() - logTime_laser) > Laser_log_intervall and RS232.available()) {  //Flush RS232 buffer if data were are not read for too long.
      while (RS232.read() >= 0)
        ;
      Serial.println("Laser buffer flushed");
    }

    // flush IMX5 buffer if stuck
    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-
    if ((millis() - logTime_IMX5) > IMX5_log_intervall and IMX5.available()) {  //Flush RS232 buffer if data were are not read for too long.
      while (IMX5.read() >= 0)
        ;
      Serial.println("IMX5 buffer flushed");
    }

    // flush SDCard every "flushSD_intervall"
    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-
    if (lastTime_flushSD + flushSD_intervall < millis()) {
      dataFile.flush();
      dataFile.flush();
      lastTime_flushSD = millis();
      Serial.println("flushed");
    }

    // Read battery voltage
    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-
	  if (lastTime_VBat + VBat_intervall < millis()) {
      lastTime_VBat = millis();
      logVBat();
	  }

    // Get Temperature
    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-
    if (lastTime_Temp + Temp_intervall < millis()) {
      lastTime_Temp = millis();
      // long time=millis();
      logTemp(tempsens1,1);
      logTemp(tempsens2,2);
	  logTemp(tempsens3,3);
	  logTemp(tempsens4,4);
      // Serial.printf("Time: %d ms", millis()-time);
      // for  (int i = 0; i < N_TempSens; i++) {
      //     Serial.printf("Reading Sensor: %d", i);
      //     time=millis();
      //     logTemp2(i);
      //     Serial.printf("Time: %d ms", millis()-time);
      // }
    }
	
	// // Send strobe pulse every "strobe_intervall"
    // // =-=-=-=-=-=-=-=-=-=-=-=-=-=-
    // if (lastTime_STROBE + STROBE_intervall < millis()) {
      // pulseStrobeIMX5();
      // Serial.println("STROBE");
      // lastTime_STROBE = millis(); 
    // }
	
  } else {
    delay(50);  // wait 50 ms if not measuring
  }



  //################################# Do other stuff #############################################

  // Check BLE connection
  if (deviceConnected && BLE_message) {
    Serial.println("Sending BLE message");
    pTxCharacteristic->setValue(txString);
    pTxCharacteristic->notify();
    BLE_message = false;
    //delay(10); // bluetooth stack will go into congestion, if too many packets are sent
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    //delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising();  // restart advertising
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }



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


  delay(5);
}
