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

#include "FS.h"
#include "SD.h"
#include "SPI.h"


//HardwareSerial
#include <HardwareSerial.h>
HardwareSerial RS232(2);
HardwareSerial IMX5(1);
// HardwareSerial Serial(0);

#define version "IMX5+Laser v1.0"

long lastTime = 0;  //Simple local timer
long lastTime1 = 0;  //Simple local timer
long lastTime2 = 0;  //Simple local timer
long lastTime3 = 0;  //Simple local timer
long lastTime_STROBE =0;
long lastTime_flushSD = 0;
long logTime_laser;
long logTime_IMX5;




// settings PIN SPI 
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
#define SCK 18
#define MISO 19
#define MOSI 23
#define SPI_rate 80000000  // 10000000   %toCheck
#define CS_Sd 5   // cip select  SD card
#define CS_DAC  14
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

#define CD_pin 36  // chip detect pin is shorted to GND if a card is inserted. (Otherwise pulled up by 10kOhm)
#define triggerDAC 32     

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
#define WriteSize_IMX5 126  // Write data to buffer in blocks (should be shorter than expected message)
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


bool logging = false;
unsigned long lastPrint;  // Record when the last Serial print took place



// settings altimeter LSD70A
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
//PINs can be changed to any pin. // For Hardware Serial use Pin 16 an 17. SoftwareSerial worked on pins 12 and 27
int PIN_Rx = 16;  // 16 = Hardware RX pin,
int PIN_Tx = 17;  // 17 = Hardware TX pin,
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#define baudrateRS232  230400  //115200
#define Laser_BufferSize 2048  // Allocate 1024 Bytes of RAM for UART serial storage
#define Laser_log_intervall 2000


// settings INS IMX5
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
//PINs can be changed to any pin. // For Hardware Serial use Pin 16 an 17. SoftwareSerial worked on pins 12 and 27
int  IMX5_Rx = 26;  //  Hardware RX pin, to PIN10 on IMX5
int  IMX5_Tx =25;  //  Hardware TX pin, to PIN8 on IMX5
int  PIN_PPS =36;  //  GPIO for PPS, to  PIN5 on IMX5
int  IMX5_strobe = 4; //  GPIO for STROBE input (first), to  PIN2 on IMX5
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#define baudrateIMX5 230400  //115200
#define IMX5_BufferSize 2048  // Allocate 1024 Bytes of RAM for UART serial storage
#define IMX5_log_intervall 2000
#define flushSD_STROBE  5000
char asciiMessage[] = "$ASCB,512,,,100,,,,30000,,30000,,,";  // // Get PINS1 @ 2Hz on the connected serial port, leave all other broadcasts the same, and save persistent messages.

char asciiMessageformatted[128];
int IMX5freq=100;

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
#define  MA12070P_address 0x20
#define  CSwitchTx_address 0x24
#define  CSwitchCal_address 0x26
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
bool BLE_stop = false;   // Stop datalogger over BLE
bool BLE_start = false;  // Start datalogger over BLE

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"  // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLEname "GNSS BLE UART"



void LED_blink(int len, int times) {  // Used for blinking LED  times at interval len in milliseconds
  int a;
  for (a = 0; a < times; a = a + 1) {
    digitalWrite(statLED, HIGH);
    delay(len);
    digitalWrite(statLED, LOW);
    if (times > 1) delay(len);  // Wait after blinking next time
  }
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



//-=-=-=-=-=-=-=-=-=-=-=-
// Function generation
//-=-=-=-=-=-=-=-=-=-=-=-
uint64_t clock_divider=1;
uint64_t AWG_clock_freq =125000000; // 7372800; // 125000000; 
uint16_t freqAdd;
uint16_t freqDat;
float gain=0;
uint16_t gainDAT = 0x1000;



//resonant frequencies:
#define F1 1061
#define F2 2283
#define F3 4618
#define F4 8651

uint64_t freqs[] ={F1,F2,F3,F4};
uint64_t freq=F3;
int Nfreq=4;  //Number of frequencies to use

// states of CSwitch for each frequencz
#define stateF1 0x80
#define stateF2 0x40
#define stateF3 0x02
#define stateF4 0x01
#define stateF5 0x10
#define stateF6 0x08
uint8_t CSw_states[] ={stateF1,stateF2,stateF3,stateF4};

#define CALF1 1055
#define CALF2 4744
#define CALF3 8805
int CAL_states[]={0,1,2,4};

#define MAXGAIN 0x3000


//-=-=-=-=-=-=-=-=-=-=-=-


// /////////////////////////////////////////////
// ---------------------------------------------
// Communication to Microcontroller
// ---------------------------------------------
// /////////////////////////////////////////////

// to be done SPI


// /////////////////////////////////////////////
// ---------------------------------------------
// Signal generation: running and test functions
// ---------------------------------------------
// /////////////////////////////////////////////

void testLong(){
	delay(500);
	strcpy(txString,"Starting long test");
	Send_tx_String(txString) ;
	startMicro();   // start recording of ADC data for given duration in seconds
  digitalWrite(PIN_MUTE , LOW);  // mute
  delay(5000);
  digitalWrite(PIN_MUTE , HIGH);  // unmute
  delay(1000);
  digitalWrite(PIN_MUTE , LOW);  // mute
  
	for (int i=0; i<Nfreq; ++i) {
		freq=freqs[i];
		configureSineWave();
    setCswitchTx(CSw_states[i]);
    delay(10);
    digitalWrite(PIN_MUTE , HIGH);  // unmute
    run2();
    trigger();
    delay(5000);
    // Pause for a tenth of a second between notes.
    stop_trigger();
    digitalWrite(PIN_MUTE , LOW);  // mute
    delay(200);
		}
   
   delay(1000);
   stopMicro();
   delay(200);
   strcpy(txString,"End of test");
   Send_tx_String(txString) ;
}

void testCal(){
	delay(500);
	strcpy(txString,"Starting cal test");
	Send_tx_String(txString) ;
	startMicro();   // start recording of ADC data for given duration in seconds
  digitalWrite(PIN_MUTE , LOW);  // mute
  delay(1000);
  digitalWrite(PIN_MUTE , HIGH);  // unmute
  delay(1000);
  digitalWrite(PIN_MUTE , LOW);  // mute
  
	for (int i=0; i<Nfreq; ++i) {
		freq=freqs[i];
		configureSineWave();
    
		for (int j=0; j<4; ++j) {
			setCswitchCal(CAL_states[j]);
      delay(10);
			digitalWrite(PIN_MUTE , HIGH);  // unmute
      run2();
      trigger();
      delay(1000);
      // Pause for a tenth of a second between notes.
      stop_trigger();
      digitalWrite(PIN_MUTE , LOW);  // mute
			delay(200);
		}
   }
   delay(1000);
   stopMicro();
   delay(200);
   strcpy(txString,"End of test");
   Send_tx_String(txString) ;
}

void frequencySweep(int start,int stp,int Delta){
	  int A=start/Delta;
	  int B=stp/Delta;
	  delay(500);
	  strcpy(txString,"Starting frequency sweep");
	  Send_tx_String(txString) ;
	  startMicro();   // start recording of ADC data for given duration in seconds
	  delay(2000);
    digitalWrite(PIN_MUTE , HIGH);  // unmute
    
	  for (int i=A; i<B; ++i) {
		  // Play the note for a quarter of a second.
		  freq=i*Delta;
		  configureSineWave();
		  // sprintf(subString,"f: %d",freq );
      // strcat(txString,subString);
		  digitalWrite(PIN_MUTE , HIGH);  // unmute
		  // run();
      run2();
      trigger();
		  delay(200);
		  // Pause for a tenth of a second between notes.
		  stop_trigger();
		  digitalWrite(PIN_MUTE , LOW);  // mute
		  delay(50);
	  }
    Send_tx_String(txString);
    delay(1000);
    stopMicro();
	  strcpy(txString,"End of sweep");
	  Send_tx_String(txString) ;
}	  

void GainSweep(int start,int stp,int Delta){

    if(stp>MAXGAIN){
      sprintf(txString,"Stop gain is larger then maximum gain: %04X",MAXGAIN );
      Serial.print("STOP");
      Serial.println(stp);
      Serial.print("Maxgain");
      Serial.println(MAXGAIN);
      Send_tx_String(txString) ;
      return;
      }
    
	  int A=start/Delta;
	  int B=stp/Delta;
    // Serial.print("A");
    // Serial.println(A);
    // Serial.print("B");
    // Serial.println(B);
	  
	  delay(500);
	  strcpy(txString,"Starting gain sweep");
	  Send_tx_String(txString) ;
	  startMicro();   // start recording of ADC data for given duration in seconds
	  delay(2000);
      
      
	  for (int j=0; j<Nfreq; ++j) {
		  freq=freqs[j];
		  configureSineWave();
		  // sprintf(subString,"f: %d",freq );
      // strcat(txString,subString);
      setCswitchTx(CSw_states[j]);
      // if(j==0){ 
      //   digitalWrite(PIN_GPIOswitch , HIGH);
      //   delay(10);
      // }else{
      //   digitalWrite(PIN_GPIOswitch , LOW);
      //   delay(10);
      // }

      
		  for (int i=A; i<B; ++i) {
			  // Play the note for a quarter of a second.
			  setGain2(i*Delta);
			  // sprintf(txString,",g: %d",gainDAT ); 
			  digitalWrite(PIN_MUTE , HIGH);  // unmute
			  run2();
			  trigger();
			  delay(200);
			  // Pause for a tenth of a second between notes.
			  stop_trigger();
			  digitalWrite(PIN_MUTE , LOW);  // mute
			  delay(50);
		  }
		delay(1000);
    }
    Send_tx_String(txString);
    stopMicro();
	  strcpy(txString,"End of sweep");
	  Send_tx_String(txString) ;
}


void GainSweep2(int start,int stp,int Delta){
  // Same as GainSweep but don't go over different frequencies!

    if(stp>MAXGAIN){
      sprintf(txString,"Stop gain is larger then maximum gain: %04X",MAXGAIN );
      Serial.print("STOP");
      Serial.println(stp);
      Serial.print("Maxgain");
      Serial.println(MAXGAIN);
      Send_tx_String(txString) ;
      return;
      }
    
	  int A=start/Delta;
	  int B=stp/Delta;
	  
	  delay(500);
	  strcpy(txString,"Starting gain sweep V2");
	  Send_tx_String(txString) ;
	  startMicro();   // start recording of ADC data for given duration in seconds
	  delay(2000);      
		  for (int i=A; i<B; ++i) {
			  // Play the note for a quarter of a second.
			  setGain2(i*Delta);
			  // sprintf(txString,",g: %d",gainDAT ); 
			  digitalWrite(PIN_MUTE , HIGH);  // unmute
			  run2();
			  trigger();
			  delay(1000);
			  // Pause for a tenth of a second between notes.
			  stop_trigger();
			  digitalWrite(PIN_MUTE , LOW);  // mute
			  delay(100);
		  }
		delay(1000);
    Send_tx_String(txString);
    stopMicro();
	  strcpy(txString,"End of sweep");
	  Send_tx_String(txString) ;
}

void FsubSweep(int Df,int Delta){
	  delay(200);
	  strcpy(txString,"Starting f sweep around res. freqs.");
	  Send_tx_String(txString) ;
	  startMicro();   // start recording of ADC data for given duration in seconds
	  delay(1000);

	  for (int j=0; j<Nfreq; ++j) {
		  int mainfreq=freqs[j];
		  int start=mainfreq-Df;
      int stop=mainfreq+Df;
      int A=start/Delta;
	    int B=stop/Delta;
      Serial.print("Main freq:");
      Serial.println(mainfreq);
    
      for (int i=A; i<B+1; ++i) {
        freq=i*Delta;
        configureSineWave();
        setCswitchTx(CSw_states[j]);
        sprintf(subString,"f: %d",freq );
        // strcat(txString,subString);
        Serial.print(subString);
        digitalWrite(PIN_MUTE , HIGH);  // unmute
        run2();
        trigger();
        delay(500);
        stop_trigger();
        digitalWrite(PIN_MUTE , LOW);  // mute
        delay(50);
        Serial.print('.');
		  }
		
    delay(300);
    }
    Send_tx_String(txString);
    stopMicro();
	  strcpy(txString,"End of sweep");
	  Send_tx_String(txString) ;
}


// /////////////////////////////////////////////
// ---------------------------------------------
// SignalGen functions
// ---------------------------------------------
// /////////////////////////////////////////////

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
  out=readReg(0x1E);
  Serial.print("Current run mode:");
  Serial.println(out,BIN);
  writeReg(0x1E,0x0001);
  trigger();
  out=readReg(0x1E);
  Serial.print("New run mode:");
  Serial.println(out,BIN);
}

void run2(){
  writeReg(0x1E,0x0001);
  trigger();
}

void program(){
  digitalWrite(triggerGPIO,HIGH);
  writeReg(0x1E,0x0000);
}
void trigger(){
  // Serial.println("[AWG] Triggerring");
  digitalWrite(triggerGPIO,HIGH);
  delay(1);
  digitalWrite(triggerGPIO,LOW);
}

void stop_trigger(){
  // Serial.println("Stop triggerring");
  digitalWrite(triggerGPIO,LOW);
  delay(1);
  digitalWrite(triggerGPIO,HIGH);
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
// I2C communication
//---------------------------------
// /////////////////////////////////////////////
void setCswitchTx ( uint8_t state ){

  //  CSwitch_code = 0xFF-((state>>2)*3 )<<6 ;// parse first bit
  //  CSwitch_code = CSwitch_code - ((state>>1)%2*3 )<<4 ;// parse second bit
  //  CSwitch_code = CSwitch_code - ((state)%2*3 )<<2  ;  // parse third bit
   CSwitch_code = 0xFF-state;

	 //Write message to the slave
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

uint32_t spiCommand( uint32_t msg ) {  //use it as you would the regular arduino SPI API
  spi.beginTransaction(SPISettings(SPI_rate, MSBFIRST, SPI_MODE0));
  digitalWrite(CS, LOW); //pull SS slow to prep other end for transfer
  uint32_t out= spi.transfer32(msg);
  digitalWrite(CS, HIGH); //pull ss high to signify end of data transfer
  spi.endTransaction();

  return out;
}

void writeMSG (uint16_t addr,uint16_t dat) {
  uint32_t msg = (0x00 << 24) + (addr << 16) + dat;
  spiCommand(  msg );
}

void writeReg (uint16_t addr,uint16_t dat) {
  writeMSG(  addr,dat);
  delay(1);
  writeMSG( 0x1D , 0x01 );
}


uint32_t readReg(uint16_t addr){
//   Serial.println("Reading register");
   uint32_t msg = (0x80 << 24) + (addr << 16) + 0x0000;
   uint16_t out=spiCommand(  msg );
   return out;
}


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
	sprintf(asciiMessage, "$ASCB,512,,,%d,,,,,,,,,",IMX5freq);
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
  dataFile.println(version);
  dataFile.print(F("# IMX5 NMEA settings: "));
  dataFile.println(asciiMessage);

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
  sprintf(dataFileName, "/INS%02d%02d%02d_%02d%02d.dat", Year % 1000, Month, Day, Hour, Minute);  // create name of data file
                                                                                                  //sprintf(dataFileName, "/testfile.csv");   // create name of data file
  Serial.println(dataFileName);
  dataFile = fs.open(dataFileName, FILE_WRITE);
  if (!dataFile) {
    Serial.println(F("Failed to create data file! Freezing..."));
    Serial.println(dataFileName);
    while (1)
      ;
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
  LED_blink(100, 10);
}


void Write_stop() {
  dataFile.println(F("#STOP"));
  // dataFile.print(F("# Measurements stopped on: "));
  // getDateTime( );
  // dataFile.print(printDateTime());
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

void startLogging() {
  BLE_message = true;

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


  strcpy(txString, "Starting logging data!");
  Serial.println(txString);
}




// Stop data logging process
void stop_logging(fs::FS &fs) {
  logging = false;  // Set flag to false
  BLE_message = true;
  Serial.println("Turning datalogging OFF!");

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
  LED_blink(25, 4);
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
	restart_logging();
  } else if (rxValue.indexOf("STOP") != -1) {
	logging = false;
	stop_logging(SD);
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
  } else if (rxValue.indexOf("INSFREQ") != -1) {
	  Serial.println("Setting new frequency value! ");
    int index = rxValue.indexOf(":");\
    int index2 = rxValue.indexOf(":",index+1);
    if (index !=-1 and index2 !=-1){
		if (!logging) {
		  IMX5freq=rxValue.substring(index+1,index2).toInt();
		  sprintf(txString,"New frequency is: %d ms",IMX5freq );
		  Serial.println(txString);
		  setIMX5message(); //update ASCII setting message
		  setupINS(); //send setting message to IMX5
		  } else {
			Serial.println("Datalogging running. Can't read file list now. First stop measurment!");
		}
    } else {
      sprintf(txString,"INS requency (ms) can not be parsed from string '%s''",rxValue);
      Serial.println(txString);
    }
  
  
  } else {
	BLE_message = true;
	strcpy(txString, "Input can not be parsed retry!");
	Serial.println(txString);
  }
}



// Restart data logging process
void restart_logging() {
  if (logging) {
    strcpy(txString, "Already logging.");
    Serial.println(txString);
    return;
  }

  LED_blink(100, 5);
  logging = true;
  startLogging();
}


// change bool setting
void setLogFlag(String rxValue, bool &flag, String flagname) {
  if (!logging) {
    int index = rxValue.indexOf(":");
    int index2 = rxValue.indexOf(":", index + 1);
    if (index != -1 and index2 != -1) {
      flag = (rxValue.substring(index + 1, index2).toInt() > 0);
      BLE_message = true;
      sprintf(txString, "New %s log setting is: %d", flagname, flag);
      Serial.println(txString);

    } else {
      BLE_message = true;
      sprintf(txString, "%s log setting can not be parsed from string '%s'. Valid format is 'FLAGNAME:0:'", flagname, rxValue);
      Serial.println(txString);
    }
  } else {
    BLE_message = true;
    strcpy(txString, "Datalogging running. Can't change flag log setting now. First stop measurment!");
    Serial.println(txString);
  }
  Send_tx_String(txString);
}





void readFiles(String rxValue) {
  /*
    Read last files (haderfile and datafile) if no argument is passed  ("READ" ) otherwise read passed files (READ:<<filepath>>:).
    */
  if (!logging) {
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
    strcpy(txString, "Datalogging running. Can't read files now. First stop measurment!");
    Serial.println(txString);
  }
  Send_tx_String(txString);
}

void getFileList() {
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




// Get Laser data for 1 second and send it over BLE and serial
void check_laser() {
  Send_tx_String(txString);
  strcpy(txString, "");
  if (!logging) {
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
  if (!logging) {
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


// Print commands
void commands() {
  Send_tx_String(txString);
  strcpy(txString, "");
  strcat(txString, "\nCommands \n# --------------------------------------\n");
  strcat(txString, "\n Serial only\n# .................\n");
  strcat(txString, "READ Read last file of last measurement. If argument is passed with READ:<< nfile>>: read file nfile \nLIST  Get list of files in SD card \n ");

  strcat(txString, "\n Serial and BLE:\n# .................\n");
  strcat(txString, " STOP  Stops measurement \nSTART Starts new measurement \nRATE:<<N>>: Set sampling rate to N \nCHECKATT  Get attitude angles from IMU anf GPS coordinates \n CHECKLASER  Get Laser data for 1 second and send it over BLE and serial \nCOMS  List commands \nLOGGPS:<<b>>: Log GPS data if b=1  (Default) ");
  Send_tx_String(txString);
  delay(100);
  strcpy(txString, "");
  strcat(txString, "\n BLE only:\n# .................\n");
  strcat(txString, "CHECKIMU  Get IMU data and send them over BLE+serial for manual check \nDYNMODEL:<<n>>: Set dynamic model to n.  Sensor need to be recalibrated after changing the setting.\n        n=4: Automotive (default) \n        n=0: Applications with low acceleration. \n        n=6: Airborne. Greater vertical acceleration  \n");
  strcat(txString, "IMUCAL:<<b>>: Change setting for regular check for IMU calibration. b=0,1 \n LOGRMX:<<b>>: Log RMX messages or not. b=0,1 \nNAVPVT:<<b>>: same \nNAVPVAT:<<b>>: same  \n");
  strcat(txString, "ESFINS:<<b>>: same\nESFRAW:<<b>>:  same \nESFMEAS:<<b>>:  same \nESFALG:<<b>>: same \n");
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
  Serial.println("bluetooth connection: 'GNSS BLE UART' ");
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




// /////////////////////////////////////////////
// -%--------------------------------------------
// Setup and loop
// %%------------------------------------------------------------------------------------------------------------------------------
void setup() {

  Serial.begin(115200);

	//Mute and Enable PINS
	pinMode(PIN_EN , OUTPUT); 
	digitalWrite(PIN_EN , HIGH);   // must be =1 at startup
	pinMode(PIN_MUTE , OUTPUT); 
	digitalWrite(PIN_MUTE , LOW);  // must be =0 at startup
	pinMode(PIN_GPIOswitch , OUTPUT); 
	digitalWrite(PIN_GPIOswitch , LOW);  // switch off at startup

	// Initialize the I2C transmitter.	
	Wire.begin();	
	
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
	


	//enable MA12070P to be allow to acces registers
	digitalWrite(PIN_EN , LOW);

  // Setup RS232 connection to Laser
  //--------------------
  RS232.begin(baudrateRS232, SERIAL_8N1, PIN_Rx, PIN_Tx);  // Use this for HardwareSerial
  RS232.setRxBufferSize(Laser_BufferSize);
  //    delay(500);
  //  	RS232.write(0x1B);  // stop sending data
  //    delay(500);
  //  	RS232.write("SD 0 3");  // set data format
  //  	RS232.write(0x0D);
  //    delay(100);


  // Setup INS connection
  //--------------------
  Serial.println("Connecting to INS");
  IMX5.begin(baudrateIMX5, SERIAL_8N1, IMX5_Rx, IMX5_Tx);  // Use this for HardwareSerial
  IMX5.setRxBufferSize(IMX5_BufferSize);
  Serial.println("Started serial to IMX5");
  pinMode(IMX5_strobe, OUTPUT);
  digitalWrite(IMX5_strobe, LOW);


  // Setup BLE connection
  //--------------------
  setup_BLE();


  // Setup SD connection
  //--------------------
  Serial.print("CD pin value:");
  Serial.println(digitalRead(CD_pin));
  if (!digitalRead(CD_pin)) {
    Serial.println("No SD card inserted. Waiting for it.");
    while (1) {
      LED_blink(200, 3);
      delay(2000);
      if (digitalRead(CD_pin)) {
        Serial.println(F("SD card inset=rted continuing."));
        break;
      }
    }
  }

spi.begin(SCK, MISO, MOSI, CS);  //%toCheck  SPI could be a huge mess!!!!
  if (!SD.begin(CS, spi, SPI_rate)) {
    Serial.println("Card Mount Failed");
    while (1)
      ;
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

  Serial.println(F("Setup completeded."));

  if (logging) {
    startLogging();
  } else {
    BLE_message = true;
    strcpy(txString, "Waiting for command 'START' over Serial of BLE for starting logging data!");
    Send_tx_String(txString);
  }
}


void loop() {

  //################################# Data logging #############################################

  if (logging) {  // if logging is active check for INS and Laser data and save them to SD card

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
        // Serial.print('#');
      }

      // Serial.print("Dump IMX5 data to buffer:");
      // Serial.println(bitesToWrite);
      // Serial.write(tempBuffer, bitesToWrite);     
      // Serial.print("I");
      // Serial.print(bitesToWrite);
      // Serial.print("t");
      // Serial.print(logTime_IMX5);
      
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
      
      // Serial.print("Dump Laser data to buffer:");
      // Serial.println(bitesToWrite);
      // Serial.print("L");
      // Serial.print(bitesToWrite);
      // Serial.print("t");
      // Serial.print(logTime_laser);
      // Serial.write(tempBuffer, bitesToWrite);
      
      delay(1);
    }



    //  Write data to SD
    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    if (((myBufferSize + BufferTail - BufferHead) % myBufferSize) > sdWriteSize) {
      // Serial.println(" ");
      // Serial.print("Total bytes in buffer:");
      // Serial.print((myBufferSize + BufferTail - BufferHead) % myBufferSize);
      // Serial.print(", buffer tail:");
      // Serial.print(BufferTail);
      // Serial.print(", buffer head:");
      // Serial.print(BufferHead);
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
      // Write position of bufferHead and marker at end of buffer. Used for debugging!
      // tempBuffer[sdWriteSize-5]=BufferHead/1000+'0';
      // tempBuffer[sdWriteSize-4]=BufferHead/100%10+'0';
      // tempBuffer[sdWriteSize-3]=BufferHead/10%10+'0';
      // tempBuffer[sdWriteSize-2]=BufferHead%10+'0';
      // tempBuffer[sdWriteSize-1]='%';


      dataFile.write(tempBuffer, sdWriteSize);
      lastTime3 = millis();
      Serial.print(",Data written:");
      Serial.print(sdWriteSize);
      // Serial.print(" ,write to tempBuffer (ms):");
      // Serial.print(lastTime2-lastTime);
      Serial.print(" , in time (ms):");
      Serial.println(lastTime3-lastTime);
      // Serial.write( tempBuffer, sdWriteSize);
      delay(1);
    }


    //==========================================================


    // Send strobe pulse every "strobe_intervall"
    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-
    if (lastTime_STROBE + flushSD_STROBE < millis()) {
      pulseStrobeIMX5();
      Serial.println("STROBE");
      lastTime_STROBE = millis(); 
    }
   

    // // // check data in buffer once per 10 seconds
    // // // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    // if (millis() > (lastPrint + 10000)) {  //
    //   Serial.print("Data in Buffer:");
    //   Serial.println((myBufferSize + BufferTail - BufferHead) % myBufferSize);
    //   Serial.print("Data in IMX5:");
    //   Serial.println(IMX5.available());
    //   Serial.print("Data in Laser:");
    //   Serial.println(RS232.available());
    //   BLE_message = true;
    //   strcpy(txString, ":");
    //   lastPrint = millis();  // Update lastPrint
    // }

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
      lastTime_flushSD = millis();
      Serial.println("flushed");
    }



  } else {
    delay(50);  // wait 50 ms if not logging
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


  if (BLE_stop) {
    stop_logging(SD);
    BLE_stop = false;
  } else if (BLE_start) {
    restart_logging();
    BLE_start = false;
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
