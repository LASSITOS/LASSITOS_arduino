/*
  Getting data from u-blox GNSS module and from LDS70A Laser altimeter and saving them to SD card.
  By: AcCapelli
  Date: August 18th, 2022
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.
  version v1.0
  
  Based on Examples from used libraries and hardware.

  Hardware Connections:

  Connect the ClickBoard to the following pins (using V_SPI):
   * ClickBoard | ESP32 micro usb
   SCK  18
   MISO  19
   MOSI  16
   CS  5
   TRG  21 

   * ClickBoard | ESP32 usb-C
   SCK  23
   MISO  19
   MOSI  16
   CS  18
   TRG  04 
  
  BLE:
  Set up BLE connections with UART communication
  Start new data files if "START" is received and stop current data files if "STOP" is received. See below for more BLE commands.
  
  
*/
#include "SPI.h"
// #include "PCF8574.h"

long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.


int statLED = 13;

// settings SPI
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
#define SCK  23
#define MISO  19
#define MOSI  16
#define CS  18
#define SPI_rate 10000000
#define triggerGPIO 04        
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=


SPIClass spi = SPIClass(VSPI);

uint16_t addr;
char addrStr[5];
uint16_t dat;
char datStr[19];
uint32_t msg;
uint16_t out;


char txString2[50];
char txString[500];   // String containing messages to be send to BLE terminal
char subString[20];																			  			   



// settings UART
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

#include <HardwareSerial.h>
HardwareSerial UARTmicro(2);

int PIN_Rx = 14; //  Hardware RX pin,
int PIN_Tx = 32; // Hardware TX pin,

#define baudrateUART 115200 
int loggingTime=10;   // duration of data recording 

#define myBufferSize 2048
#define UARTBufferSize 2048
char myBuffer[myBufferSize];

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=



// --------------------------------
// Settings I2C
//---------------------------------

#include <Wire.h>
#define  MA12070P_address 0x20
#define  CSwitchTx_address 0x21
#define  CSwitchCal_address 0x26
uint8_t CSwitch =0 ;  // Controlling witch switch is open. 0 all a close.  1 for first, 2 for second,   4  for third. Sum for combinations. 
uint8_t  CSwitch_code =0xFF;

// PCF8574 I2C_cSwitchTx(CSwitchTx_address);
// PCF8574 I2C_Calib(CSwitchCal_address);

#define PIN_EN 33    // must be =1 at startup
#define PIN_MUTE 15  // must be =0 at startup

#define PIN_GPIOswitch 27 //GPIO used for switching SSR directly over GPIO


// Sinus function variables
//-=-=-=-=-=-=-=-=-=-=-=-
uint64_t clock_divider=1;
uint64_t AWG_clock_freq = 125000000;

uint16_t freqAdd;
// uint64_t freq;
uint16_t freqDat;
float gain=0;
uint16_t gainDAT = 0x1000;

//-=-=-=-=-=-=-=-=-=-=-=-





//-=-=-=-=-=-=-=-=-=-=-=-
// Settings for BLE connection
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;

bool BLE_message=false;
bool BLE_stop=false;   // Stop datalogger over BLE
bool BLE_start=false;  // Start datalogger over BLE

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"


// /////////////////////////////////////////////
// ---------------------------------------------
// declare functions to avoid trowing errors
// --------------------------------------------
void Send_tx_String(char *txString);



// /////////////////////////////////////////////
// ---------------------------------------------
// running and test functions
// ---------------------------------------------
// /////////////////////////////////////////////


//resonant frequencies:

#define F1 1061
#define F2 4618
#define F3 8651
uint64_t freqs[] ={F1,F3};
uint64_t freq=F3;

#define CALF1 1055
#define CALF2 4744
#define CALF3 8805
int CAL_states[]={0,1,2,4};

#define MAXGAIN 0x2000

void testBasics(){
	delay(500);
	strcpy(txString,"Starting basic test");
	Send_tx_String(txString) ;
	startMicro();   // start recording of ADC data for given duration in seconds
  digitalWrite(PIN_MUTE , LOW);  // mute
  delay(1000);
  digitalWrite(PIN_MUTE , HIGH);  // unmute
  delay(1000);
  digitalWrite(PIN_MUTE , LOW);  // mute
  
	for (int i=0; i<2; ++i) {
		freq=freqs[i];
		configureSineWave();
		sprintf(txString,"New frequency is: %d",freq );
		Send_tx_String(txString); 
    if(i==0){ 
      digitalWrite(PIN_GPIOswitch , HIGH);
    }else{
      digitalWrite(PIN_GPIOswitch , LOW);
    }
    
		for (int j=0; j<4; ++j) {
			setCswitchCal(CAL_states[j]);
      delay(10);
			digitalWrite(PIN_MUTE , HIGH);  // unmute
      run();
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

    Serial.print("A");
    Serial.println(A);
    Serial.print("B");
    Serial.println(B);

    
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
		  sprintf(txString,",f: %d",freq );
		  Send_tx_String(txString); 
		  digitalWrite(PIN_MUTE , HIGH);  // unmute
		  run();
		  trigger();
		  delay(250);
		  // Pause for a tenth of a second between notes.
		  stop_trigger();
		  digitalWrite(PIN_MUTE , LOW);  // mute
		  delay(50);
	  }
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
    Serial.print("A");
    Serial.println(A);
    Serial.print("B");
    Serial.println(B);
	  
	  delay(500);
	  strcpy(txString,"Starting gain sweep");
	  Send_tx_String(txString) ;
	  startMicro();   // start recording of ADC data for given duration in seconds
	  delay(2000);
      
      
	  for (int j=0; j<2; ++j) {
		  freq=freqs[j];
		  configureSineWave();
		  sprintf(txString,",f: %d",freq );
		  Send_tx_String(txString); 
      if(j==0){ 
        digitalWrite(PIN_GPIOswitch , HIGH);
        delay(10);
      }else{
        digitalWrite(PIN_GPIOswitch , LOW);
        delay(10);
      }

      
		  for (int i=A; i<B; ++i) {
			  // Play the note for a quarter of a second.
			  gainDAT=i*Delta;
			  setGain2(gainDAT);
			  sprintf(txString,",g: %d",gainDAT );
			  Send_tx_String(txString); 
			  digitalWrite(PIN_MUTE , HIGH);  // unmute
			  run();
			  trigger();
			  delay(200);
			  // Pause for a tenth of a second between notes.
			  stop_trigger();
			  digitalWrite(PIN_MUTE , LOW);  // mute
			  delay(50);
		  }
		delay(1000);
    }
    stopMicro();
	  strcpy(txString,"End of sweep");
	  Send_tx_String(txString) ;
}

// /////////////////////////////////////////////
// ---------------------------------------------
// UART to microcontroller function
// ---------------------------------------------
// /////////////////////////////////////////////

void recordMicro(float duration){

  // UARTmicro.write('N');
  // delay(1000);
  // UARTmicro.write('D');
  // delay(1000);
  // UARTmicro.write('C');
  // delay(2000);
  // UARTmicro.write('S');
  // delay(int(1000*duration));  
  // UARTmicro.write('T');
  startMicro();
  delay(int(1000*duration));  
  stopMicro();
}

void startMicro(){
  while (UARTmicro.read() >= 0)
    ;	
	
  UARTmicro.write('N');
  delay(1000);
  UARTmicro.write('D');
  delay(1000);
  UARTmicro.write('C');
  delay(2000);
  UARTmicro.write('S');
  delay(3000);
  if (UARTmicro.available()) {
      int bitesToWrite = UARTmicro.available();
      Serial.print("UARTmicro available (bytes): ");
      Serial.println(bitesToWrite);
      for (int i=0; i<bitesToWrite;i++){
        myBuffer[i]=UARTmicro.read();
      }
      
      myBuffer[bitesToWrite]='\0';
	  // Serial.write(myBuffer, bitesToWrite);
      strcpy(txString,"Output Microcontroller:");
      Send_tx_String(txString);
      Send_tx_String(myBuffer);
    } else{
      strcpy(txString,"No output from Microcontroller!/n");
      Send_tx_String(txString);
    }
  
}

void stopMicro(){ 
  UARTmicro.write('T');
  delay(1000);
  if (UARTmicro.available()) {
      int bitesToWrite = UARTmicro.available();
      Serial.print("UARTmicro available (bytes): ");
      Serial.print(bitesToWrite);
      for (int i=0; i<bitesToWrite;i++){
        myBuffer[i]=UARTmicro.read();
      }
      
      myBuffer[bitesToWrite]='\0';
	  // Serial.write(myBuffer, bitesToWrite);
      strcpy(txString,"Output Microcontroller:");
      Send_tx_String(txString);
      Send_tx_String(myBuffer);
    } else{
      strcpy(txString,"No output from Microcontroller!");
      Send_tx_String(txString);
    }
    

}



// /////////////////////////////////////////////
// --------------------------------
// I2C communication
//---------------------------------
// /////////////////////////////////////////////
void setCswitchTx ( uint8_t state ){

   CSwitch_code = 0xFF-((state>>2)*3 )<<6 ;// parse first bit
   CSwitch_code = CSwitch_code - ((state>>1)%2*3 )<<4 ;// parse second bit
   CSwitch_code = CSwitch_code - ((state)%2*3 )<<2  ;  // parse third bit

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
//  Serial.print("freq");
//  Serial.println(freq);
//  Serial.print("temp");
//  Serial.println(temp,BIN);
//  Serial.print("freqTW");
//  Serial.println(freqTW,BIN);
//  Serial.print("freqMSB");
//  Serial.println(freqMSB,BIN);
//  Serial.print("freqLSB");
//  Serial.println(freqLSB,BIN);      
//  program()
  
  writeReg(0x27,0x0031);  //Sinewave Mode channel 1
//  writeReg(0x27,0x3131);  //Sinewave Mode, channel 1 and 2
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
void program(){
  digitalWrite(triggerGPIO,HIGH);
  writeReg(0x1E,0x0000);
}
void trigger(){
  Serial.println("[AWG] Triggerring");
  digitalWrite(triggerGPIO,HIGH);
  delay(100);
  digitalWrite(triggerGPIO,LOW);
}

void stop_trigger(){
  Serial.println("Stop triggerring");
  digitalWrite(triggerGPIO,LOW);
  delay(100);
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

void setGain(float value){
  if (value >= 2 or value <= -2){
    sprintf(txString,"Gain (%f) must be a number between -2 and 2",value);
    Serial.println(txString);
    return;
  }
  gain=value;
  if (gain < 0){
    gainDAT=int(0x400*abs(gain))<<4|(0x8000);
  }else{
    gainDAT=int(0x400*abs(gain))<<4;
  }
  writeReg(0x35,gainDAT); //digital gain
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
  //  Serial.println("Writing register");
  writeMSG(  addr,dat);
  delay(1);
  writeMSG( 0x1D , 0x01 );
//  delay(2);
//  uint32_t out=spiCommand( (0x80 << 24) + (addr << 16) + 0x0000 ); // Read register
//  uint16_t out2=out & ~(~0U << 16); // or out & 0xFFFF
//  if ( dat != out){
//      Serial.println("Write command unsuccessful");
//      Serial.println(dat,BIN);
//      Serial.println(out,BIN);
//      Serial.println(out2,BIN);
//  }
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
// Input parsing functions
// ---------------------------------------------
// /////////////////////////////////////////////
void parse( String rxValue){
	  //Start new data files if START is received and stop current data files if STOP is received
  if (rxValue.indexOf("START") != -1 or rxValue.indexOf("start") != -1) { 
    run();
    trigger();
    digitalWrite(PIN_MUTE , HIGH);  // unmute
  
  } else if (rxValue.indexOf("STOP") != -1 or rxValue.indexOf("stop") != -1 ) {
    stop_trigger();
    digitalWrite(PIN_MUTE , LOW);  // mute
     
  } else if (rxValue.indexOf("TEST") != -1 or rxValue.indexOf("test") != -1) { 
	testBasics();
    
  } else if (rxValue.indexOf("FSWEEP") != -1 or rxValue.indexOf("fsweep") != -1) { 
    Serial.println("Got freq sweep command");
    int index = rxValue.indexOf(":");
    int index2 = rxValue.indexOf(":",index+1);
	  int index3 = rxValue.indexOf(":",index2+1);
	  int index4 = rxValue.indexOf(":",index3+1);
    if (index !=-1 and index2 !=-1 and index3 !=-1 and index4 !=-1){
		  int A=rxValue.substring(index+1,index2).toInt();
	    int B=rxValue.substring(index2+1,index3).toInt();
	    int delta=rxValue.substring(index3+1,index4).toInt();
		  Serial.print("START");
      Serial.println(A);
      Serial.print("STOP");
      Serial.println(B);
      Serial.print("Delta");
      Serial.println(delta);
		  frequencySweep(A,B,delta);
	  } else {
      sprintf(txString,"Start, Stop and delta can not be parsed form string: '%s''",rxValue);
      Serial.println(txString);
	  }
  } else if (rxValue.indexOf("GSWEEP") != -1 or rxValue.indexOf("gsweep") != -1) { 
    Serial.println("Got gain sweep command"); 
    int index = rxValue.indexOf(":");\
    int index2 = rxValue.indexOf(":",index+1);
	  int index3 = rxValue.indexOf(":",index2+1);
	  int index4 = rxValue.indexOf(":",index3+1);
    if (index !=-1 and index2 !=-1 and index3 !=-1 and index4 !=-1){
		  rxValue.substring(index+1,index2).toCharArray(datStr ,index2-index+1);
      int A=strtoul (datStr, NULL, 16);
	    rxValue.substring(index2+1,index3).toCharArray(datStr ,index3-index2+1);
      int B= strtoul (datStr, NULL, 16);
	    rxValue.substring(index3+1,index4).toCharArray(datStr ,index4-index3+1);
      int delta=strtoul (datStr, NULL, 16);
		  Serial.print("START");
      Serial.println(A);
      Serial.print("STOP");
      Serial.println(B);
      Serial.print("Delta");
      Serial.println(delta);
		  GainSweep(A,B,delta);
    } else {
      sprintf(txString,"Start, Stop and delta can not be parsed form string: '%s''",rxValue);
      Serial.println(txString);
    }  
    
   } else if (rxValue.indexOf("RECORD") != -1 or rxValue.indexOf("record") != -1 ) {
	  strcat(txString,"ADC recording started");
    Send_tx_String(txString) ;
    recordMicro(loggingTime); 
    
 
  } else if (rxValue.indexOf("MEAS") != -1 or rxValue.indexOf("meas") != -1 ) {
	  Serial.println("Msg: ");
    Serial.print(rxValue);
    int index = rxValue.indexOf(":");
    int index2 = rxValue.indexOf(":",index+1);
    Serial.print("index1: ");
    Serial.print(index);
    Serial.println("index2: ");
    Serial.print(index2);
    if (index !=-1 and index2 !=-1){
      loggingTime=rxValue.substring(index+1,index2).toInt();
      Serial.println("Logging time: ");
      Serial.print(loggingTime);
      sprintf(txString,"Record duration is: %d s",loggingTime );
      strcat(txString,", ADC recording started");
      Send_tx_String(txString) ;
      recordMicro(loggingTime); 
      strcpy(txString,"ADC recording stopped");
      Send_tx_String(txString) ;

    } else {
      sprintf(txString,"Recording duration (s) can not be parsed from string '%s''",rxValue);
      Serial.println(txString);
    }
    

    
  } else if (rxValue.indexOf("TRIGGER") != -1) {
	  trigger();
	
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
    } else {
      sprintf(txString,"Gain can not be parsed from string '%s''",rxValue);
      Serial.println(txString);
    }  
  } else if (rxValue.indexOf("SETGAIN") != -1) {
    Serial.println("Setting new digital gain value! ");
    int index = rxValue.indexOf(":");
    int index2 = rxValue.indexOf(":",index+1);
    if (index !=-1 and index2 !=-1){
      setGain(rxValue.substring(index+1,index2).toFloat());
      sprintf(txString,"New gain is:%f, and gain tuning word is: %04X",gain,gainDAT );
      Serial.println(txString);
    } else {
      sprintf(txString,"Gain can not be parsed from string '%s''",rxValue);
      Serial.println(txString);
    }

    
   // Read SPI register 	
  } else if (rxValue.substring(0,4)== "SPIR"  and rxValue.charAt(5)== 'R') {

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
  
    // Set TX CSwitch over I2C
  }else if (rxValue.indexOf("SETTSW") != -1) {
	
	Serial.println("Setting new state for Tx CSwitch.");
    int index = rxValue.indexOf(":");
    int index2 = rxValue.indexOf(":",index+1);
    if (index !=-1 and index2 !=-1){
      rxValue.substring(index+1,index2).toCharArray(addrStr,5);
      CSwitch=strtoul (addrStr, NULL, 16);
      setCswitchTx (CSwitch);

    } else {
      sprintf(txString,"CSwitch state can not be parsed from string '%s''",rxValue);
      Serial.println(txString);
    }

  
    // Set CalibCoil CSwitch over I2C
  }else if (rxValue.indexOf("SETCSW") != -1) {
 
  Serial.println("Setting new state for calibration coil CSwitch.");
    int index = rxValue.indexOf(":");\
    int index2 = rxValue.indexOf(":",index+1);
    if (index !=-1 and index2 !=-1){
      rxValue.substring(index+1,index2).toCharArray(addrStr,5);
      CSwitch=strtoul (addrStr, NULL, 16);
      setCswitchCal (CSwitch);

    } else {
      sprintf(txString,"CSwitch state can not be parsed from string '%s''",rxValue);
      Serial.println(txString);
    }

    // Set new state for GPIO controlled CSwitch
  }else if (rxValue.indexOf("SETGPIOSW") != -1) {
 
  Serial.println("Setting new state for GPIO controlled CSwitch.");
    int index = rxValue.indexOf(":");\
    int index2 = rxValue.indexOf(":",index+1);
    if (index !=-1 and index2 !=-1){
      if (rxValue.substring(index+1,index2).toInt()==1){
			   digitalWrite(PIN_GPIOswitch , HIGH);
      }else {
			   digitalWrite(PIN_GPIOswitch , LOW);
	    }

    } else {
      sprintf(txString,"CSwitch state can not be parsed from string '%s''",rxValue);
      Serial.println(txString);
    }

  
    // Read register  I2C
  }else if(rxValue.substring(0,4)== "I2CR" and rxValue.charAt(8)== 'R') {
	  char addrStr[5];
	  rxValue.substring(4,8).toCharArray(addrStr,5);
	  uint8_t addr=strtoul (addrStr, NULL, 16);
	  Serial.print("Reading register: ");
	  Serial.println(addr, HEX);
	  uint8_t out=readRegI2C ( MA12070P_address,addr);
	  sprintf(txString2,"Addr:%#02X Data:",addr);
	  Serial.print(txString2);
	  Serial.println(out,BIN);

	  
  // Write to register  I2C  
  } else if (rxValue.substring(0,4)== "I2CW" and (rxValue.charAt(8) == 'X' or rxValue.charAt(8) == 'B')) {
	  char addrStr[5];
	  char datStr[19];
	  uint8_t addr;
	  uint8_t dat;
	  
	  if(rxValue.charAt(8) == 'X'){
		  rxValue.substring(9,13).toCharArray(datStr,5);
		  Serial.print(datStr);
		  dat=strtoul (datStr, NULL, 16);
		
	  }else if(rxValue.charAt(8)== 'B'){
		   rxValue.substring(9,17).toCharArray(datStr,17);
           Serial.print(datStr);
		   dat=strtoul (datStr, NULL, 2);
	  }
	  rxValue.substring(4,8).toCharArray(addrStr,5);
	  addr=strtoul (addrStr, NULL, 16);
	  sprintf(txString2,"Writing register:%#02X Data:%#04X",addr);
	  Serial.print(txString2);
	  Serial.println(dat,BIN);
	  write_I2C(MA12070P_address,addr,dat);
  
  }else{
	Serial.println("Input could not be parsed!");
  } 	
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
  Serial.println("bluetooth connection: 'ESP32 AD9106 controlMicro' ");
  // Create the BLE Device
  BLEDevice::init("ESP32 AD9106 controlMicro");

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
    Wire.write(0x00); // all ports off
    uint8_t error = Wire.endTransmission();
    Serial.printf("endTransmission on CSwitch Tx: %u\n", error);
    Wire.begin();
    Wire.beginTransmission(CSwitchCal_address); // device 2
    Wire.write(0x00); // all ports off
    error = Wire.endTransmission();
    Serial.printf("endTransmission on CSwitch CalibCoil: %u\n", error);
	
	
	  // // Set pinMode to OUTPUT
	  // I2C_cSwitchTx.pinMode(P3, OUTPUT);
	  // I2C_cSwitchTx.pinMode(P2, OUTPUT);
	  // I2C_cSwitchTx.pinMode(P4, OUTPUT);
	  // I2C_cSwitchTx.pinMode(P5, OUTPUT);
	  // I2C_cSwitchTx.pinMode(P6, OUTPUT);
	  // I2C_cSwitchTx.pinMode(P7, OUTPUT);
	  // I2C_Calib.pinMode(P0, OUTPUT);
	  // I2C_Calib.pinMode(P1, OUTPUT);
	  // I2C_Calib.pinMode(P2, OUTPUT);

	  // Serial.print("Init cSwitches");
	  // if (I2C_Calib.begin()){
		// Serial.println("I2C_Calib is OK");
	  // }else{
		// Serial.println("I2C_Calib is KO");
	  // }

	  // if (I2C_cSwitchTx.begin()){
		// Serial.println("I2C_SwitchTx is OK");
	  // }else{
		// Serial.println("I2C_SwitchTx is KO");
	  // }


	//enable MA12070P to be allow to acces registers
	digitalWrite(PIN_EN , LOW);
	
	
	

	//SPI
	spi.begin(SCK, MISO, MOSI, CS);							   
    pinMode(CS, OUTPUT); //set up slave select pins as outputs as the Arduino API doesn't handle automatically pulling SS low
    digitalWrite(CS, HIGH);
    pinMode(triggerGPIO, OUTPUT); //set up slave select pins as outputs as the Arduino API doesn't handle automatically pulling SS low
    digitalWrite(triggerGPIO, HIGH);

	setup_BLE();		 

    // setup AD9106
    delay(100);
    configureSineWave();
	
	//setup UART to microcontroller
	UARTmicro.begin(baudrateUART,SERIAL_8N1, PIN_Rx, PIN_Tx);  // Use this for HardwareSerial
    UARTmicro.setRxBufferSize(UARTBufferSize);
   
}


void loop(){
 
  if (Serial.available()){ // Check Serial inputs
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
  
  //################################# Manage BLE #############################################
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
  
  
  delay(50);
}
