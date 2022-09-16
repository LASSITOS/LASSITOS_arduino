;/*
  Getting data from u-blox GNSS module and from LDS70A Laser altimeter and saving them to SD card.
  By: AcCapelli
  Date: August 18th, 2022
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.
  version v1.0
  
  Based on Examples from used libraries and hardware.

  Hardware Connections:

  Connect the ClickBoard to the following pins (using V_SPI):
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
#include "SPI.h"


long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.


int statLED = 13;

// settings SD
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
#define SCK  18
#define MISO  19
#define MOSI  32
#define CS  5
#define SPI_rate 10000000
#define triggerGPIO 27         
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=


SPIClass spi = SPIClass(VSPI);

uint16_t addr;
char addrStr[5];
uint16_t dat;
char datStr[19];
uint32_t msg;
uint16_t out;

char txString[50];  
char txString2[50];

// Sinus function variables
//-=-=-=-=-=-=-=-=-=-=-=-
uint64_t clock_divider=1;
uint64_t AWG_clock_freq = 125000000;

uint16_t freqAdd;
uint64_t freq=3000;
uint16_t freqDat;
float gain=0;
uint16_t gainDAT = 0x0800;


//-=-=-=-=-=-=-=-=-=-=-=-



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
  writeReg(0x27,0x0031);  //Sinewave Mode
  writeReg(0x45,0x0000); //Static phase/freq
  writeReg(0x3E,freqMSB); //Freq MSB
  writeReg(0x3F,freqLSB); //Freq LSB

//  gainDAT=int((gain+2)*4095/4) << 4;
  writeReg(0x35,gainDAT); //digital gain
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
  writeReg(0x35,gainDAT); //digital gain
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
    
  } else if (rxValue.indexOf("STOP") != -1 or rxValue.indexOf("stop") != -1 ) {
	  stop_trigger();
   
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
   // Read register 	
  } else if (rxValue.charAt(0)== 'R' and rxValue.charAt(5)== 'R') {

	  rxValue.substring(1,5).toCharArray(addrStr,5);
	  addr=strtoul (addrStr, NULL, 16);
	  Serial.print("Reading register: ");
//          Serial.println(addr, HEX);
	  out=readReg(addr);
	  sprintf(txString2,"Addr:%#02X Data:",addr);
	  Serial.print(txString2);
	  Serial.println(out,BIN);

	  
  // Write to register    
  } else if (rxValue.charAt(0)== 'W' and (rxValue.charAt(5) == 'X' or rxValue.charAt(5) == 'B')) {
	  if(rxValue.charAt(5) == 'X'){
		  rxValue.substring(6,12).toCharArray(datStr,7);
		  Serial.print(datStr);
		  dat=strtoul (datStr, NULL, 16);
		
	  }else if(rxValue.charAt(5)== 'B'){
		   rxValue.substring(6,22).toCharArray(datStr,17);
//               Serial.print(datStr);
		   dat=strtoul (datStr, NULL, 2);
	  }
	  rxValue.substring(1,5).toCharArray(addrStr,5);
	  addr=strtoul (addrStr, NULL, 16);
	  
//          Serial.print("Writing register: ");
//          Serial.print(addr, HEX);
//          Serial.print(", data: ");
//          Serial.println(dat, HEX);
	  sprintf(txString2,"Writing register:%#02X Data:%#04X",addr);
	  Serial.print(txString2);
	  Serial.println(dat,BIN);
	  writeReg(addr,dat);
  
  }else{
	Serial.println("Input could not be parsed!");
  } 	
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

    

	spi.begin(SCK, MISO, MOSI, CS);							   
    pinMode(CS, OUTPUT); //set up slave select pins as outputs as the Arduino API doesn't handle automatically pulling SS low
    digitalWrite(CS, HIGH);
    pinMode(triggerGPIO, OUTPUT); //set up slave select pins as outputs as the Arduino API doesn't handle automatically pulling SS low
    digitalWrite(triggerGPIO, HIGH);


    // setup AD9106
    delay(100);
    configureSineWave();
        
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
  

  delay(50);
}
