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
#define triggerGPIO 27         // chip detect pin is shorted to GND if a card is inserted. (Otherwise pulled up by 10kOhm)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=


SPIClass spi = SPIClass(VSPI);

uint16_t addr;
char addrStr[5];
uint16_t dat;
char datStr[19];
uint32_t msg;


char txString[50];  
char txString2[50];

// Sinus function variables
//-=-=-=-=-=-=-=-=-=-=-=-
uint16_t freqAdd;
int freq;
uint16_t freqDat;

//-=-=-=-=-=-=-=-=-=-=-=-



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
  delay(2);
  uint32_t out=spiCommand( (0x80 << 24) + (addr << 16) + 0x0000 ); // Read register
  uint16_t out2=out & ~(~0U << 16); // drop first part 
  if ( dat != out){
      Serial.println("Write command unsuccessful");
      Serial.println(dat,BIN);
      Serial.println(out,BIN);
      Serial.println(out2,BIN);
  }
}


uint32_t readReg(uint16_t addr){
//   Serial.println("Reading register");
   uint32_t msg = (0x80 << 24) + (addr << 16) + 0x0000;
   uint32_t out=spiCommand(  msg );
   sprintf(txString2,"Addr:%#02X Data:",addr);
   Serial.print(txString2);
   Serial.println(out,BIN);
   return out;
}

// /////////////////////////////////////////////
// ---------------------------------------------
// SignalGen functions
// ---------------------------------------------
// /////////////////////////////////////////////

void trigger(){
  Serial.println("[AWG] Triggerring");
  digitalWrite(triggerGPIO,HIGH);
  digitalWrite(triggerGPIO,LOW);
}

void stop_trigger(){
  Serial.println("Stop triggerring");
  digitalWrite(triggerGPIO,LOW);
  digitalWrite(triggerGPIO,HIGH);
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

      //Start new data files if START is received and stop current data files if STOP is received
      if (rxValue.indexOf("START") != -1) { 
        trigger();
      } else if (rxValue.indexOf("STOP") != -1) {
        stop_trigger();
      
      // Reading register    
      } else if (rxValue.charAt(0)== 'R' and rxValue.charAt(5)== 'R') {

          rxValue.substring(1,5).toCharArray(addrStr,5);
          addr=strtoul (addrStr, NULL, 16);
          Serial.print("Reading register: ");
//          Serial.println(addr, HEX);
          readReg(addr);
      
      // Writing to register    
      } else if (rxValue.charAt(0)== 'W') {
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
        Serial.println(txString);
      }
      Serial.println("*********");
      }
  }  
  

  delay(50);
}
