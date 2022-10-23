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
   * ClickBoard | ESP32
   SCK  18
   MISO  19
   MOSI  16
   CS  5
   TRG  21 
  
  BLE:
  Set up BLE connections with UART communication
  Start new data files if "START" is received and stop current data files if "STOP" is received. See below for more BLE commands.
  
  
*/
#include "SPI.h"


long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.


int statLED = 13;

// settings SPI
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
#define SCK  18
#define MISO  19
#define MOSI  16
#define CS  5
#define SPI_rate 10000000
#define triggerGPIO 21         
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
int loggingTime=30;   // duration of data recording 

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=








// Sinus function variables
//-=-=-=-=-=-=-=-=-=-=-=-
uint64_t clock_divider=1;
uint64_t AWG_clock_freq = 125000000;

uint16_t freqAdd;
uint64_t freq=3000;
uint16_t freqDat;
float gain=0;
uint16_t gainDAT = 0x1000;

// Define a C-major scale to play all the notes up and down.
uint64_t freqList[] = { 100,200,300,400,500,600,700,800,900,1000,1500,2000,2500,3000,4000,5000,6000,7000,8000,9000 };
uint64_t freqList2[] = { 100,250,500,750,1000,2000,2500,4000,5000,6000,7000,8000,9000 };

uint16_t gainList[] = {0x2000,0x1400,0x1200,0x1000,0x0800 ,0x0400 ,0x0200 ,0x0100 };
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
// UART to microcontroller function
// ---------------------------------------------
// /////////////////////////////////////////////

void recordMicro(float duration){

  UARTmicro.write('N');
  delay(1000);
  UARTmicro.write('D');
  delay(1000);
  UARTmicro.write('C');
  delay(2000);
  UARTmicro.write('S');
  delay(int(1000*duration));  
  UARTmicro.write('T');
}

void startMicro(){
  UARTmicro.write('N');
  delay(1000);
  UARTmicro.write('D');
  delay(1000);
  UARTmicro.write('C');
  delay(2000);
  UARTmicro.write('S');
}

void stopMicro(){ 
  UARTmicro.write('T');
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
    
  } else if (rxValue.indexOf("S2") != -1 or rxValue.indexOf("sweep2") != -1) { 
    delay(500);
    strcpy(txString,"Start sweep");
    Send_tx_String(txString) ;
    startMicro();  // start recording of ADC data for given duration in seconds
	  delay(2000);
   
	  for (int j=0; j<sizeof(gainList)/sizeof(uint16_t); ++j) {
		  strcpy(txString,"New gain");
//		  sprintf(txString,"New gain is: %04X",gainList[j] );
      strcpy(txString,"Changing gain" );
		  Send_tx_String(txString); 
		  gainDAT=gainList[j];
		  delay(500);
		  for (int i=0; i<sizeof(freqList2)/sizeof(uint64_t); ++i) {
			  // Play the note for a quarter of a second.
			  freq=freqList2[i];
			  configureSineWave();
			  sprintf(txString,"New frequency is: %d ",freq );
			  Send_tx_String(txString); 
			  run();
			  trigger();
			  delay(1000);
			  // Pause for a tenth of a second between notes.
			  stop_trigger();
			  delay(200);
			}
	  }
    stopMicro();
	  strcpy(txString,"End of sweep");
	  Send_tx_String(txString); 
	  
  } else if (rxValue.indexOf("SWEEP") != -1 or rxValue.indexOf("sweep") != -1) { 
	  delay(500);
	  strcpy(txString,"Start sweep");
	  Send_tx_String(txString) ;
	  startMicro();   // start recording of ADC data for given duration in seconds
	  delay(2000);
	  for (int i=0; i<sizeof(freqList)/sizeof(uint64_t); ++i) {
		  // Play the note for a quarter of a second.
		  freq=freqList[i];
		  configureSineWave();
		  sprintf(txString,"New frequency is: %d",freq );
		  Send_tx_String(txString); 
		  run();
		  trigger();
		  delay(1000);
		  // Pause for a tenth of a second between notes.
		  stop_trigger();
		  delay(200);
      
	  }
    stopMicro();
	  strcpy(txString,"End of sweep");
	  Send_tx_String(txString) ;
	  
      
  } else if (rxValue.indexOf("STOP") != -1 or rxValue.indexOf("stop") != -1 ) {
	  stop_trigger();
  } else if (rxValue.indexOf("RECORD") != -1 or rxValue.indexOf("record") != -1 ) {
	  strcpy(txString,"ADC recording started");
    Send_tx_String(txString) ;
	  recordMicro(loggingTime); 
    strcpy(txString,"ADC recording stopped");
    Send_tx_String(txString) ;
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
