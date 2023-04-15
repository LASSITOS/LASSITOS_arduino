/*
  Getting data from u-blox GNSS module and from LDS70A Laser altimeter and saving them to SD card.
  By: AcCapelli
  Date: April 11, 2023
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.
  version v1.0
  
  Based on Examples from used libraries and hardware.

  
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
#define SPI_rate 1000000
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





// Sinus function variables
//-=-=-=-=-=-=-=-=-=-=-=-
uint64_t clock_divider=1;
uint64_t AWG_clock_freq = 7372800; // 125000000; 

uint16_t freqAdd;
// uint64_t freq;
uint16_t freqDat;
float gain=0;
uint16_t gainDAT = 0x1000;

uint64_t freq=2000;
#define MAXGAIN 0x3000


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
// running and test functions
// ---------------------------------------------
// /////////////////////////////////////////////





void frequencySweep(int start,int stp,int Delta){
	  int A=start/Delta;
	  int B=stp/Delta;
	  delay(500);
	  strcpy(txString,"Starting frequency sweep");
	  Send_tx_String(txString) ;
	  delay(1000);

    
	  for (int i=A; i<B; ++i) {
		  // Play the note for a quarter of a second.
		  freq=i*Delta;
		  configureSineWave();
		  printf(subString,"f: %d",freq );
		  strcat(txString,subString);
		  Send_tx_String(txString);
		  // run();
          run2();
          trigger();
		  delay(1000);
		  // Pause for a tenth of a second between notes.
		  stop_trigger();
		  delay(200);
	  }
    
      delay(1000);
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
	  delay(1000);
      
      
	  for (int i=A; i<B; ++i) {
			  // Play the note for a quarter of a second.
			  gainDAT=i*Delta;
			  setGain2(gainDAT);
			  // sprintf(txString,",g: %d",gainDAT ); 
			  run2();
			  trigger();
			  delay(1000);
			  // Pause for a tenth of a second between notes.
			  stop_trigger();
			  delay(200);
    }
    Send_tx_String(txString);
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

  writeReg(0x45,0x0000); //Static phase/freq
  writeReg(0x3E,freqMSB); //Freq MSB
  writeReg(0x3F,freqLSB); //Freq LSB

  writeReg(0x35,gainDAT); //digital gain Ch1
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
	  out=readReg(0x35);
      Serial.print("New gain:");
      Serial.println(out,BIN);				
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
  } else if (rxValue.substring(0,4)== "SPIW"  and (rxValue.charAt(8) == 'X' or rxValue.charAt(8) == 'B')) {
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
          Serial.print(rxValue2);
          Serial.println("*********");
      		BLE_message=true;
      		// sprintf(subString,"BLE input: %s\n",rxValue2);
          // strcpy(txString,subString);
          // Send_tx_String(txString);
          // delay(100);
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
