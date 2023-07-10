/*
 Uses library 
 by Mischianti Renzo <http://www.mischianti.org>

 https://www.mischianti.org/2019/01/02/pcf8574-i2c-digital-i-o-expander-fast-easy-usage/
*/



// --------------------------------
// Settings I2C
//---------------------------------

#include <Wire.h>


#define  CSwitchTx_address 0x24
#define  CSwitchCal_address 0x26
#define  MA12070P_address 0x20
uint8_t CSwitch =0 ;  // Controlling witch switch is open. 0 all a close.  1 for first, 2 for second,   4  for third. Sum for combinations. 
uint8_t  CSwitch_code =0xFF;


uint16_t addr;
char addrStr[5];
uint16_t dat;
char datStr[19];
uint32_t msg;
uint16_t out;
char txString2[50];
char txString[500];   // String containing messages to be send to BLE terminal
char subString[20];    


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
bool BLE_message = false;


// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"  // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLEname "RCSwitchTest BLE"



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
// Input parsing functions
// ---------------------------------------------
// /////////////////////////////////////////////
void parse( String rxValue){
	  //Start new data files if START is received and stop current data files if STOP is received
  if (rxValue.indexOf("SETTSW") != -1) {
	
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

  
  // Set new values on CSwitch calibration 
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

  } else if (rxValue.indexOf("SCANI2C") != -1) {
      Serial.println("Scanning I2C devices");
      scanI2C ();


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
void setup(){

    Serial.begin(115200);
  
	// Setup BLE connection
	//--------------------
	setup_BLE();

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
  
    // Check BLE connection
  if (deviceConnected && BLE_message) {
    // Serial.println("Sending BLE message");
    pTxCharacteristic->setValue(txString);
    pTxCharacteristic->notify();
    BLE_message = false;
    strcpy(txString, "");
    delay(10); // bluetooth stack will go into congestion, if too many packets are sent
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

  delay(50);
}
