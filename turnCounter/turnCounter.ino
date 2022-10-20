int const PULSE_SENSOR_PIN = 26;   // 'S' Signal pin connected to A0

int Signal;                // Store incoming ADC data. Value can range from 0-1024
int Threshold = 500;       // Determine which Signal to "count as a beat" and which to ignore.
int count =0;
bool isup= 0;
long timer=0;
bool counting=0;
long intervall=500;
char txString[256];   // String containing messages to be send to BLE terminal
char subString[32];  


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
//#define SERVICE_UUID           "1448079E-7F48-4029-ADBB-637450A7AEF2" // UART service UUID
//#define CHARACTERISTIC_UUID_RX "1448079E-7F48-4029-ADBB-637450A7AEF2"
//#define CHARACTERISTIC_UUID_TX "1448079E-7F48-4029-ADBB-637450A7AEF2"
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"



// Parse input
void parse( String rxValue){
    //Start new data files if START is received and stop current data files if STOP is received
  if (rxValue.indexOf("START") != -1 or rxValue.indexOf("start") != -1) { 
     counting=1;
     count =0;
  } else if (rxValue.indexOf("STOP") != -1 or rxValue.indexOf("stop") != -1 ) {
    counting=0;
  } else if (rxValue.indexOf("cont") != -1 or rxValue.indexOf("CONT") != -1) {
    counting=1;  
  }else{
  Serial.println("Input could not be parsed!");
  strcat(txString,"Input could not be parsed!");
  pTxCharacteristic->setValue(txString);
  pTxCharacteristic->notify();
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
          pTxCharacteristic->setValue(txString);
          pTxCharacteristic->notify();
          delay(100);
          parse(rxValue2);
          delay(100);
      }
    }
};


void setup_BLE() {
  Serial.println("bluetooth connection: 'ESP32 BLE UART' ");
  // Create the BLE Device
  BLEDevice::init("ESP32 BLE turnCounter");

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






//----------------------------------------
//----------------------------------------

void setup() {
  pinMode(LED_BUILTIN,OUTPUT);  // Built-in LED will blink to your heartbeat
  Serial.begin(115200);           // Set comm speed for serial plotter window
  setup_BLE();  
  delay(500);  
}

void loop() {

  Signal = analogRead(PULSE_SENSOR_PIN); // Read the sensor value
//  Serial.print(Signal); 
//  Serial.print(" "); // Send the signal value to serial plotter
//  Serial.println(count); 
  
  if(Signal > Threshold){                // If the signal is above threshold, turn on the LED
    digitalWrite(LED_BUILTIN,HIGH);
    if (!isup){
    if (counting){
      count++;
    }
    isup=1;
    }
  } else {
    digitalWrite(LED_BUILTIN,LOW);     // Else turn off the LED
    if (isup){
    isup=0;
    }
  }
  
  if (millis() - timer > intervall and counting){   
    Serial.print("turns:");
    Serial.println(count); 
    timer = millis(); //Update the timer
    if (deviceConnected) {
      sprintf(subString,"turns: %i \n",count);
//      sprintf(subString,"%i",count);
      strcpy(txString,subString);
      pTxCharacteristic->setValue(txString);
      pTxCharacteristic->notify();
      BLE_message=false;
    }
  }
  
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
	// if (deviceConnected && BLE_message ) {
		// pTxCharacteristic->setValue(txString);
		// pTxCharacteristic->notify();
		// BLE_message=false;
		// //delay(10); // bluetooth stack will go into congestion, if too many packets are sent
	// }		
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
  
  
  
  delay(1);
}
