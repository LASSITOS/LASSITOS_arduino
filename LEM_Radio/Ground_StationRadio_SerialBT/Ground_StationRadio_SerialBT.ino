/*
  Code for mixing RTK RTCM messages and LEM specific NMEA message and send them to serial radio for passing them the LASSITOS LEM. Worjing on ESP32
  By: AcCapelli
  Date: February, 202a
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  Based on Examples from multiple libraries and hardware.
  
*/

//HardwareSerial
#include <HardwareSerial.h>
HardwareSerial Radio(2);
HardwareSerial RTCM_in(1);

int RTCM_Rx = 33; //  Hardware RX pin,
int RTCM_Tx = 27; // Hardware TX pin,

int Radio_Rx = 16; //  Hardware RX pin,
int Radio_Tx = 17; // Hardware TX pin,
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

int baudrateRadio= 57600 ;
int baudrateRTCM= 57600 ;


unsigned long lastTime;

String rxValueRadio;
String rxValue;


// Setting for BLE connection
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
char txString[512];  // String containing messages to be send to BLE terminal
char subString[64];
char buffStr[512];
bool BLE_message = false;
bool BLEinput=false;
String rxValueBLE;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"  // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLEname "LEM Ground Station BLE"






void setup() {
  Serial.begin(115200);  
  Serial.println("Starting ");

  Radio.begin(baudrateRadio,SERIAL_8N1, Radio_Rx, Radio_Tx);  
  RTCM_in.begin(baudrateRTCM,SERIAL_8N1, RTCM_Rx, RTCM_Tx);  


  while(Radio.read() >= 0);
  while(Serial.read() >= 0);
  while(RTCM_in.read() >= 0);
}

void loop() {
  // / This should go in task
  if (RTCM_in.available()) { 
    while(RTCM_in.available())  {   // If anything comes in Serial (USB),
      Radio.write(RTCM_in.read());   // read it and send it out Serial1 (pins 0 & 1)
    }
  }
  // / This should go in task


  if (Radio.available()) {      // If anything comes in Serial (USB),
    Serial.print("Got message radio");
    String RadioTx = Radio.readString(); // read it and send it out Serial1 (pins 0 & 1)
    Serial.println(RadioTx);
    BLE_message = true;
    RadioTx.toCharArray(buffStr, 512);
    strcat(txString, buffStr );
    Send_tx_String(txString);
    }


  // Send out BLE messges if necessary
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
  if(BLEinput){  // Check BLE inputs
		Serial.println("Got BLE message");
    Serial.print("Parsing:");
    Serial.print(rxValueBLE);
    sprintf(subString,"BLE input: %s\n",rxValueBLE);
		strcat(txString,subString);
    Send_tx_String(txString);
    delay(10);
    BLEinput=false;
    parse(rxValueBLE);
    Serial.println("BLE message processed");
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

  delay(100);
}
