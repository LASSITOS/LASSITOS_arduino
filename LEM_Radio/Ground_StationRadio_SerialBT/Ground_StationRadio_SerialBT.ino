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
#include "BluetoothSerial.h"

//#define USE_PIN // Uncomment this to use PIN during pairing. The pin is specified on the line below
const char *pin = "1234"; // Change this to more secure PIN.

String device_name = "LEM Ground Station BT";

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;

char txString[512];  // String containing messages to be send to BLE terminal
char subString[64];
char buffStr[512];



// Send txSTring to BLE and Serial
void Send_tx_String(char *txString) {

  Serial.print(txString);
  SerialBT.print(txString);

  strcpy(txString, "");
}






void setup() {
  Serial.begin(115200);  
  Serial.println("Starting ");

  Radio.begin(baudrateRadio,SERIAL_8N1, Radio_Rx, Radio_Tx);  
  RTCM_in.begin(baudrateRTCM,SERIAL_8N1, RTCM_Rx, RTCM_Tx);  

  
  // Bluetooth Serial setup
  //------------------------
  SerialBT.begin(device_name); //Bluetooth device name
  Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());
  #ifdef USE_PIN
    SerialBT.setPin(pin);
    Serial.println("Using PIN");
  #endif




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
    RadioTx.toCharArray(buffStr, 512);
    strcat(txString, buffStr );
    Send_tx_String(txString);
    }




  if (Serial.available()) {  // Check Serial inputs
    String rxValue = Serial.readString();
    if (rxValue.length() > 0) {
      Serial.println("*********");
      Serial.print("Received Value: ");
      for (int i = 0; i < rxValue.length(); i++)
        Serial.print(rxValue[i]);
		Serial.println("*********");
    parse(rxValue);
      
    }
  }

  if (SerialBT.available()) {  // Check SerialBT inputs
    String rxValue = SerialBT.readString();
    if (rxValue.length() > 0) {
      SerialBT.println("*********");
      SerialBT.print("Received Value: ");
      for (int i = 0; i < rxValue.length(); i++)
        SerialBT.print(rxValue[i]);
      SerialBT.println("*********");
      parse(rxValue);
    }
  }

  delay(10);
}
