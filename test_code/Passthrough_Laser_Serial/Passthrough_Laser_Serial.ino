/*
  Code for mixing RTK RTCM messages and LEM specific NMEA message and send them to serial radio for passing them the LASSITOS LEM. Worjing on ESP32
  By: AcCapelli
  Date: April 7th, 2023
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  Based on Examples from multiple libraries and hardware.
  
  BLE:
  Set up BLE connections with UART communication
  Start new data files if "START" is received and stop current data files if "STOP" is received. See below for more BLE commands.
  
*/

//HardwareSerial
#include <HardwareSerial.h>
HardwareSerial Radio(2);
HardwareSerial RTCM_in(1);

int PIN_Rx = 14; //  Hardware RX pin,
int PIN_Tx = 32; // Hardware TX pin,
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

int baudrateRadio= 230400 ;
unsigned long lastTime;
#define sdWriteSize 256 // Write data to the SD card in blocks of 512 bytes
uint8_t *myBuffer; // A buffer to hold the data while we write it to SD car


void setup() {
  Serial.begin(115200);
  Radio.begin(baudrateRadio,SERIAL_8N1, PIN_Rx, PIN_Tx);  // Use this for HardwareSerial
  Serial.println("Starting ");

  myBuffer = new uint8_t[sdWriteSize]; // Create our own buffer to hold the data while we write it to SD card
  while(Radio.read() >= 0);
}

void loop() {
  if (Serial.available()) {      // If anything comes in Serial (USB),
     Radio.write(Serial.read());   // read it and send it out Serial1 (pins 0 & 1)
    // Serial.println("Got data from Serial ");
     }

  if (Radio.available()) {      // If anything comes in Serial (USB),
     Serial.write(Radio.read());   // read it and send it out Serial1 (pins 0 & 1)
    // Serial.println("Got data from Serial ");
     }

  delay(1);
}
