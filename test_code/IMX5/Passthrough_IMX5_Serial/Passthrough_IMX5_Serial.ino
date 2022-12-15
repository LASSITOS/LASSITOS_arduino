/*
  SerialPassthrough sketch

  Some boards, like the Arduino 101, the MKR1000, Zero, or the Micro, have one
  hardware serial port attached to Digital pins 0-1, and a separate USB serial
  port attached to the IDE Serial Monitor. This means that the "serial
  passthrough" which is possible with the Arduino UNO (commonly used to interact
  with devices/shields that require configuration via serial AT commands) will
  not work by default.

  This sketch allows you to emulate the serial passthrough behaviour. Any text
  you type in the IDE Serial monitor will be written out to the serial port on
  Digital pins 0 and 1, and vice-versa.

  On the 101, MKR1000, Zero, and Micro, "Serial" refers to the USB Serial port
  attached to the Serial Monitor, and "Serial1" refers to the hardware serial
  port attached to pins 0 and 1. This sketch will emulate Serial passthrough
  using those two Serial ports on the boards mentioned above, but you can change
  these names to connect any two serial ports on a board that has multiple ports.

  created 23 May 2016
  by Erik Nyquist

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/SerialPassthrough
*/

//HardwareSerial
#include <HardwareSerial.h>
HardwareSerial IMX5(2);

int PIN_Rx = 25; //  Hardware RX pin,
int PIN_Tx = 26; // Hardware TX pin,
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

int baudrateIMX5= 921600 ;
unsigned long lastTime;
#define sdWriteSize 256 // Write data to the SD card in blocks of 512 bytes
uint8_t *myBuffer; // A buffer to hold the data while we write it to SD car

const char* asciiMessage = "ASCB,512,,,1000,,,,,,,,,";  // Get PINS1 @ 10Hz on the connected serial port, leave all other broadcasts the same, and save persistent messages.

void setup() {
  Serial.begin(115200);
 Serial.println("Started serial to PC");
  
  IMX5.begin(baudrateIMX5,SERIAL_8N1, PIN_Rx, PIN_Tx);  // Use this for HardwareSerial
  Serial.println("Started serial to IMX5");

  myBuffer = new uint8_t[sdWriteSize]; // Create our own buffer to hold the data while we write it to SD card
  while(IMX5.read() >= 0);

  IMX5.write(asciiMessage); //send instruction for sending ASCII messages
  Serial.println("Send instruction for sending ASCII messages to IMX5");
}

void loop() {
  if (Serial.available()) {      // If anything comes in Serial (USB),
     IMX5.write(Serial.read());   // read it and send it out Serial1 (pins 0 & 1)
     Serial.println("Got data from Serial ");
     }

  if ( IMX5.available()) {     // If anything comes in Serial1 (pins 0 & 1)
    Serial.write( IMX5.read());   // read it and send it out Serial (USB)
//    Serial.println("Got data from IMX5. ");
  }
//  if ( IMX5.available() >= sdWriteSize) {   
//      
//      IMX5.readBytes(myBuffer, sdWriteSize);
//      Serial.println(",");
//      Serial.write( myBuffer, sdWriteSize);   
//      Serial.println(".");
//    }
    
//  if ( (millis() - lastTime > 10000)){   
//    lastTime = millis(); //Update the timer
//    Serial.println("10 s are passed. ");
//    SA

//  }

  delay(5);
}