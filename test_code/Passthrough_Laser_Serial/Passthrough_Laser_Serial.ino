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
HardwareSerial RS232(2);

int PIN_Rx = 16; //  Hardware RX pin,
int PIN_Tx = 17; // Hardware TX pin,
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

int baudrateRS232= 115200 ;
unsigned long lastTime;

void setup() {
  Serial.begin(115200);
  RS232.begin(baudrateRS232,SERIAL_8N1, PIN_Rx, PIN_Tx);  // Use this for HardwareSerial
  Serial.println("Starting ");
}

void loop() {
  if (Serial.available()) {      // If anything comes in Serial (USB),
     RS232.write(Serial.read());   // read it and send it out Serial1 (pins 0 & 1)
     }

  if ( RS232.available()) {     // If anything comes in Serial1 (pins 0 & 1)
    Serial.write( RS232.read());   // read it and send it out Serial (USB)
//    Serial.println("Got data from RS232. ");
  }

  if ( (millis() - lastTime > 10000)){   
    lastTime = millis(); //Update the timer
    Serial.println("5 s are passed. ");
  }

  
}
