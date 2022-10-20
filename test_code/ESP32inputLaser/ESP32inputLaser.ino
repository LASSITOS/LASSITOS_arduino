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
#define sdWriteSize 256 // Write data to the SD card in blocks of 512 bytes
uint8_t *myBuffer; // A buffer to hold the data while we write it to SD car


void setup() {
  Serial.begin(115200);
  RS232.begin(baudrateRS232,SERIAL_8N1, PIN_Rx, PIN_Tx);  // Use this for HardwareSerial
  Serial.println("Starting ");

  myBuffer = new uint8_t[sdWriteSize]; // Create our own buffer to hold the data while we write it to SD card
  while(RS232.read() >= 0);
}

void loop() {
   if (Serial.available()){ // Check Serial inputs
   String rxValue = Serial.readString();
    if (rxValue.length() > 0) {
      Serial.println("*********");
      Serial.print("Received Value: ");
      for (int i = 0; i < rxValue.length(); i++)
        Serial.print(rxValue[i]);
      Serial.println();

      //Start new data files if START is received and stop current data files if STOP is received
      if (rxValue.indexOf("DT") != -1) { 
        RS232.write("DT");
        RS232.write(0x0D);
        delay(100);
        
      } else if (rxValue.indexOf("STOP") != -1) {
        RS232.write(0x1B);
        delay(500);
//        while ( RS232.available()){ RS232.read();}
        while(RS232.read() >= 0) ; // flush the receive buffer.
      } else if (rxValue.indexOf("SD03") != -1) {
        RS232.write("SD 0 3");
		    RS232.write(0x0D);
	  } else if (rxValue.indexOf("SD01") != -1) {
        RS232.write("SD 0 1");
		RS232.write(0x0D);
	  } else if (rxValue.indexOf("SD00") != -1) {
        RS232.write("SD 0 0");
		    RS232.write(0x0D);
	  } else if (rxValue.indexOf("PA") != -1) {
        RS232.write("PA");
		    RS232.write(0x0D);
	 }else{
        Serial.println("Input can not be parsed retry!");
      }
      Serial.println("*********");
      }
  } 





  if ( RS232.available()) {     // If anything comes in Serial1 (pins 0 & 1)
    Serial.write( RS232.read());   // read it and send it out Serial (USB)
//    Serial.println("Got data from RS232. ");
  }
//  if ( RS232.available() >= sdWriteSize) {   
//      
//      RS232.readBytes(myBuffer, sdWriteSize);
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
