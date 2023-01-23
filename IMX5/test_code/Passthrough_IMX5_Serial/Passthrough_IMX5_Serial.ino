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

int PIN_Rx = 16; //  Hardware RX pin,
int PIN_Tx = 17; // Hardware TX pin,
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

int baudrateIMX5= 115200 ;
unsigned long lastTime;
#define sdWriteSize 256 // Write data to the SD card in blocks of 512 bytes
uint8_t *myBuffer; // A buffer to hold the data while we write it to SD car

char asciiMessage[] = "$ASCB,512,,,500,,,,,,,,,";  // Get PINS1 @ 2Hz on the connected serial port, leave all other broadcasts the same, and save persistent messages.
char asciiMessageformatted[100];


// add checksum to asciiMessage
int FormatAsciiMessage(  char *Message,int messageLength, char *outMessage ){
  int checkSum = 0;
  unsigned int ptr = 0;
  char buf[16];
  outMessage[0] = '\0';
  
  if (Message[0] == '$'){
    ptr++;
  } else {
    strcpy(outMessage, "$");
  }

  // concatenate Message to outMessage
  strcat(outMessage,Message);

  // compute checksum
  while (ptr < messageLength){
    checkSum ^= Message[ptr];
    ptr++;
  }
  Serial.print("Size of message: ");
  Serial.println(messageLength);
  
  sprintf(buf, "*%.2x\r\n", checkSum);
  strcat(outMessage,buf);
 
  return sizeof(outMessage);
}



void setup() {
  Serial.begin(115200);
  Serial.println("Started serial to PC");
  
  IMX5.begin(baudrateIMX5,SERIAL_8N1, PIN_Rx, PIN_Tx);  // Use this for HardwareSerial
  Serial.println("Started serial to IMX5");

  myBuffer = new uint8_t[sdWriteSize]; // Create our own buffer to hold the data while we write it to SD card
  

//  if (!FormatAsciiMessage( asciiMessage,  sizeof(asciiMessage),asciiMessageformatted)){
//      Serial.println("Failed to encode ASCII get INS message\r\n");
//  }else {
//      Serial.println(asciiMessageformatted);
//      IMX5.write(asciiMessageformatted); //send instruction for sending ASCII messages
//      Serial.println("Send instruction for sending ASCII messages to IMX5"); 
//  }
  while(IMX5.read() >= 0);

}


void loop() {
  if (Serial.available()) {      // If anything comes in Serial (USB),
    
    int availableBytes = Serial.available();
    
    if(availableBytes<95){
      int i=0;
      char Message[100];
      while( i<availableBytes){
          Message[i] = Serial.read();
          i++;
      }
      Message[i]= '\0';
      Serial.print("Original message: ");
      if (!FormatAsciiMessage( Message, i,asciiMessageformatted)){
            Serial.println("Failed to encode ASCII get INS message\r\n");
        }else {
            Serial.print("Message formatted to  ASCII messages for IMX5: ");
            Serial.println(asciiMessageformatted);
            IMX5.write(asciiMessageformatted); //send instruction for sending ASCII messages
             
      }
    }

//	IMX5.write(Serial.read());   // read it and send it out Serial1 (pins 0 & 1)
  }


  
  if ( IMX5.available()) {     // If anything comes in Serial1 (pins 0 & 1)
    Serial.write( IMX5.read());
//    Serial.print( IMX5.readString());   // read it and send it out Serial (USB)
//    Serial.write( IMX5.read());   // read it and send it out Serial (USB)
//    Serial.println("Got data from IMX5. ");
  }
//  if ( IMX5.available() >= sdWriteSize) {   
//      
//      IMX5.readBytes(myBuffer, sdWriteSize);
//      Serial.println(",");
//      Serial.write( myBuffer, sdWriteSize);   
//      Serial.println(".");
//    }
    


  delay(5);
}
