/*
  Code for parsing LEM specific NMEA message and separate  them from data flow coming over serial radio. Worjing on ESP32.
  By: AcCapelli
  Date: February, 2024
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  Based on Examples from multiple libraries and hardware.
  
*/

//HardwareSerial
#include <HardwareSerial.h>
HardwareSerial Radio(2);
HardwareSerial IMX5(1);

int IMX5_Rx = 33; //  Hardware RX pin,
int IMX5_Tx = 27; // Hardware TX pin,

int Radio_Rx = 16; //  Hardware RX pin,
int Radio_Tx = 17; // Hardware TX pin,
int Radio_CTS = 19;
int Radio_RTS = 23; 

unsigned long  Radio_inbytes=0;
int  Radio_inbytes_buff=0;
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

int baudrateRadio= 57600 ;
int baudrateIMX5= 57600 ;


unsigned long lastTime;

String rxValueRadio;
String rxValue;
char RxVal;



#include <NMEAParser.h>
/* A parser is declared with 1 handlers at most */
NMEAParser<3> parser;


void LEM_Handler(){
  Serial.print("Got $LEM message with ");
  Serial.print(parser.argCount());
  Serial.print(" arguments.  ");
  char arg0[16];
  char msgOut[64];
  int arg;
  char valStr[16];

  if (parser.getArg(0,arg0)) {
    Serial.print(" Message handle: ");
    Serial.println(arg0);
    strcpy(msgOut,arg0);
  }
  else{
    Serial.print("Coud not get messge handle! ");
    return;
  }

  
  if (parser.argCount()>1){
    strcat(msgOut,":");
    for (int i=1;i<parser.argCount();i++){
      if (parser.getArg(i,arg)){
        sprintf(valStr,"%d:",arg);
        strcat(msgOut,valStr);
      }
    }
  } 
  Serial.print(" LEM command: ");
  Serial.println(msgOut);
}

void unknownCommand()
{
  Serial.print("*** Unkown command : ");
  char buf[6];
  parser.getType(buf);
  Serial.println(buf);
}
void errorHandler()
{
  if (parser.error()>1){
    Serial.print("*** Error: ");
    Serial.println(parser.error()); 
  }

}

void setup() {
  Serial.begin(115200);  
  Serial.println("Starting ");

  Radio.begin(baudrateRadio,SERIAL_8N1, Radio_Rx, Radio_Tx);  
  IMX5.begin(baudrateIMX5,SERIAL_8N1, IMX5_Rx, IMX5_Tx);  

  // SEtup parser NMEA messages //
  parser.setErrorHandler(errorHandler);
  parser.addHandler("LEMMS", LEM_Handler);
  parser.setDefaultHandler(unknownCommand);

  while(Radio.read() >= 0);
  while(Serial.read() >= 0);
  while(IMX5.read() >= 0);
}

void loop() {

  if (Radio.available()) {      // If anything comes in Radio,
      while(Radio.available())  {   // read it and send it out IMX5 and to NMEA parser.
            RxVal=Radio.read();
            IMX5.write(RxVal);  
            parser << RxVal; 
            Radio_inbytes_buff++;
      }

    }

  if (Serial.available()) {      // If anything comes in Serial is passed to NMEA parser
      while(Serial.available())  {   
            RxVal=Serial.read();
            Serial.write(RxVal);  
            parser << RxVal; 
      }
    }
  
  if (Radio_inbytes_buff>1000) {      // If anything comes in Serial is passed to NMEA parser
      Radio_inbytes+=Radio_inbytes_buff/1000;
      Radio_inbytes_buff=0;
      Serial.print("Total kb received:");
      Serial.println(Radio_inbytes);
    }
  delay(5);
}
