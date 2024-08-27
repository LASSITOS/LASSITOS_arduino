/*
  Code for parsing LEM specific NMEA message and separate  them from data flow coming over serial radio. Worjing on ESP32.
  By: AcCapelli
  Date: February, 2024
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  Based on Examples from multiple libraries and hardware.
  
*/




unsigned long lastTime;

String rxValueRadio;
String rxValue;
char RxVal;



#include "LaserParser.h"
/* A parser is declared with 1 handlers at most */
LaserParser<3> parserLaser;


void Laser_Handler(){
  Serial.print(" ");

    float Temp;
    float Dist;
    float SigQual;
    int LaserError=parserLaser.getLaserError( );

  if (LaserError==0) {
    Dist=parserLaser.getDist();
    Temp=parserLaser.getTemp();
    SigQual=parserLaser.getSigQual();
    if (Dist!=-1){
      Serial.print("Laser distance: ");
      Serial.print(Dist);
      Serial.print(" m");
      }
    
    if (SigQual!=-1){
      Serial.print(", Signal quality: ");
      Serial.print(SigQual);
      Serial.print(" ");
      }

    if (Temp!=999){
      Serial.print(", Temperature: ");
      Serial.print(Temp);
      Serial.print(" degC");
      }
      Serial.println("");
  }
  else{
    Serial.print("Laser error ");
    Serial.print(LaserError);
    switch(LaserError){
      case 2:
        Serial.println(": No distance identified ");
        break;
      case 4:
        Serial.println(": Device error ");
        break;
      case 6:
        Serial.println(": Temperature out of range ");
        break;
      case 10:
        Serial.println(": laser voltage lower than min. voltage");
        break;
    }
  }
  return;
}


void errorHandlerLaser()
{
  if (parserLaser.error()>1){
    Serial.print("*** Error: ");
    Serial.println(parserLaser.error()); 
  }

}



void setup() {
  Serial.begin(115200);  
  Serial.println("Starting ");


  // SEtup parser Laser messages //
  parserLaser.setErrorHandler(errorHandlerLaser);
  parserLaser.setLaserHandler(Laser_Handler);

  while(Serial.read() >= 0);
  Serial.println("Ready ");
}



void loop() {

  if (Serial.available()) {      // If anything comes in Serial is passed to NMEA parserLaser
      while(Serial.available())  {   
            RxVal=Serial.read();
            Serial.write(RxVal);  
            parserLaser << RxVal; 
      }
    }

  delay(20);
}
