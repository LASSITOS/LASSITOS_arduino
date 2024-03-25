#include <HardwareSerial.h>
#include <NMEAParser.h>
#include "Support.h"

HardwareSerial GPS(1);
NMEAParser<3> parser;

const unsigned int GPS_RX = 33; //  Hardware RX pin,
const unsigned int GPS_TX = 27; // Hardware TX pin,

const unsigned int BAUDGPS= 57600 ;

unsigned long  ESP_inbytes=0;
int  ESP_inbytes_buff=0;

char RxVal;


long lastTime = 0;  //Simple local timer
long timer = 0;  //Simple local timer


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
  sendToBase(msgOut);
  
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
  // put your setup code here, to run once:
  Serial.begin(115200);  
  Serial.println("Starting ");
  InitESPNow();
    while(!ScanForSlave()){
    Serial.println("REP ESP not found. trying again in 5 seconds.");
    delay(5000);
  }
  Serial.println("REP ESP found.");
  xLemToBaseQueue = xQueueCreate( QUEUE_LENGTH, sizeof(ESP_Msg_t) );   // create a queue of 10 messages
  xTaskCreate(espNOWsendTask, "espNOWsendTask", 1024*6, NULL, 4, NULL);
  //xTaskCreate(serialEventRadioTask, "serialEventRadioTask", 1024*6, NULL, 5, NULL);
  GPS.begin(BAUDGPS,SERIAL_8N1, GPS_RX, GPS_TX);  

  // SEtup parser NMEA messages //
  parser.setErrorHandler(errorHandler);
  parser.addHandler("LEMMS", LEM_Handler);
  parser.setDefaultHandler(unknownCommand);

  lastTime=millis(); 

}

void loop() {

  
  if (Serial.available()) {      // If anything comes in Serial is passed to NMEA parser
      while(Serial.available())  {   
            RxVal=Serial.read();
            Serial.write(RxVal);  
            parser << RxVal; 
      }
    }
  

  if (GPS.available()) {      // If anything comes in Serial is passed to NMEA parser
      while(GPS.available())  {   
            RxVal=GPS.read();
            Serial.write(RxVal);  
      }
    }


  if (ESP_inbytes_buff>1000) {      // If anything comes in Serial is passed to NMEA parser
      ESP_inbytes+=ESP_inbytes_buff/1000;
      ESP_inbytes_buff=0;
      Serial.print("Total kb received:");
      Serial.println(ESP_inbytes);
    }

  if (millis()-lastTime >5000) {      // If anything comes in Serial is passed to NMEA parser
      sendToBase("hey");
      lastTime=millis();
    }
  delay(5);

}

void sendToBase(char * msg){
  ESP_Msg_t espMsg = {0};
  memcpy(&espMsg.msg, msg, strlen(msg));
  espMsg.length = strlen(msg);
  xQueueSend( xLemToBaseQueue, ( void * ) &espMsg, ( TickType_t ) 0 );
}
