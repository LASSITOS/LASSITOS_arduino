
#include <HardwareSerial.h>
#include "Support.h"
HardwareSerial Radio(2);


const int RADIO_RX = 16;
const int RADIO_TX = 17;
const int BAUD_RADIO = 57600;

char txString[500];  // String containing messages to be send to BLE terminal
char txString2[50];
char subString[64];


// Battery regulator MAX17048
#include <Wire.h> // Needed for I2C
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h> // Click here to get the library: http://librarymanager/All#SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library

SFE_MAX1704X lipo(MAX1704X_MAX17048); // Create a MAX17048

uint time_lipo=0;
uint time_lipo2=0;
uint intervall_lipo=60000;  //Intervall between cheking lipo status in ms.
char lipoString[64];
float BatV=0;
float BatPer=0;
float BatRate=0;







void setup() {
  
  Serial.begin(115200);
  
  InitESPNow();
  while(!ScanForSlave()){
    Serial.println("LEM ESP not found. trying again in 5 seconds.");
    delay(5000);
  }

  Serial.println("LEM ESP found.");
  xBaseToLemQueue = xQueueCreate( QUEUE_LENGTH, sizeof(ESP_Msg_t) );   // create a queue of 10 messages
  xTaskCreate(espNOWsendTask, "espNOWsendTask", 1024*6, NULL, 4, NULL);
  xTaskCreate(serialEventRadioTask, "serialEventRadioTask", 1024*6, NULL, 5, NULL);
  Radio.begin(BAUD_RADIO, SERIAL_8N1, RADIO_RX, RADIO_TX);



  // Set up the MAX17048 LiPo fuel gauge:
  Wire.begin();
  lipo.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial
  Serial.print(F("Setting up MAX17048"));
  if (lipo.begin() == false) // Connect to the MAX17048 using the default wire port
  {
    Serial.println(F("MAX17048 not detected."));
    // while (1)
    //   ;
  }
  lipo.quickStart();
  time_lipo=millis();
  Serial.println(F(", done."));
}

void loop() {
  if(foundLem == 0){
    ScanForSlave();
  }
  
  if(millis()>time_lipo+intervall_lipo){
    time_lipo=millis();
    BatV=lipo.getVoltage();
    BatPer=lipo.getSOC();
    BatRate=lipo.getChangeRate();
    strcpy(lipoString,"Battery UAV repeater: ");
    Serial.print(lipoString);  // Print the battery voltage
    Radio.print(lipoString);
    sprintf(lipoString," %.2f V,",BatV);
    Serial.print(lipoString);  // Print the battery voltage
    Radio.print(lipoString);
    sprintf(lipoString," %.2f",BatPer );
    strcat(lipoString, "%,");
    Serial.print(lipoString);  // Print the battery voltage
    Radio.print(lipoString);
    sprintf(lipoString,"rate:  %.2f",BatRate);
    strcat(lipoString, "%/hr\n");
    Serial.print(lipoString);  // Print the battery voltage
    Radio.print(lipoString);

    if( lipo.isVoltageLow()){
      strcpy(lipoString,"Repeater Battery is low!! ");
      Serial.print(lipoString);  // Print the battery voltage
      Radio.print(lipoString);
    }

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
}





void serialEventRadioTask(void * params){
  unsigned int numBytes = 0;
  
  
  for(;;){
    if(Radio.available()){
      while(Radio.available()){
        ESP_Msg_t msg;
        msg.length = Radio.readBytes(msg.msg, BUFFER_LEN);
        xQueueSend( xBaseToLemQueue, ( void * ) &msg, ( TickType_t ) 0 );
        
      }
      
    }

    // if(Radio.available()){
    //   while(Radio.available()){
    //     IMX5.write(Radio.read());
    //   }
      
    // }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}


void sendToLEM(char * msg){
  ESP_Msg_t espMsg = {0};
  memcpy(&espMsg.msg, msg, strlen(msg));
  espMsg.length = strlen(msg);
  xQueueSend( xBaseToLemQueue, ( void * ) &espMsg, ( TickType_t ) 0 );
  Serial.print(msg);
}


void Send_tx_String(char * msg){
  // sendToLEM(msg);
  Serial.print(msg);
}

