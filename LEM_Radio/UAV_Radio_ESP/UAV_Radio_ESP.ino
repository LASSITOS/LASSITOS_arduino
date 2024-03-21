
#include <HardwareSerial.h>
#include "Support.h"
HardwareSerial Radio(2);
HardwareSerial IMX5(1);

const int RADIO_RX = 16;
const int RADIO_TX = 17;
const int BAUD_RADIO = 57600;

const unsigned int IMX5_RX = 33; //  Hardware RX pin,
const unsigned int IMX5_TX = 27; // Hardware TX pin,

const unsigned int BAUDIMX5= 57600 ;




void setup() {
  
  Serial.begin(115200);
  IMX5.begin(BAUDIMX5,SERIAL_8N1, IMX5_RX, IMX5_TX);
  
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

  
 
}

void loop() {
  if(foundLem == 0){
    ScanForSlave();
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





