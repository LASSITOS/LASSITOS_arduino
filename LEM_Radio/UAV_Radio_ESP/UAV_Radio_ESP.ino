
#include <HardwareSerial.h>
#include "Support.h"
HardwareSerial Radio(2);


const int RADIO_RX = 16;
const int RADIO_TX = 17;
const int BAUD_RADIO = 57600;

// HardwareSerial IMX5(1);
// const unsigned int IMX5_RX = 33; //  Hardware RX pin,
// const unsigned int IMX5_TX = 27; // Hardware TX pin,
// const unsigned int BAUDIMX5= 57600 ;


// Battery regulator MAX17048
#include <Wire.h> // Needed for I2C
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h> // Click here to get the library: http://librarymanager/All#SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library

SFE_MAX1704X lipo(MAX1704X_MAX17048); // Create a MAX17048

uint time_lipo=0;
uint time_lipo2=0;
uint intervall_lipo=30000;  //Intervall between cheking lipo status in ms.
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
    strcpy(lipoString,"Battery: ");
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

  // if(millis()>time_lipo2+5000){
  //   time_lipo2=millis();
  //   Serial.print("Voltage: ");
  //   Serial.print(lipo.getVoltage());  // Print the battery voltage
  //   Serial.print("V");
  // }
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



