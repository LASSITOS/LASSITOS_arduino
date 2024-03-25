#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h> // only for esp_wifi_set_channel()


const unsigned int BUFFER_LEN = 200; // buffer length

QueueHandle_t xLemToBaseQueue = NULL; // message queue
const int QUEUE_LENGTH = 10;

typedef struct ESP_Msg_t {     // structure for the queue 
  uint8_t length;
  char msg[BUFFER_LEN];
} ESP_Msg_t;

int foundRep = 0; 

#define CHANNEL 3
#define LEM_ESP_AP_NAME "LEM_AP_1000"
#define REP_ESP_AP_NAME "REP_AP_1000"
#define DELETEBEFOREPAIR 0
#define PRINTSCANRESULTS 0

esp_now_peer_info_t slave;