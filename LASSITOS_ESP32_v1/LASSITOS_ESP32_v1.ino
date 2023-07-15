/*
  Code for controlling the LASSITOS sea ice EM esp32 microprocessor.
  By: AcCapelli
  Date: April 7th, 2023
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  Based on Examples from multiple libraries and hardware.
  
  BLE:
  Set up BLE connections with UART communication
  Start new data files if "START" is received and stop current data files if "STOP" is received. See below for more BLE commands.
  
*/

#include "FS.h"
#include "SD.h"
#include "SPI.h"


//HardwareSerial
#include <HardwareSerial.h>
HardwareSerial RS232(2);
HardwareSerial IMX5(1);
// HardwareSerial Serial(0);

#define baudrateSerial  115200 //230400  // 115200

#define Version "LASSITOS EM sounder ESP32 v1.1"

long lastTime = 0;  //Simple local timer
long lastTime1 = 0;  //Simple local timer
long lastTime2 = 0;  //Simple local timer
long lastTime3 = 0;  //Simple local timer
long lastTime_STROBE =0;
long lastTime_flushSD = 0;
long logTime_laser;
long logTime_IMX5;
long lastTime_VBat; 
long lastTime_Temp; 
long lastTime_CAL =0;


TaskHandle_t TaskStart;
TaskHandle_t TaskEnd;


// settings PIN SPI 
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
#define SCK 18
#define MISO 19
#define MOSI 23
#define SPI_rate    400000  // DAC up to 80 MHz  %toCheck
#define SPI_rate_SD 40000000  
#define CS_SD 5   // cip select  SD card
#define CS_DAC 14
#define CS_MSP430  15  // Chip select MSP430
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

SPIClass spi = SPIClass(VSPI);

uint16_t addr;
char addrStr[5];
uint16_t dat;
char datStr[19];
uint32_t msg;
uint16_t out;


//Other PINs
//-=-=-=-=-=-=-=-=-=-=-=-=-=
#define CD_pin 36  // chip detect pin is shorted to GND if a card is inserted. (Otherwise pulled up by 10kOhm)
#define triggerDAC 32     
#define PIN_VBat 34    

float VBat=0;
long VBat_intervall = 10000;



// settings SD card
//------------------

#define sdWriteSize 16384     // Write data to the SD card in blocks of 512 bytes
#define WriteSize_Laser 23  // Write data to buffer in blocks (should be shorter than expected message)
#define WriteSize_IMX5 30  // Write data to buffer in blocks (should be shorter than expected message)
#define myBufferSize 49152
#define tempBufferSize 16384      // must be bigger than sdWriteSize,WriteSize_Laser and WriteSize_IMX5
#define LaserBufferSize 2048 
#define flushSD_intervall 120000  // Flush SD every ** milliseconds to avoid losing data if ESP32 crashes

uint8_t myBuffer[myBufferSize];       // Create our own buffer to hold the data while we write it to SD card
uint8_t tempBuffer[tempBufferSize];   // Create temporay buffer
uint8_t myBuffer_laser[LaserBufferSize];  // Create buffer for laser data

int BufferTail = 0;
int BufferHead = 0;
int bitesToWrite;


char dataFileName[24];  //Max file name length is 23 characters)
File dataFile;          //File where data is written to


bool measuring = false;
bool StartFlag = false;
bool StopFlag = false;
bool closefileFlag = true;

unsigned long lastPrint;  // Record when the last Serial print took place



// Setting for BLE connection
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
char txString[500];  // String containing messages to be send to BLE terminal
char txString2[50];
char subString[64];
bool BLE_message = false;
bool BLEinput=false;
String rxValueBLE;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"  // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLEname "LEM BLE"




// settings altimeter LSD70A
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
int PIN_Rx = 17;  // 16 = Hardware RX pin,
int PIN_Tx = 16;  // 17 = Hardware TX pin,
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#define baudrateRS232  230400  //115200
#define Laser_BufferSize 2048  // Allocate 1024 Bytes of RAM for UART serial storage
#define Laser_log_intervall 2000


// settings INS IMX5
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
int  IMX5_Rx = 26;  //  Hardware RX pin, to PIN10 on IMX5
int  IMX5_Tx =25;  //  Hardware TX pin, to PIN8 on IMX5
int  IMX5_strobe = 4; //  GPIO for STROBE input (first), to  PIN2 on IMX5 !!!!!!!!Reserved for radio but can be used meanwhile!!!!!!!!
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#define baudrateIMX5 230400  //115200
#define IMX5_BufferSize 8192  // Allocate 1024 Bytes of RAM for UART serial storage
#define IMX5_log_intervall 2000
#define STROBE_intervall 5000
// char asciiMessage[] = "$ASCB,512,,,200,,,,30000,,30000,,,";  // // Get PINS1 @ 100ms, 30s  on the connected serial port, leave all other broadcasts the same, and save persistent messages.
char asciiMessage[] = "$ASCB,512,,,6,,,,2000,,2000,,,";     // new IMX5 has different IMU data rate  (16 ms). I dont know why!! But it is ok.
char asciiMessageformatted[128];
int IMXrate=100;
int IMUdataRate=16;

long Year =2023;
long Month =4;
long Day =29;
long Hour =13;
long Minute = 20;
long Second = 0;


// --------------------------------
// Settings I2C, Amp. Cswitches, Temp.
//---------------------------------
#include <Wire.h>
#include "Adafruit_MCP9808.h"

Adafruit_MCP9808 sensor = Adafruit_MCP9808();
Adafruit_MCP9808 tempsens1 = Adafruit_MCP9808();
Adafruit_MCP9808 tempsens2 = Adafruit_MCP9808();
Adafruit_MCP9808 tempsens3 = Adafruit_MCP9808();
Adafruit_MCP9808 tempsens4 = Adafruit_MCP9808();


// I2C addresses
#define  MA12070P_address 0x20
#define  CSwitchTx_address 0x24
#define  CSwitchCal_address 0x26
#define  Temp1_address 0x18  // 000 ; Tx, Tail
#define  Temp2_address 0x19  // 001 ;  Rx, Tip
#define  Temp3_address 0x1A  // 010 ;  Battery
#define  Temp4_address 0x1B  // 011


#define N_TempSens 2
int TempSens_addr[] ={Temp1_address,Temp2_address};
long Temp_intervall = 10000;  // logging rate for temperature in ms


uint8_t CSwitch =0 ;  // Controlling witch switch is open. 0 all a close.  1 for first, 2 for second,   4  for third. Sum for combinations. 
uint8_t  CSwitch_code =0xFF;

#define PIN_EN 27    // must be =1 at startup
#define PIN_MUTE 12  // must be =0 at startup




//-=-=-=-=-=-=-=-=-=-=-=-
// Function generation settings
//-=-=-=-=-=-=-=-=-=-=-=-
uint64_t clock_divider=1;
uint64_t AWG_clock_freq = 7372800; // 125000000; 
uint16_t freqAdd;
uint16_t freqDat;
float gain=0;
uint16_t gainDAT = 0x1000;


//resonant frequencies:
#define F1 684
#define F2 1079
#define F3 1908
#define F4 5754
#define F5 6848
#define F6 8207

#define F46 4725
#define F456 3900


uint64_t freqs[] ={F1,F2,F3,F456,F46,F4,F5,F6};
uint16_t Nfreq=8;  //Number of frequencies to use


int ifreq =2;
uint64_t freq=freqs[ifreq];
int Nmulti =0;
int i_multi =0;
// uint64_t Multifreqs[]={1,3,7};//Multifreqs[10];

int Nmulti_reg=3;
uint64_t Multifreqs[]={1,3,7,0,0,0,0,0,0,0}; //Must be same or larger than Nfreq
int MultiPeriod=300;
long MultiTime=0;

// states of CSwitch for each frequency
#define stateF1 0x80
#define stateF2 0x40
#define stateF3 0x02
#define stateF4 0x01
#define stateF5 0x10
#define stateF6 0x08

uint8_t CSw_states[] ={stateF1,stateF2,stateF3,stateF4+stateF5+stateF6,stateF4+stateF6,stateF4,stateF5,stateF6};
uint8_t CSw_state=CSw_states[ifreq];


// #define CALF1 1055
// #define CALF2 4744
// #define CALF3 8805
int CAL_states[]={0,1,2,4,6};
int CAL_state=0;
int N_cal=5;
bool calibrating=0;
bool cal_on=0;
int  CAL_intervall_on=3000;
int  CAL_intervall_off=1000;

#define MAXGAIN 0x1800

//-=-=-=-=-=-=-=-=-=-=-=-





//-=-=-=-=-=-=-=-=-=-=-=-
// Microcontroller communication settings
//-=-=-=-=-=-=-=-=-=-=-=-
# define CDM_start 0x81
# define CDM_stop 0x82
# define CDM_status 0xFF
# define CDM_status_read 0x00
# define CDM_status_read32 0X00
# define CDM_reset 0x84							 

// status commands
#define TSTAT_CMD_OK        0x80
#define TSTAT_CALIBRATING   0x08
#define TSTAT_STARTING      0x04
#define TSTAT_SD_ERR        0x02
#define TSTAT_RUNNING       0x01


uint32_t status_out=0;

// status flags Micro
bool MSP430_CMD_OK =0;
bool MSP430_CALIBRATING =0;
bool MSP430_STARTING =0;
bool MSP430_SD_ERR =0;
bool MSP430_measuring =0;


// other flags
bool ADCstarted=0;  //flag checking if ADC responded to start command
bool SD_mounted =0;
bool SD_filecreated =0;






void CalInFlight(){
	delay(500);
	strcpy(txString,"Starting cal test");
	Send_tx_String(txString) ;
  
	for (int j=1; j<4; ++j) {
			setCswitchCal(CAL_states[j]);
			delay(1000);
			setCswitchCal(0);
			delay(1000);
		}
   strcpy(txString,"End of calibration");
   Send_tx_String(txString) ;
}



// /////////////////////////////////////////////
// --------------------------------
// other functions
//---------------------------------
// /////////////////////////////////////////////
float readVBat(){
	int VBat_int= analogRead(PIN_VBat);
	VBat =float(VBat_int)/4095*3.2*10;
	// Serial.printf("Voltage battery GPIO value: %d \n", VBat_int);
  // Serial.printf("Voltage battery: %05.2f V\n", VBat);
	return VBat;
}

void logVBat(){
    readVBat();
	  sprintf(subString,"VBat %05.2f\n",VBat );
	  int msglen=11;
    for (int i = 0; i < msglen; i++){
      tempBuffer[i] = uint8_t(subString[i]);
    } 
    writeToBuffer(tempBuffer, msglen);
}


void logTemp(Adafruit_MCP9808 sensor, int  sensorNumber){
  float c = sensor.readTempC();
  // Serial.printf("Temp%d: %.4f* C\n",sensorNumber,c); 
  sprintf(subString,"Temp%d %07.4f\n",sensorNumber,c );
  int msglen=strlen(subString);
  for (int i = 0; i < msglen; i++){
      tempBuffer[i] = uint8_t(subString[i]);
    } 
    writeToBuffer(tempBuffer, msglen);
}

void readTemp(Adafruit_MCP9808 sensor, int  sensorNumber){
  float c = sensor.readTempC();
  sprintf(txString,"Temperature: %.4f* C\n",c);
  Serial.println(txString);
  
}







void writeToBuffer(uint8_t *tempBuf, int &bitesToWrite) {
  // move BufferHead if circular buffer is full
  int nDump;
  if (BufferTail >= BufferHead) {
    nDump = bitesToWrite - (myBufferSize + BufferHead - BufferTail);
  } else {
    nDump = bitesToWrite - (BufferHead - BufferTail);
  }
  if (nDump >= 0) {
    Serial.print("Data buffer is getting full. Dumping N data:");
    Serial.println(nDump + 1);
    BufferHead = (BufferHead + nDump + 1) % myBufferSize;
  }

  // write data to buffer
  if ((BufferTail + bitesToWrite) < myBufferSize) {
    for (int i = 0; i < bitesToWrite; i++) {
      myBuffer[BufferTail] = tempBuf[i];
      BufferTail++;
    }
  } else {
    int a = myBufferSize - BufferTail;
    for (int i = 0; i < a; i++) {
      myBuffer[BufferTail] = tempBuf[i];
      BufferTail++;
    }
    BufferTail = 0;
    for (int i = 0; i < (bitesToWrite - a); i++) {
      myBuffer[BufferTail] = tempBuf[i + a];
      BufferTail++;
    }
  }
}






void startMeasuring() {
  // BLE_message = true;
  
  // Configure DAC
  //-----------
  stop_trigger();
  delay(10);
  configureResFreq(ifreq);
  setCswitchCal(0);
  delay(10);
  digitalWrite(PIN_MUTE , HIGH);  // unmute
  
  
  makeFiles(SD);
  delay(500);

  lastPrint = millis();      // Initialize lastPrint
  logTime_laser = millis();  // logTime_laser
  logTime_IMX5 = logTime_laser;
  BufferTail = 0;
  BufferHead = 0;

  // setting up GPS for automatic messages
  setupINS();
  delay(50);
  
  // Write start of data to datafile
  dataFile.println(F("###Data###"));  
  
  //start Laser measuremens
  RS232.write("DT");
  RS232.write(0x0D);
  delay(200);
	
	
  while (RS232.read() >= 0){
    ;  // flush the receive buffer.
  }

  // while (IMX5.read() >= 0)
    // ;  // flush the receive buffer.
  while (IMX5.available() > 0){
    char k = IMX5.read();
  }
  
  char infoMsg[] = "$INFO";
  FormatAsciiMessage(infoMsg, sizeof(infoMsg), asciiMessageformatted);
  IMX5.write(asciiMessageformatted);  //send instruction for sending ASCII messages



  xTaskCreatePinnedToCore(
                    TaskStartCode,  /* Task function. */
                    "TaskStart",    /* name of task. */
                    10000,       	/* Stack size of task */
                    NULL,        	/* parameter of the task */
                    1,           	/* priority of the task */
                    &TaskStart,     /* Task handle to keep track of created task */
                    0); 			/* Core where the task should run */ 

}


void TaskStartCode(void * pvParameters) {
  
  while(true){
  // Start MSP430
  //-----------
  startMSP430();
  
  // Start DAC
  //-----------
  if (Nmulti==0){
	  run();
	}

  if (SD_filecreated){
    strcat(txString, "\r\nFile succesfully created on ESP32 SD card");
    Serial.println(txString);

  } else{
    strcat(txString, "\r\n ESP32 SD not working. Abort.");
    Serial.println(txString);
	  interrupt_Measuring();
	  vTaskDelete(TaskStart);
  }
  
  strcat(txString, "\r\nStarted measurement succesfully!");
  Serial.println(txString);
  BLE_message = true; //Send_tx_String(txString);
  
  vTaskDelete(TaskStart);
  
  }
}



// Interrupt data measuring process when there is an error at startup. No successful measurement
void interrupt_Measuring() {
  measuring = false;  // Set flag to false
  BLE_message = true;
  Serial.println("abort mesurment!");
  
  //Stop DAC
  stop_trigger();
  digitalWrite(PIN_MUTE , LOW);  // mute
  
  //Stop Micro
  delay(50);
  stopMicro();

  delay(25);
  dataFile.println(F("# unsuccesful measurement"));

  closefileFlag = true;

  BLE_message = true; //Send_tx_String(txString);
}


// Stop data measuring process
void stopTask(void * pvParameters) {
  while(true){
  Serial.println("Turning dataMeasuring OFF!");
  
  //Stop DAC
  stop_trigger();
  digitalWrite(PIN_MUTE , LOW);  // mute
  
  //Stop Micro
  delay(1000);
  stopMicro();
  
  measuring = false;  // Set flag to false
  
  delay(100);
  emptyBuffer();
  delay(100);
  closefileFlag = true;

  // Serial.println("Datafile closed.");                     
  strcat(txString, "Measurement stopped successfully");
  BLE_message = true; //Send_tx_String(txString);

  vTaskDelete(TaskEnd);
  }
}


void closeFile(fs::FS &fs){
  // output file string
  char filesstring[50];
  strcpy(txString, "Closed file:");
  sprintf(filesstring, " File: %s", dataFileName);
  strcat(txString, filesstring);
  Serial.print("Closing file:");
  Serial.println(filesstring);
  delay(25);
  Write_stop();
  dataFile.close();  // Close the data file
  Serial.println("Datafile closed."); 
  BLE_message = true;  
}

void emptyBuffer(){
  //  Empty data buffer
  // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  Serial.print("Total bytes in buffer:");
  Serial.print((myBufferSize + BufferTail - BufferHead) % myBufferSize);
  Serial.print(", buffer tail:");
  Serial.print(BufferTail);
  Serial.print(", buffer head:");
  Serial.println(BufferHead);
  int bytesTot = 0;
  int remainingBytes = (myBufferSize + BufferTail - BufferHead) % myBufferSize;  // Check if there are any bytes remaining in the file buffer

  while (remainingBytes > 0)  // While there is still data in the file buffer
  {
    int bytesToWrite = remainingBytes;  // Write the remaining bytes to SD card sdWriteSize bytes at a time
    if (bytesToWrite > sdWriteSize) {
      bytesToWrite = sdWriteSize;
    }
    // write data to buffer
    if ((BufferHead + bytesToWrite) < myBufferSize) {  // case head + bytesToWrite smaller than buffer size
      for (int i = 0; i < bytesToWrite; i++) {
        tempBuffer[i] = myBuffer[BufferHead];
        BufferHead++;
        bytesTot++;
      }
    } else {
      int a = myBufferSize - BufferHead;
      for (int i = 0; i < a; i++) {
        myBuffer[i] = tempBuffer[BufferHead];
        BufferHead++;
        bytesTot++;
      }
      BufferHead = 0;
      for (int i = a; i < (bytesToWrite); i++) {
        tempBuffer[i] = myBuffer[BufferHead];
        BufferHead++;
        bytesTot++;
      }
    }
    dataFile.write(tempBuffer, bytesToWrite);
    remainingBytes -= bytesToWrite;
  }
  Serial.print("Buffer emptied! Data written:");
  Serial.println(bytesTot);
  
}


// Stop data measuring process
void stop_Measuring(fs::FS &fs) {
  measuring = false;  // Set flag to false
  BLE_message = true;
  Serial.println("Turning dataMeasuring OFF!");
  
  //Stop DAC
  stop_trigger();
  digitalWrite(PIN_MUTE , LOW);  // mute
  
  //Stop Micro
  delay(1000);
  stopMicro();
  


  //Read data INS+Laser
  // =-=-=-=-=-=-=-=-=-
   //  Laser data
  int times=0;
  while (RS232.available() ) {
    bitesToWrite = RS232.available();
    if (tempBufferSize < bitesToWrite) {
      bitesToWrite = tempBufferSize;
    }
    RS232.readBytes(tempBuffer, bitesToWrite);
    writeToBuffer(tempBuffer, bitesToWrite);
    times++;
    delay(1);
  }
  Serial.print("Wrote Laser data to buffer N times. N=");
  Serial.println(times);
   
  //  INS data
  times=0;
  while (IMX5.available() ) {
    bitesToWrite = IMX5.available();
    if (tempBufferSize < bitesToWrite) {
      bitesToWrite = tempBufferSize;
    }

    IMX5.readBytes(tempBuffer, bitesToWrite);
    writeToBuffer(tempBuffer, bitesToWrite);
    times++;
    delay(1);
  }
  Serial.print("Wrote INS data to buffer N times. N=");
  Serial.println(times);


  emptyBuffer();

  // output file string
  char filesstring[50];
  strcpy(txString, "Closed file:");
  sprintf(filesstring, " File: %s", dataFileName);
  strcat(txString, filesstring);
  Serial.print("Closing file:");
  Serial.println(filesstring);
  delay(25);
  Write_stop();
  dataFile.close();  // Close the data file
  // Serial.println("Datafile closed.");                     
  strcat(txString, "Measurement stopped successfully");
  BLE_message = true; //Send_tx_String(txString);
}



	




// Get Laser data for 1 second and send it over BLE and serial
void check_laser() {
  Send_tx_String(txString);
  strcpy(txString, "");
  if (!measuring) {
    strcat(txString, "\nLaser data \n# --------------------------------------\n");
    Send_tx_String(txString);
	strcpy(txString, "");
    lastTime = millis();
    while (RS232.read() >= 0)
      ;  // flush the receive buffer.
    while (millis() - lastTime < 1000)
      ;
    int availableBytes = RS232.available();
    if (LaserBufferSize < availableBytes) {
        availableBytes = LaserBufferSize;
		    strcat(txString, "# Buffer full. Data were cut.\n");
    }
	
	
	RS232.readBytes(myBuffer_laser, availableBytes);
    Serial.write(myBuffer_laser, availableBytes);
    if (deviceConnected) {
      pTxCharacteristic->setValue(myBuffer_laser, availableBytes);
      pTxCharacteristic->notify();
      BLE_message = false;
    }
  strcat(txString, "# --------------------------------------\n");
  Send_tx_String(txString);
  } else {
    strcpy(txString, "Datalogging running. Can't run LASER_check now!");
  }
  Send_tx_String(txString);
}

// Get Laser data for 1 second and send it over BLE and serial
void check_INS() {
  Send_tx_String(txString);
  strcpy(txString, "");
  if (!measuring) {
    strcat(txString, "\nIMX5 data \n# --------------------------------------\n");
    Send_tx_String(txString);
	strcpy(txString, "");
    while (IMX5.read() >= 0)
      ;  // flush the receive buffer.
    
    delay(1000);
    int availableBytes = IMX5.available();
	
	if (LaserBufferSize < availableBytes) {
        availableBytes = LaserBufferSize;
		strcat(txString, "# Buffer full. Data were cut.\n");
    }
	
    IMX5.readBytes(myBuffer_laser, availableBytes);
    Serial.write(myBuffer_laser, availableBytes);
    if (deviceConnected) {
      pTxCharacteristic->setValue(myBuffer_laser, availableBytes);
      pTxCharacteristic->notify();
      BLE_message = false;
    }
    
    strcat(txString, "# --------------------------------------\n");
    Send_tx_String(txString);
  } else {
    strcpy(txString, "Datalogging running. Can't run INS_check now!");
  }
  Send_tx_String(txString);
}








// /////////////////////////////////////////////
// -%--------------------------------------------
// Setup and loop
// %%------------------------------------------------------------------------------------------------------------------------------
void setup() {

  Serial.begin(baudrateSerial);

	//Mute and Enable PINs Amplifier MA12070 
	pinMode(PIN_EN , OUTPUT); 
	digitalWrite(PIN_EN , HIGH);   // must be =1 at startup
	pinMode(PIN_MUTE , OUTPUT); 
	digitalWrite(PIN_MUTE , LOW);  // must be =0 at startup
	

	// Initialize the I2C transmitter.	
	Wire.begin();	
	Serial.println("Wire set up");
  
	// Set the initial state of the pins on the PCF8574 devices
	Wire.beginTransmission(CSwitchTx_address); // device 1
    Wire.write(0xff); // all ports off
    uint8_t error = Wire.endTransmission();
    Serial.printf("endTransmission on CSwitch Tx: %u\n", error);
    Wire.begin();
    Wire.beginTransmission(CSwitchCal_address); // device 2
    Wire.write(0xff); // all ports off
    error = Wire.endTransmission();
    Serial.printf("endTransmission on CSwitch CalibCoil: %u\n", error);
	

	if (!tempsens1.begin(Temp1_address)) {
		Serial.println("Couldn't Temperature sensor 1! Check your connections and verify the address is correct.");
	  }	
	if (!tempsens2.begin(Temp2_address)) {
		Serial.println("Couldn't Temperature sensor 2! Check your connections and verify the address is correct.");
	  }
	if (!tempsens3.begin(Temp3_address)) {
		Serial.println("Couldn't Temperature sensor 2! Check your connections and verify the address is correct.");
	  }
	  if (!tempsens4.begin(Temp4_address)) {
		Serial.println("Couldn't Temperature sensor 2! Check your connections and verify the address is correct.");
	  }
	  
  
	//enable MA12070 to be allow to acces registers
	digitalWrite(PIN_EN , LOW);
	write_I2C (MA12070P_address , 0x1D, 0x01 ); // use power mode BBB
	write_I2C (MA12070P_address , 0x25, 0x00 ); // low gain better SignaltoNoise
	

  // Setup RS232 connection to Laser
  //--------------------
  Serial.println("Connecting to LASER");
  RS232.begin(baudrateRS232, SERIAL_8N1, PIN_Rx, PIN_Tx);  // Use this for HardwareSerial
  RS232.setRxBufferSize(Laser_BufferSize);
  Serial.println("Connected to LASER");
  //    delay(500);
  //  	RS232.write(0x1B);  // stop sending data
  //    delay(500);
  //  	RS232.write("SD 0 3");  // set data format
  //  	RS232.write(0x0D);
  //    delay(100);


  // Setup IMX5 connection
  //--------------------
  Serial.println("Connecting to IMX5");
  IMX5.begin(baudrateIMX5, SERIAL_8N1, IMX5_Rx, IMX5_Tx);  // Use this for HardwareSerial
  IMX5.setRxBufferSize(IMX5_BufferSize);
  Serial.println("Started serial to IMX5");
  pinMode(IMX5_strobe, OUTPUT);
  digitalWrite(IMX5_strobe, LOW);


  // Setup BLE connection
  //--------------------
  setup_BLE();


  // Setup SPI connection
  //--------------------
  spi.begin(SCK, MISO, MOSI, CS_SD);  
  Serial.println("Started SPI");
  pinMode(CS_SD, OUTPUT); //set up slave select pins as outputs as the Arduino API doesn't handle automatically pulling SS low
  digitalWrite(CS_SD, HIGH);
  pinMode(CS_DAC, OUTPUT); //set up slave select pins as outputs as the Arduino API doesn't handle automatically pulling SS low
  digitalWrite(CS_DAC, HIGH);
  pinMode(CS_MSP430 , OUTPUT); //set up slave select pins as outputs as the Arduino API doesn't handle automatically pulling SS low
  digitalWrite(CS_MSP430 , HIGH);
  
  pinMode(triggerDAC , OUTPUT); 
  digitalWrite(triggerDAC, HIGH);
  
  
  // Setup SD connection
  //--------------------
  if (!SD.begin(CS_SD, spi, SPI_rate_SD)) {
    Serial.println("Card Mount Failed");
    // while (1)
      ;
  } else{
    Serial.println("SD mounted");
	SD_mounted=1;
  }

  uint8_t cardType = SD.cardType();
  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else if (cardType == CARD_NONE) {
    Serial.println("No SD card attached. ");
  } else {
    Serial.println("UNKNOWN");
  }
  //    listDir(SD, "/", 0);
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
  Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
  Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));


  BLE_message = true;
  strcpy(txString, "Setup completeded. Waiting for command 'START' over Serial of BLE for starting measuring data!");
  Send_tx_String(txString);

}


void loop() {

  //################################# Data logging #############################################

  if (measuring) {  // if measuring is active check for INS and Laser data and save them to SD card

    //  INS data
    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    if (IMX5.available() >= WriteSize_IMX5) {
      bitesToWrite = IMX5.available();
      if (tempBufferSize < bitesToWrite) {
        bitesToWrite = tempBufferSize;
      }

      IMX5.readBytes(tempBuffer, bitesToWrite);
      writeToBuffer(tempBuffer, bitesToWrite);
      logTime_IMX5 = millis();

      if (tempBuffer[bitesToWrite-5]!='*' and IMX5.available()>0 ){
        bitesToWrite = IMX5.available();
        if (tempBufferSize < bitesToWrite) {
          bitesToWrite = tempBufferSize;
        }
        IMX5.readBytes(tempBuffer, bitesToWrite);
        writeToBuffer(tempBuffer, bitesToWrite);
      }
      delay(1);
    }



    //  Laser data
    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    if (RS232.available() >= WriteSize_Laser) {
      bitesToWrite = RS232.available();
      if (tempBufferSize < bitesToWrite) {
        bitesToWrite = tempBufferSize;
      }
      RS232.readBytes(tempBuffer, bitesToWrite);
      writeToBuffer(tempBuffer, bitesToWrite);
      logTime_laser = millis();
      delay(1);
    }



    //  Switch multifreq
    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    if (Nmulti>0 & millis()-MultiTime >MultiPeriod ){
      // long tik =  millis();
      stop_trigger();
      digitalWrite(PIN_MUTE , LOW);  // mute
      delay(3);
      int i=Multifreqs[ i_multi ];
      freq=freqs[i];
      configureSineWave();
      setCswitchTx(CSw_states[i]);
      delay(3);
      digitalWrite(PIN_MUTE , HIGH);  // unmute
      run2();
      trigger();
      
      // int tok=millis();
      // Serial.printf("New frequency is: %d,",freq);
      // Serial.printf("Total switching time: %d ms,",tok-tik);
      // Serial.printf("Switching intervall: %d ms,",tik-MultiTime);
      // Serial.println(" ");
      MultiTime=millis();
      i_multi=(i_multi+1)%Nmulti;
      
    }







    //  Write data to SD
    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    if (((myBufferSize + BufferTail - BufferHead) % myBufferSize) > sdWriteSize) {
      lastTime = millis();
	  
      // write data to buffer
      if ((BufferHead + sdWriteSize) < myBufferSize) {  // case head + sdWritesize smaller than buffer size
        for (int i = 0; i < sdWriteSize; i++) {
          tempBuffer[i] = myBuffer[BufferHead];
          BufferHead++;
        }
      } else {
        int a = myBufferSize - BufferHead;
        for (int i = 0; i < a; i++) {
          tempBuffer[i] = myBuffer[BufferHead];
          BufferHead++;
        }
        BufferHead = 0;
        for (int i = a; i < sdWriteSize; i++) {
          tempBuffer[i] = myBuffer[BufferHead];
          BufferHead++;
        }
      }
      lastTime2 = millis();


      dataFile.write(tempBuffer, sdWriteSize);
      lastTime3 = millis();
      Serial.print(",Data written:");
      Serial.print(sdWriteSize);
      // Serial.print(" ,write to tempBuffer (ms):");
      // Serial.print(lastTime2-lastTime);
      Serial.print(" , in time (ms):");
      Serial.println(lastTime3-lastTime);
      delay(1);
    }


    //==========================================================


    // flush Laser buffer if stuck
    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-
    if ((millis() - logTime_laser) > Laser_log_intervall and RS232.available()) {  //Flush RS232 buffer if data were are not read for too long.
      while (RS232.read() >= 0)
        ;
      Serial.println("Laser buffer flushed");
    }

    // flush IMX5 buffer if stuck
    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-
    if ((millis() - logTime_IMX5) > IMX5_log_intervall and IMX5.available()) {  //Flush RS232 buffer if data were are not read for too long.
      while (IMX5.read() >= 0)
        ;
      Serial.println("IMX5 buffer flushed");
    }

    // flush SDCard every "flushSD_intervall"
    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-
    if (lastTime_flushSD + flushSD_intervall < millis()) {
      dataFile.flush();
      dataFile.flush();
      lastTime_flushSD = millis();
      Serial.println("flushed");
    }

    // Read battery voltage
    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-
	  if (lastTime_VBat + VBat_intervall < millis()) {
      lastTime_VBat = millis();
      logVBat();
	  }

    // Get Temperature
    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-
    if (lastTime_Temp + Temp_intervall < millis()) {
      lastTime_Temp = millis();
      logTemp(tempsens1,1);
      logTemp(tempsens2,2);
	    logTemp(tempsens3,3);
	    // logTemp(tempsens4,4);
    }
	

	

	// Execute calibration if necessary
    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-
    if (calibrating){
      
      if (cal_on & (lastTime_CAL + CAL_intervall_on < millis())) {
        lastTime_CAL = millis(); 
        cal_on=0;
        setCswitchCal(0);
        int msglen=7;
        writeToBuffer((uint8_t*)"CalOff\n", msglen);
        if (CAL_state==N_cal-1){
          calibrating=0;
          CAL_state=0;
          strcpy(txString,"End of calibration");
	        Send_tx_String(txString) ;
        }
        
      } else if(!cal_on & (lastTime_CAL + CAL_intervall_off < millis())){
        lastTime_CAL = millis(); 
        cal_on=1;
        CAL_state++;
        setCswitchCal(CAL_states[CAL_state]);
        
        sprintf(subString,"CalOn %1d\n",CAL_states[CAL_state] );
        int msglen=8;
        for (int i = 0; i < msglen; i++){
          tempBuffer[i] = uint8_t(subString[i]);
        } 
        writeToBuffer(tempBuffer, msglen);
        
        sprintf(subString,"Set cal switch to: %1d",CAL_states[CAL_state] );
        strcpy(txString,subString);
	      Send_tx_String(txString) ;
        
      }
    }

  } else {
    delay(50);  // wait 50 ms if not 
	//  Serial.print("loop() running on core ");
	// Serial.println(xPortGetCoreID());
  }


  //################################# Do other stuff #############################################

  // Moving STOP and START in loop should make them independent of BLE
  if (StopFlag) {
    StopFlag = false;
    // measuring = false;
	  // stop_Measuring(SD);
	xTaskCreatePinnedToCore(stopTask,"TaskEnd",10000, NULL, 1,  &TaskEnd,0); 
  }

  if (closefileFlag) {
    closefileFlag = false;
	  closeFile(SD);
    Serial.println("End of meaurement");  
  }

  if (StartFlag) {
    StartFlag = false;
    measuring = true;
    startMeasuring();
  }


  // Check BLE connection
  if (deviceConnected && BLE_message) {
    // Serial.println("Sending BLE message");
    pTxCharacteristic->setValue(txString);
    pTxCharacteristic->notify();
    BLE_message = false;
    strcpy(txString, "");
    delay(10); // bluetooth stack will go into congestion, if too many packets are sent
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    //delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising();  // restart advertising
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }

  if(BLEinput){  // Check BLE inputs
		Serial.println("Got BLE message");
    Serial.print("Parsing:");
    Serial.print(rxValueBLE);
    sprintf(subString,"BLE input: %s\n",rxValueBLE);
		strcat(txString,subString);
    Send_tx_String(txString);
    delay(10);
    BLEinput=false;
    parse(rxValueBLE);
    Serial.println("BLE message processed");
  }

  if (Serial.available()) {  // Check Serial inputs
    String rxValue = Serial.readString();
    if (rxValue.length() > 0) {
      Serial.println("*********");
      Serial.print("Received Value: ");
      for (int i = 0; i < rxValue.length(); i++)
        Serial.print(rxValue[i]);
        Serial.println();
		parse(rxValue);
      Serial.println("*********");
    }
  }


  delay(5);
}
