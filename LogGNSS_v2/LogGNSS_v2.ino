/*
  Getting time and date using u-blox commands an saving them to SD card.
  By: AcCapellis
  Date: December 19th, 2021
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  Based on Examples from used libraries and hardware.

  Hardware Connections:
  
  Plug a Qwiic cable into the GNSS and a BlackBoard
  Open the serial monitor at 115200 baud to see the output
  
  Connect RXI of OpenLog to pin 27 on ESP32 and TXI of OpenLog to pin 12 on ESP32
  
  
  BLE:
  Set up BLE connections with UART communication
  Start new data files if "START" is received and stop current data files if "STOP" is received
  
*/

#include <Wire.h> //Needed for I2C to GNSS

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> 
SFE_UBLOX_GNSS myGNSS;

#include <SoftwareSerial.h> //Needed for Software Serial to OpenLog
SoftwareSerial OpenLog;

long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.
int statLED = 13;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
int PIN_Rx = 12; // 12 = Soft RX pin,
int PIN_Tx = 27; // 27 = Soft TX pin, //Connect RXI of OpenLog to pin 27 on Arduino 
int resetOpenLog = 25; //This pin resets OpenLog. Connect pin 25 to pin GRN on OpenLog.
//PINs can be changed to any pin.
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bool logging = true;

int NavigationFrequency = 1;
int BufferSize = 301;

char headerFileName[24]; //Max file name length is 23 characters)
char dataFileName[24]; //Max file name length is 23 characters)
char hFN2[12] ; 
char dataFileName2[12];

#define packetLength 100 // NAV PVT is 92 + 8 bytes in length (including the sync chars, class, id, length and checksum bytes)

long Year ;
long Month ;
long Day ;
long Hour ;
long Minute ;
long Second;


// Setting for BLE connection
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;
char txString[100];
bool BLE_message=false;
bool BLE_stop=false;
bool BLE_start=false;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"






void LED_blink(int len, int times ) { // Used for blinking LED  times at interval len in milliseconds
  int a;
  for( a = 0; a < times; a = a + 1 ){
      digitalWrite(statLED, HIGH);
      delay(len);
      digitalWrite(statLED, LOW);
      if  (times > 1) delay(len); // Wait after blinking next time
   }   
}





// /////////////////////////////////////////////
// ---------------------------------------------
//GNSS code
// ---------------------------------------------
// /////////////////////////////////////////////


// Callback: printPVTdata will be called when new NAV PVT data arrives
// See u-blox_structs.h for the full definition of UBX_NAV_PVT_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setAutoPVTcallback
//        /                  _____  This _must_ be UBX_NAV_PVT_data_t
//        |                 /               _____ You can use any name you like for the struct
//        |                 |              /
//        |                 |              |
void printPVTdata(UBX_NAV_PVT_data_t ubxDataStruct)
{
    Serial.println();

    Serial.print(F("Time: ")); // Print the time
    uint8_t hms = ubxDataStruct.hour; // Print the hours
    if (hms < 10) Serial.print(F("0")); // Print a leading zero if required
    Serial.print(hms);
    Serial.print(F(":"));
    hms = ubxDataStruct.min; // Print the minutes
    if (hms < 10) Serial.print(F("0")); // Print a leading zero if required
    Serial.print(hms);
    Serial.print(F(":"));
    hms = ubxDataStruct.sec; // Print the seconds
    if (hms < 10) Serial.print(F("0")); // Print a leading zero if required
    Serial.print(hms);
    Serial.print(F("."));
    unsigned long millisecs = ubxDataStruct.iTOW % 1000; // Print the milliseconds
    if (millisecs < 100) Serial.print(F("0")); // Print the trailing zeros correctly
    if (millisecs < 10) Serial.print(F("0"));
    Serial.print(millisecs);

    long latitude = ubxDataStruct.lat; // Print the latitude
    Serial.print(F(" Lat: "));
    Serial.print(latitude);

    long longitude = ubxDataStruct.lon; // Print the longitude
    Serial.print(F(" Long: "));
    Serial.print(longitude);
    Serial.print(F(" (degrees * 10^-7)"));

    long altitude = ubxDataStruct.hMSL; // Print the height above mean sea level
    Serial.print(F(" Height above MSL: "));
    Serial.print(altitude);
    Serial.println(F(" (mm)"));
	
	  BLE_message=true;
	  char filesstring[200];
    sprintf(txString, "Time:  %02d:%02d:%02d.%04d,  Lat: %d, Long: %d, Elev: %.1d (m) /n",
			   ubxDataStruct.hour,ubxDataStruct.min,ubxDataStruct.sec,
			   latitude * 0.0000001,longitude * 0.0000001,altitude/1000);
}




// /////////////////////////////////////////////
// ---------------------------------------------
// SD card funtions und code
// ---------------------------------------------
// /////////////////////////////////////////////

//This function pushes OpenLog into command mode
void gotoCommandMode(void) {
  //Send three control z to enter OpenLog command mode
  //Works with Arduino v1.0
  OpenLog.write(26);
  OpenLog.write(26);
  OpenLog.write(26);

  //Wait for OpenLog to respond with '>' to indicate we are in command mode
  while (1) {
    if (OpenLog.available()){
      if (OpenLog.read() == '>') break; 
    }
  }
  Serial.println("In command mode");
}

//This function pushes OpenLog into command mode if it is not there jet. 
//Use with caution. It will push garbage data to Openlog that will be saved to Logfile if it is not in Command mode. 
void checkandgotoCommandMode(void) {
  bool resp = false;
  int count = 0;
  Serial.print("Going to command mode.");
  while (!resp){
    ++count;
    //Send three control z to enter OpenLog command mode
    //Works with Arduino v1.0
    OpenLog.write(26);
    OpenLog.write(26);
    OpenLog.write(26);
  
    //Wait for OpenLog to respond with '>' to indicate we are in command mode
    for (int timeOut = 0 ; timeOut < 500 ; timeOut++) {
      if (OpenLog.available()){
        if (OpenLog.read() == '>') {
          resp = true;
          Serial.println(F("OpenLog is in Command Mode"));
          break;  
        }
      }
      delay(1);
    }
    if (count > 2) { 
      Serial.println(F("Can't get in command mode. Freezing."));
      while (1){
      }
    } else if (!resp) {
      Serial.println(F("Problem getting in command mode. Retry in case OpenLog was already in command mode"));
    }
  }
}


//Setups up the software serial, resets OpenLog so we know what state it's in, and waits
//for OpenLog to come online and report '<' that it is ready to receive characters to record
void setupOpenLog(void) {
  pinMode(resetOpenLog, OUTPUT);
  Serial.println(F("Connecting to openLog"));  
  OpenLog.begin(9600, SWSERIAL_8N1, PIN_Rx, PIN_Tx, false, 256); // 12 = Soft RX pin, 27 = Soft TX pin

  if (!OpenLog) { // If the object did not initialize, then its configuration is invalid
    Serial.println(F("Invalid SoftwareSerial pin configuration, check config")); 
    while (1) { // Don't continue with invalid configuration. Blink LED 3 times every 2 seconds
      LED_blink(200, 3);
      delay(2000);
    }
  }

  //Reset OpenLog
  digitalWrite(resetOpenLog, LOW);
  delay(5);
  digitalWrite(resetOpenLog, HIGH);

  //Wait for OpenLog to respond with '<' to indicate it is alive and recording to a file
  while (1) {
    if (OpenLog.available())
      if (OpenLog.read() == '<') break;
  }
  Serial.println("OpenLog online");
}


//This function creates a given file and then opens it in append mode (ready to record characters to the file)
//Then returns to listening mode
//This function assumes the OpenLog is in command mode
void createFile(char *fileName) {
  //New way
  OpenLog.print("new ");
  OpenLog.print(fileName);
  OpenLog.write(13); //This is \r
//  OpenLog.println(fileName); //regular println works with OpenLog v2.51 and above

  //Wait for OpenLog to return to waiting for a command
    //Wait for OpenLog to respond with '>' to indicate we are in command mode
  while (1) {
    if (OpenLog.available())
      if (OpenLog.read() == '>') break;
  }
  Serial.print("Made new file");

  OpenLog.print("append ");
//  OpenLog.println(fileName); //regular println works with OpenLog v2.51 and above
  OpenLog.print(fileName);
  OpenLog.write(13); //This is \r

  //Wait for OpenLog to indicate file is open and ready for writing
    //Wait for OpenLog to respond with '>' to indicate we are in command mode
//  while (1) {
//    if (OpenLog.available())
//      if (OpenLog.read() == '>') break;
//  }
  Serial.println("OpenLog is now waiting for characters and will record them to the new file");
  //OpenLog is now waiting for characters and will record them to the new file
}


//Reads the contents of a given file and dumps it to the serial terminal
//This function assumes the OpenLog is in command mode
void readFile(char *fileName) {
  
  while(OpenLog.available()) OpenLog.read(); //Clear incoming buffer

  OpenLog.print("read ");
  OpenLog.print(fileName);
  OpenLog.write(13); //This is \r

  //The OpenLog echos the commands we send it by default so we have 'read log823.txt\r' sitting
  //in the RX buffer. Let's try to not print this.
  while (1) {
    if (OpenLog.available())
      if (OpenLog.read() == '\r') break;
  }

  Serial.print("Reading from file: ");
  Serial.println(fileName);
  //This will listen for characters coming from OpenLog and print them to the terminal
  //This relies heavily on the SoftSerial buffer not overrunning. This will probably not work
  //above 38400bps.
  //This loop will stop listening after 1 second of no characters received
  for (int timeOut = 0 ; timeOut < 1000 ; timeOut++) {
    while (OpenLog.available()) {
      char tempString[100];

      int spot = 0;
      while (OpenLog.available()) {
        tempString[spot++] = OpenLog.read();
        if (spot > 98) break;
      }
      tempString[spot] = '\0';
      Serial.write(tempString); //Take the string from OpenLog and push it to the Arduino terminal
      timeOut = 0;
    }

    delay(1);
  }

  //This is not perfect. The above loop will print the '.'s from the log file. These are the two escape characters
  //recorded before the third escape character is seen.
  //It will also print the '>' character. This is the OpenLog telling us it is done reading the file.

  //This function leaves OpenLog in command mode
}


//Check the stats of the SD card via 'disk' command
//This function assumes the OpenLog is in command mode
void readDisk() {
  OpenLog.println("disk");
  
  //The OpenLog echos the commands we send it by default so we have 'disk\r' sitting 
  //in the RX buffer. Let's try to not print this.
  while(1) {
    if(OpenLog.available())
      if(OpenLog.read() == '\r') break;
  }  

  //This will listen for characters coming from OpenLog and print them to the terminal
  //This relies heavily on the SoftSerial buffer not overrunning. This will probably not work
  //above 38400bps.
  //This loop will stop listening after 1 second of no characters received
  for(int timeOut = 0 ; timeOut < 1000 ; timeOut++) {
    while(OpenLog.available()) {
      char tempString[100];
      
      int spot = 0;
      while(OpenLog.available()) {
        tempString[spot++] = OpenLog.read();
        if(spot > 98) break;
      }
      tempString[spot] = '\0';
      Serial.write(tempString); //Take the string from OpenLog and push it to the Arduino terminal
      timeOut = 0;
    }

    delay(1);
  }

  //This is not perfect. The above loop will print the '.'s from the log file. These are the two escape characters
  //recorded before the third escape character is seen.
  //It will also print the '>' character. This is the OpenLog telling us it is done reading the file.  

  //This function leaves OpenLog in command mode
}



void readDateTime(){
  Year = myGNSS.getYear();
  Month = myGNSS.getMonth();
  Day = myGNSS.getDay();
  Hour = myGNSS.getHour();
  Minute = myGNSS.getMinute();
  Second = myGNSS.getSecond();
}

void  printDateTime() {  
  Serial.print(Year);
  Serial.print("-");
  Serial.print(Month);
  Serial.print("-");
  Serial.print(Day);
  Serial.print("  ");
  Serial.print(Hour);
  Serial.print(":");
  Serial.print(Minute);
  Serial.print(":");
  Serial.print(Second);
}

void  Write_header() {  
  OpenLog.println(F("# GNSS log file"));
  OpenLog.print("# Date: ");
  OpenLog.print(Year);
  OpenLog.print("-");
  OpenLog.print(Month);
  OpenLog.print("-");
  OpenLog.println(Day);
  OpenLog.print("# Time: ");
  OpenLog.print(Hour);
  OpenLog.print(":");
  OpenLog.print(Minute);
  OpenLog.print(":");
  OpenLog.println(Second);
  OpenLog.println(F("# --------------------------------------"));
  OpenLog.print(F("# UBX binary data are in  file: "));
  OpenLog.print(dataFileName);
  OpenLog.println(F(" "));
}

char nth_letter(int n)
{
    return "abcdefghijklmnopqrstuvwxyz"[n];
}


// Make new files in SD card. file names are derived from GNSS time if available. 
void  makeFiles() {  
  Serial.println(F("Making new files"));

  // Check for GPS to have good time and date
  int count = 0;
  while (1) {  // Check for GPS to have good time and date
    ++count;
    if ((myGNSS.getTimeValid() == true) && (myGNSS.getDateValid() == true)) {
      Serial.print(F("Date and time are valid. It is: "));
      readDateTime();
      printDateTime();
      Serial.println();
      break;
    }else if (count > 5) {
        Year = 2000 ;
        Month = 01;
        Day = 01;
        Hour = random(23) ;
        Minute = random(60);
        Second = random(60);
        break;
        Serial.println(F("GPS is not good. Making random filename with date:"));
        printDateTime();
        Serial.println(); 
    }
    
    Serial.println(F("Date or time are not valid. Waiting for better GPS connection."));
    readDateTime();
    Serial.print(F("Date and time: "));
    printDateTime();
    Serial.println();
    LED_blink(1000, 5);
  }
  
  sprintf(headerFileName, "%02d%02d%02d_%02d%02d.txt", Year%1000 ,Month ,Day ,Hour,Minute);  // create name of header file
  sprintf(dataFileName, "%02d%02d%02d_%02d%02d.ubx", Year%1000 ,Month ,Day ,Hour,Minute);   // create name of data file
  Serial.println(headerFileName);
  Serial.println(dataFileName);
  
  
  gotoCommandMode(); //Puts OpenLog in command mode.
//  readDisk();
//  checkandgotoCommandMode();
  Serial.print(F("Making header file:"));
  createFile(headerFileName); //Creates a new file called Date_Time.txt
  Serial.print(F("created file: "));
  Serial.println(headerFileName);
  Write_header();
  
  
  gotoCommandMode();  //Puts OpenLog in command mode.
//  readFile(headerFileName);

  Serial.print(F("Making data file:"));
  Serial.println(dataFileName);
  createFile(dataFileName); //Creates a new file called Date_Time.ubx
  Serial.print(F("Created file: "));
  Serial.println(dataFileName);
  LED_blink(100,10);
}


void  Write_stop() {  
  OpenLog.print(F("Measurements stopped on: "));
  OpenLog.print(Year);
  OpenLog.print("-");
  OpenLog.print(Month);
  OpenLog.print("-");
  OpenLog.print(Day);
  OpenLog.print(", ");
  OpenLog.print(Hour);
  OpenLog.print(":");
  OpenLog.print(Minute);
  OpenLog.print(":");
  OpenLog.println(Second);
  OpenLog.println(F("# --------------------------------------"));
}


// Stop data logging process
void stop_logging() {
	logging = false; // Set flag to false
	BLE_message=true;
	strcpy(txString,"Turning datalogging OFF!");
	char filesstring[50];
	sprintf(filesstring, " Files: %s, %s",headerFileName, dataFileName);
	strcat(txString,filesstring);
	Serial.println("Turning datalogging OFF!");
	Serial.println(filesstring);
	LED_blink(25, 4);
	
	gotoCommandMode(); //Puts OpenLog in command mode.
	OpenLog.print("append ");
	OpenLog.print(headerFileName);
	OpenLog.write(13); //This is 
	 
	// while (1) {
		// if (OpenLog.available())
		  // if (OpenLog.read() == '>') break;
	// }
	Serial.println("Ready to append");	
	LED_blink(25, 4);
	readDateTime();
	Write_stop();
	Serial.println("Measurement stopped successfully");	
}

// Restart data logging process
void restart_logging() {
	logging = false;
	Serial.println("Turning ON datalogging with new files!");
	BLE_message=true;
	strcpy(txString,"Turning ON datalogging with new files!");
	LED_blink(100, 5);

	// //Reset OpenLog
	// digitalWrite(resetOpenLog, LOW);
	// delay(5);
	// digitalWrite(resetOpenLog, HIGH);

	// //Wait for OpenLog to respond with '<' to indicate it is alive and recording to a file
	// while (1) {
		// if (OpenLog.available())
		  // if (OpenLog.read() == '<') break;
	// }
	// Serial.println("OpenLog resete and back online");
	// LED_blink(100, 5);
	makeFiles();
	delay(500);
	logging = true;
}




// /////////////////////////////////////////////
// ---------------------------------------------
// BLE funtions und code
// ---------------------------------------------
// /////////////////////////////////////////////

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("Received Value: ");
        for (int i = 0; i < rxValue.length(); i++)
          Serial.print(rxValue[i]);
        Serial.println();

        //Start new data files if START is received and stop current data files if STOP is received
        if (rxValue.find("START") != -1) { 
			BLE_start=true;
       }
        else if (rxValue.find("STOP") != -1) {
			BLE_stop=true;
        }
        Serial.println();
        Serial.println("*********");
      }
    }
};



void setup_BLE() {
  Serial.println("bluetooth connection: 'GNSS BLE UART' ");
  // Create the BLE Device
  BLEDevice::init("GNSS BLE UART");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
										CHARACTERISTIC_UUID_TX,
										BLECharacteristic::PROPERTY_NOTIFY
									);
                      
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
											 CHARACTERISTIC_UUID_RX,
											BLECharacteristic::PROPERTY_WRITE
										);

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}









// /////////////////////////////////////////////
// -%--------------------------------------------
// Setup and loop
// %%------------------------------------------------------------------------------------------------------------------------------
void setup(){
    pinMode(statLED, OUTPUT);
    digitalWrite(statLED, HIGH);
    delay(1000);
    digitalWrite(statLED, LOW);

    pinMode(resetOpenLog, OUTPUT);
    
    Serial.begin(115200);
    while (!Serial) {
      ; //Wait for user to open terminal
      LED_blink(100, 4);
      delay(1000);
    }  
    Serial.println(F("Loggin GPS data to OpenLog"));
    
    // Setup GPS connection
    Serial.println("Connecting to GPS");
    Wire.begin();
    myGNSS.setFileBufferSize(BufferSize); // setFileBufferSize must be called _before_ .begin
    if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
    {
      Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
      while (1){
        LED_blink(200, 2);
        delay(2000);
        if (myGNSS.begin() == false) { //Connect to the u-blox module using Wire port
          Serial.println(F("u-blox GNSS detected I2C address. Reconnection was successfull."));
          break;
        }
      }
    }
    
    myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
    Serial.println(F("Connection to GPS succesful"));

	setup_BLE();

    //Setup OpenLog for saving data on SD
    setupOpenLog(); //Resets logger and waits for the '<' I'm alive character
    Serial.println(F("Connection to OpenLog succesful!"));
    //Write header file and create data file ready for writing binary data
    makeFiles();
    delay(1000);


    // setting up GPS for automatic messages
    Serial.println(F("setting up GPS for automatic messages"));
    myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
    myGNSS.setNavigationFrequency(NavigationFrequency); //Produce one navigation solution per second
    myGNSS.setAutoPVTcallback(&printPVTdata); // Enable automatic NAV PVT messages with callback to printPVTdata
    myGNSS.logNAVPVT(); // Enable NAV PVT data logging

    Serial.println(F("Setup completeded."));
}


void loop(){
  
  // if logging is avctive check for GNSS data and save them to SD card in binary file
	if (logging){   
      	Serial.print(".");
		if (millis() - lastTime > 5000)  {  // make new line every 5 seconds
			lastTime = millis(); //Update the timer
			Serial.println(" ");
		}
		
		
		myGNSS.checkUblox(); // Check for the arrival of new data and process it.
		myGNSS.checkCallbacks(); // Check if any callbacks are waiting to be processed.

		if (myGNSS.fileBufferAvailable() >= packetLength) // Check to see if a new packetLength-byte NAV PVT message has been stored
			{
			uint8_t myBuffer[packetLength]; // Create our own buffer to hold the data while we write it to SD card

			myGNSS.extractFileBufferData((uint8_t *)&myBuffer, packetLength); // Extract exactly packetLength bytes from the UBX file buffer and put them into myBuffer

			OpenLog.write(myBuffer, packetLength); // Write exactly packetLength bytes from myBuffer to the ubxDataFile on the SD card

			//printBuffer(myBuffer); // Uncomment this line to print the data as Hexadecimal bytes

			LED_blink(10, 1);
		}
	}
  
    // Check BLE connection
	if (deviceConnected && BLE_message ) {
		pTxCharacteristic->setValue(txString);
		pTxCharacteristic->notify();
		BLE_message=false;
		//delay(10); // bluetooth stack will go into congestion, if too many packets are sent
	}
	// disconnecting
	if (!deviceConnected && oldDeviceConnected) {
		//delay(500); // give the bluetooth stack the chance to get things ready
		pServer->startAdvertising(); // restart advertising
		Serial.println("start advertising");
		oldDeviceConnected = deviceConnected;
	}
	// connecting
	if (deviceConnected && !oldDeviceConnected) {
		// do stuff here on connecting
		oldDeviceConnected = deviceConnected;
	}
  
	if (BLE_stop){
		stop_logging();
		BLE_stop= false;
	} else if (BLE_start){
		restart_logging();
		BLE_start= false;
	}
		
  
	if (Serial.available()) // Check if the user wants to stop logging
		{
		stop_logging();
	}	



	delay(50);
}
