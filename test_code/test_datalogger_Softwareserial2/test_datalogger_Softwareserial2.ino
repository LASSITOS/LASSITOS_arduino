#include <SoftwareSerial.h> //Needed for Software Serial to OpenLog
SoftwareSerial OpenLog;


float dummyVoltage = 3.50; //This just shows to to write variables to OpenLog

long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.
int statLED = 13;
int resetOpenLog = 25; //This pin resets OpenLog. Connect pin 25 to pin GRN on OpenLog.

int NavigationFrequency = 4;
int BufferSize = 501;

char headerFileName[16]; //Max file name length is "12345678.123" (12 characters)
char dataFileName[16]; //Max file name length is "12345678.123" (12 characters)
char hFN2[10] ; 
char dataFileName2[10];

#define packetLength 100 // NAV PVT is 92 + 8 bytes in length (including the sync chars, class, id, length and checksum bytes)

long Year ;
long Month ;
long Day ;
long Hour ;
long Minute ;
long Second;


void LED_blink(int len, int times ) { // Used for blinking LED  times at interval len in milliseconds
  int a;
  for( a = 0; a < times; a = a + 1 ){
      digitalWrite(statLED, HIGH);
      delay(len);
      digitalWrite(statLED, LOW);
      if  (times > 1) delay(len); // Wait after blinking next time
   }   
}

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
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  //Connect RXI of OpenLog to pin 27 on Arduino
  OpenLog.begin(9600, SWSERIAL_8N1, 12, 27, false, 256); // 12 = Soft RX pin (not used), 27 = Soft TX pin
  //27 can be changed to any pin.
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

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
  OpenLog.println(fileName); //regular println works with OpenLog v2.51 and above
//  OpenLog.print(fileName);
//  OpenLog.write(13); //This is \r

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

void  makeFiles() {  // write header in SD file
  Serial.println(F("Making new files"));

  // Check for GPS to have good time and date
  Year = 2021 ;
  Month = 01;
  Day = 01;
  Hour = random(23) ;
  Minute = random(60);
  Second = random(60);

  LED_blink(100,10);
  Serial.println("Survived 1 second");
  
  sprintf(headerFileName, "%02d%02d%02d_%02d%02d.txt", Year%1000 ,Month ,Day ,Hour,Minute);  // create name of header file
  sprintf(dataFileName, "%02d%02d%02d_%02d%02d.ubx", Year%1000 ,Month ,Day ,Hour,Minute);   // create name of data file
  Serial.println(headerFileName);
  Serial.println(dataFileName);
  
  
  gotoCommandMode(); //Puts OpenLog in command mode.
//  readDisk();
//  checkandgotoCommandMode();
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
  Serial.println("Survived 1 second");
  
}




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
    


    //Setup OpenLog for saving data on SD
    setupOpenLog(); //Resets logger and waits for the '<' I'm alive character
    Serial.println(F("Connection to OpenLog succesful!"));
    //Write header file and create data file ready for writing binary data
    makeFiles();
    delay(1000);

}


void loop() {
  digitalWrite(statLED, HIGH);
  delay(500);
  dummyVoltage++;
//  OpenLog.print("Voltage: ");
//  OpenLog.println(dummyVoltage);
  OpenLog.write(random(999));
  Serial.println("New entry in file over SoftwareSerial!");
  digitalWrite(statLED, LOW);
  delay(1000);

  if (Serial.available())   { // Check if the user wants to stop logging
    Serial.println(F("\r\nLogging stopped. Freezing..."));
    gotoCommandMode();
    readDisk();
    while(1); // Do nothing more
  }

}
