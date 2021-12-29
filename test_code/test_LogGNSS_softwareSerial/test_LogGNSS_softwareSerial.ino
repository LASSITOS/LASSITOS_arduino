/*
  Getting time and date using u-blox commands an saving them to SD card.
  By: AcCapellis
  Date: December 19th, 2021
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  Based on Examples of Lybraries.

  Hardware Connections:
  
  Plug a Qwiic cable into the GNSS and a BlackBoard
  Open the serial monitor at 115200 baud to see the output

  Connect RXI of OpenLog to pin 27 on ESP32
  
*/

#include <Wire.h> //Needed for I2C to GNSS

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> 
SFE_UBLOX_GNSS myGNSS;

#include <SoftwareSerial.h> //Needed for Software Serial to OpenLog
SoftwareSerial OpenLog;

long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.
int statLED = 13;


void setup()
{
  pinMode(statLED, OUTPUT);
  digitalWrite(statLED, HIGH);
  delay(1000);
  digitalWrite(statLED, LOW);
  LED_blink(100, 10);
  
  Serial.begin(115200);
  
  while (!Serial)
    ; //Wait for user to open terminal
    LED_blink(100, 4);
    delay(1000);
  Serial.println("Test of loggin GPS data to OpenLog");
  
    
  // Setup GPS connection
  Serial.println("Connecting to GPS");
  Wire.begin();
  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1){
      LED_blink(200, 2);
      delay(2000);
    }
  }
  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  //myGNSS.saveConfiguration();        //Optional: Save the current settings to flash and BBR
  Serial.println("Connection to GPS succesful");


  //Setup OpenLog for saving data on SD
  Serial.println("Connecting to openLog");  
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  //Connect RXI of OpenLog to pin 27 on Arduino
  OpenLog.begin(9600, SWSERIAL_8N1, 12, 27, false, 256); // 12 = Soft RX pin (not used), 27 = Soft TX pin
  //27 can be changed to any pin.
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  if (!OpenLog) { // If the object did not initialize, then its configuration is invalid
    Serial.println("Invalid SoftwareSerial pin configuration, check config"); 
    while (1) { // Don't continue with invalid configuration. Blink LED 3 times every 2 seconds
      LED_blink(200, 3);
      delay(2000);
    }
  }

  //Write header
  OpenLog.println("GNSS log file");
  Serial.println("Connection to OpenLog succesful!"); 
  
}

void loop()
{
  //Query module only every second. Doing it more often will just cause I2C traffic.
  //The module only responds when a new position is available
  if (millis() - lastTime > 1000)
  {
    lastTime = millis(); //Update the timer
    digitalWrite(statLED, HIGH);
    delay(200);
    digitalWrite(statLED, LOW);
    
    long latitude = myGNSS.getLatitude();
    Serial.print(F("Lat: "));
    Serial.print(latitude);

    long longitude = myGNSS.getLongitude();
    Serial.print(F(" Long: "));
    Serial.print(longitude);
    Serial.print(F(" (degrees * 10^-7)"));

    long altitude = myGNSS.getAltitude();
    Serial.print(F(" Alt: "));
    Serial.print(altitude);
    Serial.print(F(" (mm)"));

    byte SIV = myGNSS.getSIV();
    Serial.print(F(" SIV: "));
    Serial.print(SIV);

    Serial.println();
    Serial.print(myGNSS.getYear());
    Serial.print("-");
    Serial.print(myGNSS.getMonth());
    Serial.print("-");
    Serial.print(myGNSS.getDay());
    Serial.print(" ");
    Serial.print(myGNSS.getHour());
    Serial.print(":");
    Serial.print(myGNSS.getMinute());
    Serial.print(":");
    Serial.print(myGNSS.getSecond());
    Serial.println();
    
    OpenLog.print(F(" Alt: "));
    OpenLog.print(altitude);
    Serial.println("New entry in file over SoftwareSerial!");

  }
}

void LED_blink(int len, int times) {
  int a;
  for( a = 0; a < times; a = a + 1 ){
      digitalWrite(statLED, HIGH);
      delay(len);
      digitalWrite(statLED, LOW);
      delay(len);
   }
      
}
