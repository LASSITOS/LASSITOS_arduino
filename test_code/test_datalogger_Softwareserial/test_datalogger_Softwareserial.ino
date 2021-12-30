#include <SoftwareSerial.h> //Needed for Software Serial to OpenLog
SoftwareSerial OpenLog;


int statLED = 13;
float dummyVoltage = 3.50; //This just shows to to write variables to OpenLog


void setup() {
	delay(2000);
	pinMode(statLED, OUTPUT);
	Serial.begin(115200);
	Serial.println(PSTR("This serial prints to the COM port"));

	
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  //Connect RXI of OpenLog to pin 27 on Arduino
  OpenLog.begin(38400, SWSERIAL_8N1, 12, 27, false, 256); // 12 = Soft RX pin (not used), 27 = Soft TX pin
  //27 can be changed to any pin.
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  if (!OpenLog) { // If the object did not initialize, then its configuration is invalid
    Serial.println("Invalid SoftwareSerial pin configuration, check config"); 
    while (1) { // Don't continue with invalid configuration
      delay (1000);
    }
  }
  OpenLog.println("This serial records to the OpenLog text file over SoftwareSerial");

  //Write something to OpenLog
  OpenLog.print("Voltage: ");
  OpenLog.println(dummyVoltage);
  dummyVoltage++;
  OpenLog.print("Voltage: ");
  OpenLog.println(dummyVoltage);

  Serial.println("Text written to file. Go look!");
}

void loop() {
  digitalWrite(statLED, HIGH);
  delay(500);
  dummyVoltage++;
  OpenLog.print("Voltage: ");
  OpenLog.println(dummyVoltage);
  Serial.println("New entry in file over SoftwareSerial!");
  digitalWrite(statLED, LOW);
  delay(2000);
}
