#include <HardwareSerial.h>
HardwareSerial OpenLog(2);


int statLED = 13;
float dummyVoltage = 3.50; //This just shows to to write variables to OpenLog


void setup() {
	delay(2000);
	pinMode(statLED, OUTPUT);
	Serial.begin(115200);
	Serial.println(PSTR("This serial prints to the COM port"));
 
	
	
	//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  //Connect RXI of OpenLog to pin 17 and 16 on ESP32
  OpenLog.begin(115200);
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

	
	// high speed half duplex, turn off interrupts during tx
//	if (!OpenLog) { // If the object did not initialize, then its configuration is invalid
//    Serial.println("Invalid SoftwareSerial pin configuration, check config"); 
//    while (1) { // Don't continue with invalid configuration
//      delay (1000);
//    }
//  }
  OpenLog.println("This serial records to the OpenLog text file");

  //Write something to OpenLog
  OpenLog.println("Hi there! How are you today?");
  OpenLog.print("Voltage: ");
  OpenLog.println(dummyVoltage);
  dummyVoltage++;
  OpenLog.print("Voltage: ");
  OpenLog.println(dummyVoltage);

  Serial.println("Text written to file. Go look!");
}

void loop() {
  digitalWrite(statLED, HIGH);
  delay(5);
  dummyVoltage++;
  OpenLog.print("Voltage: ");
  OpenLog.println(dummyVoltage);
  OpenLog.println("Papla pirla che tra po te parlat piu. Te mandi gio a l'inferno da dont ca tes vegnu.");
  OpenLog.println( "The purpose of our thesis is to test the new GNSS module, ZED-F9P released by u-blox. In order to do this, we analyze the measurements to see which applications the module is best suited for. The measurement methods we choose to use for the analysis, are different types of real-time measurements, as well as different types of phase-based static measurements. The real-time measurements consist of dynamic and stationary (static) measurements, as well as the accuracy we can expect.");
  Serial.print(".");
  digitalWrite(statLED, LOW);
  delay(20);
}
