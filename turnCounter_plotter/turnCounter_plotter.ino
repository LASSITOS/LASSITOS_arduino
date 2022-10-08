int const PULSE_SENSOR_PIN = 26;   // 'S' Signal pin connected to A0

int Signal;                // Store incoming ADC data. Value can range from 0-1024
int Threshold = 500;       // Determine which Signal to "count as a beat" and which to ignore.
int count =0;
bool isup= 0;
long timer=0;


void setup() {
  pinMode(LED_BUILTIN,OUTPUT);  // Built-in LED will blink to your heartbeat
  Serial.begin(9600);           // Set comm speed for serial plotter window
}

void loop() {

  Signal = analogRead(PULSE_SENSOR_PIN); // Read the sensor value
  Serial.print(Signal); 
  Serial.print(" "); // Send the signal value to serial plotter
  Serial.println(count); 
  
  if(Signal > Threshold){                // If the signal is above threshold, turn on the LED
    digitalWrite(LED_BUILTIN,HIGH);
    if (!isup){
    count++;
    isup=1;
    }
  } else {
    digitalWrite(LED_BUILTIN,LOW);     // Else turn off the LED
    if (isup){
    isup=0;
    }
  }
//  if (millis() - timer > 1000){   
//    Serial.print("turns:");
//    Serial.println(count); 
//    timer = millis(); //Update the timer
//  }
//  
  delay(20);
}
