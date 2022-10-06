/*----------------------------------------------------------
* Title: I2C basic communication set-up
* Author: Rien Oortgiesen
* This code demonstrates basic I2C communication
* using Arduino UNO together with MA120XXX devices
* Use:
* The code uses I2C lib from Wayne Truchsess which allows repeated
* start and can be used in an interrupt service routine
*
* I2C hardware config:
* Uno breakout: SCL = A5; SDA = A4 GND = GND;
* Reference board CONN_COM: SCL = pin 4; SDA = pin 3; GND = pin 2
*
* Revisions:
* D1a: use of external lib initial test working
* F1: final version for demonstration
*
* This code is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 2.1 of the License, or (at your option) any later version.
*/

#include <I2C.h>
const byte LED = 13; // LED pin number
const byte BUTTON = 2; // BUTTON pin number
volatile int state = LOW;


// Interrupt Service Routine (ISR)
void switchPressed ()
{
state = !state; // change state
digitalWrite(LED, state); //write state to LED
write_I2C(state); //jump to I2C handling
}

void setup ()
{
pinMode (LED, OUTPUT); // so we can update the LED
digitalWrite (BUTTON, HIGH); // internal pull-up resistor

// attach interrupt handler (0 is the internal interrupt attached to pin 2)
attachInterrupt (0, switchPressed, RISING);
// start with LED off
digitalWrite(LED, 0);
// set audio_in_mode_ext
I2c.begin();
I2c.write(0x20,0x27,0x28); //audio_in_mode_ext = 1
I2c.end();
// set in 26dB audio_in_mode
I2c.begin();
I2c.write(0x20,0x25,0x30); //audio_in_mode = 1
I2c.end();
// set in 20dB audio_in_mode
//I2c.write(0x20,0x25,0x10); //audio_in_mode = 0
//digitalWrite(LED, 0);
} // end of setup
void loop ()
{
// wait for interrupt
}
void write_I2C (bool dB){
	I2c.begin();
	if( dB == true ){
		I2c.write(0x20,0x25,0x30); //audio_in_mode = 1
	}
	else	{
		I2c.write(0x20,0x25,0x10); //audio_in_mode = 0
	}
	I2c.end();
}