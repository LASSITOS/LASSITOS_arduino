/*


 This example generates a sinus waves (and other periodic waveforms) based tone at a specified frequency
 and sample rate. Then outputs the data using the I2S interface to a MA12070P Breakout board.
 Circuit:
   * MA12070P  :  ESP32
   * -------------------
   * I2S:
   * ---------------
   * BKLC  : 12   # I2S bit clock.  sckPin or constant PIN_I2S_SCK in ESP32 library 
   * WS    : 27      # I2S word clock. fsPin or constant PIN_I2S_FS ESP32 library 
   * GND   : GND
   * SDO   : 33      # I2S audio data
   * MCLK  : 26      # I2S master clock
   * 
   * I2C:
   * ---------------
   * SCL   :  SCL 22   #I2C clock,  need pull up resistor (e.g 10k)
   * SDA   :  SDA 23   # I2C data,  need pull up resistor  (e.g 10k)
   * GND   :  GND
   * EN    :  32      # enable or disable the amplifier   ENABLE = 1 -> disabled. On board pulled to GND
   * MUTE  :  14      # mute or unmute the amplifier   /MUTE = 0 -> mute.  On board pulled to VDD 
  
  By: AcCapelli
  Date: September 15th, 2022
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.
  version v1.0
  
  Based on Examples from used libraries and hardware. EG: Code created 17 November 2016 by Sandeep Mistry
 */


// --------------------------------
// Settings I2S
//---------------------------------
#define PIN_I2S_SCK 12

#define PIN_I2S_FS 27

#define PIN_I2S_SD 33

#define PIN_I2S_SD_OUT 26

//#define PIN_I2S_SD_IN 35

//#include <driver/i2s.h> // or  
#include <I2S.h>



#define SAMPLERATE_HZ 16000  // The sample rate of the audio.  ??ESP32 max 16kHz??
                             
#define BITS_SAMPLE   16     // The number of bits per sample. ??ESP32 max 16??

#define AMPLITUDE     ((1<<12)-1)   // Set the amplitude of generated waveforms.  This controls how loud
                             // the signals are, and can be any value from 0 to 2**BITS_SAMPLE - 1.  Start with
                             // a low value to prevent damaging speakers!

#define WAV_SIZE      256    // The size of each generated waveform.  The larger the size the higher
                             // quality the signal.  A size of 256 is more than enough for these simple
                             // waveforms.


// --------------------------------
// Settings I2C
//---------------------------------

#include <I2C.h>
#define  I2C_address 0x20

#define PIN_EN 32
#define PIN_MUTE 14

// --------------------------------
// Settings Function generation
//---------------------------------


// Define the frequency of music notes (from http://www.phy.mtu.edu/~suits/notefreqs.html):
#define C4_HZ      261.63
#define D4_HZ      293.66
#define E4_HZ      329.63
#define F4_HZ      349.23
#define G4_HZ      392.00
#define A4_HZ      440.00
#define B4_HZ      493.88

// Define a C-major scale to play all the notes up and down.
float scale[] = { C4_HZ, D4_HZ, E4_HZ, F4_HZ, G4_HZ, A4_HZ, B4_HZ, A4_HZ, G4_HZ, F4_HZ, E4_HZ, D4_HZ, C4_HZ };

// Store basic waveforms in memory.
int32_t sine[WAV_SIZE]     = {0};
int32_t sawtooth[WAV_SIZE] = {0};
int32_t triangle[WAV_SIZE] = {0};
int32_t square[WAV_SIZE]   = {0};
















void generateSine(int32_t amplitude, int32_t* buffer, uint16_t length) {
  // Generate a sine wave signal with the provided amplitude and store it in
  // the provided buffer of size length.
  for (int i=0; i<length; ++i) {
    buffer[i] = int32_t(float(amplitude)*sin(2.0*PI*(1.0/length)*i));
  }
}
void generateSawtooth(int32_t amplitude, int32_t* buffer, uint16_t length) {
  // Generate a sawtooth signal that goes from -amplitude/2 to amplitude/2
  // and store it in the provided buffer of size length.
  float delta = float(amplitude)/float(length);
  for (int i=0; i<length; ++i) {
    buffer[i] = -(amplitude/2)+delta*i;
  }
}

void generateTriangle(int32_t amplitude, int32_t* buffer, uint16_t length) {
  // Generate a triangle wave signal with the provided amplitude and store it in
  // the provided buffer of size length.
  float delta = float(amplitude)/float(length);
  for (int i=0; i<length/2; ++i) {
    buffer[i] = -(amplitude/2)+delta*i;
  }
    for (int i=length/2; i<length; ++i) {
    buffer[i] = (amplitude/2)-delta*(i-length/2);
  }
}

void generateSquare(int32_t amplitude, int32_t* buffer, uint16_t length) {
  // Generate a square wave signal with the provided amplitude and store it in
  // the provided buffer of size length.
  for (int i=0; i<length/2; ++i) {
    buffer[i] = -(amplitude/2);
  }
    for (int i=length/2; i<length; ++i) {
    buffer[i] = (amplitude/2);
  }
}

void playWave(int32_t* buffer, uint16_t length, float frequency, float seconds) {
  // Play back the provided waveform buffer for the specified
  // amount of seconds.
  // First calculate how many samples need to play back to run
  // for the desired amount of seconds.
  uint32_t iterations = seconds*SAMPLERATE_HZ;
  // Then calculate the 'speed' at which we move through the wave
  // buffer based on the frequency of the tone being played.
  float delta = (frequency*length)/float(SAMPLERATE_HZ);
  // Now loop through all the samples and play them, calculating the
  // position within the wave buffer for each moment in time.
  for (uint32_t i=0; i<iterations; ++i) {
    uint16_t pos = uint32_t(i*delta) % length;
    int32_t sample = buffer[pos];
    // Duplicate the sample so it's sent to both the left and right channel.
    // It appears the order is right channel, left channel if you want to write
    // stereo sound.
    I2S.write(sample, sample);
  }
}

void setup() {
  // Configure serial port.
  Serial.begin(115200);
  Serial.println("ESP32 I2S Audio Tone Generator");

  // Initialize the I2S transmitter.
  if (!I2S.begin(I2S_PHILIPS_MODE, SAMPLERATE_HZ,BITS_SAMPLE)) {
    Serial.println("Failed to initialize I2S transmitter!");
    while (1);
  }
  I2S.enableTx();
	
  // Initialize the I2S transmitter.	
	
	
  // Generate waveforms.
  generateSine(AMPLITUDE, sine, WAV_SIZE);
  generateSawtooth(AMPLITUDE, sawtooth, WAV_SIZE);
  generateTriangle(AMPLITUDE, triangle, WAV_SIZE);
  generateSquare(AMPLITUDE, square, WAV_SIZE);
}

void loop() {
  Serial.println("Sine wave");
  for (int i=0; i<sizeof(scale)/sizeof(float); ++i) {
    // Play the note for a quarter of a second.
    playWave(sine, WAV_SIZE, scale[i], 0.25);
    // Pause for a tenth of a second between notes.
    delay(100);
  }
  Serial.println("Sawtooth wave");
  for (int i=0; i<sizeof(scale)/sizeof(float); ++i) {
    // Play the note for a quarter of a second.
    playWave(sawtooth, WAV_SIZE, scale[i], 0.25);
    // Pause for a tenth of a second between notes.
    delay(100);
  }
  Serial.println("Triangle wave");
  for (int i=0; i<sizeof(scale)/sizeof(float); ++i) {
    // Play the note for a quarter of a second.
    playWave(triangle, WAV_SIZE, scale[i], 0.25);
    // Pause for a tenth of a second between notes.
    delay(100);
  }
  Serial.println("Square wave");
  for (int i=0; i<sizeof(scale)/sizeof(float); ++i) {
    // Play the note for a quarter of a second.
    playWave(square, WAV_SIZE, scale[i], 0.25);
    // Pause for a tenth of a second between notes.
    delay(100);
  }
}
