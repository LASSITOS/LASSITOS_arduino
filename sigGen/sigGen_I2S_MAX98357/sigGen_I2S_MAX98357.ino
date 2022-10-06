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
   * MCLK  : 26      # I2S master clock. May be that not every PIN can generate clock.
   * 

              
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

#define PIN_I2S_WS 27

#define PIN_I2S_SD 33

// #define PIN_I2S_SD_OUT 15  //not used since I2S is used 1 way only.

#define PIN_I2S_MKLC 26

//#define PIN_I2S_SD_IN 35

#include <driver/i2s.h> // or  
// #include <I2S.h>



#define SAMPLERATE_HZ 16000  // The sample rate of the audio.  ??ESP32 max 16kHz??
#define WAV_SIZE      1024    // The size of each generated waveform.  The larger the size the higher
                             // quality the signal.  A size of 256 is more than enough for these simple
                             // waveforms.

//const i2s_port_t I2S_PORT = I2S_NUM_0;
const i2s_port_t I2S_PORT = I2S_NUM_0;  //=0;



// --------------------------------
// Settings Function generation
//---------------------------------
#define BITS_SAMPLE 32

// Define a C-major scale to play all the notes up and down.
uint64_t freqList[] = { 100,200,300,400,500,600,700,800,900,1000,1500,2000,2500,3000,4000,5000,6000,7000,8000,9000 };

unsigned int AMPLITUDE=((1<<8)-1);   // Set the amplitude of generated waveforms.  This controls how loud
                             // the signals are, and can be any value from 0 to 2**BITS_SAMPLE - 1.  Start with
                             // a low value to prevent damaging speakers!
							 
unsigned int FREQ=500; 			// Frequency of waveform in Hz


unsigned int TIME_LENGTH=20;	//  Time of waveform generation in seconds

// Store basic waveforms in memory.
int32_t sine[WAV_SIZE]     = {0};
int32_t sawtooth[WAV_SIZE] = {0};
int32_t triangle[WAV_SIZE] = {0};
int32_t square[WAV_SIZE]   = {0};


char txString2[50];
char txString[500];   // String containing messages to be send to BLE terminal
char subString[20];    


void i2sInit(){
  esp_err_t err;
// The I2S config as per the example
  const i2s_config_t i2s_config = {
      .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_TX), // transfer
      .sample_rate = SAMPLERATE_HZ,                         // 16KHz
      .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT, // could only get it to work with 32bits
      .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT, // use right channel
      .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,     // Interrupt level 1
      .dma_buf_count = 4,                           // number of buffers
      .dma_buf_len = 8                              // 8 samples per buffer (minimum)
  };

  // The pin config as per the setup
  const i2s_pin_config_t pin_config = {
      .bck_io_num = PIN_I2S_SCK,   // Serial Clock (SCK)
      .ws_io_num = PIN_I2S_WS,    // Word Select (WS)
      .data_out_num = PIN_I2S_SD, // Serial Data (SD)n
      .data_in_num = I2S_PIN_NO_CHANGE   // ot used (only for microphone)
  };
  
  // Configuring the I2S driver and pins.
  // This function must be called before any I2S driver read/write operations.
  err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("Failed installing driver: %d\n", err);
    while (true);
  }
  err = i2s_set_pin(I2S_PORT, &pin_config);
  if (err != ESP_OK) {
    Serial.printf("Failed setting pin: %d\n", err);
    while (true);
  }
  Serial.println("I2S driver installed.");
}






// --------------------------------
// Function generation
//---------------------------------

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

void generateSine(int32_t amplitude, int32_t* buffer, uint16_t length) {
  // Generate a sine wave signal with the provided amplitude and store it in
  // the provided buffer of size length.
  for (int i=0; i<length; ++i) {
    buffer[i] = int32_t(float(amplitude)*sin(2.0*PI*(1.0/length)*i));
  }
}

void playWave(int32_t* buffer, uint16_t length, float frequency, float seconds) {
  // Play back the provided waveform buffer for the specified
  // // amount of seconds.
  
  // set clock
  ESP_LOGI(TAG, "set clock");
    i2s_set_clk(I2S_PORT, SAMPLERATE_HZ, BITS_SAMPLE, I2S_CHANNEL_MONO);
    
	
	//Using push
    // for(i = 0; i < SAMPLE_PER_CYCLE; i++) {
    //     if (bits == 16)
    //         i2s_push_sample(0, &samples_data[i], 100);
    //     else
    //         i2s_push_sample(0, &samples_data[i*2], 100);
    // }
    // or write
	uint32_t iterations = seconds*SAMPLERATE_HZ;
	size_t i2s_bytes_write = 0;
	float delta = (frequency*length)/float(SAMPLERATE_HZ);
	for (uint32_t i=0; i<iterations; ++i) {
		uint16_t pos = uint32_t(i*delta) % length;
		int32_t sample = buffer[pos];
		ESP_LOGI(TAG, "write data");
		i2s_write(I2S_PORT, &sample, BITS_SAMPLE, &i2s_bytes_write, 100);
  }
}

void playWave_old(int32_t* buffer, uint16_t length, float frequency, float seconds) {
  // Play back the provided waveform buffer for the specified
  // amount of seconds.
  // First calculate how many samples need to play back to run
  // for the desired amount of seconds.
  uint32_t iterations = seconds*SAMPLERATE_HZ;
  size_t i2s_bytes_write = 0;
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
    i2s_write(I2S_PORT, &sample, BITS_SAMPLE, &i2s_bytes_write, 50);

  }
}

 
// /////////////////////////////////////////////
// ---------------------------------------------
// Input parsing functions
// ---------------------------------------------
// /////////////////////////////////////////////
void parse( String rxValue){
	  //Start new data files if START is received and stop current data files if STOP is received
  if (rxValue.indexOf("START") != -1 or rxValue.indexOf("start") != -1) { 
	  playWave(sine, WAV_SIZE, FREQ, TIME_LENGTH);
	  
	
  } else if (rxValue.indexOf("SETFREQ") != -1) {
	  Serial.println("Setting new frequency value! ");
    int index = rxValue.indexOf(":");\
    int index2 = rxValue.indexOf(":",index+1);
    if (index !=-1 and index2 !=-1){
      FREQ=rxValue.substring(index+1,index2).toInt();
      sprintf(txString,"New frequency is: %d Hz",FREQ );
      Serial.println(txString);
    } else {
      sprintf(txString,"Frequency can not be parsed from string '%s''",rxValue);
      Serial.println(txString);
    }

  } else if (rxValue.indexOf("SETAMP") != -1) {
    Serial.println("Setting new digital gain value! ");
    int index = rxValue.indexOf(":");
    int index2 = rxValue.indexOf(":",index+1);
    if (index !=-1 and index2 !=-1){
      AMPLITUDE=rxValue.substring(index+1,index2).toInt();
	    generateSine(AMPLITUDE, sine, WAV_SIZE);
      sprintf(txString,"New gain is:%f, and gain tuning word is: %04X",AMPLITUDE,AMPLITUDE );
      Serial.println(txString);
    } else {
      sprintf(txString,"Amplitude can not be parsed from string '%s''. Amplitude can be any value from 0 to 2**BITS_SAMPLE - 1",rxValue);
      Serial.println(txString);
    }
  
    } else if (rxValue.indexOf("SETLEN") != -1) {
	  Serial.println("Setting new frequency value! ");
    int index = rxValue.indexOf(":");\
    int index2 = rxValue.indexOf(":",index+1);
    if (index !=-1 and index2 !=-1){
      TIME_LENGTH=rxValue.substring(index+1,index2).toInt();
      sprintf(txString,"New frequency is: %d Hz",TIME_LENGTH );
      Serial.println(txString);
    } else {
      sprintf(txString,"Frequency can not be parsed from string '%s''",rxValue);
      Serial.println(txString);
    }
  
  }else{
	Serial.println("Input could not be parsed!");
  } 	
}










void setup() {
  // Configure serial port.
  Serial.begin(115200);
  Serial.println("ESP32 I2S Audio Tone Generator");

  // Initialize the I2S transmitter.
  i2sInit();
    


  
  // Initialize the I2C transmitter.	
	
	
  // Generate waveforms.
  generateSine(AMPLITUDE, sine, WAV_SIZE);
  generateSawtooth(AMPLITUDE, sawtooth, WAV_SIZE);
  generateTriangle(AMPLITUDE, triangle, WAV_SIZE);
  generateSquare(AMPLITUDE, square, WAV_SIZE);
}

void loop() {

  
    if (Serial.available()){ // Check Serial inputs
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
  
  delay(50);
}
