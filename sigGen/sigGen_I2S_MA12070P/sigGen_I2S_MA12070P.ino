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
   * MCLK  : 15      # I2S master clock. May be that not every PIN can generate clock.
   * 
   * I2C:
   * ---------------
   * SCL   :  SCL 22   #I2C clock,  need pull up resistor (e.g 10k)
   * SDA   :  SDA 21   # I2C data,  need pull up resistor  (e.g 10k)  (23 for Micro-USB board)
   * GND   :  GND
   
   * Other:
   * ---------------
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
#define PIN_I2S_WS 27
#define PIN_I2S_SD 33
#define PIN_I2S_MKLC 0
//#define PIN_I2S_SD_IN 35

#include <driver/i2s.h> // or  

#define SAMPLERATE_HZ 192000  // The sample rate of the audio.  ??ESP32 max 16kHz??
#define WAV_SIZE      256    // The size of each generated waveform.  The larger the size the higher
                             // quality the signal.  A size of 256 is more than enough for these simple
                             // waveforms.

const i2s_port_t I2S_PORT = I2S_NUM_0;  //=0;




// --------------------------------
// Settings I2C
//---------------------------------

#include <Wire.h>
#define  MA12070P_address 0x20

#define PIN_EN 32    // must be =1 at startup
#define PIN_MUTE 14  // must be =0 at startup




// --------------------------------
// Settings Function generation
//---------------------------------
#define BITS_SAMPLE 32

// Define a C-major scale to play all the notes up and down.
uint64_t freqList[] = { 100,200,300,400,500,600,700,800,900,1000,1500,2000,2500,3000,4000,5000,6000,7000,8000,9000 };

unsigned int AMPLITUDE=((1<<20)-1);   // Set the amplitude of generated waveforms.  This controls how loud
                             // the signals are, and can be any value from 0 to 2**BITS_SAMPLE - 1.  Start with
                             // a low value to prevent damaging speakers!
							 
unsigned int FREQ=1000; 			// Frequency of waveform in Hz


unsigned int TIME_LENGTH=5;	//  Time of waveform generation in seconds

// Store basic waveforms in memory.
int32_t sine[WAV_SIZE]     = {0};
int32_t sawtooth[WAV_SIZE] = {0};
int32_t triangle[WAV_SIZE] = {0};
int32_t square[WAV_SIZE]   = {0};


int32_t sample=0;

char txString2[50];
char txString[500];   // String containing messages to be send to BLE terminal
char subString[20];    


void i2sInit(uint32_t buffer_len){
  esp_err_t err;
// The I2S config as per the example
  const i2s_config_t i2s_config = {
      .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_TX), // transfer
      .sample_rate = SAMPLERATE_HZ,                         // 16KHz
      .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT, // could only get it to work with 32bits
      .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, // use right channel,I2S_CHANNEL_FMT_ONLY_RIGHT or I2S_CHANNEL_FMT_RIGHT_LEFT
      .communication_format = I2S_COMM_FORMAT_STAND_MSB,//i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,     // Interrupt level 1
      .dma_buf_count = 2,                           // number of buffers
      .dma_buf_len = buffer_len/2                              // 8 samples per buffer (minimum)
  };

  // The pin config as per the setup
  const i2s_pin_config_t pin_config = {
	    .mck_io_num = PIN_I2S_MKLC,   // Master Clock (MLCK)
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
// I2C communication
//---------------------------------

void write_I2C (uint8_t address, uint8_t msg ){
  //Write message to the slave
  Serial.printf("Writing to reg: %x, msg: %x\n", address,msg);
  Wire.beginTransmission(MA12070P_address);
  Wire.write(address);
  Wire.write(msg);
  uint8_t error = Wire.endTransmission(true);
  Serial.printf("endTransmission: %u\n", error);
}


uint8_t readRegI2C (uint8_t address){
  uint8_t bytesReceived=0;
  int temp=0;
  Serial.printf("Reading reg: %#02X\n", address);
  Wire.beginTransmission(MA12070P_address);
  Wire.write(address);
  uint8_t error = Wire.endTransmission(false);
  Serial.printf("endTransmission error: %u\n", error);
  //Read 16 bytes from the slave
  bytesReceived = Wire.requestFrom(MA12070P_address, 1);
  temp = Wire.read();
  Serial.printf("bytes received: %d\n", bytesReceived);
  Serial.printf("value: %x\n", temp);
  Serial.printf("value in decimal: %d\n", temp);
  return  temp;
  // uint8_t bytesReceived = Wire.requestFrom(MA12070P_address, 1);
  // uint8_t temp[bytesReceived]={0};
  // if((bool)bytesReceived){ //If received more than zero bytes
    // Wire.readBytes(temp, bytesReceived);
    // log_print_buf(temp, bytesReceived);
    // }
	// Serial.printf("bytes received: %d\n", bytesReceived);
	// Serial.printf("value: %x\n", temp);
  // return  temp[0];
}


// --------------------------------
// Function generation
//---------------------------------

void playWave2(int32_t amplitude, float frequency, float seconds) {

	size_t i2s_bytes_write = 0;
	uint32_t length=SAMPLERATE_HZ/frequency;
	int32_t buffer[length*2] = {0}; 
  for (uint32_t i=0; i<length; ++i) {
			buffer[i*2]=int32_t(float(amplitude)*sin(2*PI/length*i));
			buffer[i*2+1]=buffer[i*2];
		}
   
  Serial.print("length: ");
  Serial.println(length);
  //  for (uint32_t i=0; i<length; ++i) {Serial.println(buffer[i*2]);}
  delay(1000);
  
  // Initialize the I2S transmitter.
  i2s_driver_uninstall(I2S_PORT); //stop & destroy i2s driver
  i2sInit(length*2);
  i2s_set_clk(I2S_PORT, SAMPLERATE_HZ, BITS_SAMPLE, I2S_CHANNEL_MONO);     // set clock
//  
  //write buffer
  i2s_write(I2S_PORT, &buffer, BITS_SAMPLE*length*2, &i2s_bytes_write, 100);
   
  digitalWrite(PIN_MUTE , HIGH);  //unmute MA12070P 
  delay(1000*seconds);
  digitalWrite(PIN_MUTE , LOW);  //mute MA12070P 
   
  for (uint32_t i=0; i<length; ++i) {
      buffer[i*2]=0;
     buffer[i*2+1]=0;
  }
  i2s_write(I2S_PORT, &buffer, BITS_SAMPLE*length*2, &i2s_bytes_write, 100);
  
  Serial.println("Iteration end");
//   i2s_driver_uninstall(I2S_PORT); //stop & destroy i2s driver
}




// /////////////////////////////////////////////
// ---------------------------------------------
// Input parsing functions
// ---------------------------------------------
// /////////////////////////////////////////////
void parse( String rxValue){
	  //Start new data files if START is received and stop current data files if STOP is received
  if (rxValue.indexOf("START") != -1 or rxValue.indexOf("start") != -1) { 
	  // playWave(sine, WAV_SIZE, FREQ, TIME_LENGTH);
	  playWave2(AMPLITUDE,FREQ, TIME_LENGTH);
	
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
//	  generateSine(AMPLITUDE, sine, WAV_SIZE);
      sprintf(txString,"New gain is:%f, and gain tuning word is: %04X",AMPLITUDE,AMPLITUDE );
      Serial.println(txString);
    } else {
      sprintf(txString,"Amplitude can not be parsed from string '%s''. Amplitude can be any value from 0 to 2**BITS_SAMPLE - 1",rxValue);
      Serial.println(txString);
    }
  
    } else if (rxValue.indexOf("SETLEN") != -1) {
	  Serial.println("Setting new lenght in seconds ");
    int index = rxValue.indexOf(":");\
    int index2 = rxValue.indexOf(":",index+1);
    if (index !=-1 and index2 !=-1){
      TIME_LENGTH=rxValue.substring(index+1,index2).toInt();
      sprintf(txString,"New length is: %d s",TIME_LENGTH );
      Serial.println(txString);
    } else {
      sprintf(txString,"Lenght can not be parsed from string '%s''",rxValue);
      Serial.println(txString);
    }
  
  
  // Mute/Unmute 	
  }  else if (rxValue.indexOf("MUTE") != -1) {
    Serial.println("Setting new lenght in seconds ");
    int index = rxValue.indexOf(":");\
    int index2 = rxValue.indexOf(":",index+1);
    if (index !=-1 and index2 !=-1){
      if (rxValue.substring(index+1,index2).toInt()){
        digitalWrite(PIN_MUTE , LOW);  //mute MA12070P 
        Serial.println("Mute");
      }else{
        digitalWrite(PIN_MUTE , HIGH);  //mute MA12070P 
        Serial.println("Unmute");
      }
    } else {
      sprintf(txString,"Lenght can not be parsed from string '%s''",rxValue);
      Serial.println(txString);
    }
  
  
  // Read register  
  }else if (rxValue.charAt(0)== 'R' and rxValue.charAt(5)== 'R') {
	  char addrStr[5];
	  rxValue.substring(1,5).toCharArray(addrStr,5);
	  uint8_t addr=strtoul (addrStr, NULL, 16);
	  Serial.print("Reading register: ");
	  Serial.println(addr, HEX);
	  uint8_t out=readRegI2C ( addr);
	  sprintf(txString2,"Addr:%#02X Data:",addr);
	  Serial.print(txString2);
	  Serial.println(out,BIN);

	  
  // Write to register    
  } else if (rxValue.charAt(0)== 'W' and (rxValue.charAt(5) == 'X' or rxValue.charAt(5) == 'B')) {
	  char addrStr[5];
	  char datStr[19];
	  uint8_t addr;
	  uint8_t dat;
	  
	  if(rxValue.charAt(5) == 'X'){
		  rxValue.substring(6,10).toCharArray(datStr,5);
		  Serial.print(datStr);
		  dat=strtoul (datStr, NULL, 16);
		
	  }else if(rxValue.charAt(5)== 'B'){
		   rxValue.substring(6,14).toCharArray(datStr,17);
           Serial.print(datStr);
		   dat=strtoul (datStr, NULL, 2);
	  }
	  rxValue.substring(1,5).toCharArray(addrStr,5);
	  addr=strtoul (addrStr, NULL, 16);
	  
//          Serial.print("Writing register: ");
//          Serial.print(addr, HEX);
//          Serial.print(", data: ");
//          Serial.println(dat, HEX);
	  sprintf(txString2,"Writing register:%#02X Data:%#04X",addr);
	  Serial.print(txString2);
	  Serial.println(dat,BIN);
	  write_I2C(addr,dat);
  } else{
	Serial.println("Input could not be parsed!");
  } 	
}







void setup() {
  // Configure serial port.
  Serial.begin(115200);
  Serial.println("ESP32 I2S Audio Tone Generator");

  
  //Mute and Enable PINS
  pinMode(PIN_EN , OUTPUT); 
  digitalWrite(PIN_EN , HIGH);   // must be =1 at startup
  pinMode(PIN_MUTE , OUTPUT); 
  digitalWrite(PIN_MUTE , LOW);  // must be =0 at startup
    
  // Initialize the I2C transmitter.	
  Wire.begin();
  
  // Initialize the I2S transmitter.
  i2sInit(SAMPLERATE_HZ/FREQ*2);
  i2s_set_clk(I2S_PORT, SAMPLERATE_HZ, BITS_SAMPLE, I2S_CHANNEL_MONO);     // set clock
  
  //enable MA12070P to be allow to acces registers
  digitalWrite(PIN_EN , LOW); 
  
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
