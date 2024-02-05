





// /////////////////////////////////////////////
// ---------------------------------------------
// I2c to SPI functions
// ---------------------------------------------
// /////////////////////////////////////////////

uint32_t I2CspiCommand( uint32_t msg ) {  //use it as you would the regular arduino SPI API
  uint8_t out[4];
  uint8_t in[4];
  Serial.print("Message to send: ");
  Serial.printf("%x \n",msg);

  in[3]=msg & 0xff;
  in[2]=(msg >> 8 )& 0xff;
  in[1]=(msg >> 16 )& 0xff;
  in[0]=(msg >> 24 )& 0xff;
  
  Serial.print("decomposed msg: ");
  Serial.printf("%x, %x, %x, %x\n ",in[0],in[1],in[2],in[3]);

  spiBridge.spiTransfer(0,in,4,out);
  Serial.print("decomposed out: ");
  Serial.printf("%x, %x, %x, %x\n ",out[0],out[1],out[2],out[3]);

  return out[3]+ (out[2]<< 8) +  (out[1]<< 16) +(out[0] << 24) ;
}

void writeMSG (uint16_t addr,uint16_t dat) {
  uint32_t msg = (0x00 << 24) + (addr << 16) + dat;
  I2CspiCommand(  msg );
}

void writeReg (uint16_t addr,uint16_t dat) {
  //  Serial.println("Writing register");
  writeMSG(  addr,dat);
  delay(1);
  writeMSG( 0x1D , 0x01 );
}


uint32_t readReg(uint16_t addr){
//   Serial.println("Reading register");
   uint32_t msg = (0x80 << 24) + (addr << 16) + 0x0000;
   uint16_t out=I2CspiCommand(  msg );  //first 4 bytes are dropped by saving into uint16_t 
   return out;
}









// /////////////////////////////////////////////
// --------------------------------
// I2C communication
//---------------------------------
// /////////////////////////////////////////////


void parsePrint_I2C_error(int error){
  if (error>0){
      Serial.printf("EndTransmission with error: %u\n", error);
      switch(error){
        case 1:
          Serial.println("data too long to fit in transmit buffer");
          break;
        case 3:
          Serial.println("received NACK on transmit of data");
          break;
        case 2:
          Serial.println("received NACK on transmit of address");
          break;
        case 4:
          Serial.println("other error");
          break;
        case 5:
          Serial.println("timeout. Device can't be reached");
          break;
      }
  }
}

void write_I2C (uint8_t device, uint8_t address, uint8_t msg ){
  //Write message to the slave
  Serial.printf("Writing to reg: %x, msg: %x\n", address,msg);
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(msg);
  uint8_t error = Wire.endTransmission(true);
  Serial.printf("endTransmission: %u\n", error);
}


uint8_t readRegI2C (uint8_t device, uint8_t address){
  uint8_t bytesReceived=0;
  int temp=0;
  Serial.printf("Reading reg: %#02X\n", address);
  Wire.beginTransmission(device);
  Wire.write(address);
  uint8_t error = Wire.endTransmission(false);
  Serial.printf("endTransmission error: %u\n", error);
  //Read 16 bytes from the slave
  bytesReceived = Wire.requestFrom(device, 1);
  temp = Wire.read();
  Serial.printf("bytes received: %d\n", bytesReceived);
  Serial.printf("value: %x\n", temp);
  Serial.printf("value in decimal: %d\n", temp);
  return  temp;
}

void scanI2C (){
  byte error, address;
  int nDevices = 0;

  delay(5000);

  Serial.println("Scanning for I2C devices ...");
  for(address = 0x01; address < 0x7f; address++){
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0){
      Serial.printf("I2C device found at address 0x%02X\n", address);
      nDevices++;
    } else if(error != 2){
      Serial.printf("Error %d at address 0x%02X\n", error, address);
    }
  }
  if (nDevices == 0){
    Serial.println("No I2C devices found");
  }
}



