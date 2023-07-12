
// /////////////////////////////////////////////
// ---------------------------------------------
// Communication to Microcontroller
// ---------------------------------------------
// /////////////////////////////////////////////


void startMicro(){
	uint8_t startMSG[13];
  char timestr[13];
	sprintf(timestr, "%02d%02d%02d%02d%02d", Year % 100, Month, Day, Hour, Minute); 
	startMSG[0]=CDM_start;
	for (int i=0;i<11;i++){
		startMSG[1+i] = uint8_t(timestr[i]);
		}

	startMSG[12]=0;
	spiTransfer2( startMSG, 12, CS_MSP430 );
}


void startMicro2(){
  Hour = random(23);
  Minute = random(59);
  Second = random(59);
  printDateTime();
  resetMicro();
  startMicro();	  
  delay(3000)   ;  // Wait until microcontroller started the measurement
  status_out = statusMicro();
  Serial.print("Microcontroller status: 0x");
  Serial.printf("%02X, ",status_out);
	Serial.print("BIN: ");
  Serial.println(status_out,BIN);
}


void stopMicro(){ 
    spiCommand8( CDM_stop  , CS_MSP430 );
	  delay(500)   ;  // Check that this is enought time fot the microcontroller stopping the measurements
	  // spiCommand8( CDM_status   , CS_MSP430 );
    // Serial.print("Send stop to Microcontroller!");
    // delay(2);
    // statusMicro();
}

uint32_t statusMicro(){   
	spiCommand8( CDM_status   , CS_MSP430 );
	delay(2);
	// Serial.printf("Status: %08X, ",spiCommand32( CDM_status_read32,  CS_MSP430));
    // delay(2);
	return spiCommand32( CDM_status_read32,  CS_MSP430);
}

void statusMicro8(){   
	  spiCommand8( CDM_status   , CS_MSP430 );
    delay(1);
    uint8_t out=spiCommand8( CDM_status_read   , CS_MSP430 );
	  // ParseStatus(out);
    Serial.printf("%04X",out);
    Serial.println(" ");
	  Serial.print("BIN: ");
    Serial.println(out,BIN);
}

void statusMicroLong(){   
	  spiCommand8( CDM_status   , CS_MSP430 );
    delay(10);
    Serial.print("Send long STATUS request");
    status_out =spiCommand32( CDM_status_read32,  CS_MSP430);
	  Serial.printf("%04X",status_out);
    Serial.println(" ");
	  Serial.print("BIN: ");
    Serial.println(status_out,BIN);
	  
}



void ParseStatus(int out){   	
  Serial.print("Parsing status message:");
  Serial.printf("%03X, ",status_out);
	MSP430_CMD_OK =0;
	MSP430_measuring  =0;
  MSP430_SD_ERR = 0;
  MSP430_CALIBRATING = 0;
	MSP430_STARTING = 0;
  
  // Parsing  response from microcontroller
	if ((out& TSTAT_CMD_OK )==TSTAT_CMD_OK ){
		MSP430_CMD_OK =1;
		
		if((out& TSTAT_RUNNING)==TSTAT_RUNNING){
			MSP430_measuring = 1;
		}else if((out& TSTAT_SD_ERR)==TSTAT_SD_ERR){
			MSP430_SD_ERR = 1;
			Serial.print("MSP430 SD not running!");
		}else if((out& TSTAT_CALIBRATING)==TSTAT_CALIBRATING){
			MSP430_CALIBRATING = 1;
			Serial.print("MSP430 is calibrating!");
		}else if((out& TSTAT_STARTING)==TSTAT_STARTING){
			MSP430_STARTING = 1;
			Serial.print("MSP430 is starting!");
		}

	} else {
    Serial.print("MSP430 can't be reached or got wrong message");
	}

}


void resetMicro(){
  spiCommand8( CDM_reset  , CS_MSP430 );
	delay(300)   ;  // Check that this is enought time for the microcontroller stopping the measurements
	Serial.print("Send reset to Microcontroller!");
	// spiCommand8( CDM_status , CS_MSP430 );
	// delay(2);
	// uint8_t out=spiCommand8( CDM_status_read   , CS_MSP430 );
  //   Serial.printf("Micro stat: %02X",out);
  //   Serial.println(" ");
}


//start Microcontroller data logger
//----------------------------------
void startMSP430() {
  resetMicro();
  startMicro();	  
  delay(3000)   ;  // Wait until microcontroller started the measurement
  status_out = statusMicro();
  Serial.print("\r\nMicrocontroller status: 0x");
  Serial.printf("%08X, ",status_out);
	// Serial.print("BIN: ");
  // Serial.println(status_out,BIN);
  sprintf(subString,"\r\nStatus Micro: 0x%08X, ",status_out);
  strcat(txString, subString);
  
  ParseStatus( (status_out & 0xFFF00000)>>20);
  if(!MSP430_measuring){
        strcat(txString, "\r\nMeasurement on Microcontroller not started!");
    if (MSP430_SD_ERR){
      strcat(txString, " SD not working");
	  }
    Serial.println(txString);  
    interrupt_Measuring();
    vTaskDelete(TaskStart);
  }
}