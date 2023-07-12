// /////////////////////////////////////////////
// ---------------------------------------------
// Input parsing functions
// ---------------------------------------------
// /////////////////////////////////////////////
void parse( String rxValue){
	  //Start new data files if START is received and stop current data files if STOP is received
  if (rxValue.indexOf("SETTSW") != -1) {
	
	Serial.println("Setting new state for Tx CSwitch.");
    int index = rxValue.indexOf(":");
    int index2 = rxValue.indexOf(":",index+1);
    if (index !=-1 and index2 !=-1){
      rxValue.substring(index+1,index2).toCharArray(addrStr,5);
      CSwitch=strtoul (addrStr, NULL, 16);
      setCswitchTx (CSwitch);

    } else {
      sprintf(txString,"CSwitch state can not be parsed from string '%s''",rxValue);
      Serial.println(txString);
    }

  
  // Set new values on CSwitch calibration 
  }else if (rxValue.indexOf("SETCSW") != -1) {
 
  Serial.println("Setting new state for calibration coil CSwitch.");
    int index = rxValue.indexOf(":");\
    int index2 = rxValue.indexOf(":",index+1);
    if (index !=-1 and index2 !=-1){
      rxValue.substring(index+1,index2).toCharArray(addrStr,5);
      CSwitch=strtoul (addrStr, NULL, 16);
      setCswitchCal (CSwitch);

    } else {
      sprintf(txString,"CSwitch state can not be parsed from string '%s''",rxValue);
      Serial.println(txString);
    }

  } else if (rxValue.indexOf("SCANI2C") != -1) {
      Serial.println("Scanning I2C devices");
      scanI2C ();


    // Read register  I2C
  }else if(rxValue.substring(0,4)== "I2CR" and rxValue.charAt(8)== 'R') {
	  char addrStr[5];
	  rxValue.substring(4,8).toCharArray(addrStr,5);
	  uint8_t addr=strtoul (addrStr, NULL, 16);
	  Serial.print("Reading register: ");
	  Serial.println(addr, HEX);
	  uint8_t out=readRegI2C ( MA12070P_address,addr);
	  sprintf(txString2,"Addr:%#02X Data:",addr);
	  Serial.print(txString2);
	  Serial.println(out,BIN);

	  
  // Write to register  I2C  
  } else if (rxValue.substring(0,4)== "I2CW" and (rxValue.charAt(8) == 'X' or rxValue.charAt(8) == 'B')) {
	  char addrStr[5];
	  char datStr[19];
	  uint8_t addr;
	  uint8_t dat;
	  
	  if(rxValue.charAt(8) == 'X'){
		  rxValue.substring(9,13).toCharArray(datStr,5);
		  Serial.print(datStr);
		  dat=strtoul (datStr, NULL, 16);
		
	  }else if(rxValue.charAt(8)== 'B'){
		   rxValue.substring(9,17).toCharArray(datStr,17);
           Serial.print(datStr);
		   dat=strtoul (datStr, NULL, 2);
	  }
	  rxValue.substring(4,8).toCharArray(addrStr,5);
	  addr=strtoul (addrStr, NULL, 16);
	  sprintf(txString2,"Writing register:%#02X Data:%#04X",addr);
	  Serial.print(txString2);
	  Serial.println(dat,BIN);
	  write_I2C(MA12070P_address,addr,dat);
  
  }else{
	Serial.println("Input could not be parsed!");
  } 	
}