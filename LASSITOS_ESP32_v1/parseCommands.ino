
// /////////////////////////////////////////////
// ---------------------------------------------
// Input parsing functions    %toCheck
// ---------------------------------------------
// /////////////////////////////////////////////
void parse( String rxValue){     //%toCheck
  //Start new data files if START is received and stop current data files if STOP is received
  BLE_message = true;
  if (rxValue.indexOf("START") != -1) {
	  if (measuring) {
      strcpy(txString, "Already measuring.");
      Serial.println(txString);
      return;
    }
    // measuring = true;
    // startMeasuring();
    StartFlag = true;
  
  } else if (rxValue.indexOf("STOP") != -1) {
	  // measuring = false;
	  // stop_Measuring(SD);
    StopFlag = true;
	  
  } else if (rxValue.indexOf("STATUS") != -1) {
	  statusMicro8();

	} else if (rxValue.indexOf("STATLONG") != -1) {
	  statusMicroLong();	

	} else if (rxValue.indexOf("STRMICRO") != -1) {
	  startMicro();	

  } else if (rxValue.indexOf("STPMICRO") != -1) {
	  stopMicro();	
  		   
  } else if(rxValue.indexOf("RESET") != -1){
   ESP.restart();
	
  } else if(rxValue.indexOf("RESMICRO") != -1){
    resetMicro();

  } else if (rxValue.indexOf("TESTCAL") != -1 or rxValue.indexOf("testcal") != -1) { 
    Serial.println("Starting calibration test");
    testCal();

  } else if (rxValue.indexOf("CAL") != -1 or rxValue.indexOf("cal") != -1) { 
    calibrating=1;

  } else if (rxValue.indexOf("TESTLONG") != -1 or rxValue.indexOf("testlong") != -1) { 
    Serial.println("Starting long test");
    testLong();

  } else if (rxValue.indexOf("LONGSINGLE") != -1 ) { 
    Serial.println("Starting long test single freqency");
    int index = rxValue.indexOf(":");
    int index2 = rxValue.indexOf(":",index+1);
	  int index3 = rxValue.indexOf(":",index2+1);
    if (index !=-1 and index2 !=-1 and index3 !=-1 ){
		  int i=rxValue.substring(index+1,index2).toInt();
	    int len=rxValue.substring(index2+1,index3).toInt();
		  Serial.print("Frequency index:");
      Serial.println(i);
      Serial.print("Measuring lenght (s):");
      Serial.println(len);
      testLongSingle(i,len*1000);
	  } else {
      sprintf(txString,"Frequency index and measuring lenght could not be parsed from: '%s''",rxValue);
      Serial.println(txString);
	  }
  } else if (rxValue.indexOf("FSUBSWEEP") != -1 or rxValue.indexOf("fsubsweep") != -1) { 
    Serial.println("Got freq sub sweep command");
    int index = rxValue.indexOf(":");
    int index2 = rxValue.indexOf(":",index+1);
	  int index3 = rxValue.indexOf(":",index2+1);
    if (index !=-1 and index2 !=-1 and index3 !=-1 ){
		  int Df=rxValue.substring(index+1,index2).toInt();
	    int delta=rxValue.substring(index2+1,index3).toInt();
		  Serial.print("F range");
      Serial.println(Df);
      Serial.print("Delta");
      Serial.println(delta);
		  FsubSweep(Df,delta);
	  } else {
      sprintf(txString,"Frequency range and delta can not be parsed form string: '%s''",rxValue);
      Serial.println(txString);
	  }
  } else if (rxValue.indexOf("FSWEEP") != -1 or rxValue.indexOf("fsweep") != -1) { 
    Serial.println("Got freq sweep command");
    int index = rxValue.indexOf(":");
    int index2 = rxValue.indexOf(":",index+1);
	  int index3 = rxValue.indexOf(":",index2+1);
	  int index4 = rxValue.indexOf(":",index3+1);
    if (index !=-1 and index2 !=-1 and index3 !=-1 and index4 !=-1){
		  int A=rxValue.substring(index+1,index2).toInt();
	    int B=rxValue.substring(index2+1,index3).toInt();
	    int delta=rxValue.substring(index3+1,index4).toInt();
		  Serial.print("START");
      Serial.println(A);
      Serial.print("STOP");
      Serial.println(B);
      Serial.print("Delta");
      Serial.println(delta);
		  frequencySweep(A,B,delta);
	  } else {
      sprintf(txString,"Start, Stop and delta can not be parsed form string: '%s''",rxValue);
      Serial.println(txString);
	  }
  } else if (rxValue.indexOf("GSWEEP") != -1 or rxValue.indexOf("gsweep") != -1) { 
    Serial.println("Got gain sweep command"); 
    int index = rxValue.indexOf(":");\
    int index2 = rxValue.indexOf(":",index+1);
	  int index3 = rxValue.indexOf(":",index2+1);
	  int index4 = rxValue.indexOf(":",index3+1);
    if (index !=-1 and index2 !=-1 and index3 !=-1 and index4 !=-1){
		  rxValue.substring(index+1,index2).toCharArray(datStr ,index2-index+1);
      int A=strtoul (datStr, NULL, 16);
	    rxValue.substring(index2+1,index3).toCharArray(datStr ,index3-index2+1);
      int B= strtoul (datStr, NULL, 16);
	    rxValue.substring(index3+1,index4).toCharArray(datStr ,index4-index3+1);
      int delta=strtoul (datStr, NULL, 16);
		  Serial.print("START");
      Serial.println(A);
      Serial.print("STOP");
      Serial.println(B);
      Serial.print("Delta");
      Serial.println(delta);
      if (rxValue.substring(index-1,index)=="2"){
        GainSweep2(A,B,delta);
      } else{
        GainSweep(A,B,delta);
      }
      } else {
      sprintf(txString,"Start, Stop and delta can not be parsed form string: '%s''",rxValue);
      Serial.println(txString);
    } 
		  
      }else if (rxValue.indexOf("SETTSW") != -1) {
	
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

  
    // Set CalibCoil CSwitch over I2C
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
	
	 
  
  
  
  } else if (rxValue.indexOf("READ") != -1) {
	readFiles(rxValue);
  
  } else if (rxValue.indexOf("LIST") != -1) {
	getFileList();
  } else if (rxValue.indexOf("COMS") != -1 or rxValue.indexOf("?") != -1) {
	commands();
  } else if (rxValue.indexOf("CHECKLASER") != -1) {
	check_laser();
  } else if (rxValue.indexOf("CHECKINS") != -1) {
	check_INS();
  } else if (rxValue.indexOf("INSRATE") != -1) {
	  Serial.println("Setting new frequency value! ");
    int index = rxValue.indexOf(":");
    int index2 = rxValue.indexOf(":",index+1);
    if (index !=-1 and index2 !=-1){
		if (!measuring) {
		  IMXrate=rxValue.substring(index+1,index2).toInt();
		  sprintf(txString,"New rate is: %d ms",IMXrate );
		  Serial.println(txString);
		  setIMX5message(); //update ASCII setting message
		  setupINS(); //send setting message to IMX5
		  } else {
			sprintf(txString,"Data measuring is running. Can't change IMX rate now. First stop measurment!");
			Serial.println(txString);
		}
    } else {
      sprintf(txString,"INS frequency (ms) can not be parsed from string '%s''",rxValue);
      Serial.println(txString);
    }
	
	
  } else if (rxValue.indexOf("SETRESFREQ") != -1) {
	  Serial.println("Setting new frequency value! ");
    int index = rxValue.indexOf(":");\
    int index2 = rxValue.indexOf(":",index+1);
    if (index !=-1 and index2 !=-1){
      ifreq=rxValue.substring(index+1,index2).toInt();
      if (ifreq<Nfreq and ifreq>=0 ){
		  stop_trigger();
		  configureResFreq(ifreq);
		  sprintf(txString,"New resonant frequency is: %d",freqs[ifreq]);
		  Serial.println(txString);
		  Nmulti =0;
	  }else {
		  sprintf(txString,"Error! Pass a valid index for resonant frequency between 0 and '%d''", Nfreq-1);
	  }
    } else {
      sprintf(txString,"Resonant frequency can not be parsed from string '%s''",rxValue);
      Serial.println(txString);
    }
  
  } else if (rxValue.indexOf("SETMULTIFREQ") != -1) {
	  Serial.println("Setting measurement to multiple frequencies ");
      stop_trigger();
	  Nmulti =Nmulti_reg;

  } else if (rxValue.indexOf("DEF_MFREQ") != -1) {
	Serial.println("Define frequencies for multiple frequencies meaurement");
	stop_trigger();
	// msglen=rxValue.length()
	bool validMsg=true;   
	int indexes[10];
  int _Multifreqs[10];
	int i=0;
	indexes[0]= rxValue.indexOf(":");
	while(rxValue.indexOf(":",indexes[i]+1)!=-1){
		indexes[i] = rxValue.indexOf(":",indexes[i]+1);
		i++;
		if (i==Nfreq-1){
			validMsg=false; 
			break;
		}
  }
  
  if (indexes[0] >0 and indexes[0] <= Nfreq){

		int _Nmulti =indexes[0];
		for (int j =0;j<=_Nmulti;j++){
			rxValue.substring(indexes[j],indexes[j+1]).toCharArray(datStr ,indexes[j+1]-indexes[j]);
			int A=strtoul (datStr, NULL, 16);
			
			if (A>Nfreq-1){
				validMsg=false;  
				break;
			}
			_Multifreqs[j-1]=A;
		}
		
		Nmulti_reg =_Nmulti;
		Nmulti=Nmulti_reg;
		for ( int j =0;j<Nmulti;j++){
			Multifreqs[j]=_Multifreqs[j];
		}
	
	} else{
		validMsg=false;
	}
    
	if(!validMsg){
		sprintf(txString,"Error! Pass a valid messages with frequency between 0 and '%d''. E.g.:  DEF_MFREQ:3:1:4:8:", Nfreq-1);
	}

  } else if (rxValue.indexOf("SETFREQ") != -1) {
	  Serial.println("Setting new frequency value! ");
    int index = rxValue.indexOf(":");\
    int index2 = rxValue.indexOf(":",index+1);
    if (index !=-1 and index2 !=-1){
      freq=rxValue.substring(index+1,index2).toInt();
      configureSineWave();
      sprintf(txString,"New frequency is: %d",freq );
      Serial.println(txString);
    } else {
      sprintf(txString,"Frequency can not be parsed from string '%s''",rxValue);
      Serial.println(txString);
    }
  } else if (rxValue.indexOf("SETGAIN2") != -1) {
      Serial.println("Setting new digital gain value! ");
      int index = rxValue.indexOf(":");\
      int index2 = rxValue.indexOf(":",index+1);
      if (index !=-1 and index2 !=-1){
        rxValue.substring(index+1,index2).toCharArray(datStr ,index2-index+1);
        setGain2(strtoul (datStr, NULL, 16));
        Serial.println(datStr);
        sprintf(txString,"New gain tuning word is: %04X",gainDAT );
        Serial.println(txString);
        out=readReg(0x35);
        Serial.print("New gain:");
        Serial.println(out,BIN);
      } else {
        sprintf(txString,"Gain can not be parsed from string '%s''",rxValue);
        Serial.println(txString);
    }  
	
  } else if (rxValue.indexOf("SCANI2C") != -1) {
      Serial.println("Scanning I2C devices");
      scanI2C ();
	
  } else if (rxValue.indexOf("VBAT") != -1) {
      BLE_message = true;
      VBat=readVBat();
      sprintf(txString,"Voltage battery: %05.2f V\n", VBat);
      Serial.println(txString);
    
  } else if (rxValue.indexOf("TEMP") != -1) {
      BLE_message = true;
      float c = tempsens1.readTempC();
      sprintf(txString,"Temperature: %.4f* C\n",c);
      Serial.println(txString);

  } else if (rxValue.indexOf("STROBE") != -1) {
      Serial.println("strobe pulse");
      pulseStrobeIMX5(); 
	
  } else if (rxValue.substring(0,4)== "SPIR"  and rxValue.charAt(8)== 'R') {
      rxValue.substring(4,8).toCharArray(addrStr,8);
      addr=strtoul (addrStr, NULL, 16);
      Serial.print("Reading register: ");
  //          Serial.println(addr, HEX);
      out=readReg(addr);
      sprintf(txString2,"Addr:%#02X Data:",addr);
      Serial.print(txString2);
      Serial.println(out,BIN);

	  
  // Write to SPI register    
  } else if (rxValue.indexOf("SPIW") != -1  and (rxValue.charAt(8) == 'X' or rxValue.charAt(8) == 'B')) {
	  if(rxValue.charAt(8) == 'X'){
		  rxValue.substring(9,15).toCharArray(datStr,7);
		  Serial.print(datStr);
		  dat=strtoul (datStr, NULL, 16);
		
	  }else if(rxValue.charAt(8)== 'B'){
		   rxValue.substring(9,25).toCharArray(datStr,17);
//               Serial.print(datStr);
		   dat=strtoul (datStr, NULL, 2);
	  }
	  rxValue.substring(4,8).toCharArray(addrStr,5);
	  addr=strtoul (addrStr, NULL, 16);
	  
//          Serial.print("Writing register: ");
//          Serial.print(addr, HEX);
//          Serial.print(", data: ");
//          Serial.println(dat, HEX);
	  sprintf(txString2,"Writing register:%#02X Data:%#04X",addr);
	  Serial.print(txString2);
	  Serial.println(dat,BIN);
	  writeReg(addr,dat);
	
	
  } else {
	BLE_message = true;
	strcpy(txString, "Input can not be parsed retry!");
	Serial.println(txString);
  }
}





// Print commands  %toCheck:  Need to make new list of commands
void commands() {
  Send_tx_String(txString);
  strcpy(txString, "");
  strcat(txString, "\nCommands \n# --------------------------------------\n");
  strcat(txString, "\n Serial only\n# .................\n");
  strcat(txString, "READ Read last file of last measurement. If argument is passed with READ:<< nfile>>: read file nfile \nLIST  Get list of files in SD card \n ");
  Send_tx_String(txString);
  delay(100);
  strcat(txString, "\n Serial and BLE:\n# .................\n");
  strcat(txString, "STOP  Stops measurement \nSTART Starts new measurement  \n" );
  strcat(txString, "CHECKLASER  Get Laser data for 1 second and send it over BLE and serial \n" );
  strcat(txString, "COMS  List commands \n");
  strcat(txString, "CHECKINS  Get INS data and send them over BLE+serial for manual check \n");
  strcat(txString, "RESET  Reset ESP32 \n");
  strcat(txString, "RESMICRO  Reset MSP430 \n");
  strcat(txString, "STATUS  Reset MSP430 \n");
  strcat(txString, "STATLONG  Reset MSP430 \n");
  Send_tx_String(txString);
  delay(100);
  strcat(txString, "STRMICRO,STPMICRO  Start,Stop data logging on MSP430 \n");
  strcat(txString, "SETRESFREQ  Set resonant frequuency for measurements \n");
  strcat(txString, "SETFREQ  Set frequuency of signal generator (Do not set the right resonant capacitor) \n");
  strcat(txString, "SETGAIN2  Set digital gain DAC and signal strength\n");
  strcat(txString, "FSUBSWEEP:range:step:  sweep around each resonatn frequency \n");
  strcat(txString, "FSWEEP:range:step:  sweep  frequency \n");
  strcat(txString, "TEMP  get temperature \n");
  strcat(txString, "TEMP  get temperature \n");
  strcat(txString, "SCANI2C  Reset MSP430 \n");
  strcat(txString, "VBAT  get battery voltage\n");
  strcat(txString, "TEMP  get temperature \n");

  strcat(txString, "# --------------------------------------\n");
  Send_tx_String(txString);
  delay(100);
  strcpy(txString, "");
}
