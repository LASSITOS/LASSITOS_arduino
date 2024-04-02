
// /////////////////////////////////////////////
// ---------------------------------------------
// Input parsing functions    %toCheck
// ---------------------------------------------
// /////////////////////////////////////////////
void parse( String rxValue){     //%toCheck
  //Start new data files if START is received and stop current data files if STOP is received
  if (rxValue.indexOf("START") != -1) {
	  makeLEMNMEAMSG("START",5);
  
  } else if (rxValue.indexOf("STOP") != -1) {
	  makeLEMNMEAMSG("STOP",4);
	  
  } else if (rxValue.indexOf("STATUS") != -1) {
	  makeLEMNMEAMSG("STATUS",6);

  		   
  } else if(rxValue.indexOf("RESET") != -1){
    makeLEMNMEAMSG("RESET",5);
	
  } else if(rxValue.indexOf("RESMICRO") != -1){
    makeLEMNMEAMSG("RESMICRO",8);


  } else if (rxValue.indexOf("CAL") != -1 or rxValue.indexOf("cal") != -1) { 
    makeLEMNMEAMSG("CAL",3);

  } else if (rxValue.indexOf("COMS") != -1 or rxValue.indexOf("?") != -1) {
	  commands();
  } else if (rxValue.indexOf("CHECKLASER") != -1) {
	  makeLEMNMEAMSG("CHECKLASER",11);
  } else if (rxValue.indexOf("CHECKINS") != -1) {
	  makeLEMNMEAMSG("CHECKINS",8);

	
  } else if (rxValue.indexOf("SETRESFREQ") != -1) {
	  Serial.println("Setting new frequency value! ");
    int index = rxValue.indexOf(":");\
    int index2 = rxValue.indexOf(":",index+1);
    int valout[2];
    if (index !=-1 and index2 !=-1){
      valout [0]=rxValue.substring(index+1,index2).toInt();
      makeLEMNMEAMSG_values("SETRESFREQ", 10, valout, 1);

    } else {
      sprintf(txString,"Resonant frequency can not be parsed from string '%s''",rxValue);
      Serial.println(txString);
    }
  
  } else if (rxValue.indexOf("SETMULTIFREQ") != -1) {
	  makeLEMNMEAMSG("SETMULTIFREQ",12);

  } else if (rxValue.indexOf("DEF_MFREQ") != -1) {
    Serial.println("Define frequencies for multiple frequencies meaurement");
    bool validMsg=true;   
    int indexes[16];
    int _Multifreqs[16];
    int i=0;
    indexes[0]= rxValue.indexOf(":");
    while(rxValue.indexOf(":",indexes[i]+1)!=-1){
      indexes[i+1] = rxValue.indexOf(":",indexes[i]+1);
      i++;
    }
    if (i >0 and i <= 15){
      int _Nmulti =rxValue.substring(indexes[0]+1,indexes[1]).toInt();
      _Multifreqs[0]=_Nmulti;
      // Serial.println(rxValue.substring(indexes[0]+1,indexes[1]));
      // Serial.println(_Nmulti);
      // Serial.println(i);
      for (int j =1;j<=_Nmulti;j++){      
        _Multifreqs[j]=rxValue.substring(indexes[j]+1,indexes[j+1]).toInt();
      }
      // Serial.println(_Multifreqs[_Nmulti-1]);
      makeLEMNMEAMSG_values("DEF_MFREQ", 9, _Multifreqs, _Nmulti+1);
    } else{
      validMsg=false;
    }
      
    if(!validMsg){
      strcpy(txString,"Error! Pass a valid message with number f freq as first vale'. E.g.:  DEF_MFREQ:3:1:4:8:");
    }


  } else if (rxValue.indexOf("SETGAIN2") != -1) {
      Serial.println("Setting new digital gain value! ");
      int index = rxValue.indexOf(":");\
      int index2 = rxValue.indexOf(":",index+1);
      int valout[2];
      char datStr[24];
      if (index !=-1 and index2 !=-1){
        rxValue.substring(index+1,index2).toCharArray(datStr ,index2-index+1);
        valout [0]=strtoul (datStr, NULL, 16);
        makeLEMNMEAMSG_values("SETGAIN", 7, valout, 1);
      
      } else {
        sprintf(txString,"Gain can not be parsed from string '%s''",rxValue);
        Serial.println(txString);
    }  
	
	
  } else if (rxValue.indexOf("VBAT") != -1) {
      makeLEMNMEAMSG("VBAT",4);
    
  } else if (rxValue.indexOf("TEMP") != -1) {
      makeLEMNMEAMSG("TEMP",4);
	
  } else {
	strcpy(txString, "Input can not be parsed retry!");
	
  Send_tx_String(txString);
  }
}





// add checksum to asciiMessage
void makeLEMNMEAMSG(char *Message, int messageLength) {
  char LEMNMEA[32];
  char outMessage[32];
  sprintf(outMessage,"$LEMMS,%s",Message);
  // Serial.println(outMessage);
  FormatAsciiMessage(outMessage,  messageLength+8,  LEMNMEA);
  // Serial.println(LEMNMEA);
  // Radio.print(LEMNMEA);
  sendToLEM(LEMNMEA);
  Serial.println("Sent NMEA");
  Serial.println(LEMNMEA);
}

// add checksum to asciiMessage
void makeLEMNMEAMSG_values(char *Message, int messageLength, int *values, int values_number) {
  char outMessage[128];
  char LEMNMEA[128];
  char valStr[16];
  sprintf(outMessage,"$LEMMS,%s",Message);
  int out_length=messageLength+8;
  Serial.print("length");
  Serial.println(out_length);
  for (int j =0;j<values_number;j++){
    sprintf(valStr,",%d",values[j]);
    strcat(outMessage, valStr);
    out_length+= strlen (valStr);
    Serial.print("valStr:");
    Serial.println(valStr);
    Serial.print("length");
    Serial.println(out_length);
  }
  FormatAsciiMessage(outMessage, out_length,LEMNMEA);
  // Radio.print(LEMNMEA);
  sendToLEM(LEMNMEA);
  Serial.print("Sent NMEA:");
  Serial.println(LEMNMEA);
  Serial.print("length");
  Serial.println(out_length);
  Serial.print("outMessage:");
  Serial.println(outMessage);
}

// add checksum to asciiMessage
int FormatAsciiMessage(char *Message, int messageLength, char *outMessage) {
  int checkSum = 0;
  unsigned int ptr = 0;
  char buf[16];
  outMessage[0] = '\0';

  if (Message[0] == '$') {
    ptr++;
  } else {
    strcpy(outMessage, "$");
  }

  // concatenate Message to outMessage
  strcat(outMessage, Message);

  // compute checksum
  while (ptr < messageLength) {
    checkSum ^= Message[ptr];
    ptr++;
  }
  // Serial.print("Size of message: ");
  // Serial.println(messageLength);
  sprintf(buf, "*%.2x\r\n", checkSum);
  strcat(outMessage, buf);
  return sizeof(outMessage);
}









// Print commands  %toCheck:  Need to make new list of commands
void commands() {
  Send_tx_String(txString);
  strcpy(txString, "");
  strcat(txString, "\nCommands \n# --------------------------------------\n");
  strcpy(txString, "");
  strcat(txString, "\n Serial and BLE:\n# .................\n");
  strcat(txString, "STOP  Stops measurement \nSTART Starts new measurement  \n" );
  strcat(txString, "SETRESFREQ  Set resonant frequuency for measurements. E.G. SETRESFREQ:1: \n");
  strcat(txString, "SETMULTIFREQ Go to multifrequency mode.   \n");
  strcat(txString, "SETGAIN2  Set digital gain DAC and signal strength\n");
  strcat(txString, "DEF_MFREQ  Used to change frequencies of multifreq mode. E.g.:  DEF_MFREQ:3:1:4:8: \n");
  
  Send_tx_String(txString);
  delay(100);
  strcpy(txString, "");
  strcat(txString, "CAL  Start calibration process. To be called during measurement \n");
  strcat(txString, "COMS  List commands \n");
  strcat(txString, "CHECKLASER  Get Laser data for 1 second and send it over BLE and serial \n" );
  strcat(txString, "CHECKINS  Get INS data and send them over BLE+serial for manual check \n");
  strcat(txString, "TEMP  get temperature \n");
  strcat(txString, "VBAT  get battery voltage\n");
  strcat(txString, "RESET  Reset ESP32 \n");
  
  Send_tx_String(txString);
  delay(100);
  strcpy(txString, "");
  strcat(txString, "\n MSP430 commands:\n# ,,,,,,,,,,,,,,,,\n");
  strcat(txString, "RESMICRO  Reset MSP430 \n");
  strcat(txString, "STATUS  Check status MSP430 \n");

  strcat(txString, "# --------------------------------------\n");
  Send_tx_String(txString);
  delay(100);
  strcpy(txString, "");
}
