
// /////////////////////////////////////////////
// ---------------------------------------------
// Input parsing functions    %toCheck
// ---------------------------------------------
// /////////////////////////////////////////////
void parse( String rxValue){     //%toCheck
  //Start new data files if START is received and stop current data files if STOP is received
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



  
  } else if (rxValue.indexOf("READ") != -1) {
	readFiles(rxValue);
  
  } else if (rxValue.indexOf("LIST") != -1) {
	getFileList();
  } else if (rxValue.indexOf("COMS") != -1 or rxValue.indexOf("?") != -1) {
	commands();

  } else if (rxValue.indexOf("CHECKGPS") != -1) {
	check_INS();
 
  } else {
	strcpy(txString, "Input can not be parsed retry!");
	Serial.println(txString);
  }
}





// Print commands  %toCheck:  Need to make new list of commands
void commands() {
  sendToBase(txString);
  strcpy(txString, "");
  strcat(txString, "\nCommands \n# --------------------------------------\n");
  strcat(txString, "\n Serial only\n# .................\n");
  strcat(txString, "READ Read last file of last measurement. If argument is passed with READ:<< nfile>>: read file nfile \nLIST  Get list of files in SD card \n ");
  sendToBase(txString);
  delay(100);
  strcpy(txString, "");
  strcat(txString, "\n Serial and BLE:\n# .................\n");
  strcat(txString, "STOP  Stops measurement \nSTART Starts new measurement  \n" );
  strcat(txString, "CHECKGPS  Check if GPS data are received\n");
  strcat(txString, "LIST  List files in SD card\n");
  sendToBase(txString);
  delay(100);
  strcpy(txString, "");
}
