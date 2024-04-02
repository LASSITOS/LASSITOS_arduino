
// /////////////////////////////////////////////
// ---------------------------------------------
// Input parsing functions    %toCheck
// ---------------------------------------------
// /////////////////////////////////////////////
void parse( String rxValue){     //%toCheck
  //Start new data files if START is received and stop current data files if STOP is received
  if (rxValue.indexOf("START") != -1) { 
    restart_logging();
  } else if (rxValue.indexOf("STOP") != -1) {
    logging = false;
    stop_logging(SD);
  } else if (rxValue.indexOf("RATE") != -1) {
    setRate(rxValue);
  } else if (rxValue.indexOf("READ") != -1) {
    readFiles(rxValue);
  } else if (rxValue.indexOf("LIST") != -1) {
    getFileList();
  } else if (rxValue.indexOf("CHECKATT") != -1) {
    check_attitude( );
  } else if (rxValue.indexOf("COMS") != -1 or rxValue.indexOf("?") != -1)  {
    commands(); 
	
  } else {
	strcpy(txString, "Input can not be parsed retry!");
	
  Send_tx_String(txString);
  }
}



// Print commands
void  commands(){  
   Send_tx_String(txString);
   strcpy(txString,"");

   strcat(txString,"\n Serial and BLE:\n# .................\n");
   strcat(txString," STOP  Stops measurement \nSTART Starts new measurement \nRATE:<<N>>: Set sampling rate to N \nCHECKATT  Get attitude angles from IMU anf GPS coordinates \nCOMS  List commands \n");
   strcat(txString,"CHECKIMU  Get IMU data and send them over BLE+serial for manual check \nDYNMODEL:<<n>>: Set dynamic model to n.  Sensor need to be recalibrated after changing the setting.\n        n=4: Automotive (default) \n        n=0: Applications with low acceleration. \n        n=6: Airborne. Greater vertical acceleration  \n");
   Send_tx_String(txString);
   delay(100);
   strcpy(txString,"");
   strcat(txString,"IMUCAL:<<b>>: Change setting for regular check for IMU calibration. b=0,1 \n LOGRMX:<<b>>: Log RMX messages or not. b=0,1 \nNAVPVT:<<b>>: same \nNAVPVAT:<<b>>: same  \n");
   strcat(txString,"ESFINS:<<b>>: same\nESFRAW:<<b>>:  same \nESFMEAS:<<b>>:  same \nESFALG:<<b>>: same \n");
   strcat(txString,"# --------------------------------------\n");   
   strcat(txString,"\nCommands \n# --------------------------------------\n");
   strcat(txString,"\n Serial only\n# .................\n");
   strcat(txString,"READ Read last file of last measurement. If argument is passed with READ:<< nfile>>: read file nfile \nLIST  Get list of files in SD card \n ");

   Send_tx_String(txString);
   delay(100);
   strcpy(txString,"");
}

