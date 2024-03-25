
// Parsing NMEA messages
//-------------------------
void LEM_Handler(){
  Serial.print("Got $LEM message with ");
  Serial.print(parser.argCount());
  Serial.print(" arguments.  ");
  char arg0[16];
  char msgOut[64];
  int arg;
  char valStr[16];

  if (parser.getArg(0,arg0)) {
    Serial.print(" Message handle: ");
    Serial.println(arg0);
    strcpy(msgOut,arg0);
  }
  else{
    Serial.print("Coud not get messge handle! ");
    return;
  }


  
  if (parser.argCount()>1){
    strcat(msgOut,":");
    for (int i=1;i<parser.argCount();i++){
      if (parser.getArg(i,arg)){
        sprintf(valStr,"%d:",arg);
        strcat(msgOut,valStr);
      }
    }
  } 
  // Serial.print(" LEM command: ");
  // Serial.println(msgOut);
  // sendToBase(msgOut);
  parse( msgOut);
  
}



void GGA_Handler(){
  Serial.print("Got GGA message with ");
  Serial.print(parser.argCount());
  Serial.print(" arguments.  ");
  char arg0[16];
  char msgOut[64];
  int arg;
  char valStr[16];

  if (parser.getArg(0,arg0)) {
    Serial.print(" Time: ");
    Serial.println(arg0);
    strncpy(valStr,arg0,2);
    Hour = atoi(valStr);
    strncpy(valStr,arg0+2,2);
    Minute = atoi(valStr);
    // Second = GPDZA.substring(4, 6).toInt();
  }
  else{
    Serial.print("Coud not get messge handle! ");
    return ;
  }
  return ;
}




void unknownCommand()
{
  // Serial.print("*** Unkown command : ");
  // char buf[6];
  // parser.getType(buf);
  // Serial.println(buf);
}

void errorHandler()
{
  if (parser.error()>1){
    Serial.print("*** Error: ");
    Serial.println(parser.error()); 
  }

}











// /////////////////////////////////////////////
// ---------------------------------------------
//GPS code
// ---------------------------------------------
// /////////////////////////////////////////////


void getDateTime() {
  lastTime = millis();
  validDate=0; 
  while (!validDate) {
    if (GPS.available()) {  // Check Serial inputs
      while(GPS.available())  {   
            parser << GPS.read();
      }
    }
    if (millis() - lastTime > 5000) {
      // Serial.print(F("No GPS. Setting date to default and random time!"));
      out = 1;
    }
    delay(10);
  }


  Serial.print(F("Current date:"));
  strcpy(txString, printDateTime().c_str());
  Serial.println(txString);
}




String printDateTime() {
  sprintf(subString, "%d-%d-%d %d:%d:%d", Year, Month, Day, Hour, Minute, Second);
  Serial.println(subString);
  return subString;
}






// Get INS data for 1 second and send it over BLE and serial
void check_INS() {
  sendToBase(txString);
  strcpy(txString, "");
  if (!measuring) {
    strcat(txString, "\nGPS data \n# --------------------------------------\n");
    sendToBase(txString);
	strcpy(txString, "");
    while (GPS.read() >= 0)
      ;  // flush the receive buffer.
    delay(1000);
    int availableBytes = GPS.available();
	
	if (LaserBufferSize < availableBytes) {
        availableBytes = LaserBufferSize;
		  Serial.println("# Buffer full. Data were cut.");
    }
	
    GPS.readBytes(myBuffer_laser, availableBytes);
    Serial.write(myBuffer_laser, availableBytes);

    strcat(txString, "\n# --------------------------------------\n");
    sendToBase(txString);
  } else {
    strcpy(txString, "Datalogging running. Can't run INS_check now!");
  }
  sendToBase(txString);
}


