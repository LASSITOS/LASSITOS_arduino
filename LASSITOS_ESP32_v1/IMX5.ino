
// /////////////////////////////////////////////
// ---------------------------------------------
//INS code
// ---------------------------------------------
// /////////////////////////////////////////////


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


// setting up INS
void setupINS() {
  if (!FormatAsciiMessage(asciiMessage, sizeof(asciiMessage), asciiMessageformatted)) {
    Serial.println("Failed to encode ASCII get INS message\r\n");
  } else {
    Serial.println(asciiMessageformatted);
    IMX5.write(asciiMessageformatted);  //send instruction for sending ASCII messages
    Serial.println("Send instruction for sending ASCII messages to IMX5");
  }
  while (IMX5.read() >= 0)
    ;
  Serial.println(F("INS setting updated"));
  strcat(txString, "INS setting updated");

  pulseStrobeIMX5();  
  lastTime_STROBE = millis();
}




void getDateTime() {
  char msgOut[256];
  char msgStart[] = "$ASCB,0,,,,,,,,,,,12,";
  FormatAsciiMessage(msgStart, sizeof(msgStart), msgOut);
  int out = 0;
  String msg = "NotValidMessage";
  IMX5.write("$STPC*14\r\n");
  Serial.println(msgOut);
  IMX5.write(msgOut);
  lastTime = millis();
  while (!out) {
    if (IMX5.available()) {  // Check Serial inputs
      // String msg = Serial.readString();
      bitesToWrite = IMX5.available();
      if (tempBufferSize < bitesToWrite) {
        bitesToWrite = tempBufferSize;
      }
      IMX5.readBytes(tempBuffer, bitesToWrite);
      tempBuffer[bitesToWrite]='\0';
      String msg = (char*)tempBuffer;
      // Serial.println(msg);
      msg[bitesToWrite]='\0';
      Serial.printf("BitestoWrite: %d",bitesToWrite);
      out = parseDateTime(msg);
    }
    if (millis() - lastTime > 5000) {
      Serial.print(F("No GPS. Setting date to default and random time!"));
      // out = parseDateTime("$GPZDA,001924,06,01,1980,00,00*41");
      out = parseDateTime("NotAValidMessage");
      out = 1;
    }
    delay(10);
  }
  IMX5.write("$STPC*14\r\n");
  char msgStop[] = "$ASCB,0,,,,,,,,,,,0,";
  FormatAsciiMessage(msgStop, sizeof(msgStop), msgOut);
  Serial.println(msgOut);
  IMX5.write(msgOut);

  Serial.print(F("Current date:"));
  strcpy(txString, printDateTime().c_str());
  Serial.println(txString);
}

int parseDateTime(String GPDZA) {
  //$GPZDA,001924,06,01,1980,00,00*41\r\n
  //$GPZDA,032521,08,02,2023,00,00*46   
  int a=GPDZA.indexOf("$GPZDA");
  int b=GPDZA.indexOf("*",a+1);
  if (a != -1 and b!= -1 and a<b ) {
    Serial.print("Valid msg:");
    Serial.println(GPDZA.substring(a, b+3));
    int index[6];
    index[0] = a + 7;
    // Serial.print(index[0]);
    for (int i = 0; i < 5; i++) {
      index[i + 1] = GPDZA.indexOf(',', index[i]) + 1;
      // Serial.print(i+1);
      // Serial.println(index[i+1]);
    }
    // Serial.println(GPDZA.substring(index[1],index[2]-1));
    Day = GPDZA.substring(index[1], index[2] - 1).toInt();
    // Serial.print(Year);
    // Serial.println(GPDZA.substring(index[2],index[3]-1));
    Month = GPDZA.substring(index[2], index[3] - 1).toInt();
    // Serial.print(Month);
    // Serial.println(GPDZA.substring(index[3],index[4]-1));
    Year = GPDZA.substring(index[3], index[4] - 1).toInt();
    // Serial.print(Day);
    // Serial.println(GPDZA.substring(index[0],index[0]+2));
    Hour = GPDZA.substring(index[0], index[0] + 2).toInt();
    // Serial.print(Hour);
    // Serial.println(GPDZA.substring(index[0]+2,index[0]+4));
    Minute = GPDZA.substring(index[0] + 2, index[0] + 4).toInt();
    // Serial.print(Minute);
    // Serial.println(GPDZA.substring(index[0]+4,index[0]+6));
    Second = GPDZA.substring(index[0] + 4, index[0] + 6).toInt();
    // Serial.print(Second);
    // Serial.println("Date parsed");
    return 1;
  } else {
    Year = 2000;
    Month = 01;
    Day = 01;
    Hour = random(23);
    Minute = random(59);
    Second = random(59);

    Serial.print("Msg not valid:");
    Serial.println(GPDZA);
    
    return 0;
  }
}


String printDateTime() {
  sprintf(subString, "%d-%d-%d %d:%d:%d", Year, Month, Day, Hour, Minute, Second);
  Serial.println(subString);
  return subString;
}


void setIMX5message(){
	int IMXrate2=int( round(IMXrate/IMUdataRate));
	sprintf(asciiMessage, "$ASCB,512,,,%d,,,,,,,,,",IMXrate2);
}


void pulseStrobeIMX5(){
 digitalWrite(IMX5_strobe, HIGH);
 delay(1);
 digitalWrite(IMX5_strobe, LOW);
}

