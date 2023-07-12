// /////////////////////////////////////////////
// ---------------------------------------------
// SD card funtions und code
// ---------------------------------------------
// /////////////////////////////////////////////

void listDir(fs::FS &fs, const char *dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels) {
        listDir(fs, file.path(), levels - 1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

void createDir(fs::FS &fs, const char *path) {
  Serial.printf("Creating Dir: %s\n", path);
  if (fs.mkdir(path)) {
    Serial.println("Dir created");
  } else {
    Serial.println("mkdir failed");
  }
}

void removeDir(fs::FS &fs, const char *path) {
  Serial.printf("Removing Dir: %s\n", path);
  if (fs.rmdir(path)) {
    Serial.println("Dir removed");
  } else {
    Serial.println("rmdir failed");
  }
}

void readFile(fs::FS &fs, const char *path) {
  Serial.printf("## Reading file: %s\n", path);
  int nmax=50000;
  File file = fs.open(path);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  //    Serial.println("Read from file: ");
  while (file.available()) {
    Serial.write(file.read());
	nmax--;
	if (nmax==0){
		Serial.println("####### maximum number of characters has been written!");
		break;
	}
  }
  file.close();
}

void writeFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

void renameFile(fs::FS &fs, const char *path1, const char *path2) {
  Serial.printf("Renaming file %s to %s\n", path1, path2);
  if (fs.rename(path1, path2)) {
    Serial.println("File renamed");
  } else {
    Serial.println("Rename failed");
  }
}

void deleteFile(fs::FS &fs, const char *path) {
  Serial.printf("Deleting file: %s\n", path);
  if (fs.remove(path)) {
    Serial.println("File deleted");
  } else {
    Serial.println("Delete failed");
  }
}







void Write_header() {
  dataFile.println(F("#LEM INS and Laser altimeter log file"));
  dataFile.print("# Date: ");
  dataFile.print(Year);
  dataFile.print("-");
  dataFile.print(Month);
  dataFile.print("-");
  dataFile.println(Day);
  dataFile.print("# Time: ");
  dataFile.print(Hour);
  dataFile.print(":");
  dataFile.print(Minute);
  dataFile.print(":");
  dataFile.println(Second);
  dataFile.print(F("# Script version: "));
  dataFile.println(Version);
  dataFile.print(F("# IMX5 NMEA settings: "));
  dataFile.println(asciiMessage);
  dataFile.println(F("#Signal generation settings: "));
  dataFile.println(F("# --------------------------"));
  if (Nmulti==0){
    dataFile.printf("# Frequency: %d",freq);
  }else{
	  dataFile.print("# Frequency: multifrequency"); 
	for (int i=0; i < Nmulti ;i++){
		dataFile.printf("# \t F%d: %d kHz",i+1,freqs[Multifreqs[i]]); 
	  }
  }
  dataFile.println("#");
}


void log_Laser_settings() {
  dataFile.println(F("#Laser settings: "));
  dataFile.println(F("# --------------"));
  // for (int i = 0; i < 3; i++) {
  //   RS232.write(0x1B);
  //   delay(50);
  // }
  Serial.println("Reading Laser setting:");
  RS232.write(0x1B);
  delay(250);
  RS232.write(0x0D);
  delay(100);
  while (RS232.read() >= 0)
    ;  // flush the receive buffer.
  uint8_t *Buffer_lasersetting = new uint8_t[4096];
  unsigned long startTime = millis();
  bitesToWrite=0;
  RS232.write("ID");
  RS232.write(0x0D);
  delay(100);
  RS232.write("PA");
  RS232.write(0x0D);
  delay(100);
  while ((millis() -startTime) < 500) {
    while (RS232.available()){
      Buffer_lasersetting[bitesToWrite]=RS232.read();
      bitesToWrite++;
    }
    delay(5);
  }
  if( Buffer_lasersetting>0){
    dataFile.write(Buffer_lasersetting, bitesToWrite);
    Serial.write(Buffer_lasersetting, bitesToWrite);
  } else{
    Serial.println("No data from Laser");
  }
  delete[] Buffer_lasersetting;
  dataFile.println(F("# --------------------------------------"));
  dataFile.println(F(" "));
}



// Make new files in SD card. file names are derived from GNSS time if available.
void makeFiles(fs::FS &fs) {
  Serial.println(F("Making new files"));

  // Check for GPS to have good time and date
  getDateTime();
  sprintf(dataFileName, "/INS%02d%02d%02d_%02d%02d.csv", Year % 100, Month, Day, Hour, Minute);  // create name of data file
                                                                                                  //sprintf(dataFileName, "/testfile.csv");   // create name of data file
  Serial.println(dataFileName);
  dataFile = fs.open(dataFileName, FILE_WRITE);
  if (!dataFile) {
    Serial.println(F("Failed to create data file! "));
    Serial.println(dataFileName);
    SD_filecreated=0;
  } else{
	SD_filecreated=1;
  }
  
  delay(100);
  Write_header();
  delay(100);
  log_Laser_settings();
  Serial.println("Logged Laser settings");
  delay(100);

  strcpy(txString, "");
  strcat(txString, "Created file: ");
  strcat(txString, dataFileName);
  Send_tx_String(txString);
}




void Write_stop() {
  dataFile.println(F("#STOP"));
  dataFile.println(F("#--------------------------------------"));
}





void readFiles(String rxValue) {
  /*
    Read last files (haderfile and datafile) if no argument is passed  ("READ" ) otherwise read passed files (READ:<<filepath>>:).
    */
  if (!measuring) {
    //    Serial.println(rxValue);
    int index = rxValue.indexOf(":");
    int index2 = rxValue.indexOf(":", index + 1);
    //    Serial.println(index );
    //    Serial.println(index2);

    if (index != -1 and index2 != -1) {
      char path[32];
      rxValue.substring(index + 1, index2).toCharArray(path, 32);
      Serial.println("");
      //      Serial.print("%% Reading file:");
      //      Serial.println(path);
      readFile(SD, path);
      Serial.println(" \n## End of file");

    } else if (index = -1) {
      Serial.print("Reading file:");
      Serial.println(dataFileName);
      readFile(SD, dataFileName);

    } else {
      BLE_message = true;
      sprintf(txString, "File can not be parsed form string '%s'. Valid format is 'READ' or 'READ:<<filepath>>:' !", rxValue);
      Serial.println(txString);
    }
  } else {
    BLE_message = true;
    strcpy(txString, "Data Measuring running. Can't read files now. First stop measurment!");
    Serial.println(txString);
  }
  Send_tx_String(txString);
}

void getFileList() {
  /*
    Return list of files in SD card
    */
  if (!measuring) {
    Serial.println("");
    Serial.println("%% File list:");
    listDir(SD, "/", 0);
    Serial.println("%% end");

  } else {
    Serial.println("Data Measuring running. Can't read file list now. First stop measurment!");
  }
}

