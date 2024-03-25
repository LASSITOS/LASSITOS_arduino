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



void readFileLong(fs::FS &fs, const char *path) {
  Serial.printf("## Reading file: %s\n", path);
  File file = fs.open(path);
  if (!file) {
    Serial.println("Failed to open file for reading");
    Serial.println("&END&");
    return;
  }
  uint64_t totalbites=0;
  int bufferfull=0;
  //    Serial.println("Read from file: ");
  Serial.println("&START&");
  Serial.flush();
  uint8_t SDTempBuffer[128];
  while (128<file.available()) {
    // while (Serial.availableForWrite()<32){
    //   Serial.println(Serial.availableForWrite());
    //   delay(1);
    //   bufferfull++;
    // }
    // totalbites+=Serial.write(file.read());
    file.read( SDTempBuffer,128);
    Serial.write( SDTempBuffer,128);
    totalbites+=128;
    // 
  }
  // Serial.println("&tosinglebite&");
  while (file.available()) {
    // while (Serial.availableForWrite()<32){
    //   Serial.println(Serial.availableForWrite());
    //   delay(1);
    //   bufferfull++;
    // }
    totalbites+=Serial.write(file.read());
    // 
  }
  file.close();
  Serial.println("&STOP&");
  // Serial.printf("Buffer was full n=%d times\n", bufferfull);
  Serial.printf("#%%TotalBites: %d\r\n", totalbites);
  Serial.println("&END&");
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
  dataFile.println(F("#RTK GPS log file"));
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
  dataFile.println(F("#Other settings ESP32: "));
  dataFile.println(F("# --------------------------"));
  dataFile.print("# Rate SPI SD: ");
  dataFile.println(SPI_rate_SD);
  
  dataFile.println("#");
}




// Make new files in SD card. file names are derived from GNSS time if available.
void makeFiles(fs::FS &fs) {
  Serial.println(F("Making new files"));

  // Check for GPS to have good time and date
  //getDateTime();
  // sprintf(dataFileName, "/INS%02d%02d%02d_%02d%02d%02d.csv", Year % 100, Month, Day, Hour, Minute,Second);  // create name of data file
  strcpy(dataFileName, "/GPS_test.csv" );                                                                                        //sprintf(dataFileName, "/testfile.csv");   // create name of data file
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

  strcpy(txString, "");
  strcat(txString, "Created file: ");
  strcat(txString, dataFileName);
  sendToBase(txString);
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
      if (rxValue.indexOf("READTOT") != -1) {
		readFileLong(SD, path);
		Serial.println(" \n## End of file");
	  } else {
		readFile(SD, path);
		Serial.println(" \n## End of file");
	  }
    } else if (index = -1) {
      Serial.print("Reading file:");
      Serial.println(dataFileName);
      readFile(SD, dataFileName);

    } else {
      sprintf(txString, "File can not be parsed form string '%s'. Valid format is 'READ' or 'READ:<<filepath>>:' !", rxValue);
      Serial.println(txString);
    }
  } else {
    strcpy(txString, "Data Measuring running. Can't read files now. First stop measurment!");
    Serial.println(txString);
  }
  sendToBase(txString);
}



void getFileList() {
  /*
    Return list of files in SD card
    */
  if (!measuring) {
    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);
    Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
    Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
    Serial.println("");
    Serial.println("");
    Serial.println("%% File list:");
    listDir(SD, "/", 0);
    Serial.println("%% end");

  } else {
    Serial.println("Data Measuring running. Can't read file list now. First stop measurment!");
  }
}

