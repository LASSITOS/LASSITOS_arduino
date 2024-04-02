//Parsing LEM settings from SD car LEM_settings.txt file. 


void readSettingsFile(fs::FS &fs) {
  #define pathSettings "/LEM_settings.txt"
  Serial.printf("\n## Reading LEM settings from file: %s\n---------------------------------\n", pathSettings);
  File file = fs.open(pathSettings);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  char line[256];
  char c;
  int i=0;
  while (file.available()) {
    c=file.read();
    
    if (c=='\n' ){
      line[i]='\0';
      // Serial.print("Parsing line: ");
      // Serial.println(line);
      parseSettingLine(line);
      i=0;
    } else if(c=='\r' ){
      ;
    } else if(i==255){
      line[i]='\0';
      Serial.print("Line too long: ");
      Serial.println(line);
      parseSettingLine(line);
      i=0;
    } else{
        line[i]=c;
        i++;
    }

	}
  file.close();
}


void parseSettingLine(String line){
	int index;
	bool out;
	// cut comments
	index=line.indexOf("#");
	if (index!=-1){
		line.remove( index);
		// Serial.println("line has # character. Comments discarded");
	}
	
	index=line.indexOf("=");
	if (index!=-1){
		// Serial.println(line);
		out=lineToVariable(line.substring(0, index),line.substring(index+1));
	}
	
	updateAfterLoadSettings();
}


bool lineToVariable(String var_string, String val_string){
	
	int *int_vars[]={&F1, &F2, &F3, &F4, &F5, &F6, &F46, &F456,&ifreq,&Nmulti_reg, &MultiPeriod, 
                   & N_cal, &CAL_intervall_on, &CAL_intervall_off, &N_CalMulti_on, &N_CalMulti_off, &N_CalCoil, &N_RxCoil, &N_BuckCoil }; 
	String int_var_names[]={"F1","F2","F3","F4","F5","F6","F46","F456","ifreq","Nmulti_reg","MultiPeriod",
                          "N_cal","CAL_intervall_on","CAL_intervall_off" ,"N_CalMulti_on","N_CalMulti_off","N_CalCoil","N_RxCoil","N_BuckCoil"}; 
	int N_int_vars=18;
  
  float *fl_vars[]={ &d_Rx, &d_Bx, &d_Cx, &L_CalCoil, &A_CalCoil , &A_RxCoil, &A_BuckCoil, &distCenter}; 
	String fl_var_names[]={"d_Rx","d_Bx","d_Cx","L_CalCoil","A_CalCoil","A_RxCoil","A_BuckCoil","distCenter"}; 
	float N_fl_vars=8;

  // list non int: ifreq,gainDAT

  var_string.trim() ; //remove white spaces (\t,space)
  val_string.trim() ; //remove white spaces (\t,space)

  for (int i=0;i<N_int_vars;i++){
    if (var_string.equals(int_var_names[i]) ) {
        *int_vars[i]=val_string.toInt();
        Serial.print(int_var_names[i]);
        Serial.print(":");
        Serial.println(*int_vars[i]);
        return 1;
    }
  }

  for (int i=0;i<N_fl_vars;i++){
    if (var_string.equals(fl_var_names[i]) ) {
        *fl_vars[i]=val_string.toFloat();
        Serial.print(fl_var_names[i]);
        Serial.print(":");
        Serial.println(*fl_vars[i]);
        return 1;
    }
  }
	
	if (var_string.equals("LEM_version") ) {
		LEM_version=val_string;
		Serial.print("LEM_version:");
		Serial.println(LEM_version);
    return 1;
		
	} else if (var_string.equals("LEM_ID") ) {
		LEM_ID=val_string;
		Serial.print("LEM_ID:");
		Serial.println(LEM_ID);
    return 1;

	} else if (var_string.equals("BLEname") ) {
		BLEname=val_string;
		Serial.print("BLEname:");
		Serial.println(BLEname);
    return 1;

  } else if (var_string.equals("ifreq") ) {
		ifreq=val_string.toInt();
		Serial.print("ifreq:");
		Serial.println(ifreq);
    return 1;
} else if (var_string.equals("Nfreq") ) {
		Nfreq=val_string.toInt();
		Serial.print("Nfreq:");
		Serial.println(Nfreq);
    return 1;
	} else if (var_string.equals("gainDAT") ) {
		gainDAT=val_string.toInt();
		Serial.print("gainDAT:");
		Serial.println(gainDAT);
    return 1;

	} else if (var_string.equals("Multifreqs") ) {
    for( int i=0;i<10;i++) Multifreqs[i]=0;
		
    int index= 0;
		int ind2 ;
		int i=0;
    Serial.print(var_string);
		Serial.print(":");
    // Serial.print(val_string);
		// Serial.print(":");
		while(val_string.indexOf(",",index)!=-1){
			ind2 = val_string.indexOf(",",index+1)+1;
      // Serial.print(index);
      // Serial.print(":");
      // Serial.print(ind2);
      // Serial.print(":");
      // Serial.print(val_string.substring(index,ind2-1));
			// Serial.print(":");
			val_string.substring(index,ind2-1).toCharArray(datStr ,ind2-index);
			// Serial.print(datStr);
			// Serial.print(":");
			Multifreqs[i]=strtoul (datStr, NULL, 16);
      Serial.print(Multifreqs[i]);
			Serial.print(",");
			i++;
			index=ind2;
    }
		Serial.println(' ');
    return 1;
		
    
  } else if (var_string.equals("CAL_states") ) {
      for( int i=0;i<10;i++) CAL_states[i]=0;
      
      int index= 0;
      int ind2 ;
      int i=0;
      Serial.print(var_string);
      Serial.print(":");
      while(val_string.indexOf(",",index)!=-1){
        ind2 = val_string.indexOf(",",index+1)+1;
        CAL_states[i]=val_string.substring(index,ind2-1).toInt();
        Serial.print(CAL_states[i]);
        Serial.print(",");
        i++;
        index=ind2;
      }
      Serial.println(' ');
      return 1;
    
    
  } else if (var_string.equals("CAL_states") ) {
    for( int i=0;i<10;i++) Rs[i]=0;
		
    int index= 0;
		int ind2 ;
		int i=0;
    Serial.print(var_string);
		Serial.print(":");
		while(val_string.indexOf(",",index)!=-1){
			ind2 = val_string.indexOf(",",index+1)+1;
			Rs[i]=val_string.substring(index,ind2-1).toFloat();
      Serial.print(Rs[i]);
			Serial.print(",");
			i++;
			index=ind2;
    }
		Serial.println(' ');
    return 1;

	} else {
		Serial.print("Variable was not found for:");
		Serial.println(var_string);
    return 0;
	}
  
	
}


void updateAfterLoadSettings(){
//Update some variables based on SD settings. E.G. freqs array
freqs[0]=F1;
freqs[1]=F2;
freqs[2]=F3;
freqs[3]=F456;
freqs[4]=F46;
freqs[5]=F4;
freqs[6]=F5;
freqs[7]=F6;
freq=freqs[ifreq];

}