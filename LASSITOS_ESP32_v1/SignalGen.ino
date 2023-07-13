

// /////////////////////////////////////////////
// ---------------------------------------------
// SignalGen functions
// ---------------------------------------------
// /////////////////////////////////////////////
void configureResFreq(int ifreq){   //set resonant frequency from set or possible frequencies.
  freq=freqs[ifreq];
  CSw_state=CSw_states[ifreq];
  configureSineWave();
  setCswitchTx ( CSw_state );
}


void configureSineWave(){

  // Prepare the parameters:
  uint64_t temp=freq*0x1000000*clock_divider;
  uint64_t freqTW=temp/AWG_clock_freq;  // get frequency tuning word 0x1000000=2**24
  uint16_t freqMSB=freqTW>>8;
  uint16_t freqLSB=(freqTW&0xFF)<<8;

  
  writeReg(0x27,0x0031);  //Sinewave Mode channel 1
  // writeReg(0x27,0x3131);  //Sinewave Mode, channel 1 and 2
  writeReg(0x26,0x0031);  //Sinewave Mode channel 3
//  
  writeReg(0x45,0x0000); //Static phase/freq
  writeReg(0x3E,freqMSB); //Freq MSB
  writeReg(0x3F,freqLSB); //Freq LSB

//  gainDAT=int((gain+2)*4095/4) << 4;
  writeReg(0x35,gainDAT); //digital gain Ch1
  writeReg(0x34,gainDAT); //digital gain Ch2
  writeReg(0x33,gainDAT); //digital gain Ch3
}

void run(){
  // out=readReg(0x1E);
  // Serial.print("Current run mode:");
  // Serial.println(out,BIN);
  writeReg(0x1E,0x0001);
  delay(1);
  trigger();
  out=readReg(0x1E);
  Serial.print("New run mode:");
  Serial.println(out,BIN);
  if ((out & 0x3)!=0x03){
	ADCstarted=0;
	Serial.print("DAC didn't start correctly!");
  }else{
	ADCstarted=1;  
	Serial.print("DAC started correctly!");
  }
}

void run2(){
  writeReg(0x1E,0x0001);
  trigger();
  out=readReg(0x1E);
  if ((out & 0x3)!=0x03){
	ADCstarted=0;
  }else{
	ADCstarted=1;  
  }
}


void trigger(){
  // Serial.println("[AWG] Triggerring");
  digitalWrite(triggerDAC,HIGH);
  delay(1);
  digitalWrite(triggerDAC,LOW);
}

void stop_trigger(){
  // Serial.println("Stop triggerring");
  digitalWrite(triggerDAC,LOW);
  delay(1);
  digitalWrite(triggerDAC,HIGH);
  writeReg(0x1E,0x0000);
}

void setGain2(int value){
//  gainDAT=int((gain+2)*4095/4) << 4;
  gainDAT=value;
  writeReg(0x35,gainDAT); //digital gain ch1
  writeReg(0x34,gainDAT); //digital gain Ch2
  writeReg(0x33,gainDAT); //digital gain Ch3  
}



// /////////////////////////////////////////////
// ---------------------------------------------
// Signal generation: running and test functions
// ---------------------------------------------
// /////////////////////////////////////////////


void testLong(){
	delay(500);
	strcpy(txString,"Starting long test");
	Send_tx_String(txString) ;
	startMicro2();   // start recording of ADC data for given duration in seconds
  digitalWrite(PIN_MUTE , LOW);  // mute
  delay(5000);
  digitalWrite(PIN_MUTE , HIGH);  // unmute
  delay(1000);
  digitalWrite(PIN_MUTE , LOW);  // mute
  
	for (int i=0; i<Nfreq; ++i) {
		freq=freqs[i];
		configureSineWave();
		// sprintf(subString,"f: %d",freq );
    // strcat(txString,subString);
		// Send_tx_String(txString); 
    // if(i==0){ 
    //   digitalWrite(PIN_GPIOswitch , HIGH);
    // }else{
    //   digitalWrite(PIN_GPIOswitch , LOW);
    // }
    setCswitchTx(CSw_states[i]);
    delay(10);
    digitalWrite(PIN_MUTE , HIGH);  // unmute
    run2();
    trigger();
    delay(2000);
    // Pause for a tenth of a second between notes.
    stop_trigger();
    digitalWrite(PIN_MUTE , LOW);  // mute
    delay(200);
		}
   
   delay(1000);
   stopMicro();
   delay(200);
   strcpy(txString,"End of test");
   Send_tx_String(txString) ;
}



void testLongSingle(int i, int len){
	delay(500);
	strcpy(txString,"Starting long test");
	Send_tx_String(txString) ;
	startMicro2();   // start recording of ADC data for given duration in seconds
  digitalWrite(PIN_MUTE , LOW);  // mute
  delay(2000);
  digitalWrite(PIN_MUTE , HIGH);  // unmute
  delay(2000);
  digitalWrite(PIN_MUTE , LOW);  // mute
  
  freq=freqs[i];
	configureSineWave();
  setCswitchTx(CSw_states[i]);
  delay(10);
  digitalWrite(PIN_MUTE , HIGH);  // unmute
  run2();
  trigger();

  delay(len);

  stop_trigger();
  digitalWrite(PIN_MUTE , LOW);  // mute
  
  delay(1000);
  stopMicro();
  delay(200);
  strcpy(txString,"End of test");
  Send_tx_String(txString) ;
}


void testCal(){
	delay(500);
	strcpy(txString,"Starting cal test");
	Send_tx_String(txString) ;
	startMicro2();   // start recording of ADC data for given duration in seconds
  digitalWrite(PIN_MUTE , LOW);  // mute
  delay(1000);
  
	for (int i=0; i<Nfreq; ++i) {
		freq=freqs[i];
		configureSineWave();
    setCswitchTx(CSw_states[i]);
    delay(10);
    digitalWrite(PIN_MUTE , HIGH);  // unmute
    delay(100);
    run2();
    trigger();
    delay(1000);
		
		for (int j=1; j<4; ++j) {
			setCswitchCal(CAL_states[j]);
			delay(3000);
			setCswitchCal(0);
			delay(2000);
		}
		
		stop_trigger();
    digitalWrite(PIN_MUTE , LOW);  // mute
   }
   delay(1000);
   stopMicro();
   delay(200);
   strcpy(txString,"End of test");
   Send_tx_String(txString) ;
}





void frequencySweep(int start,int stp,int Delta){
	  int A=start/Delta;
	  int B=stp/Delta;
	  delay(500);
	  strcpy(txString,"Starting frequency sweep");
	  Send_tx_String(txString) ;
	  startMicro2();   // start recording of ADC data for given duration in seconds
	  delay(2000);
    digitalWrite(PIN_MUTE , HIGH);  // unmute
    
	  for (int i=A; i<B; ++i) {
		  freq=i*Delta;
		  configureSineWave();
		  digitalWrite(PIN_MUTE , HIGH);  // unmute
		  // run();
      run2();
      trigger();
		  delay(200);
		  stop_trigger();
		  digitalWrite(PIN_MUTE , LOW);  // mute
		  delay(50);
	  }
    Send_tx_String(txString);
    delay(1000);
    stopMicro();
	  strcpy(txString,"End of sweep");
	  Send_tx_String(txString) ;
}	  



void GainSweep(int start,int stp,int Delta){

    if(stp>MAXGAIN){
      sprintf(txString,"Stop gain is larger then maximum gain: %04X",MAXGAIN );
      Serial.print("STOP");
      Serial.println(stp);
      Serial.print("Maxgain");
      Serial.println(MAXGAIN);
      Send_tx_String(txString) ;
      return;
      }
    
	  int A=start/Delta;
	  int B=stp/Delta;
	  delay(500);
	  strcpy(txString,"Starting gain sweep");
	  Send_tx_String(txString) ;
	  startMicro2();   // start recording of ADC data for given duration in seconds
	  delay(2000);
      
      
	  for (int j=0; j<Nfreq; ++j) {
		  freq=freqs[j];
		  configureSineWave();
      setCswitchTx(CSw_states[j]);
     
      
		  for (int i=A; i<B; ++i) {
			  // Play the note for a quarter of a second.
			  setGain2(i*Delta);
			  // sprintf(txString,",g: %d",gainDAT ); 
			  digitalWrite(PIN_MUTE , HIGH);  // unmute
			  run2();
			  trigger();
			  delay(5000);
			  // Pause for a tenth of a second between notes.
			  stop_trigger();
			  digitalWrite(PIN_MUTE , LOW);  // mute
			  delay(200);
		  }
		delay(1000);
    }
    Send_tx_String(txString);
    stopMicro();
	  strcpy(txString,"End of sweep");
	  Send_tx_String(txString) ;
}


void GainSweep2(int start,int stp,int Delta){
  // Same as GainSweep but don't go over different frequencies!

    if(stp>MAXGAIN){
      sprintf(txString,"Stop gain is larger then maximum gain: %04X",MAXGAIN );
      Serial.print("STOP");
      Serial.println(stp);
      Serial.print("Maxgain");
      Serial.println(MAXGAIN);
      Send_tx_String(txString) ;
      return;
      }
    
	  int A=start/Delta;
	  int B=stp/Delta;
	  
	  delay(500);
	  strcpy(txString,"Starting gain sweep V2");
	  Send_tx_String(txString) ;
	  startMicro2();   // start recording of ADC data for given duration in seconds
	  delay(2000);      
		  for (int i=A; i<B; ++i) {
			  // Play the note for a quarter of a second.
			  setGain2(i*Delta);
			  // sprintf(txString,",g: %d",gainDAT ); 
			  digitalWrite(PIN_MUTE , HIGH);  // unmute
			  run2();
			  trigger();
			  delay(1000);
			  // Pause for a tenth of a second between notes.
			  stop_trigger();
			  digitalWrite(PIN_MUTE , LOW);  // mute
			  delay(100);
		  }
		delay(1000);
    Send_tx_String(txString);
    stopMicro();
	  strcpy(txString,"End of sweep");
	  Send_tx_String(txString) ;
}

void FsubSweep(int Df,int Delta){
	  delay(200);
	  strcpy(txString,"Starting f sweep around res. freqs.");
	  Send_tx_String(txString) ;
	  startMicro2();   // start recording of ADC data for given duration in seconds
	  delay(1000);

	  for (int j=0; j<Nfreq; ++j) {
		  int mainfreq=freqs[j];
		  int start=mainfreq-Df;
      int stop=mainfreq+Df;
      int A=start/Delta;
	    int B=stop/Delta;
      Serial.print("Main freq:");
      Serial.println(mainfreq);
    
      for (int i=A; i<B+1; ++i) {
        freq=i*Delta;
        configureSineWave();
        setCswitchTx(CSw_states[j]);
        sprintf(subString,"f: %d",freq );
        // strcat(txString,subString);
        Serial.print(subString);
        digitalWrite(PIN_MUTE , HIGH);  // unmute
        run2();
        trigger();
        delay(500);
        stop_trigger();
        digitalWrite(PIN_MUTE , LOW);  // mute
        delay(50);
        Serial.print('.');
		}
    delay(300);
    }
    Send_tx_String(txString);
    stopMicro();
	  strcpy(txString,"End of sweep");
	  Send_tx_String(txString) ;
}

