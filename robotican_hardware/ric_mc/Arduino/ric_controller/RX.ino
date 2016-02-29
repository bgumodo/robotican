void ReadRxCommands() {
  // read data into variables
  cli(); // disable interrupts
  RX1=RX[0];
  RX2=RX[1];
  RX3=RX[2];
  RX4=RX[3];
  RX5=RX[4];
  RX6=RX[5];
  sei(); // enable interrupts
  if (!rc_calib_ok) {
    if (!first_rc) {
      CENTER_RX1=RX1;
      CENTER_RX2=RX2;
      first_rc=true; 
    }

    if (RX1>MAX_RX1) MAX_RX1=RX1;
    else if (RX1<MIN_RX1) MIN_RX1=RX1;
    if (RX2>MAX_RX2) MAX_RX2=RX2;
    else if (RX2<MIN_RX2) MIN_RX2=RX2;
  }

  if ((RX1<CENTER_RX1+RX_DEAD_BAND)&&(RX1>CENTER_RX1-RX_DEAD_BAND)) RX1=CENTER_RX1;
  else if (RX1>CENTER_RX1) RX1=RX1-RX_DEAD_BAND;
  else if (RX1<CENTER_RX1) RX1=RX1+RX_DEAD_BAND;

  if ((RX2<CENTER_RX2+RX_DEAD_BAND)&&(RX2>CENTER_RX2-RX_DEAD_BAND)) RX2=CENTER_RX2;
  else if (RX2>CENTER_RX2) RX2=RX2-RX_DEAD_BAND;
  else if (RX2<CENTER_RX2) RX2=RX2+RX_DEAD_BAND;
}

boolean isRxConnected() {
  return !failsafeEnabled;

}

void CommandsFromRX() {
  drive_command=0;
  turn_command=0;

  if (RX2>CENTER_RX2) drive_command=map(RX2,CENTER_RX2,MAX_RX2-RX_DEAD_BAND,0,MAX_RC_COMMAND);
  else if (RX2<CENTER_RX2) drive_command=map(RX2,MIN_RX2+RX_DEAD_BAND,CENTER_RX2,-MAX_RC_COMMAND,0);
  if (RX1>CENTER_RX1) turn_command=map(RX1,CENTER_RX1,MAX_RX1-RX_DEAD_BAND,0,MAX_RC_COMMAND);
  else if (RX1<CENTER_RX1) turn_command=map(RX1,MIN_RX1+RX_DEAD_BAND,CENTER_RX1,-MAX_RC_COMMAND,0);

  if (turn_command>127) turn_command=127;
  else if (turn_command<-127) turn_command=-127;

  if (drive_command>127) drive_command=127;
  else if (drive_command<-127) drive_command=-127;


  if (RX5>1500) {   
    DriveMode=RX_ARM_MODE;
  }
  else {
    DriveMode=RX_DRIVE_MODE;
  }

}



void pub_rx() {
  cmdMessenger.sendCmdStart(kRx);
  cmdMessenger.sendCmdArg(RX1);
  cmdMessenger.sendCmdArg(RX2);
  cmdMessenger.sendCmdArg(RX3);
  cmdMessenger.sendCmdArg(RX4);
  cmdMessenger.sendCmdArg(RX5);
  cmdMessenger.sendCmdArg(RX6);
  cmdMessenger.sendCmdEnd();
}


void EEPROMWriteInt(int p_address, int p_value)
{
  byte lowByte = ((p_value >> 0) & 0xFF);
  byte highByte = ((p_value >> 8) & 0xFF);

  EEPROM.write(p_address, lowByte);
  EEPROM.write(p_address + 1, highByte);
}

unsigned int EEPROMReadInt(int p_address)
{
  byte lowByte = EEPROM.read(p_address);
  byte highByte = EEPROM.read(p_address + 1);

  return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
}



void start_rc_calib() {
  loginfo("Starting RC calibration...");
  CENTER_RX1_cal=1500;
  MIN_RX1_cal=2000;
  MAX_RX1_cal=1000;

  CENTER_RX2_cal=1500;
  MIN_RX2_cal=2000;
  MAX_RX2_cal=1000;
  do_rc_calib=true;

stop_motors( );

}

void rc_calib() {
  if (RX1>MAX_RX1_cal) {
    MAX_RX1_cal=RX1;
    print_rc_calib(1);
  }
  else if (RX1<MIN_RX1_cal){
    MIN_RX1_cal=RX1;
    print_rc_calib(1);
  }

  if (RX2>MAX_RX2_cal) {
    MAX_RX2_cal=RX2;
    print_rc_calib(2);
  }
  else if (RX2<MIN_RX2_cal){
    MIN_RX2_cal=RX2;
    print_rc_calib(2);
  }
}

void save_rc_calib() {
  CENTER_RX1_cal=RX1;
  CENTER_RX2_cal=RX2;
  do_rc_calib=false;
  //print_rc_calib();
  EEPROMWriteInt(rc_calib_addr,MIN_RX1_cal);
  EEPROMWriteInt(rc_calib_addr+2,CENTER_RX1_cal);
  EEPROMWriteInt(rc_calib_addr+4,MAX_RX1_cal);
  EEPROMWriteInt(rc_calib_addr+6,MIN_RX2_cal);
  EEPROMWriteInt(rc_calib_addr+8,CENTER_RX2_cal);
  EEPROMWriteInt(rc_calib_addr+10,MAX_RX2_cal);
  loginfo("RC calibration saved to EEPROM");
  load_rc_calib();
}

void cancel_rc_calib() {
  loginfo("RC calibration canceled");
  do_rc_calib=false;
}
void load_rc_calib() {

  int MIN_RX1_t=EEPROMReadInt(rc_calib_addr);
  int CENTER_RX1_t=EEPROMReadInt(rc_calib_addr+2);
  int MAX_RX1_t=EEPROMReadInt(rc_calib_addr+4);
  int MIN_RX2_t=EEPROMReadInt(rc_calib_addr+6);
  int CENTER_RX2_t=EEPROMReadInt(rc_calib_addr+8);
  int MAX_RX2_t=EEPROMReadInt(rc_calib_addr+10);

  if ((MIN_RX1_t<MAX_RX1_t)&&(MIN_RX1_t<CENTER_RX1_t)&&(MIN_RX1_t<1500)&&(MIN_RX1_t>500)&&(MAX_RX1_t>MIN_RX1_t)&&(MAX_RX1_t>CENTER_RX1_t)&&(MAX_RX1_t>1500)&&(MAX_RX1_t<2500)&&(CENTER_RX1_t>1000)&&(CENTER_RX1_t<2000) && 
    (MIN_RX2_t<MAX_RX2_t)&&(MIN_RX2_t<CENTER_RX2_t)&&(MIN_RX2_t<1500)&&(MIN_RX2_t>500)&&(MAX_RX2_t>MIN_RX2_t)&&(MAX_RX2_t>CENTER_RX2_t)&&(MAX_RX2_t>1500)&&(MAX_RX2_t<2500)&&(CENTER_RX2_t>1000)&&(CENTER_RX2_t<2000)) {

    loginfo("RC calibration loaded from EEPROM");

    MIN_RX1=MIN_RX1_t;
    CENTER_RX1=CENTER_RX1_t;
    MAX_RX1=MAX_RX1_t;

    MIN_RX2=MIN_RX2_t;
    CENTER_RX2=CENTER_RX2_t;
    MAX_RX2=MAX_RX2_t;

    rc_calib_ok=true;
    first_rc=true;
  }
  else   {
    logwarn("Could not load RC calibration from EEPROM");
    rc_calib_ok=false;
  }
  //print_rc_calib(0);

}


void print_rc_calib(int in_cal) {
  if (in_cal==1) {
    cmdMessenger.sendCmdStart(kRcCalib);
    cmdMessenger.sendCmdArg(1);
    cmdMessenger.sendCmdArg(MIN_RX1_cal);
    cmdMessenger.sendCmdArg(CENTER_RX1_cal);
    cmdMessenger.sendCmdArg(MAX_RX1_cal);
    cmdMessenger.sendCmdEnd();
  }
  else if (in_cal==2) {
    cmdMessenger.sendCmdStart(kRcCalib);
    cmdMessenger.sendCmdArg(2);
    cmdMessenger.sendCmdArg(MIN_RX2_cal);
    cmdMessenger.sendCmdArg(CENTER_RX2_cal);
    cmdMessenger.sendCmdArg(MAX_RX2_cal);
    cmdMessenger.sendCmdEnd();
  }
  else {
    cmdMessenger.sendCmdStart(kRcCalib);
    cmdMessenger.sendCmdArg(1);
    cmdMessenger.sendCmdArg(MIN_RX1);
    cmdMessenger.sendCmdArg(CENTER_RX1);
    cmdMessenger.sendCmdArg(MAX_RX1);
    cmdMessenger.sendCmdEnd();

    cmdMessenger.sendCmdStart(kRcCalib);
    cmdMessenger.sendCmdArg(2);        
    cmdMessenger.sendCmdArg(MIN_RX2);
    cmdMessenger.sendCmdArg(CENTER_RX2);
    cmdMessenger.sendCmdArg(MAX_RX2);
    cmdMessenger.sendCmdEnd();
  }
}

void OnRcCalib() {
  int cal = cmdMessenger.readInt32Arg();
  if (cal==1)  start_rc_calib(); 
  else if (cal==2) save_rc_calib();
  else if (cal==3) cancel_rc_calib();
  else if (cal==4) print_rc_calib(0);
}








