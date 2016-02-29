

void setup_driver() {


  CONTROLLER_PORT.flush();
  //stop_motors();
  got_parameters=false;
  ask_parameters=false;
  asks=1;
  encoders_ok=true;

  //gotp_t=millis();
  enc_ok_t=millis();
  got_first_enc_read=false;
}

void send_parameters() {
  float pid_constatns[5];
  if (!nh.getParam("pid_constants", pid_constatns, 5)) {
    nh.logwarn("No PID parameters found, using defaults.");
  }
  else {
    nh.loginfo("PID parameters found, sending to controller...");
    cmdMessenger.sendCmdStart(kSetParameters);
    cmdMessenger.sendCmdArg(pid_constatns[0],5);
    cmdMessenger.sendCmdArg(pid_constatns[1],5);
    cmdMessenger.sendCmdArg(pid_constatns[2],5);
    cmdMessenger.sendCmdArg(pid_constatns[3],5);
    cmdMessenger.sendCmdArg((int)pid_constatns[4]);
    cmdMessenger.sendCmdEnd();

  }
}

void OnGetParametersAck() {
  got_parameters=true;
  nh.loginfo("Got parameters from controller:");
  asks=1;
  float kp = cmdMessenger.readFloatArg();
  float ki = cmdMessenger.readFloatArg();
  float kd = cmdMessenger.readFloatArg();
  float alpha = cmdMessenger.readFloatArg();
  int cdt = cmdMessenger.readInt32Arg();

  char log_msg[30];
  char log_msg1[10];

  dtostrf(kp,3,4,log_msg1);
  sprintf(log_msg, "    kp = %s", log_msg1);
  nh.loginfo(log_msg);

  dtostrf(ki,3,4,log_msg1);
  sprintf(log_msg, "    ki = %s", log_msg1);
  nh.loginfo(log_msg);

  dtostrf(kd,3,4,log_msg1);
  sprintf(log_msg, "    kd = %s", log_msg1);
  nh.loginfo(log_msg);

  dtostrf(alpha,3,4,log_msg1);
  sprintf(log_msg, "    alpha = %s", log_msg1);
  nh.loginfo(log_msg);

  sprintf(log_msg, "    Control loop dt = %d", cdt);
  nh.loginfo(log_msg);
  nh.loginfo("Controller ready");
}


void OnEncoders() {
  left_enc = cmdMessenger.readInt32Arg();
  right_enc = cmdMessenger.readInt32Arg();
  enc_ok_t=millis();
  if ((!encoders_ok)||(!got_first_enc_read)){
    nh.loginfo("Communication with controller is OK");
    got_parameters=false;
    send_parameters();
    ask_parameters=true;
    gotp_t=millis();
    encoders_ok=true;
    got_first_enc_read=true;
  }
}

void OnRx() {
  RX1 = cmdMessenger.readInt32Arg();
  RX2 = cmdMessenger.readInt32Arg();
  RX3 = cmdMessenger.readInt32Arg();
  RX4 = cmdMessenger.readInt32Arg();
  RX5 = cmdMessenger.readInt32Arg();
  RX6 = cmdMessenger.readInt32Arg();

  rc_msg.RX1 = RX1;
  rc_msg.RX2 = RX2;
  rc_msg.RX3 = RX3;
  rc_msg.RX4 = RX4;
  rc_msg.RX5 = RX5;
  rc_msg.RX6 = RX6;
  p_rc.publish(&rc_msg);

}

void check_encoders() {
  if (millis()-enc_ok_t>2000) {
    if (encoders_ok){
      nh.logwarn("No communication with controller");
      encoders_ok=false;
    }
  } 
}

void OnStatus() {
  RxStatus = cmdMessenger.readBoolArg();
  controller_bat_v = cmdMessenger.readFloatArg();
}


void pub_status() {

  int gps_fault_bit=0;
#ifdef USE_GPS
  if (gps.location.age()>GPS_IS_OLD) gps_fault_bit=1;
#endif

  status_msg.faults = 8 * (int)imu_fault + 4 * gps_fault_bit + 2*(int)(!encoders_ok) + 1*(int)(RxStatus);
  status_msg.sensors_battery_voltage = (float)analogRead(BATTERY_MONITOR_PIN) * 3.3 / 65535 * VOLTAGE_DIVIDER_RATIO;
  status_msg.rover_battery_voltage = controller_bat_v;
  p_status.publish(&status_msg);

}



void reset_encCb(const Empty::Request & req, Empty::Response & res) {
  cmdMessenger.sendCmd (kReset, true);

  nh.loginfo("Reset encoders");


}

void commandCb( const ric_robot::ric_command& msg) {

  cmdMessenger.sendCmdStart(kCommand);
  cmdMessenger.sendCmdArg(msg.left_wheel);
  cmdMessenger.sendCmdArg(msg.right_wheel);
  cmdMessenger.sendCmdEnd();

}

void stop_motors() {

  cmdMessenger.sendCmdStart(kCommand);
  cmdMessenger.sendCmdArg(0.0);
  cmdMessenger.sendCmdArg(0.0);
  cmdMessenger.sendCmdEnd();
}







