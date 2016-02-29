void  setup_imu() {
  Wire.begin();
  MPU.selectDevice(DEVICE_TO_USE);                       
  imu_fault = !MPU.init(MPU_UPDATE_RATE, MPU_MAG_MIX_GYRO_AND_MAG, MAG_UPDATE_RATE, MPU_LPF_RATE); // start the MPU
  if (imu_fault == false)  {
    nh.loginfo("IMU ready");
    imu_t=millis();
    if (calLibRead(DEVICE_TO_USE, &calData))
    {
      char log_msg[100];
      if (calData.magValid==true) {
        nh.loginfo("    Mag calibration data loaded:"); 
        //sprintf(log_msg, "       minX=%d minY=%d minZ=%d   maxX=%d maxY=%d maxZ=%d",calData.magMinX,calData.magMinY,calData.magMinZ,calData.magMaxX,calData.magMaxY,calData.magMaxZ );
        //nh.loginfo(log_msg);
      }
      if (calData.accelValid==true) {
        nh.loginfo("    Accel calibration data loaded:"); 
        //sprintf(log_msg, "       minX=%d minY=%d minZ=%d   maxX=%d maxY=%d maxZ=%d",calData.accelMinX,calData.accelMinY,calData.accelMinZ,calData.accelMaxX,calData.accelMaxY,calData.accelMaxZ );
        //nh.loginfo(log_msg);

      }

    }
    else {
      nh.logwarn("    No IMU calibration data found");
    }
  }
  else   nh.logerror("IMU fault");
  CHECK_IMU_INTERVAL = (unsigned long)(1000.0 / MPU_UPDATE_RATE * 100.0);

  loopState = LOOPSTATE_NORMAL;
  // pollInterval = (1000 / MPU_UPDATE_RATE) - 1; // a bit less than the minimum interval
  // lastPollTime = millis();
}




void read_imu() {

  switch (loopState) {
  case LOOPSTATE_NORMAL:

    if (MPU.read()) { // get the latest data if ready yet
      qw = MPU.m_fusedQuaternion[0];
      qx = MPU.m_fusedQuaternion[1];
      qy = MPU.m_fusedQuaternion[2];
      qz = MPU.m_fusedQuaternion[3];
      ax=MPU.m_calAccel[0];
      ay=MPU.m_calAccel[1];
      az=MPU.m_calAccel[2];
      mx=MPU.m_calMag[0];
      my=MPU.m_calMag[1];
      mz=MPU.m_calMag[2];
      gx=MPU.m_rawGyro[0];
      gy=MPU.m_rawGyro[1];
      gz=MPU.m_rawGyro[2];

      imu_t = millis();
      imu_fault=false;
    }
    break;

  case LOOPSTATE_MAGCAL:
    magCalLoop();
    imu_t = millis();
    break;

  case LOOPSTATE_ACCELCAL:
    accelCalLoop();
    imu_t = millis();
    break;
  }




}


void ric_calibCb(const ric_calib::Request & req, ric_calib::Response & res) {


  switch (loopState) {
  case LOOPSTATE_NORMAL:
    switch (req.com) {
    case 1:
      magCalStart();
      res.ack=true;
      return;

    case 2:
      accelCalStart();
      res.ack=true;      
      return;
    case 5:
      calLibErase(DEVICE_TO_USE);
      nh.loginfo("Calibration data erased");
      res.ack=true;
      return;
    case 11:
      cmdMessenger.sendCmdStart(kRcCalib);
      cmdMessenger.sendCmdArg(1);
      cmdMessenger.sendCmdEnd();
      //  nh.loginfo("sent command to start rc calib");
      res.ack=true;
      return;
    case 12:
      cmdMessenger.sendCmdStart(kRcCalib);
      cmdMessenger.sendCmdArg(2);
      cmdMessenger.sendCmdEnd();
      //  nh.loginfo("sent command to save rc calib");
      res.ack=true;
      return;
    case 13:
      cmdMessenger.sendCmdStart(kRcCalib);
      cmdMessenger.sendCmdArg(3);
      cmdMessenger.sendCmdEnd();
      //  nh.loginfo("sent command to cancel rc calib");
      res.ack=true;
      return;
    case 14:
      cmdMessenger.sendCmdStart(kRcCalib);
      cmdMessenger.sendCmdArg(4);
      cmdMessenger.sendCmdEnd();
      res.ack=true;
      //  nh.loginfo("sent command to print rc calib");
      return;
    }
    break;

  case LOOPSTATE_MAGCAL:
    switch (req.com)  {
    case 3:
      calData.magValid = true;
      calLibWrite(DEVICE_TO_USE, &calData);
      nh.loginfo("Mag cal data saved");
      break;

    case 4:
      loopState = LOOPSTATE_NORMAL;
      nh.loginfo("*** restart to use calibrated data ***");
      break;
    }
    break;

  case LOOPSTATE_ACCELCAL:
    switch (req.com) {
    case 3:

      calData.accelValid = true;
      calLibWrite(DEVICE_TO_USE, &calData);
      nh.loginfo("Accel cal data saved");
      break;

    case 4:

      loopState = LOOPSTATE_NORMAL;
      nh.loginfo("*** restart to use calibrated data ***");
      break;
    }
    break;
  }
  res.ack=true;
}

void magCalStart(void)
{
  calLibRead(DEVICE_TO_USE, &calData); // pick up existing accel data if there

  calData.magValid = false;
  calData.magMinX = 0x7fff; // init mag cal data
  calData.magMaxX = 0x8000;
  calData.magMinY = 0x7fff;
  calData.magMaxY = 0x8000;
  calData.magMinZ = 0x7fff;
  calData.magMaxZ = 0x8000;

  nh.loginfo("Entering mag calibration mode");
  loopState = LOOPSTATE_MAGCAL;
}

void magCalLoop()
{
  boolean changed;

  if (MPU.read()) { // get the latest data
    changed = false;
    if (MPU.m_rawMag[VEC3_X] < calData.magMinX) {
      calData.magMinX = MPU.m_rawMag[VEC3_X];
      changed = true;
    }
    if (MPU.m_rawMag[VEC3_X] > calData.magMaxX) {
      calData.magMaxX = MPU.m_rawMag[VEC3_X];
      changed = true;
    }
    if (MPU.m_rawMag[VEC3_Y] < calData.magMinY) {
      calData.magMinY = MPU.m_rawMag[VEC3_Y];
      changed = true;
    }
    if (MPU.m_rawMag[VEC3_Y] > calData.magMaxY) {
      calData.magMaxY = MPU.m_rawMag[VEC3_Y];
      changed = true;
    }
    if (MPU.m_rawMag[VEC3_Z] < calData.magMinZ) {
      calData.magMinZ = MPU.m_rawMag[VEC3_Z];
      changed = true;
    }
    if (MPU.m_rawMag[VEC3_Z] > calData.magMaxZ) {
      calData.magMaxZ = MPU.m_rawMag[VEC3_Z];
      changed = true;
    }

    if (changed) {
      nh.loginfo("-------");

      sprintf(temp_msg, "minX:  %d", calData.magMinX); 
      nh.loginfo(temp_msg);
      sprintf(temp_msg, "maxX:  %d", calData.magMaxX); 
      nh.loginfo(temp_msg);
      sprintf(temp_msg, "minY:  %d", calData.magMinY); 
      nh.loginfo(temp_msg);
      sprintf(temp_msg, "maxY:  %d", calData.magMaxY); 
      nh.loginfo(temp_msg);
      sprintf(temp_msg, "minZ:  %d", calData.magMinZ); 
      nh.loginfo(temp_msg);
      sprintf(temp_msg, "maxZ:  %d", calData.magMaxZ); 
      nh.loginfo(temp_msg);


    }
  }


}

void accelCalStart(void)
{
  calLibRead(DEVICE_TO_USE, &calData); // pick up existing accel data if there

  calData.accelValid = false;
  calData.accelMinX = 0x7fff; // init accel cal data
  calData.accelMaxX = 0x8000;
  calData.accelMinY = 0x7fff;
  calData.accelMaxY = 0x8000;
  calData.accelMinZ = 0x7fff;
  calData.accelMaxZ = 0x8000;

  nh.loginfo("Entering accel calibration mode");
  loopState = LOOPSTATE_ACCELCAL;
  MPU.disableAccelCal();
}

void accelCalLoop()
{
  boolean changed;

  if (MPU.read()) { // get the latest data
    changed = false;
    if (MPU.m_rawAccel[VEC3_X] < calData.accelMinX) {
      calData.accelMinX = MPU.m_rawAccel[VEC3_X];
      changed = true;
    }
    if (MPU.m_rawAccel[VEC3_X] > calData.accelMaxX) {
      calData.accelMaxX = MPU.m_rawAccel[VEC3_X];
      changed = true;
    }
    if (MPU.m_rawAccel[VEC3_Y] < calData.accelMinY) {
      calData.accelMinY = MPU.m_rawAccel[VEC3_Y];
      changed = true;
    }
    if (MPU.m_rawAccel[VEC3_Y] > calData.accelMaxY) {
      calData.accelMaxY = MPU.m_rawAccel[VEC3_Y];
      changed = true;
    }
    if (MPU.m_rawAccel[VEC3_Z] < calData.accelMinZ) {
      calData.accelMinZ = MPU.m_rawAccel[VEC3_Z];
      changed = true;
    }
    if (MPU.m_rawAccel[VEC3_Z] > calData.accelMaxZ) {
      calData.accelMaxZ = MPU.m_rawAccel[VEC3_Z];
      changed = true;
    }

    if (changed) {
      nh.loginfo("-------");

      sprintf(temp_msg, "minX:  %d", calData.accelMinX); 
      nh.loginfo(temp_msg);
      sprintf(temp_msg, "maxX:  %d", calData.accelMaxX); 
      nh.loginfo(temp_msg);
      sprintf(temp_msg, "minY:  %d", calData.accelMinY); 
      nh.loginfo(temp_msg);
      sprintf(temp_msg, "maxY:  %d", calData.accelMaxY); 
      nh.loginfo(temp_msg);
      sprintf(temp_msg, "minZ:  %d", calData.accelMinZ); 
      nh.loginfo(temp_msg);
      sprintf(temp_msg, "maxZ:  %d", calData.accelMaxZ); 
      nh.loginfo(temp_msg);

    }
  }


}






