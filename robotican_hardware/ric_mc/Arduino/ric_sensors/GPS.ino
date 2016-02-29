#ifdef USE_GPS
void read_gps() {
  while (GPS_SERIAL_PORT.available() > 0) {
    if (gps.encode(GPS_SERIAL_PORT.read())) {
      if ((gps.location.isValid())&&(gps.location.isUpdated()))
      {
        //  digitalWrite(13, !digitalRead(13));
        gps_msg.Lat = gps.location.lat();
        gps_msg.Lon = gps.location.lng();
        gps_msg.Alt = gps.altitude.meters();
        gps_msg.HDOP = gps.hdop.value();
        gps_msg.Sats = gps.satellites.value();
        gps_msg.Status = gps.location.isValid();
        p_gps.publish(&gps_msg);
      }
    }
  }
}
#endif

