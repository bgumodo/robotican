

void setup_urf() {
/*
  pinMode(URF_TX_PIN, OUTPUT);
  digitalWrite(URF_TX_PIN, HIGH);
  delayMicroseconds(25);
  digitalWrite(URF_TX_PIN, LOW);
  pinMode(URF_TX_PIN, INPUT);
*/
}


void read_urf() {
  Left_URF_Median.addValue(analogRead(LEFT_URF_PIN));
  Rear_URF_Median.addValue(analogRead(REAR_URF_PIN));
  Right_URF_Median.addValue(analogRead(RIGHT_URF_PIN));

}
