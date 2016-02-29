#ifdef HAVE_ELEVATOR
int pre_lh=HIGH;
unsigned long lh_t=0;
boolean lh_stat=false;

int pre_uh=HIGH;
unsigned long uh_t=0;
boolean uh_stat=false;



void check_lower_home() {
  int lh=digitalRead(HOME_DOWN_PIN);
  if ((lh==LOW)&&(pre_lh==HIGH)) {
    lh_t=millis();
  }
  else if((lh==LOW)&&(pre_lh==LOW)&&(!lh_stat)&&(millis()-lh_t>500)) {
    nh.loginfo("Lower home switch activated");
    set_elevator::Request req;
    set_elevator::Response res;
    req.pos = -0.01;
    elev_set_client.call(req, res);
    lh_stat=true;
  }
  else if((lh==HIGH)&&(pre_lh==LOW)&&(lh_stat)) {
    lh_stat=false;
    //nh.loginfo("Lower home switch released");
  }

  pre_lh=lh;
}

void check_upper_home() {
  int uh=digitalRead(HOME_UP_PIN);
  if ((uh==LOW)&&(pre_uh==HIGH)) {
    uh_t=millis();

  }
  else if((uh==LOW)&&(pre_uh==LOW)&&(!uh_stat)&&(millis()-uh_t>500)) {
    nh.loginfo("Upper home switch activated");
    set_elevator::Request req;
    set_elevator::Response res;
    req.pos = 0.407;
    elev_set_client.call(req, res);
    uh_stat=true;  
  }
  else if((uh==HIGH)&&(pre_uh==LOW)&&(uh_stat)) {
    uh_stat=false;
    //nh.loginfo("Upper home switch released");
  }

  pre_uh=uh;
}


void setup_homing() {

  nh.loginfo("Seting up elevator home switches...");
  pinMode(HOME_UP_PIN, INPUT_PULLUP); 
  pinMode(HOME_DOWN_PIN, INPUT_PULLUP); 

  pre_lh=HIGH;
  lh_t=millis();
  lh_stat=false;

  pre_uh=HIGH;
  uh_t=millis();

int uh=digitalRead(HOME_UP_PIN);
int lh=digitalRead(HOME_DOWN_PIN);
if (uh==LOW) {
  nh.loginfo("Upper home switch activated");
    set_elevator::Request req;
    set_elevator::Response res;
    req.pos = 0.407;
    elev_set_client.call(req, res);
    uh_stat=true;  
}
else if (lh==LOW) {
    nh.loginfo("Lower home switch activated");
    set_elevator::Request req;
    set_elevator::Response res;
    req.pos = -0.01;
    elev_set_client.call(req, res);
    lh_stat=true;
}

}

#endif

