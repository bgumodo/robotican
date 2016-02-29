
//PINS

//PWM_PINS[RX_CHANNELS] = {2,3,4,5,6,7};

#define LED_PIN 13

#define BATTERY_MONITOR_PIN A0

#define ENC1_A_PIN 15 //right motor
#define ENC1_B_PIN 16 //right motor
#define ENC2_A_PIN 17 //left motor
#define ENC2_B_PIN 18 //left motor
/*
RIGHT MOTOR RED->M1A
 RIGHT MOTOR BLACK->M1B
 RIGHT ENCODER A -> ENC1_A_PIN = 15
 RIGHT ENCODER B -> ENC1_B_PIN = 16
 
 LEFT MOTOR RED->M2A
 LEFT MOTOR BLACK->M2B
 LEFT ENCODER A -> ENC2_A_PIN = 17
 LEFT ENCODER B -> ENC2_B_PIN = 18
 */

//RX
//#include <SabertoothSimplified.h>
#include <Sabertooth.h>
Sabertooth ST(128);
//SabertoothSimplified ST;
#define MAX_COMMAND 127
#define MAX_RC_COMMAND 45
#define PUB_RX_INTERVAL 100
#include <Arduino.h>
#include <TeensyReceiver.h>

#define RX_DRIVE_MODE 1
#define RX_ARM_MODE 2
short DriveMode=1;
volatile uint16_t RX1=1500,RX2=1500,RX3=1500,RX4=0,RX5=1500,RX6=1500;
unsigned long rx_t=0;

#include <EEPROM.h>
int rc_calib_addr = 130;
boolean rc_calib_ok=false;
boolean do_rc_calib=false;

#define RX_DEAD_BAND 25

int MIN_RX1= 1000;
int MAX_RX1 =2000;
int CENTER_RX1 =1500;

int MIN_RX1_cal= 1000;
int MAX_RX1_cal =2000;
int CENTER_RX1_cal =1500;

int MIN_RX2= 1000;
int MAX_RX2 =2000;
int CENTER_RX2 =1500;

int MIN_RX2_cal= 1000;
int MAX_RX2_cal =2000;
int CENTER_RX2_cal =1500;
boolean first_rc=false;

#include <CmdMessenger.h>  // CmdMessenger
// In order to receive, attach a callback function to these events
enum
{
  // Commands
  kCommand,
  kStatus,
  kEncoders,
  kReset, 
  kSetParameters,
  kGetParameters,
  kGetParametersAck,
  kRx,
  kRcCalib,
  kloginfo,
  klogwarn,
};



#define  SERIAL_PORT_SPEED  115200
#define  SERIAL_PORT Serial2 //Serial2

#define PUB_ENC_INTERVAL 50 //20 hz
unsigned long enc_t = 0, status_t = 0, pub_t;


CmdMessenger cmdMessenger = CmdMessenger(SERIAL_PORT);



//BATTERY MONITOR
#define STATUS_INTERVAL 1000 //1 hz  
#define VOLTAGE_DIVIDER_RATIO 11.104 //Vbat ----/\/\R1=47K/\/\----A0----/\/\R2=4.7K/\/\----AGND


//DRIVER
#include <Encoder.h>
#include <PID_v1.h>
//#include <EEPROM.h>

long left_enc = 0, right_enc = 0;
long pre_left_enc = 0, pre_right_enc = 0;
float left_spd = 0, right_spd = 0;
int drive_command=0,turn_command=0;
double Setpoint1, Input1, Output1;
double Setpoint2, Input2, Output2;
#define READ_ENCODERS_INTERVAL 20  //20ms=50 hz
int  CONTROL_INTERVAL = 1; //ms
double DT;
#define  WATCHDOG_INTERVAL 1500 //ms
#define  MAX_TICKS_PER_S 10000 //tics/sec ~ 4 m/s (13cm radius wheel)
boolean wd_on =true;
float alpha = 0.05;
float kp = 0.007, ki = 0.1, kd = 0.0;
PID PID1(&Input1, &Output1, &Setpoint1, kp, ki, kd, DIRECT);
PID PID2(&Input2, &Output2, &Setpoint2, kp, ki, kd, DIRECT);

Encoder Enc1(ENC1_A_PIN, ENC1_B_PIN);
Encoder Enc2(ENC2_A_PIN, ENC2_B_PIN);
unsigned long wd_t = 0, control_t = 0,led_t=0;




void setup()
{
 // delay(4000);
  pinMode(LED_PIN, OUTPUT);

  Serial.begin(57600);

  SERIAL_PORT.begin(SERIAL_PORT_SPEED);

load_rc_calib();
  initializeReceiver();


  // Adds newline to every command
  cmdMessenger.printLfCr(); 

  // Attach my application's user-defined callback methods
  attachCommandCallbacks();

  setup_driver();

  analogReadResolution(16);
  // Setpoint1=2048;
}

void blink_led(int led_interval) {
  if (millis()-led_t>led_interval) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    led_t=millis();
  } 

}

void loginfo(String msg) {
 
   cmdMessenger.sendCmdStart(kloginfo);
cmdMessenger.sendCmdArg(msg);
  cmdMessenger.sendCmdEnd();
}

void logwarn(String msg) {
   cmdMessenger.sendCmdStart(klogwarn);
cmdMessenger.sendCmdArg(msg);
  cmdMessenger.sendCmdEnd();
}

void attachCommandCallbacks()
{
  // Attach callback methods
  //  cmdMessenger.attach(OnUnknownCommand);
  cmdMessenger.attach(kCommand, OnCommand);
  cmdMessenger.attach(kSetParameters, OnSetParameters);
  cmdMessenger.attach(kGetParameters, OnGetParameters);
  cmdMessenger.attach(kReset, OnReset);
  cmdMessenger.attach(kRcCalib, OnRcCalib);
}



void loop()
{

if (do_rc_calib) rc_calib();

  cmdMessenger.feedinSerialData();
  control_loop();
  
  /*
  wd_on =false;
  if (Serial.available()) {
    int inByte = Serial.read();
    //Serial.println(inByte);
    if ((inByte-48>=0)&&(inByte-48<=9)) {
      Setpoint1 = 512*(inByte-48);
      Setpoint2 = 512*(inByte-48);
    }
    else if (inByte=='a') {
      Setpoint1 += 512;
      Setpoint2 += 512;
    }
    else if (inByte=='z') {
      Setpoint1 -= 512;
      Setpoint2 -= 512;
    }
  }
  */
  
  if (millis() - wd_t >= WATCHDOG_INTERVAL)  {
   if (!wd_on) {
   stop_motors();
   wd_on = true;
   }
   blink_led(1500);
   }
   

  if ((isRxConnected())&&(DriveMode==RX_ARM_MODE)&&(millis() - rx_t >= PUB_RX_INTERVAL))  {
    pub_rx();
    rx_t = millis();

  }

  if (millis() - status_t >= STATUS_INTERVAL)  {
    pub_status();
    status_t = millis();




  }


  if (millis() - enc_t >= READ_ENCODERS_INTERVAL)  {
    read_encoders();
    enc_t = millis();
    //Serial.println(isRxConnected());
  }

  if (millis() - pub_t >= PUB_ENC_INTERVAL)  {
    pub_enc();
    pub_t = millis();
     
    /*
    Serial.print("Inputs: ");
     Serial.print((int)Input1);
     Serial.print("   ");
     Serial.print((int)Input2);
     
     Serial.print("    Setpoints: ");
     Serial.print((int)Setpoint1);
     Serial.print("   ");
     Serial.print((int)Setpoint2);
     
     Serial.print("    Errors: ");
     Serial.print((int)Setpoint1-Input1);
     Serial.print("   ");
     Serial.print((int)Setpoint2-Input2);
     
     Serial.print("    Outputs: ");
     Serial.print(-(int)Output1);
     Serial.print("   ");
     Serial.println(-(int)Output2);
     
     */
     /*
     Serial.print("OK: ");
    Serial.print(isRxConnected());
    Serial.print("      CH1: ");
    Serial.print(RX1);
    Serial.print("   CH2: ");
    Serial.print(RX2);
    Serial.print("   CH3: ");
    Serial.print(RX3);
    Serial.print("   CH4: ");
    Serial.print(RX4);
    Serial.print("   CH5: ");
    Serial.print(RX5);
    Serial.print("   CH6: ");
    Serial.print(RX6);
    
     Serial.print("   RX_signalReceived: ");
    Serial.print(RX_signalReceived);
         Serial.print("   oks: ");
    Serial.print(oks);
         Serial.print("   RX_failsafeStatus: ");
    Serial.println(RX_failsafeStatus);
*/

  }




}










