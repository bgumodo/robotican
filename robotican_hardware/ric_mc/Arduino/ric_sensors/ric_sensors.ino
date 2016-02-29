
#define USE_GPS

//uncomment this if you have an elevator
//#define HAVE_ELEVATOR

//uncomment this if you have an elevator
//#define HAVE_PAN_TILT

//SUBSCRIBERS
#define COMMAND_TOPIC "command"
#define PAN_TILT_TOPIC "pan_tilt"
//PUBLISHER
#define RAW_TOPIC "ric_raw"
#define GPS_TOPIC "raw_gps"
#define STATUS_TOPIC "status"
#define RC_TOPIC "RC"
//SERVICES
#define  RESET_ENCODERS_SRV "reset_encoders"
#define RIC_CALIB_SRV "ric_calib"
#define RESTART_ALL_SRV "restart"
#define ELEV_SET_SRV "elevator_controller/set_position"
#define RELAYS_SRV "relays"


//PINS

#define HOME_UP_PIN 5
#define HOME_DOWN_PIN 6

#define LED_PIN 13

#define BATTERY_MONITOR_PIN A0 //14

#define PAN_SERVO_PIN 22
#define TILT_SERVO_PIN 23

#define RELAY1_PIN 2
#define RELAY2_PIN 3

#define LEFT_URF_PIN A1 //15
#define REAR_URF_PIN A2 //16
#define RIGHT_URF_PIN A3 //17
#define URF_TX_PIN 4

/*

 GPS PIN 1 -> GND
 GPS PIN 2 -> VCC
 GPS PIN 3 -> RX3 = 7
 GPS PIN 4 -> TX3 = 8
 
 */

//CONTROLLER
#define  CONTROLLER_PORT_SPEED  115200
#define CONTROLLER_PORT Serial2
#include <CmdMessenger.h>  // CmdMessenger
#define ASK_PARAMETERS_INTERVAL 5000 //ms
#define ASK_PARAMETERS_ATTEMPTS 10 

int RX1=0,RX2=0,RX3=0,RX4=0,RX5=0,RX6=0;

CmdMessenger cmdMessenger = CmdMessenger(CONTROLLER_PORT);
int asks=1;
boolean RxStatus = 0;
float controller_bat_v = 0;
int left_enc = 0, right_enc = 0;
unsigned long enc_ok_t=0, gotp_t=0;
boolean got_parameters=false, encoders_ok=false, ask_parameters=true,got_first_enc_read=false;

enum
{
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


//ROS

#include <ros.h>
#include <ric_robot/ric_raw.h>
#include <ric_robot/ric_gps.h>
#include <ric_robot/ric_command.h>
#include <ric_robot/ric_pan_tilt.h>
#include <ric_robot/ric_status.h>
#include <ric_robot/ric_rc.h>
#include <ric_robot/relays.h>

#include <ric_robot/ric_calib.h>
#include <std_srvs/Empty.h>

#define  ROS_PORT_SPEED  57600
#define  ROS_PORT Serial
#define PUB_RAW_INTERVAL 50 //20 hz
unsigned long urf_t = 0, enc_t = 0, status_t = 0, pub_t=0, led_t=0;

ros::NodeHandle nh;
using std_srvs::Empty;
using ric_robot::ric_calib;

#include <ric_robot/set_elevator.h>
using ric_robot::set_elevator;
using ric_robot::relays;

//PROTOTYPES
void reset_encCb(const Empty::Request & req, Empty::Response & res);
void ric_calibCb(const ric_calib::Request & req, ric_calib::Response & res);
void relaysCb(const relays::Request & req, relays::Response & res);
void commandCb( const ric_robot::ric_command& msg);
#ifdef HAVE_PAN_TILT
void pantiltCb( const ric_robot::ric_pan_tilt& msg);
#endif
void restart_allCb(const Empty::Request & req, Empty::Response & res);

ros::ServiceServer<ric_calib::Request, ric_calib::Response> ric_calib_server(RIC_CALIB_SRV, &ric_calibCb);
ros::ServiceServer<relays::Request, relays::Response> relays_server(RELAYS_SRV, &relaysCb);
ros::ServiceServer<Empty::Request, Empty::Response> reset_enc_server(RESET_ENCODERS_SRV, &reset_encCb);
ros::ServiceServer<Empty::Request, Empty::Response> restart_all_server(RESTART_ALL_SRV, &restart_allCb);

#ifdef HAVE_ELEVATOR
ros::ServiceClient<set_elevator::Request, set_elevator::Response> elev_set_client(ELEV_SET_SRV);
#endif
ros::Subscriber<ric_robot::ric_command> command_sub(COMMAND_TOPIC, &commandCb );

#ifdef HAVE_PAN_TILT
ros::Subscriber<ric_robot::ric_pan_tilt> pan_tilt_sub(PAN_TILT_TOPIC, &pantiltCb );
#endif

ric_robot::ric_raw raw_msg;
ros::Publisher p_raw(RAW_TOPIC, &raw_msg);

ric_robot::ric_rc rc_msg;
ros::Publisher p_rc(RC_TOPIC, &rc_msg);

#ifdef USE_GPS
ric_robot::ric_gps gps_msg;
ros::Publisher p_gps(GPS_TOPIC, &gps_msg);
#endif

ric_robot::ric_status status_msg;
ros::Publisher p_status(STATUS_TOPIC, &status_msg);



//PAN TILT
#ifdef HAVE_PAN_TILT
#include <Servo.h>
#define MAX_PAN 35
#define MIN_PAN -35
#define MAX_TILT 30
#define MIN_TILT -30
#define PAN_CENTER 94
#define TILT_CENTER 94

Servo pan_servo;
Servo tilt_servo;
unsigned long pan_tilt_t = 0;
bool pan_tilt_moving = true;
#define PAN_TILT_MOVE_TIME 1000
#endif


#ifdef USE_GPS
//GPS
#include <TinyGPS++.h>
TinyGPSPlus gps;
#define  GPS_PORT_SPEED  9600
#define GPS_SERIAL_PORT Serial3
#define GPS_IS_OLD 3000 //ms
#endif

//IMU

#include <Wire.h>
#include "I2Cdev.h"
#include "MPU9150Lib.h"
#include "CalLib.h"
#include <dmpKey.h>
#include <dmpmap.h>
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
#include <EEPROM.h>


float qx = 0, qy = 0, qz = 1, qw = 0;
short ax=0,ay=0,az=0,gx=0,gy=0,gz=0,mx=0,my=0,mz=0;
unsigned long imu_t = 0;
unsigned long CHECK_IMU_INTERVAL;
boolean imu_fault = 0;
//  DEVICE_TO_USE selects whether the IMU at address 0x68 (default) or 0x69 is used
//    0 = use the device at 0x68
//    1 = use the device at ox69
#define  DEVICE_TO_USE    0
MPU9150Lib MPU; // the MPU object
//  MPU_UPDATE_RATE defines the rate (in Hz) at which the MPU updates the sensor data and DMP output
#define MPU_UPDATE_RATE  (10)
//  MAG_UPDATE_RATE defines the rate (in Hz) at which the MPU updates the magnetometer data
//  MAG_UPDATE_RATE should be less than or equal to the MPU_UPDATE_RATE
#define MAG_UPDATE_RATE  (10)
//  MPU_MAG_MIX defines the influence that the magnetometer has on the yaw output.
//  The magnetometer itself is quite noisy so some mixing with the gyro yaw can help
//  significantly. Some example values are defined below:
#define  MPU_MAG_MIX_GYRO_ONLY          0                   // just use gyro yaw
#define  MPU_MAG_MIX_MAG_ONLY           1                   // just use magnetometer and no gyro yaw
#define  MPU_MAG_MIX_GYRO_AND_MAG       10                  // a good mix value 
#define  MPU_MAG_MIX_GYRO_AND_SOME_MAG  50                  // mainly gyros with a bit of mag correction 
//  MPU_LPF_RATE is the low pas filter rate and can be between 5 and 188Hz
#define MPU_LPF_RATE   40

//long lastPollTime; // last time the MPU-9150 was checked
//long pollInterval; // gap between polls to avoid thrashing the I2C bus
char temp_msg[130];

int loopState; // what code to run in the loop

#define LOOPSTATE_NORMAL 0 // normal execution
#define LOOPSTATE_MAGCAL 1 // mag calibration
#define LOOPSTATE_ACCELCAL 2 // accel calibration

static CALLIB_DATA calData; // the calibration data

void magCalStart(void);
void magCalLoop(void);
void accelCalStart(void);
void accelCalLoop(void);




//FAULT AND BATTERY MONITOR
#define STATUS_INTERVAL 200 //5 hz  
#define VOLTAGE_DIVIDER_RATIO 5.779 //Vbat ----/\/\R1=22K/\/\----A0----/\/\R2=4.7K/\/\----AGND



//URF
#define URF_INTERVAL 50 //20 hz
#define sample_size 5
#include "FastRunningMedian.h"
FastRunningMedian<unsigned int, sample_size, 0> Left_URF_Median;
FastRunningMedian<unsigned int, sample_size, 0> Right_URF_Median;
FastRunningMedian<unsigned int, sample_size, 0> Rear_URF_Median;



void setup()
{

  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  digitalWrite(RELAY1_PIN,LOW);
  digitalWrite(RELAY2_PIN,LOW);

  pinMode(LED_PIN, OUTPUT);


  ROS_PORT.begin(ROS_PORT_SPEED);

  CONTROLLER_PORT.begin(CONTROLLER_PORT_SPEED);
  // Adds newline to every command
  cmdMessenger.printLfCr();
  // Attach my application's user-defined callback methods
  attachCommandCallbacks();

  analogReadResolution(16);

  nh.initNode();
  nh.advertise(p_raw);
  nh.advertise(p_rc);
#ifdef USE_GPS
  nh.advertise(p_gps);
#endif
  nh.advertise(p_status);

  nh.advertiseService(reset_enc_server);
  nh.advertiseService(ric_calib_server);
  nh.advertiseService(restart_all_server);
  nh.advertiseService(relays_server);
#ifdef HAVE_ELEVATOR
  nh.serviceClient(elev_set_client);
#endif
  nh.subscribe(command_sub);



  while (!nh.connected()) {
    blink_led(1000);
    nh.spinOnce();
  }
  nh.spinOnce();
  delay(1000);
  nh.spinOnce();
  nh.loginfo("Starting up...");
  startup_init();

}



void startup_init() {

#ifdef HAVE_ELEVATOR
  setup_homing();
#else
  //nh.loginfo("No arm elevator, ignoring elevator home switches");
#endif

#ifdef HAVE_PAN_TILT
  pan_tilt_setup();
#endif

  setup_imu();

  setup_urf();
  nh.loginfo("URF sensors ready");

#ifdef USE_GPS
  GPS_SERIAL_PORT.begin(GPS_PORT_SPEED);
  nh.loginfo("GPS ready");
#endif



  setup_driver();

}

void blink_led(int led_interval) {
  if (millis()-led_t>led_interval) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    led_t=millis();
  } 
}

void relaysCb(const relays::Request & req, relays::Response & res) {

  digitalWrite(RELAY1_PIN,req.ch1);
  digitalWrite(RELAY2_PIN,req.ch2);
  res.ack=true;
}

void restart_allCb(const Empty::Request & req, Empty::Response & res) {
  nh.loginfo("Restarting...");
  startup_init();
}

void attachCommandCallbacks()
{
  // Attach callback methods
  //  cmdMessenger.attach(OnUnknownCommand);
  cmdMessenger.attach(kGetParametersAck, OnGetParametersAck);
  cmdMessenger.attach(kStatus, OnStatus);
  cmdMessenger.attach(kEncoders, OnEncoders);
  cmdMessenger.attach(kRx, OnRx);
  cmdMessenger.attach(kRcCalib, OnRcCalib);
  cmdMessenger.attach(kloginfo, Onloginfo);
  cmdMessenger.attach(klogwarn, Onlogwarn);
}


void OnRcCalib() {
  int CH = cmdMessenger.readInt32Arg();
  int RX_MIN = cmdMessenger.readInt32Arg();
  int RX_CENTER  = cmdMessenger.readInt32Arg();
  int RX_MAX = cmdMessenger.readInt32Arg();

  sprintf(temp_msg, "Channel: %d   Min: %d   Center: %d   Max: %d", CH,RX_MIN,RX_CENTER,RX_MAX); 
  nh.loginfo(temp_msg);
}

void Onloginfo() {

  nh.loginfo(cmdMessenger.readStringArg());
}
void Onlogwarn() {
  nh.logwarn(cmdMessenger.readStringArg());
}

void pub_raw() {

  raw_msg.orientation[0] = qx;
  raw_msg.orientation[1] = qy;
  raw_msg.orientation[2] = qz;
  raw_msg.orientation[3] = qw;

  raw_msg.linear_acceleration[0]=ax;
  raw_msg.linear_acceleration[1]=ay;
  raw_msg.linear_acceleration[2]=az;

  raw_msg.angular_velocity[0]=gx;
  raw_msg.angular_velocity[1]=gy;
  raw_msg.angular_velocity[2]=gz;

  raw_msg.magnetometer[0]=mx;
  raw_msg.magnetometer[1]=my;
  raw_msg.magnetometer[2]=mz;

  raw_msg.encoders[0] = (long)left_enc;
  raw_msg.encoders[1] = (long)right_enc;

  raw_msg.urf[0] = (float)Left_URF_Median.getMedian() / 65535 * 5120 /1000;
  raw_msg.urf[1] = (float)Rear_URF_Median.getMedian() / 65535 * 5120 /1000 ; 
  raw_msg.urf[2] = (float)Right_URF_Median.getMedian() / 65535 * 5120 /1000;

  p_raw.publish(&raw_msg);
  // digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}

void loop()
{
  if (nh.connected()) blink_led(100);
  else {
    blink_led(1000);
    while (!nh.connected()) {
      nh.spinOnce();
      blink_led(1000);
    }
    nh.spinOnce();
    delay(1000);
    nh.spinOnce();
    nh.loginfo("Starting up...");
    startup_init();
  }

#ifdef HAVE_ELEVATOR
  check_lower_home();
  check_upper_home();
#endif

  cmdMessenger.feedinSerialData();

  check_encoders();

  if (millis() - status_t >= STATUS_INTERVAL)  {
    pub_status();
    status_t = millis();

  }


  if ((ask_parameters==true)&&(!got_parameters)&&(millis() - gotp_t >= ASK_PARAMETERS_INTERVAL)) {
    cmdMessenger.sendCmd(kGetParameters, true);
    char ask_msg[100];
    sprintf(ask_msg, "Asking controller parameters... (Attempt %d/%d)", asks,ASK_PARAMETERS_ATTEMPTS);
    asks=asks+1;
    if (asks>ASK_PARAMETERS_ATTEMPTS) ask_parameters=false;
    nh.loginfo(ask_msg);
    gotp_t=millis();
  }



  if (millis() - pub_t >= PUB_RAW_INTERVAL)  {
    pub_raw();
    pub_t = millis();
  }


  if (millis() - urf_t >= URF_INTERVAL)  {

    read_urf();
    //  nh.spinOnce();
    urf_t = millis();
  }

#ifdef USE_GPS
  read_gps();
#endif

  if (!imu_fault) {
    if (millis() - imu_t <= CHECK_IMU_INTERVAL)  {
      read_imu();
    }
    else {
      imu_fault = true;
      nh.logerror("IMU Fault");
    }
  }

#ifdef HAVE_PAN_TILT
  pan_tilt_wd();
#endif

  nh.spinOnce();
}






















