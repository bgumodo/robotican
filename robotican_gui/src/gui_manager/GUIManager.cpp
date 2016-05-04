//
// Created by sub on 07/04/16.
//

#include "GUIManager.h"

GUImanager::GUImanager(QMainWindow &widget, Ui::MainWindow &win, QApplication &app)
{
    //register types for QT
    qRegisterMetaType<long int>("long int");
    qRegisterMetaType<Led*>("Led*");

    //hold gui handles
    _widget = &widget;
    _win = &win;
    _app = &app;
    _timer = _nh.createTimer(ros::Duration(LOOP_RATE), &GUImanager::_loopEvents, this);

    _eventSlot.initiate(win, app);

    _initiateLbls();
    _initiateLeds();
    _connectEvents();
    _subscribeListeners();
}

void GUImanager::startGUI()
{
    _timer.start();
}

void GUImanager::_connectEvents()
{
    //battery meter updates
    QObject::connect(&_eventSignal, SIGNAL(batValChanged(int)),
                     &_eventSlot, SLOT(setBatPwr(int)));

    //leds updates
    QObject::connect(&_eventSignal, SIGNAL(ledChanged(long int, Led*)),
                     &_eventSlot, SLOT(setLed(long int, Led*)));

    //exit app button
    QObject::connect(_win->exit_btn, SIGNAL(clicked()),
                     &_eventSlot, SLOT(closeApp()));

    //execute/close launcher
    QObject::connect(_win->launch_btn, SIGNAL(clicked()),
                     &_eventSlot, SLOT(execLaunch()));
}

/*this procedure is a called as ros::Timer callback,
 * therefore it will run in loop
 */
void GUImanager::_loopEvents(const ros::TimerEvent &timerEvent) {

    _eventSignal.signalBatVal(_batListener.getBatteryPwr());
    _eventSignal.signalLed(_batListener.getLastSignal(), &_batteryLed);
    _eventSignal.signalLed(_armListener.getLastSignal(), &_armLed);
    _eventSignal.signalLed(_panTiltListenere.getLastSignal(), &_panTiltLed);
    _eventSignal.signalLed(_odomListener.getLastSignal(), &_odomLed);
    _eventSignal.signalLed(_gripperListener.getLastSignal(), &_gripperLed);
    _eventSignal.signalLed(_imuListener.getLastSignal(), &_imuLed);
    _eventSignal.signalLed(_lidarListener.getLastSignal(), &_lidarLed);
    _eventSignal.signalLed(_gpsListener.getLastSignal(), &_gpsLed);
    _eventSignal.signalLed(_frontCamListener.getLastSignal(), &_frontCamLed);
    _eventSignal.signalLed(_rearCamListener.getLastSignal(), &_rearCamLed);
    _eventSignal.signalLed(_urfLeftListener.getLastSignal(), &_urfLeftLed);
    _eventSignal.signalLed(_urfRearListener.getLastSignal(), &_urfRearLed);
    _eventSignal.signalLed(_urfRightListener.getLastSignal(), &_urfRightLed);
    _eventSignal.signalLed(_kinect2Listener.getLastSignal(), &_kinect2Led);
    _eventSignal.signalLed(_f200Listener.getLastSignal(), &_f200Led);
}

void GUImanager::_initiateLeds()
{
    _armLed.initiate(*_win->arm_led);
    _panTiltLed.initiate(*_win->pan_tilt_led);
    _odomLed.initiate(*_win->odom_led);
    _gripperLed.initiate(*_win->gripper_led);
    _imuLed.initiate(*_win->imu_led);
    _lidarLed.initiate(*_win->lidar_led);
    _gpsLed.initiate(*_win->gps_led);
    _frontCamLed.initiate(*_win->front_cam_led);
    _rearCamLed.initiate(*_win->rear_cam_led);
    _urfLeftLed.initiate(*_win->urf_left_led);
    _urfRearLed.initiate(*_win->urf_rear_led);
    _urfRightLed.initiate(*_win->urf_right_led);
    _kinect2Led.initiate(*_win->kinect2_led);
    _f200Led.initiate(*_win->f200_led);
    _batteryLed.initiate(*_win->battery_led);
}

void GUImanager::_subscribeListeners()
{
    //subscribe all listeners to appropriate topic
    _batListener.subscribe();
    _armListener.subscribe();
    _panTiltListenere.subscribe();
    _odomListener.subscribe();
    _gripperListener.subscribe();
    _imuListener.subscribe();
    _lidarListener.subscribe();
    _gpsListener.subscribe();
    _rearCamListener.subscribe();
    _frontCamListener.subscribe();
    _urfRightListener.subscribe();
    _urfRearListener.subscribe();
    _urfLeftListener.subscribe();
    _kinect2Listener.subscribe();
    _f200Listener.subscribe();
}

void GUImanager::_initiateLbls()
{
    std::string tempParam;

    _nh.param<std::string>("battery_lbl_name",tempParam, "BATTERY");
    _win->battery_lbl->setText(QApplication::translate("MainWindow", tempParam.c_str(), 0));

    _nh.param<std::string>("gps_lbl_name",tempParam, "GPS");
    _win->gps_lbl->setText(QApplication::translate("MainWindow", tempParam.c_str(), 0));

    _nh.param<std::string>("imu_lbl_name",tempParam, "IMU");
    _win->imu_lbl->setText(QApplication::translate("MainWindow", tempParam.c_str(), 0));

    _nh.param<std::string>("lidar_lbl_name",tempParam, "LIDAR");
    _win->lidar_lbl->setText(QApplication::translate("MainWindow", tempParam.c_str(), 0));

    _nh.param<std::string>("odom_lbl_name",tempParam, "ODOM");
    _win->odom_lbl->setText(QApplication::translate("MainWindow", tempParam.c_str(), 0));

    _nh.param<std::string>("arm_lbl_name",tempParam, "ARM");
    _win->arm_lbl->setText(QApplication::translate("MainWindow", tempParam.c_str(), 0));

    _nh.param<std::string>("gripper_lbl_name",tempParam, "GRIPPER");
    _win->gripper_lbl->setText(QApplication::translate("MainWindow", tempParam.c_str(), 0));

    _nh.param<std::string>("left_urf_lbl_name",tempParam, "LEFT");
    _win->left_urf_lbl->setText(QApplication::translate("MainWindow", tempParam.c_str(), 0));

    _nh.param<std::string>("right_urf_lbl_name",tempParam, "RIGHT");
    _win->right_urf_lbl->setText(QApplication::translate("MainWindow", tempParam.c_str(), 0));

    _nh.param<std::string>("rear_urf_lbl_name",tempParam, "REAR");
    _win->rear_urf_lbl->setText(QApplication::translate("MainWindow", tempParam.c_str(), 0));

    _nh.param<std::string>("kinect2_lbl_name",tempParam, "KINECT2");
    _win->kinnect2_lbl->setText(QApplication::translate("MainWindow", tempParam.c_str(), 0));

    _nh.param<std::string>("f200_lbl_name",tempParam, "F200");
    _win->f200_lbl->setText(QApplication::translate("MainWindow", tempParam.c_str(), 0));

    _nh.param<std::string>("rear_cam_lbl_name",tempParam, "REAR");
    _win->rear_cam_lbl->setText(QApplication::translate("MainWindow", tempParam.c_str(), 0));

    _nh.param<std::string>("front_cam_name",tempParam, "FRONT");
    _win->front_cam_lbl->setText(QApplication::translate("MainWindow", tempParam.c_str(), 0));

    _nh.param<std::string>("pan_tilt_name",tempParam, "PAN-TILT");
    _win->pan_tilt_lbl->setText(QApplication::translate("MainWindow", tempParam.c_str(), 0));
}


