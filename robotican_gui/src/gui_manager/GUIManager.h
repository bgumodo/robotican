//
// Created by sub on 07/04/16.
//

#ifndef ROBOTICAN_GUI_MANAGER_H
#define ROBOTICAN_GUI_MANAGER_H

#include <sstream>
#include "../../include/robotican_gui.h"
#include "event_handler/EventSignal.h"
#include "event_handler/EventSlot.h"
#include "gui_components/Led.h"

#include "listeners/Battery.h"
#include "listeners/Arm.h"
#include "listeners/Odom.h"
#include "listeners/PanTilt.h"
#include "listeners/Gripper.h"
#include "listeners/Imu.h"
#include "listeners/Lidar.h"
#include "listeners/Gps.h"
#include "listeners/FrontCam.h"
#include "listeners/RearCam.h"
#include "listeners/UrfLeft.h"
#include "listeners/UrfRear.h"
#include "listeners/UrfRight.h"
#include "listeners/Kinect2.h"
#include "listeners/F200.h"

#define LOOP_RATE 0.001

class GUImanager {
private:

    EventSignal _eventSignal;
    EventSlot _eventSlot;
    QMainWindow * _widget;
    Ui::MainWindow * _win;
    QApplication * _app;
    ros::NodeHandle _nh;
    ros::Timer _timer;

    Battery _batListener;
    Arm _armListener;
    PanTilt _panTiltListenere;
    Odom _odomListener;
    Gripper _gripperListener;
    Imu _imuListener;
    Lidar _lidarListener;
    Gps _gpsListener;
    FrontCam _frontCamListener;
    RearCam _rearCamListener;
    UrfLeft _urfLeftListener;
    UrfRear _urfRearListener;
    UrfRight _urfRightListener;
    Kinect2 _kinect2Listener;
    F200 _f200Listener;

    Led _armLed;
    Led _panTiltLed;
    Led _odomLed;
    Led _gripperLed;
    Led _imuLed;
    Led _lidarLed;
    Led _gpsLed;
    Led _frontCamLed;
    Led _rearCamLed;
    Led _urfLeftLed;
    Led _urfRearLed;
    Led _urfRightLed;
    Led _kinect2Led;
    Led _f200Led;
    Led _batteryLed;

    int _tempPublisherCounter; //delete after testing publishers

    void _loopEvents(const ros::TimerEvent& timerEvent);
    void _connectEvents();
    void _subscribeListeners();
    void _initiateLeds();
    void _initiateLbls();



public:
    GUImanager(QMainWindow &widget, Ui::MainWindow &win, QApplication &app);
    void startGUI();
    void publishAll();

};


#endif //ROBOTICAN_GUI_GUI_MANAGER_H
