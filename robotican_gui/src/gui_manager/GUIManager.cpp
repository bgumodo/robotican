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
}


