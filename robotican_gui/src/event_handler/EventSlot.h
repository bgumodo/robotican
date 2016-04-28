//
// Created by sub on 14/04/16.
//

#ifndef ROBOTICAN_GUI_EVENT_SLOT_H
#define ROBOTICAN_GUI_EVENT_SLOT_H
#include "../../include/robotican_gui.h"

#include <sys/wait.h>
#include <unistd.h>
#include <cstdlib>
#include <iostream>
#include <cstdio>
#include <memory>
#include <ros/ros.h>
#include <QThread>
#include <QFuture>
#include <QtConcurrentRun>
#include "../gui_components/Led.h"

#define OPEN_LAUNCHER_CMD "xterm -e sh -c ',~/tom.bag; exec bash;'"
#define CLOSE_LAUNCHER_CMD "echo 'close launcher cmd'"
#define TIMEOUT 0.1

class EventSlot : public QThread {
    Q_OBJECT
public:
    EventSlot();
    void initiate(Ui::MainWindow &guiHandle, QApplication &app);

    public Q_SLOTS:
    void setBatPwr(int val);
    void setLed(long int val, Led* led);
    void closeApp();
    void execLaunch();

private:
    Ui::MainWindow * _guiHandle;
    QApplication * _app;
    ros::NodeHandle _nHandle;

    bool _launcherOpened;
    std::string execShellCmd(const char* cmd);
    double calcTimeOut(long int startTime, long int endTime);



};


#endif //ROBOTICAN_GUI_EVENT_SLOT_H
