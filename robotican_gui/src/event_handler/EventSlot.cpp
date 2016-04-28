//
// Created by sub on 14/04/16.
//

#include "EventSlot.h"


EventSlot::EventSlot()
{
    _launcherOpened = false;
}

void EventSlot::initiate(Ui::MainWindow &guiHandle, QApplication &app)
{
    _guiHandle = &guiHandle;
    _app = &app;
}

void EventSlot::setBatPwr(int val)
{
    _guiHandle->battery_pbar->setValue(val);
}

void EventSlot::setLed(long int val, Led* led)
{
    //if no new signal was received by listener
    if (led->getTimeMarker() == val)
    {
        if (calcTimeOut(led->getTimeMarker(), clock()) > TIMEOUT)
            led->turnOff();
    }
    else if (calcTimeOut(led->getTimeMarker(), clock()) != -1)
    {
        led->setTimeMarker(val);
        if (!led->getState())
            led->turnOn();
    }
}

void EventSlot::closeApp()
{
    system("killall xterm");
    _app->quit();
}

void EventSlot::execLaunch() {
    std::string openCmd;
    std::string closeCmd;
    _nHandle.param<std::string>("openLauncher", openCmd, OPEN_LAUNCHER_CMD);
    _nHandle.param<std::string>("closeLauncher", closeCmd, CLOSE_LAUNCHER_CMD);

    QIcon icon;
    if (!_launcherOpened)
    {
        _launcherOpened = true;
        QFuture<std::string> execProc = QtConcurrent::run(this, &EventSlot::execShellCmd, openCmd.c_str());
        icon.addFile(QString(":/images/Shutdown.png"), QSize(), QIcon::Normal, QIcon::Off);

    }
    else
    {
        icon.addFile(QString(":/images/turnOn.png"), QSize(), QIcon::Normal, QIcon::Off);

        _launcherOpened = false;
       system("killall xterm");

    }

    _guiHandle->launch_btn->setIcon(icon);
    //_guiHandle->launch_btn->setIconSize(QSize(70, 70));



}

std::string EventSlot::execShellCmd(const char *cmd)
{
    std::string result = "";
    FILE* pipe = popen(cmd, "r");
    if (!pipe) return "ERROR";
    char buffer[128];

    while (!feof(pipe)) {
        if (fgets(buffer, 128, pipe) != NULL)
            result += buffer;
    }
    pclose(pipe);
    return result;
}

double EventSlot::calcTimeOut(long int startTime, long int endTime)
{
    double timeOut = -1;
    if (endTime > startTime)
    {
        timeOut = (endTime - 0/*startTime*/) / (double) CLOCKS_PER_SEC;
    }
    return timeOut;
}