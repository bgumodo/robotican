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

/************************************************************************
 * Goal:procedure will launch the launch file which is associated with it
 * *********************************************************************/
void EventSlot::execLaunch() {
    std::string openLauncherCmdA("xterm -e sh -c 'echo \"opening launch file...\"; echo; roslaunch ");
    std::string openLauncherCmdB("; exec bash;'");
    std::string launcherFilePath, launcherPkg;


    _nHandle.param<std::string>("launcher_file_path", launcherFilePath, "armadillo.launch");
    _nHandle.param<std::string>("launcher_file_pkg", launcherPkg, "robotican_armadillo");
    std::string launchCmd = openLauncherCmdA + launcherPkg + " " + launcherFilePath + openLauncherCmdB;

    QIcon icon;
    if (!_launcherOpened)
    {
        _launcherOpened = true;
        QFuture<std::string> execProc = QtConcurrent::run(this, &EventSlot::execShellCmd, launchCmd.c_str());
        icon.addFile(QString(":/images/Shutdown.png"), QSize(), QIcon::Normal, QIcon::Off);

    }
    else
    {
        icon.addFile(QString(":/images/turnOn.png"), QSize(), QIcon::Normal, QIcon::Off);

        _launcherOpened = false;
       system("killall xterm");

    }

    _guiHandle->launch_btn->setIcon(icon);
}

/******************************************************
 * Goal: procedure send command to shell and execute it
 * Params: @cmd is the command you wish to execute
 *****************************************************/
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

/************************************************
 * Goal: calculate time interval between times
 * Params: @startTime, @endTime
 * Pre-cond: @endTime > @startTime
 * Post-cond: returns time interval between times
 ***********************************************/
double EventSlot::calcTimeOut(long int startTime, long int endTime)
{
    double timeOut = -1;
    if (endTime > startTime)
    {
        timeOut = (endTime - startTime) / (double) CLOCKS_PER_SEC;
    }
    return timeOut;
}