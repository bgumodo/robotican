//
// Created by sub on 14/04/16.
//

#ifndef ROBOTICAN_GUI_EVENTSIGNAL_H
#define ROBOTICAN_GUI_EVENTSIGNAL_H
#include <QObject>
#include <QMainWindow>
#include <QApplication>
#include <QLabel>
#include "../gui_components/Led.h"

class EventSignal : public QObject {
    Q_OBJECT
public:
    EventSignal();
    void signalBatVal(int newVal);
    void signalLed(long int newVal, Led* led);

    Q_SIGNALS:
    void batValChanged(int newVal);
    void ledChanged(long int newVal, Led* led);

private:
    int _batPwrVal;
};

#endif //ROBOTICAN_GUI_EVENTSIGNAL_H
