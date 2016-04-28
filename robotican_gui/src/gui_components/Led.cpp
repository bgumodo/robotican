//
// Created by sub on 16/04/16.
//

#include "Led.h"

Led::Led()
{
    _state = OFF;
    _timeMarker = 0;
}

void Led::initiate(QLabel &led)
{
    _ledHandle = &led;
}

void Led::turnOn()
{
    _ledHandle->setPixmap(QPixmap(QString::fromUtf8(ON_PIC)));
    _state = ON;
}

void Led::turnOff()
{
    _ledHandle->setPixmap(QPixmap(QString::fromUtf8(OFF_PIC)));
    _state = OFF;
}

bool Led::getState()
{
    return _state;
}

long int Led::getTimeMarker()
{
    return _timeMarker;
}

void Led::setTimeMarker(long int val)
{
    _timeMarker = val;
}