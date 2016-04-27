//
// Created by sub on 14/04/16.
//

#include "EventSignal.h"

EventSignal::EventSignal()
{
    _batPwrVal = 0;
}

void EventSignal::signalBatVal(int newVal)
{
    if (_batPwrVal != newVal)
    {
        _batPwrVal = newVal;
        Q_EMIT batValChanged(newVal);
    }
}

void EventSignal::signalLed(long int newVal, Led* led)
{
    Q_EMIT ledChanged(newVal, led);
}
