//
// Created by sub on 21/04/16.
//

#ifndef ROBOTICAN_GUI_LISTENER_H
#define ROBOTICAN_GUI_LISTENER_H

#include <ros/ros.h>


class Listener
{
public:
    virtual void subscribe() = 0;
};




#endif //ROBOTICAN_GUI_LISTENER_H
