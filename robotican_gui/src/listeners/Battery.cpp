//
// Created by sub on 16/04/16.
//

//
// Created by sub on 13/04/16.
//
#include "Battery.h"

Battery::Battery()
{
  _batPower = 0;
}

void Battery::_chatterCallback(const std_msgs::UInt32::ConstPtr &msg)
{
  _batPower = msg->data;
}

int Battery::getBatteryPwr()
{
  return _batPower;
}

void Battery::subscribe()
{
  _nHandle.param<std::string>("batteryTopic",_topicName, "batteryTopic");
  _sub = _nHandle.subscribe(_topicName, 1000, &Battery::_chatterCallback, this);
}
