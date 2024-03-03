#pragma once

#include <ros.h>
#include <std_msgs/Bool.h>

#include "GlobalNodeHandle.hpp"

namespace LimitSwitchSketch
{
    const unsigned long PIN_LIMITSWITCH = 38;

    constexpr char* LIMITSWITCH_TOPIC_NAME = "/buttons/limitswitch";

    std_msgs::Bool limitswitch_msg;

    ros::Publisher limitswitch_pub(LIMITSWITCH_TOPIC_NAME, &limitswitch_msg);

    void setup() {
        Global::nh.advertise(limitswitch_pub);
        
        Global::nh.negotiateTopics();
        Global::nh.loginfo("Button publishers setup");

        pinMode(PIN_LIMITSWITCH, INPUT_PULLUP);
    }

    void loop() {
        limitswitch_msg.data = digitalRead(PIN_LIMITSWITCH);

        limitswitch_pub.publish(&limitswitch_msg);
    }
}
