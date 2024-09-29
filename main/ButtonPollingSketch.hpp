#pragma once

#include <ros.h>
#include <std_msgs/Bool.h>

#include "GlobalNodeHandle.hpp"

namespace ButtonPollingSketch
{
    const unsigned long PIN_HALL_EFFECT_1 = 12;
    const unsigned long PIN_HALL_EFFECT_2 = 13;

    constexpr char* HALL_EFFECT_1_NAME = "/buttons/hall_effect_1";
    constexpr char* HALL_EFFECT_2_NAME = "/buttons/hall_effect_2";

    std_msgs::Bool hall_effect_1_msg;
    std_msgs::Bool hall_effect_2_msg;

    ros::Publisher hall_effect_1_pub(HALL_EFFECT_1_NAME, &hall_effect_1_msg);
    ros::Publisher hall_effect_2_pub(HALL_EFFECT_2_NAME, &hall_effect_2_msg);

    void setup() {
        Global::nh.advertise(hall_effect_1_pub);
        Global::nh.advertise(hall_effect_2_pub);
        
        Global::nh.negotiateTopics();
        Global::nh.loginfo("Button publishers setup");

        pinMode(PIN_HALL_EFFECT_1, INPUT_PULLUP);
        pinMode(PIN_HALL_EFFECT_2, INPUT_PULLUP);
    }

    void loop() {
        hall_effect_1_msg.data = digitalRead(PIN_HALL_EFFECT_1);
        hall_effect_2_msg.data = digitalRead(PIN_HALL_EFFECT_2);

        hall_effect_1_pub.publish(&hall_effect_1_msg);
        hall_effect_2_pub.publish(&hall_effect_2_msg);
    }
}
