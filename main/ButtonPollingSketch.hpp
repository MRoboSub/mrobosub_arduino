#pragma once

#include <ros.h>
#include <std_msgs/Bool.h>

namespace ButtonPollingSketch
{
    const unsigned long PIN_STRANGE = 12;
    const unsigned long PIN_CHARM = 13;

    constexpr char* STRANGE_TOPIC_NAME = "/buttons/strange";
    constexpr char* CHARM_TOPIC_NAME = "/buttons/charm";

    std_msgs::Bool strange_msg;
    std_msgs::Bool charm_msg;

    ros::Publisher strange_pub(STRANGE_TOPIC_NAME, &strange_msg);
    ros::Publisher charm_pub(CHARM_TOPIC_NAME, &charm_msg);

    void setup() {
        Global::nh.advertise(strange_pub);
        Global::nh.advertise(charm_pub);
        
        pinMode(PIN_STRANGE, INPUT_PULLUP);
        pinMode(PIN_CHARM, INPUT_PULLUP);
    }

    void loop() {
        strange_msg.data = digitalRead(PIN_STRANGE);
        charm_msg.data = digitalRead(PIN_CHARM);

        strange_pub.publish(&strange_msg);
        charm_pub.publish(&charm_msg);
    }
}