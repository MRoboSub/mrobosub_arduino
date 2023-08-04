#pragma once

#include <ros.h>
#include <std_msgs/Bool.h>

#include "GlobalNodeHandle.hpp"

namespace LEDSketch
{
    constexpr unsigned long PIN_STRANGE_LED = 2;
    constexpr unsigned long PIN_CHARM_LED = 3;
    constexpr unsigned long PIN_ON_LED = 4;

    constexpr char *STRANGE_LED_TOPIC = "/led/strange";
    constexpr char *CHARM_LED_TOPIC = "/led/charm";
    constexpr char *ON_LED_TOPIC = "/led/on";

    void strangeCallback(const std_msgs::Bool& msg) {digitalWrite(PIN_STRANGE_LED, msg.data);}
    void charmCallback(const std_msgs::Bool& msg) {digitalWrite(PIN_CHARM_LED, msg.data);}
    void onCallback(const std_msgs::Bool& msg) {digitalWrite(PIN_ON_LED, msg.data);}

    ros::Subscriber<std_msgs::Bool> strange_led_subscriber(STRANGE_LED_TOPIC, &strangeCallback);
    ros::Subscriber<std_msgs::Bool> charm_led_subscriber(CHARM_LED_TOPIC, &charmCallback);
    ros::Subscriber<std_msgs::Bool> on_led_subscriber(ON_LED_TOPIC, &onCallback);

    void setup()
    {
        pinMode(PIN_STRANGE_LED, OUTPUT);
        pinMode(PIN_CHARM_LED, OUTPUT);
        pinMode(PIN_ON_LED, OUTPUT);

        Global::nh.subscribe(strange_led_subscriber);
        Global::nh.subscribe(charm_led_subscriber);
        Global::nh.subscribe(on_led_subscriber);
        
        Global::nh.negotiateTopics();
        Global::nh.loginfo("LED subscribers setup");
    }
}