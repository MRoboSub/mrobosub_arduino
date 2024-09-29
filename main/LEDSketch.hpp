#pragma once

#include <ros.h>
#include <std_msgs/Bool.h>

#include "GlobalNodeHandle.hpp"

namespace LEDSketch
{
    constexpr unsigned long PIN_HALL_EFFECT_1_LED = 52;
    constexpr unsigned long PIN_HALL_EFFECT_2_LED = 53;
    constexpr unsigned long PIN_ON_LED = 50;

    constexpr char *HALL_EFFECT_1_LED_TOPIC = "/led/hall_effect_1";
    constexpr char *HALL_EFFECT_2_LED_TOPIC = "/led/hall_effect_2";
    constexpr char *ON_LED_TOPIC = "/led/on";

    void hall_effect_1_callback(const std_msgs::Bool& msg) {digitalWrite(PIN_HALL_EFFECT_1_LED, msg.data);}
    void hall_effect_2_callback(const std_msgs::Bool& msg) {digitalWrite(PIN_HALL_EFFECT_2_LED, msg.data);}
    void onCallback(const std_msgs::Bool& msg) {digitalWrite(PIN_ON_LED, msg.data);}

    ros::Subscriber<std_msgs::Bool> hall_effect_1_led_subscriber(HALL_EFFECT_1_LED_TOPIC, &hall_effect_1_callback);
    ros::Subscriber<std_msgs::Bool> hall_effect_2_led_subscriber(HALL_EFFECT_2_LED_TOPIC, &hall_effect_2_callback);
    ros::Subscriber<std_msgs::Bool> on_led_subscriber(ON_LED_TOPIC, &onCallback);

    void setup()
    {
        pinMode(PIN_HALL_EFFECT_1_LED, OUTPUT);
        pinMode(PIN_HALL_EFFECT_2_LED, OUTPUT);
        pinMode(PIN_ON_LED, OUTPUT);

        Global::nh.subscribe(hall_effect_1_led_subscriber);
        Global::nh.subscribe(hall_effect_2_led_subscriber);
        Global::nh.subscribe(on_led_subscriber);
        
        Global::nh.negotiateTopics();
        Global::nh.loginfo("LED subscribers setup");
    }
}