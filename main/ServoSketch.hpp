/**
 * Sketch module to control the servo.
 *
 * Authors:
 * - Ivan Wei (ivanw8288@gmail.com)
 * - Ayan Chowdhury (ayanc@umich.edu)
 **/
#pragma once
#include "GlobalNodeHandle.hpp"

#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int32.h>

namespace ServoSketch
{
    // Servo Values
    Servo servo;
    constexpr int PIN = 12;

    /* ROSTopic Name */
    constexpr char *ANGLE_TOPIC_NAME = "/servo/angle";

    /* Angle Subscriber */
    void set_angle(const std_msgs::Int32 &angle_msg)
    {
        servo.write(angle_msg.data);
    }
    ros::Subscriber<std_msgs::Int32> angle_sub(ANGLE_TOPIC_NAME, &set_angle);

    void setup()
    {
        servo.attach(PIN);
        Global::nh.loginfo("Servo attached");
        Global::nh.subscribe(angle_sub);
        Global::nh.negotiateTopics();
    }

    void loop()
    {
        Global::nh.spinOnce();
        delay(15);
    }
}