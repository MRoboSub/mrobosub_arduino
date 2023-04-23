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
    Servo left_servo;
    Servo right_servo;

    constexpr int LEFT_PIN = 12;
    constexpr int RIGHT_PIN = 11;

    /* ROSTopic Name */
    constexpr char *LEFT_ANGLE_TOPIC_NAME = "/left_servo/angle";
    constexpr char *RIGHT_ANGLE_TOPIC_NAME = "/right_servo/angle";

    /* Angle Subscriber */
    void set_left_angle(const std_msgs::Int32 &angle_msg)
    {
        left_servo.write(angle_msg.data);
    }

    void set_right_angle(const std_msgs::Int32 &angle_msg)
    {
        right_servo.write(angle_msg.data);
    }

    // NOTE: Angle on servo is restricted between 25 degrees and 155 degrees.
    ros::Subscriber<std_msgs::Int32> left_angle_sub(LEFT_ANGLE_TOPIC_NAME, &set_left_angle);
    ros::Subscriber<std_msgs::Int32> right_angle_sub(RIGHT_ANGLE_TOPIC_NAME, &set_right_angle);

    void setup()
    {
        left_servo.attach(LEFT_PIN);
        right_servo.attach(RIGHT_PIN);

        Global::nh.loginfo("Servo attached");

        Global::nh.subscribe(left_angle_sub);
        Global::nh.subscribe(right_angle_sub);

        Global::nh.negotiateTopics();
    }

    void loop()
    {
        Global::nh.spinOnce();
        delay(15);
    }
}