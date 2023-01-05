#pragma once
/*
 * rosserial motor controller for BlueRobotics ESC's
 */

#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include "GlobalNodeHandle.hpp"

namespace RosSerialEscSketch
{
    constexpr size_t NUM_MOTORS = 8;

    constexpr unsigned long STOP_WIDTH = 1500;
    constexpr unsigned long MAX_FWD_WIDTH = 1900;
    constexpr unsigned long MAX_BCK_WIDTH = 1100;

    constexpr char *SUBSCRIBER_NAME = "motor";

    constexpr unsigned long SETUP_DELAY = 1000;
    constexpr unsigned long LOOP_DELAY = 1;

    const byte pins[NUM_MOTORS] = {4, 5, 6, 7, 8, 9, 10, 11};
    Servo motors[NUM_MOTORS];

    void motor_update_cb(const std_msgs::Int16MultiArray &cmd_msg)
    {
        for (int i = 0; i < NUM_MOTORS; ++i)
        {
            int width = constrain(cmd_msg.data[i], MAX_BCK_WIDTH, MAX_FWD_WIDTH);
            motors[i].writeMicroseconds(width);
        }
    }

    ros::Subscriber<std_msgs::Int16MultiArray> sub(SUBSCRIBER_NAME, motor_update_cb);

    void setup()
    {
        Global::nh.subscribe(sub);
        for (int i = 0; i < NUM_MOTORS; i++)
        {
            motors[i].attach(pins[i]);
            motors[i].writeMicroseconds(STOP_WIDTH);
        }

        delay(SETUP_DELAY);
    }

    void loop()
    {
        Global::nh.spinOnce();
        delay(LOOP_DELAY);
    }
}