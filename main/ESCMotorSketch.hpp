/**
 * Sketch to control BlueRobotics ESC motors
 * Authors:
 * - Joseph Maffetone (jmaff@umich.edu)
 * - Ivan Wei (ivanw8288@gmail.com)
 **/
#pragma once

#include "GlobalNodeHandle.hpp"
#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif
#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

namespace ESCMotorSketch
{
    constexpr size_t NUM_MOTORS = 8;
    constexpr int STOP_WIDTH = 1500;
    constexpr int MAX_FWD_WIDTH = 1900;
    constexpr int MAX_BCK_WIDTH = 1100;
    constexpr unsigned long SETUP_WAIT = 1000;

    byte pins[NUM_MOTORS] = {4, 5, 6, 7, 8, 9, 10, 11};
    Servo motors[NUM_MOTORS];

    void motor_update_cb(const std_msgs::Int16MultiArray &cmd_msg)
    {
        for (size_t i = 0; i < NUM_MOTORS; ++i)
        {
            int width = constrain(cmd_msg.data[i], MAX_BCK_WIDTH, MAX_FWD_WIDTH);
            motors[i].writeMicroseconds(width);
        }
    }

    ros::Subscriber<std_msgs::Int16MultiArray> sub("motor", motor_update_cb);

    void setup()
    {
        Global::nh.subscribe(sub);

        for (int i = 0; i < NUM_MOTORS; i++)
        {
            motors[i].attach(pins[i]);
            motors[i].writeMicroseconds(STOP_WIDTH);
        }

        delay(SETUP_WAIT); // delay to allow the ESC to recognize the stopped signal
    }

    void loop()
    {
        Global::nh.spinOnce();
        delay(15);
    }
}