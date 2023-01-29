/**
 * Main launch point for all of the modules.
 *
 * Authors:
 * - Ivan Wei (ivanw8288@gmail.com)
 * - Ayan Chowdhury (ayanc@umich.edu)
 **/
#include "GlobalNodeHandle.hpp"
#include "TestSketches.hpp"
#include "ArduinoDepthSketch.hpp"
#include "ServoSketch.hpp"

#include <Scheduler.h>
#include <ros.h>

void setup()
{
    // Initialize the node
    Global::nh.initNode();
    // Wait until connected
    while (!Global::nh.connected())
    {
        Global::nh.spinOnce();
    }
    Global::nh.loginfo("Connected NodeHandle, starting tasks.");

    // Add module callbacks below
    Scheduler.start(ArduinoDepthSensorSketch::setup, ArduinoDepthSensorSketch::loop);
    Scheduler.start(ServoSketch::setup, ServoSketch::loop);
}

void loop()
{
    // Do nothing and yield for the other scheduled tasks
    Global::nh.spinOnce();
    delay(100);
}