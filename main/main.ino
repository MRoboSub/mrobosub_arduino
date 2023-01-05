#include "GlobalNodeHandle.hpp"
#include "TestSketches.hpp"
#include "ArduinoDepthSketch.hpp"
#include "RosserialEscSketch.hpp"

#include <Scheduler.h>
#include <ros.h>

void setup() {
    Global::nh.initNode();
    while (!Global::nh.connected())
    {
        Global::nh.spinOnce();
    }
    Global::nh.loginfo("Connected NodeHandle, starting tasks.");
    
    Scheduler.start(ArduinoDepthSensorSketch::setup, ArduinoDepthSensorSketch::loop);
    Scheduler.start(RosSerialEscSketch::setup, RosSerialEscSketch::loop);    
}

void loop() {
    Global::nh.spinOnce();
    delay(100);
}