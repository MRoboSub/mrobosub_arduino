#include <Thread.h>
#include <StaticThreadController.h>
#include <ros.h>

#include "GlobalNodeHandle.hpp"
#include "SketchThread.hpp"
#include "ArduinoDepthSensor.hpp"
#include "RosSerialEsc.hpp"
#include "TestSketch.hpp"

const size_t SKETCHES = 2;

StaticThreadController<SKETCHES> setup_threads = {
    &ArduinoDepthSensorSketch::setup_thread,
    &RosSerialEscSketch::setup_thread,
};

StaticThreadController<SKETCHES> loop_threads = {
    &ArduinoDepthSensorSketch::loop_thread,
    &RosSerialEscSketch::loop_thread,
};

void setup()
{
    {
        Global::NodeHandleLock nhlock;
        nhlock.peek().initNode();
        // Wait for the node handle to connect
        while (nhlock.peek().connected())
        {
            nhlock.peek().spinOnce();
        }
    }

    setup_threads.run();
}

void loop()
{
    loop_threads.run();
}