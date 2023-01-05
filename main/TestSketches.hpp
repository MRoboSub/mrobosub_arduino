#pragma once
#include "GlobalNodeHandle.hpp"
#include <ros.h>

namespace TestSketchA {
    void setup() {
        Global::nh.logwarn("Test setup A");
    }

    void loop() {
        Global::nh.logwarn("Test loop A part 1");
        Global::nh.spinOnce();
        delay(1200);
        Global::nh.logwarn("Test loop A part 2");
        Global::nh.spinOnce();
        delay(1200);
    }
}

namespace TestSketchB {
    void setup() {
        Global::nh.logwarn("Test setup B");
    }

    void loop() {
        Global::nh.logwarn("Test loop B part 1");
        Global::nh.spinOnce();
        delay(1500);
        Global::nh.logwarn("Test loop B part 2");
        Global::nh.spinOnce();
        delay(1500);
    }
}