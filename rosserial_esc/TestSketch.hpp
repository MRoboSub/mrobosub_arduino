#pragma once

#include "GlobalNodeHandle.hpp"
#include "SketchThread.hpp"
#include <ros.h>

namespace TestSketchA {
    void setup() {
        {
            Global::NodeHandleLock().peek().logwarn("Test setup A");
        }
    }

    void loop() {
        {
            Global::NodeHandleLock nhlock;
            nhlock.peek().logwarn("Test loop A");
            nhlock.peek().spinOnce();
        }
    }

    unsigned long thread_delay()
    {
        return 1200;
    }

    LoopThread loop_thread(loop, thread_delay());
    SetupThread setup_thread(setup, loop_thread);
}

namespace TestSketchB {
    void setup() {
        {
            Global::NodeHandleLock().peek().logwarn("Test setup B");
        }
    }

    void loop() {
        {
            Global::NodeHandleLock nhlock;
            nhlock.peek().logwarn("Test loop B");
            nhlock.peek().spinOnce();
        }
    }

    unsigned long thread_delay()
    {
        return 1500;
    }

    LoopThread loop_thread(loop, thread_delay());
    SetupThread setup_thread(setup, loop_thread);
}