#pragma once

#include <Thread.h>

class LoopThread : public Thread
{
public:
    LoopThread(void (*callback)(void), unsigned long delay) : Thread(callback, delay)
    {
        enabled = false;
    }
};

class SetupThread : public Thread
{
private:
    LoopThread &loop_thread;

public:
    SetupThread(void (*callback)(void), LoopThread &loop_thread) : loop_thread(loop_thread), Thread()
    {
        enabled = true;
        onRun(callback);
    }

    void run() override
    {
        Thread::run();
        loop_thread.enabled = true;
        runned();
    }
};
