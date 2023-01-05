#pragma once

#include <ros.h>

namespace Global
{

    class NodeHandleLock
    {
    private:
        inline static ros::NodeHandle nh;
    public:
        NodeHandleLock()
        {
            noInterrupts();
        }

        ~NodeHandleLock()
        {
            interrupts();
        }

        ros::NodeHandle &peek()
        {
            return nh;
        }
    };
}