/**
 * Header containing NodeHandle for global use. ROS only
 * allows one `NodeHandle` to be initialized at a time, but
 * multiple modules need to use it, so it is initialized
 * at the start of the program and kept here.
 *
 * Authors:
 * - Ivan Wei (ivanw8288@gmail.com)
 * - Ayan Chowdhury (ayanc@umich.edu)
 **/
#pragma once

#include <ros.h>

namespace Global
{
    ros::NodeHandle nh;
}