#pragma once
/**
 * Arduino sketch to read from a BlueRobotics Bar30 Depth Sensor
 * and publish depth readings to ROS.
 *
 * Depth readings are in meters.
 *
 * Authors:
 * - Ivan Wei (ivanw8288@gmail.com)
 * - Ayan Chowdhury (ayanc@umich.edu)
 **/

#include <MS5837.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <Wire.h>
#include "GlobalNodeHandle.hpp"

namespace ArduinoDepthSensorSketch
{
    /** CONSTANTS **/

    /* Time Constants */
    // Since reading from the sensor takes up to 40ms,
    // make sure this is at least 40.
    constexpr unsigned long MEASUREMENT_DELAY_MS = 50;
    constexpr unsigned long FAILURE_WARNING_MS = 5000;

    /* Fluid Density */
    // Units are in kg/m^3
    constexpr float FLUID_DENSITY_FRESHWATER = 997;
    constexpr float _FLUID_DENSITY_SEAWATER = 1029;

    /* ROSTopic Name */
    constexpr char *DEPTH_TOPIC_NAME = "/depth/raw_depth";

    /* Sensor and ROS initialization */
    MS5837 sensor;

    // Depth readings are in meters
    std_msgs::Float32 depth_msg;
    ros::Publisher pub_depth("/depth/raw_depth", &depth_msg);

    void setup()
    {
        // Register publisher
        {
            Global::NodeHandleLock().peek().advertise(pub_depth);
        }

        // For I2C communication with sensor
        Wire.begin();

        // Wait for the pressure sensor to initialize
        while (!sensor.init())
        {
            {
                Global::NodeHandleLock().peek().logerror("Depth sensor initialization failed!\n"
                                                         "Are SDA/SCL connected correctly?\n"
                                                         "Blue Robotics Bar30: White=SDA, Green=SCL\n\n\n");
            }
            delay(FAILURE_WARNING_MS);
        }

        sensor.setFluidDensity(FLUID_DENSITY_FRESHWATER);
    }

    void loop()
    {
        /*
        // Record the start and expected end time of the loop.
        unsigned long time_loop_start = millis();
        unsigned long time_loop_end = time_loop_start + MEASUREMENT_DELAY_MS;
        */
        // Update sensor readings
        sensor.read();

        // Publish depth readings in meters
        depth_msg.data = sensor.depth();
        pub_depth.publish(&depth_msg);

        {
            Global::NodeHandleLock().peek().spinOnce();
        }
        /*
        // Delay until the end of the period for consistent delta time.
        unsigned long time_current = millis();
        if (time_current >= time_loop_end) {
            Global::nh.logwarn("Sensor read is taking longer than max loop time."
                    "Delta time may be inconsistent between published readings.");
            return;
        }
        delay(time_loop_end - time_current);
        */
    }

    unsigned long thread_delay()
    {
        return MEASUREMENT_DELAY_MS;
    }

    LoopThread loop_thread(loop, thread_delay());
    SetupThread setup_thread(setup, loop_thread);
}