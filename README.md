# mrobosub_arduino
An Arduino sketch project for Michigan Robotic Submarine. 

## Current Modules
- [DepthSensorSketch.hpp](main/DepthSensorSketch.hpp) - For getting readings from a Bar30 Depth/Pressure Sensor - namespace `DepthSensorSketch`
- [ServoSketch.hpp](main/ServoSketch.hpp) - For controlling two DSSERVO 35kg servo - namespace `ServoSketch`
- [ESCMotorSketch.hpp](main/ESCMotorSketch.hpp) - For controlling eight BlueRobotics ESC motors - namespace `ESCMotorSketch`

## Pins Used
- Depth Sensor - [DepthSensorSketch.hpp](main/DepthSensorSketch.hpp) - SDA, SCL
- Left Servo, Right Servo - [ServoSketch.hpp](main/ServoSketch.hpp) - Digital 11 (Right Servo), Digital 12 (Left Servo)
- ESC Motors - [ESCMotorSketch.hpp](main/ESCMotorSketch.hpp) - Digital 4-11

## Publishers
- `/depth/raw_depth` - type `std_msgs::Float32` - the depth sensor reading, in meters - [DepthSensorSketch.hpp](main/DepthSensorSketch.hpp)

## Subscribers
- `/left_servo/angle` - type `std_msgs::Int32` - the angle to set the left servo to, between 25 and 155 degrees - [ServoSketch.hpp](main/ServoSketch.hpp)
- `/right_servo/angle` - type `std_msgs::Int32` - the angle to set the right servo to, between 25 and 155 degrees - [ServoSketch.hpp](main/ServoSketch.hpp)
- `/motors/pwm` - type `std_msgs::Int16MultiArray` - an array of pwms to set the eight motors to - [ESCMotorSketch.hpp](main/ESCMotorSketch.hpp)

## Testing
[Docker](https://www.docker.com/) is a prerequisite.

First, build the image and fire up the Docker container:
```console
$ docker-compose up -d
```

Attach two terminals to the container (VSCode Docker extension might be helpful)

In one terminal, start `roscore`:
```console
/root/catkin_ws/src/$ roscore
```

Find which serial port your Arduino is connected to. Replace `[PORT]` in the following steps with it. 

In the second terminal, compile and upload the sketch:
```console
/root/catkin_ws/src/$ cd main
/root/catkin_ws/src/main$ arduino-cli compile --fqbn arduino:avr:mega
/root/catkin_ws/src/main$ arduino-cli upload --fqbn arduino:avr:mega -p [PORT]
```
Now, start the rosserial client application to connect the Arduino to the rest of ROS:
```console
/root/catkin_ws/src/main$ rosrun rosserial_python serial_node.py [PORT]
```
An alternate option with a `~reset_arduino` service endpoint:
```console
/root/catkin_ws/src/main$ rosrun rosserial_arduino serial_node.py _port:=[PORT]
```
The terminal should now begin displaying logs and connecting to the Arduino. Publishers, subscribers, and topics from the Arduino should now be registered and read as normal.

i.e., reading from the depth sensor's `/depth/raw_depth` topic: 
```console
/root/catkin_ws/src/$ rostopic echo /depth/raw_depth
```

This tutorial has largely been paraphrased from the [ROS Wiki `rosserial_arduino` Tutorial](http://wiki.ros.org/rosserial_arduino/Tutorials/Hello%20World).

## Development Setup
[Docker](https://www.docker.com/) is a prerequisite for working properly with dependencies. 

Start the Docker container:
```
$ docker-compose up -d
```

Attach to the Docker container. The source is in `main`.

## Tools Used
- arduino-cli, a CLI tool to allow development outside of Arduino IDE - https://github.com/arduino/arduino-cli
- ArduinoScheduler, for asynchronous operations on a single core Arduino - https://github.com/mikaelpatel/Arduino-Scheduler
- BlueRobotics MS5837 Library, for getting readings from the Bar30 Depth Sensor - https://github.com/bluerobotics/BlueRobotics_MS5837_Library
- rosserial_noetic_serial and rosserial_noetic_serial_arduino, for connecting the Arduino to the ROS network - http://wiki.ros.org/rosserial_arduino 

## Authors
- Ivan Wei (ivanw8288@gmail.com)
- Ayan Chowdhury (ayanc@umich.edu)