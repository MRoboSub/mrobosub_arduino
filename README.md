# mrobosub_arduino
An Arduino sketch project for Michigan Robotic Submarine. A fairly modular system of running multiple setup-loop tasks in parallel on an Arduino Mega.

## Current Modules
- [DepthSensorSketch.hpp](main/DepthSensorSketch.hpp) - For getting readings from a Bar30 Depth/Pressure Sensor - namespace `DepthSensorSketch`
- [ServoSketch.hpp](main/ServoSketch.hpp) - For controlling a DSSERVO 35kg servo - namespace `ServoSketch`
- [ESCMotorSketch.hpp](main/ESCMotorSketch.hpp) - For controlling eight BlueRobotics ESC motors - namespace `ESCMotorSketch`

## Pins Used
- Depth Sensor - [DepthSensorSketch.hpp](main/DepthSensorSketch.hpp) - SDA, SCL
- Servo - [ServoSketch.hpp](main/ServoSketch.hpp) - Digital 12
- ESC Motors - [ESCMotorSketch.hpp](main/ESCMotorSketch.hpp) - Digital 4-11

## Publishers
- `/depth/raw_depth` - type `std_msgs::Float32` - the depth sensor reading, in meters - [DepthSensorSketch.hpp](main/DepthSensorSketch.hpp)

## Subscribers
- `/servo/angle` - type `std_msgs::Int32` - the angle to set the servo to - [ServoSketch.hpp](main/ServoSketch.hpp)
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

## Development Guidelines
ArduinoScheduler technically allows for any callbacks, but to be organized:

To add a new module, add a new namespace - preferably in a separate header file - and implement `setup` and `loop` in that namespace. 

WARNING: Only one ROS `NodeHandle` can exist, so include `GlobalNodeHandle.hpp` and use `Global::nh` when interacting with a `NodeHandle`. The node is also initialized at the start of `main.ino`'s setup, so you don't need to do that yourself.

Otherwise, porting from a separate sketch should not require many changes.

After you finish implementing, in `main.ino`, at the bottom of `setup`, add 
```cpp
Scheduler.start(Namespace::setup, Namespace::loop);
```

An example can be seen in [ArduinoDepthSketch.hpp](main/ArduinoDepthSketch.hpp)

## Tools Used
- arduino-cli, a CLI tool to allow development outside of Arduino IDE - https://github.com/arduino/arduino-cli
- ArduinoScheduler, for asynchronous operations on a single core Arduino - https://github.com/mikaelpatel/Arduino-Scheduler
- BlueRobotics MS5837 Library, for getting readings from the Bar30 Depth Sensor - https://github.com/bluerobotics/BlueRobotics_MS5837_Library
- rosserial_noetic_serial and rosserial_noetic_serial_arduino, for connecting the Arduino to the ROS network - http://wiki.ros.org/rosserial_arduino 

## Authors
- Ivan Wei (ivanw8288@gmail.com)
- Ayan Chowdhury (ayanc@umich.edu)