/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

#define NUM_MOTORS 8

#define STOP_WIDTH 1500
#define MAX_FWD_WIDTH 1900
#define MAX_BCK_WIDTH 1100

ros::NodeHandle  nh;

byte pins[NUM_MOTORS] = {4, 5, 6, 7, 8, 9, 10, 11};
Servo motors[NUM_MOTORS];

void motor_update_cb(const std_msgs::Int16MultiArray& cmd_msg) {
  for (int i = 0; i < NUM_MOTORS; i++) {
    int width = constrain(cmd_msg.data[i], MAX_BCK_WIDTH, MAX_FWD_WIDTH);
    motors[i].writeMicroseconds(width);
  }
}

ros::Subscriber<std_msgs::Int16MultiArray> sub("motor", motor_update_cb);

void setup() {
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);

  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].attach(pins[i]);
    motors[i].writeMicroseconds(STOP_WIDTH);
  }

  delay(1000); // delay to allow the ESC to recognize the stopped signal
}

void loop(){
  nh.spinOnce();
  delay(1);
}
