#include <Servo.h>
#define NUM_MOTORS 8

#define STOP_WIDTH 1500
#define MAX_FWD_WIDTH 1900
#define MAX_BCK_WIDTH 1100

byte pins[8] = {4, 5, 6, 7, 8, 9, 10, 11};
Servo motors[8];

void setup() {
  
 Serial.begin(9600);
 for (int i = 0; i < NUM_MOTORS; i++) {
  motors[i].attach(pins[i]);
  motors[i].writeMicroseconds(STOP_WIDTH);
 }

 delay(1000); // delay to allow the ESC to recognize the stopped signal
}

void loop() {
  while (Serial.available() == 0);
  
  int val = Serial.parseInt();
  
  if(val < 1100 || val > 1900)
  {
    Serial.println("not valid");
  }
  else
  {
    servo.writeMicroseconds(val); // Send signal to ESC.
  }
}
