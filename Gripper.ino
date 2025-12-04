// This code is for the ARDUINO MEGA2560

#include <Servo.h>

Servo myGripperServo;

int OPEN_POSITION = 20; // Angle for "open"
int CLOSE_POSITION = 2; // Angle for "close"

void setup() {
  Serial.begin(9600); 
  myGripperServo.attach(9);
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readString();
    command.trim();

    if (command == "close") {
      myGripperServo.write(CLOSE_POSITION); 
    } 
    else if (command == "open") {
      myGripperServo.write(OPEN_POSITION);
    }
  }
}
