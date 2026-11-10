
#include <AccelStepper.h>

const int num_steppers = 4;

// Motor speed in steps per second
const float SPEED = 200;

// Stepper objects
// Pin order: IN1, IN3, IN2, IN4

AccelStepper steppers[num_steppers] = {
  AccelStepper(AccelStepper::FULL4WIRE, 6, 5, 3, 4),
  AccelStepper(AccelStepper::FULL4WIRE, 10, 8, 9, 7),
  AccelStepper(AccelStepper::FULL4WIRE, 18, 20, 19, 21),
  AccelStepper(AccelStepper::FULL4WIRE, 14, 16, 15, 17)
};

// Stores the latest keyboard command
byte keyState = 0;

// Stop motors if Processing stops sending data
unsigned long lastMessage = 0;
const int TIMEOUT = 300;

void setup() {

  Serial.begin(9600);

  // Set the maximum speed of each motor
  for (int i = 0; i < num_steppers; i++) {
    steppers[i].setMaxSpeed(SPEED);
    steppers[i].setSpeed(0);
  }
}

void loop() {

  // Read the latest keyboard command from Processing
  while (Serial.available() > 0) {
    keyState = Serial.read();
    lastMessage = millis();
  }

  // Stop all motors if communication is lost
  if (millis() - lastMessage > TIMEOUT) {
    keyState = 0;
  }

  // Update each motor independently
  for (int i = 0; i < num_steppers; i++) {

    // Read the forward and backward commands
    bool forward = bitRead(keyState, i * 2);
    bool backward = bitRead(keyState, i * 2 + 1);

    // Set the motor direction and speed
    if (forward && !backward) {
      steppers[i].setSpeed(SPEED);
    }
    else if (backward && !forward) {
      steppers[i].setSpeed(-SPEED);
    }
    else {
      steppers[i].setSpeed(0);
    }

    // Move the motor if another step is needed
    steppers[i].runSpeed();
  }
}
