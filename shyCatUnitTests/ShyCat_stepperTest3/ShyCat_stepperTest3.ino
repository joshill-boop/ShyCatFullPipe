
#include <AccelStepper.h>

const int STEPS = 2048;
const int num_steppers = 4;

// Speed and acceleration in steps per second
const float MAX_SPEED = 400;
const float ACCELERATION = 200;

// Arduino pin assignments for each stepper
// Pin order: IN1, IN3, IN2, IN4

const int pins[num_steppers][4] = {
  {6, 5, 3, 4},       // Stepper 1
  {10, 8, 9, 7},      // Stepper 2
  {18, 20, 19, 21},   // Stepper 3
  {14, 16, 15, 17}    // Stepper 4
};

// Stepper objects used to control movement

AccelStepper steppers[num_steppers] = {
  AccelStepper(AccelStepper::FULL4WIRE, 6, 5, 3, 4),
  AccelStepper(AccelStepper::FULL4WIRE, 10, 8, 9, 7),
  AccelStepper(AccelStepper::FULL4WIRE, 18, 20, 19, 21),
  AccelStepper(AccelStepper::FULL4WIRE, 14, 16, 15, 17)
};

void setup() {

  // Set speed and acceleration for all steppers
  for (int i = 0; i < num_steppers; i++) {
    steppers[i].setMaxSpeed(MAX_SPEED);
    steppers[i].setAcceleration(ACCELERATION);
  }
}

void loop() {

  // Move all four steppers forwards by 180 degrees
  for (int i = 0; i < num_steppers; i++) {
    steppers[i].moveTo(STEPS / 2);
  }

  // Run all steppers until they reach their targets
  bool moving = true;

  while (moving) {
    moving = false;

    for (int i = 0; i < num_steppers; i++) {
      if (steppers[i].distanceToGo() != 0) {
        steppers[i].run();
        moving = true;
      }
    }
  }

  delay(1000);

  // Move all four steppers backwards to their starting positions
  for (int i = 0; i < num_steppers; i++) {
    steppers[i].moveTo(0);
  }

  // Run all steppers until they reach their targets
  moving = true;

  while (moving) {
    moving = false;

    for (int i = 0; i < num_steppers; i++) {
      if (steppers[i].distanceToGo() != 0) {
        steppers[i].run();
        moving = true;
      }
    }
  }

  delay(1000);
}
