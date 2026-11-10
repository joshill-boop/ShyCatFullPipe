
#include <AccelStepper.h>

const int STEPS = 2048;
const int num_steppers = 4;

// Maximum speed of each motor (steps per second)
const float speeds[num_steppers] = {
  50,    // stepper 1
  50,    // stepper 2
  50,    // stepper 3
  400     // stepper 4
};

// Acceleration of each motor (steps per second squared)
const float ACCELERATION = 200;

// Stepper objects
// Pin order: IN1, IN3, IN2, IN4

AccelStepper steppers[num_steppers] = {
  AccelStepper(AccelStepper::FULL4WIRE, 6, 5, 3, 4),
  AccelStepper(AccelStepper::FULL4WIRE, 10, 8, 9, 7),
  AccelStepper(AccelStepper::FULL4WIRE, 18, 20, 19, 21),
  AccelStepper(AccelStepper::FULL4WIRE, 14, 16, 15, 17)
};

// Target position for each motor
long targets[num_steppers];

void setup() {

  // Set the speed and acceleration of each motor
  for (int i = 0; i < num_steppers; i++) {
    steppers[i].setMaxSpeed(speeds[i]);
    steppers[i].setAcceleration(ACCELERATION);

    // Set the initial target to 180 degrees
    targets[i] = STEPS / 2;
    steppers[i].moveTo(targets[i]);
  }
}

void loop() {

  // Update each motor independently
  for (int i = 0; i < num_steppers; i++) {

    // Move the motor if another step is needed
    steppers[i].run();

    // Reverse direction when the motor reaches its target
    if (steppers[i].distanceToGo() == 0) {

      // Switch between 0 and 180 degrees
      if (targets[i] == STEPS / 2) {
        targets[i] = 0;
      }
      else {
        targets[i] = STEPS / 2;
      }

      // Set the new target position
      steppers[i].moveTo(targets[i]);
    }
  }
}
