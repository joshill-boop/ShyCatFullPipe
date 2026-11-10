#include <Stepper.h>

const int STEPS = 2048;

//sets the speed of the stepper
const int RPM = 20;
const int num_steppers = 4;

// Arduino pin assignments for each steppers
// Used to release the steppers coils
// Pin order: IN1, IN3, IN2, IN4

const int pins[num_steppers][4] = {
  {6, 5, 3, 4},       // steppers 1
  {10, 8, 9, 7},      // steppers 2
  {18, 20, 19, 21},   // steppers 3
  {14, 16, 15, 17}    // steppers 4
};

// Stepper objects used to control steppers movement

Stepper steppers[num_steppers] = {
  Stepper(STEPS, 6, 5, 3, 4),
  Stepper(STEPS, 10, 8, 9, 7),
  Stepper(STEPS, 18, 20, 19, 21),
  Stepper(STEPS, 14, 16, 15, 17)
};

void setup() {

}

void loop() {
  // Set the speed of every steppers
  for (int i = 0; i < num_steppers; i++) {
    steppers[i].setSpeed(RPM);
  }

  delay(2000);

  // Run each steppers sequentially
  for (int i = 0; i < num_steppers; i++) {

    // Rotate approximately 180 degrees
    steppers[i].step(STEPS / 2);

    // Release steppers coils
    for (int j = 0; j < 4; j++) {
      digitalWrite(pins[i][j], LOW);
    }
  }
}
