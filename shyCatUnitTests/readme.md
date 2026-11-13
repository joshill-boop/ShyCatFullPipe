<u>**ShyCat Unit Tests:**</u>

ShyCat_stepperTest1:

- Rotates each stepper motor through a defined amount one at a time.

ShyCat_stepperTest2:

- Rotates all 4 stepper motors through a defined angle simultaneously.

- This is not actually simultaneously because Stepper.h doesnt allow for that so i moves all 4 1 step and then repeats.

ShyCat_stepperTest3:

- Using AccelStepper.h we genuienly move all 4 steppers at the same time.

ShyCat_stepperTest3.2:

- Operates the same as "ShyCat_stepperTest3" but has the added functionality of being able to set the inital position of each stepper before the void loop(

  | Serial command | Action                                  |
  | -------------- | --------------------------------------- |
  | `a`            | Rotate selected motor backwards         |
  | `d`            | Rotate selected motor forwards          |
  | `s`            | Stop selected motor                     |
  | `n`            | Confirm position and move to next motor |

- Open the serial monitor (9600) and see instructions.

ShyCat_stepperTest4:

- Same as unit test 3 but you can control the speed of each stepper individually.

- Each stepper doesnt have to wait for all the steppers to finish moving before we start rotating in the opposite direction.

ShyCat_processingTest1:

- This allows you to test the connection between Processing and Arduino, by moving each motor/stepper independently with keys.

  - steppers 1, 2, 3, 4 - Q, W, E, R = anticlockwise respectively

  - steppers 1, 2, 3, 4 - A, S, D, F = clockwise respectively

- The steppers can be actuated simultaneously.
