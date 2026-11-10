/*
  SHY CAT - 4x STEPPER CONTROLLER
  Arduino Nano / ATmega328P
  Uses AccelStepper library

  ========================================
  STARTUP CALIBRATION
  ========================================

  On every restart, calibrate each motor
  sequentially before normal operation.

  Controls from Processing or Serial Monitor:

    A = Rotate backwards continuously
    D = Rotate forwards continuously
    S = Stop motor
    N = Set current position as zero and
        continue to next motor

  Once all four motors are calibrated,
  normal operation begins automatically.

  ========================================
  NORMAL OPERATION
  ========================================

  Processing sends:

    A,stepA,stepB,stepC,stepD\n

  Example:
    A,-100,-200,150,250

  H = Return all motors to zero

  Serial baud: 9600

  ========================================
  IMPORTANT
  ========================================

  No physical homing sensors are fitted.

  Manual calibration establishes the
  physical closed position as step zero.

  Calibration has no physical travel limits.
  Stop motors before reaching mechanical stops.

  Motor power must come from an appropriately
  rated external 5V supply, not Nano USB.

  Connect external supply GND to Nano GND.
*/


#include <AccelStepper.h>
#include <stdlib.h>


// ==================================================
// MOTOR CONFIGURATION
// ==================================================

const uint8_t NUM_MOTORS = 4;

const uint32_t SERIAL_BAUD = 9600;


// Motor pin order: IN1, IN3, IN2, IN4

AccelStepper motors[NUM_MOTORS] = {

  AccelStepper(AccelStepper::FULL4WIRE, 6, 5, 3, 4),

  AccelStepper(AccelStepper::FULL4WIRE, 10, 8, 9, 7),

  AccelStepper(AccelStepper::FULL4WIRE, 18, 11, 19, 12),

  AccelStepper(AccelStepper::FULL4WIRE, 14, 16, 15, 17)

};


// Maximum opening travel for each motor

const long MAX_OPEN_STEPS[NUM_MOTORS] = {
  512, 341, 683, 512
};


// Direction of opening

const int8_t MOTOR_DIRECTION[NUM_MOTORS] = {
  -1, -1, +1, +1
};


// ==================================================
// NORMAL MOVEMENT SETTINGS
// ==================================================

const float MAX_SPEED_STEPS_PER_S = 280.0f;

const float ACCEL_STEPS_PER_S2 = 520.0f;


// ==================================================
// CALIBRATION SETTINGS
// ==================================================

const float CALIBRATION_SPEED = 120.0f;


// ==================================================
// SERIAL TIMEOUT
// ==================================================

const unsigned long COMMAND_TIMEOUT_MS = 1500;

unsigned long lastValidCommandAt = 0;


// ==================================================
// SERIAL BUFFER
// ==================================================

char serialBuffer[72];

uint8_t serialLength = 0;


// ==================================================
// STARTUP CALIBRATION
// ==================================================

void calibrateSteppers() {

  Serial.println(F("================================"));
  Serial.println(F("SHY CAT - MOTOR CALIBRATION"));
  Serial.println(F("================================"));

  Serial.println(F("A = backward"));
  Serial.println(F("D = forward"));
  Serial.println(F("S = stop"));
  Serial.println(F("N = next motor / set zero"));

  Serial.println();


  // Calibrate each motor sequentially

  for (uint8_t i = 0; i < NUM_MOTORS; i++) {

    Serial.print(F("CALIBRATING MOTOR "));
    Serial.println(i + 1);


    motors[i].setMaxSpeed(CALIBRATION_SPEED);

    motors[i].setSpeed(0);


    bool finished = false;


    while (!finished) {

      // Continuously run selected motor
      // at the commanded calibration speed

      motors[i].runSpeed();


      // Read commands from Processing

      if (Serial.available() > 0) {

        char command = Serial.read();


        // BACKWARDS

        if (command == 'a' || command == 'A') {

          motors[i].setSpeed(-CALIBRATION_SPEED);

          Serial.println(F("BACKWARD"));
        }


        // FORWARDS

        else if (command == 'd' || command == 'D') {

          motors[i].setSpeed(CALIBRATION_SPEED);

          Serial.println(F("FORWARD"));
        }


        // STOP

        else if (command == 's' || command == 'S') {

          motors[i].setSpeed(0);

          Serial.println(F("STOPPED"));
        }


        // NEXT MOTOR / SET ZERO

        else if (command == 'n' || command == 'N') {

          // Stop movement
          motors[i].setSpeed(0);

          // Define current physical position as zero
          motors[i].setCurrentPosition(0);

          finished = true;


          Serial.print(F("MOTOR "));
          Serial.print(i + 1);
          Serial.println(F(" ZERO SET"));

          Serial.println();
        }
      }
    }
  }


  Serial.println(F("================================"));
  Serial.println(F("ALL FOUR MOTORS CALIBRATED"));
  Serial.println(F("================================"));
}


// ==================================================
// CLAMP MOTOR TARGETS
// ==================================================

long clampMotorTarget(uint8_t i, long target) {

  // Limit each motor to its configured
  // opening direction and travel

  if (MOTOR_DIRECTION[i] > 0) {

    if (target < 0) return 0;

    if (target > MAX_OPEN_STEPS[i]) {
      return MAX_OPEN_STEPS[i];
    }

  } else {

    if (target > 0) return 0;

    if (target < -MAX_OPEN_STEPS[i]) {
      return -MAX_OPEN_STEPS[i];
    }
  }

  return target;
}


// ==================================================
// CLOSE ALL MOTORS
// ==================================================

void closeAllMotors() {

  for (uint8_t i = 0; i < NUM_MOTORS; i++) {

    if (motors[i].targetPosition() != 0) {

      motors[i].moveTo(0);
    }
  }
}


// ==================================================
// HANDLE SERIAL COMMANDS
// ==================================================

void handleCommand(char *line) {


  // ------------------------------------------------
  // HOME / CLOSE ALL MOTORS
  // ------------------------------------------------

  if (line[0] == 'H' && line[1] == '\0') {

    closeAllMotors();

    lastValidCommandAt = millis();

    return;
  }


  // ------------------------------------------------
  // NORMAL MOTOR POSITION COMMAND
  // ------------------------------------------------

  // Expected format:
  // A,stepA,stepB,stepC,stepD

  if (line[0] != 'A' || line[1] != ',') {

    return;
  }


  long values[NUM_MOTORS];

  char *cursor = line + 2;


  for (uint8_t i = 0; i < NUM_MOTORS; i++) {

    char *endPointer;

    long value = strtol(
      cursor,
      &endPointer,
      10
    );


    // Reject invalid numbers

    if (cursor == endPointer) {

      return;
    }


    // Check comma separators

    if (i < NUM_MOTORS - 1) {

      if (*endPointer != ',') {

        return;
      }

      cursor = endPointer + 1;

    } else {

      // Last number must end the packet

      if (*endPointer != '\0') {

        return;
      }
    }


    // Apply physical movement limits

    values[i] = clampMotorTarget(
      i,
      value
    );
  }


  // ------------------------------------------------
  // APPLY ALL MOTOR TARGETS
  // ------------------------------------------------

  // Only update after receiving a
  // complete valid packet

  for (uint8_t i = 0; i < NUM_MOTORS; i++) {

    if (motors[i].targetPosition() != values[i]) {

      motors[i].moveTo(values[i]);
    }
  }


  lastValidCommandAt = millis();
}


// ==================================================
// READ SERIAL LINES
// ==================================================

void readSerialLines() {

  while (Serial.available() > 0) {

    char c = (char)Serial.read();


    // Ignore carriage returns

    if (c == '\r') {

      continue;
    }


    // End of command

    if (c == '\n') {

      serialBuffer[serialLength] = '\0';

      handleCommand(serialBuffer);

      serialLength = 0;

    }


    // Add character to buffer

    else if (
      serialLength < sizeof(serialBuffer) - 1
    ) {

      serialBuffer[serialLength++] = c;

    }


    // Prevent buffer overflow

    else {

      serialLength = 0;
    }
  }
}


// ==================================================
// SETUP
// ==================================================

void setup() {

  Serial.begin(SERIAL_BAUD);


  // ----------------------------------
  // ENABLE ALL STEPPER MOTORS
  // ----------------------------------

  for (uint8_t i = 0; i < NUM_MOTORS; i++) {

    motors[i].enableOutputs();

    // Temporary position before calibration

    motors[i].setCurrentPosition(0);
  }


  // ----------------------------------
  // MANUAL CALIBRATION
  // ----------------------------------

  // This blocks normal operation until
  // all four motors have been calibrated.

  calibrateSteppers();


  // ----------------------------------
  // RESTORE NORMAL MOVEMENT SETTINGS
  // ----------------------------------

  for (uint8_t i = 0; i < NUM_MOTORS; i++) {

    motors[i].setMaxSpeed(
      MAX_SPEED_STEPS_PER_S
    );

    motors[i].setAcceleration(
      ACCEL_STEPS_PER_S2
    );

    motors[i].moveTo(0);
  }


  // ----------------------------------
  // CLEAR CALIBRATION SERIAL INPUT
  // ----------------------------------

  while (Serial.available() > 0) {

    Serial.read();
  }

  serialLength = 0;


  // Start timeout timer

  lastValidCommandAt = millis();


  Serial.println(F("================================"));
  Serial.println(F("SHY CAT CONTROLLER READY"));
  Serial.println(F("NORMAL OPERATION ACTIVE"));
  Serial.println(F("================================"));
}


// ==================================================
// MAIN LOOP
// ==================================================

void loop() {


  // ----------------------------------
  // READ PROCESSING COMMANDS
  // ----------------------------------

  readSerialLines();
  // ----------------------------------
  // CONNECTION TIMEOUT
  // ----------------------------------

  // If Processing stops sending commands,
  // automatically return all motors to zero.

  if (
    millis() - lastValidCommandAt >
    COMMAND_TIMEOUT_MS
  ) {

    closeAllMotors();
  }


  // ----------------------------------
  // RUN ALL FOUR MOTORS
  // ----------------------------------

  // AccelStepper requires run() to be
  // called continuously for every motor.
  // This allows simultaneous movement.

  for (uint8_t i = 0; i < NUM_MOTORS; i++) {

    motors[i].run();
  }
}