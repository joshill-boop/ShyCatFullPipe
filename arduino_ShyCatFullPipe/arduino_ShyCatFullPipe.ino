/*
  SHY CAT - 4x stepper controller (Arduino Nano / ATmega328P)
  Uses AccelStepper (install from Arduino IDE Library Manager).
  Non-blocking: call run() for all 4 motors every loop.

  Serial: 9600 baud, line protocol from Processing:
     A,<steps for A>,<steps for B>,<steps for C>,<steps for D>\n
  IMPORTANT WIRING CHANGE from your prior sketch:
  Old motor 3: 18,20,19,21  <-- A6/D20 and A7/D21 cannot be outputs on Nano.
  New motor 3: 18,11,19,12  <-- MOVE the two wires to D11 and D12.

  For the 28BYJ-48 + ULN2003 board, the constructor receives the four
  ARDUINO pins in the established project's IN1,IN3,IN2,IN4 order.

  CRITICAL: NO HOMING SWITCHES. At every power-on, POSITION ALL FOUR
  ARMS MANUALLY AT THEIR PHYSICAL CLOSED/HIDDEN ZERO POSITION before
  enabling power. Arduino assumes that starting position is step 0.
  Open travel and signs must be checked ONE ARM AT A TIME before use.

  Supply motors from a suitably rated external 5V source, not Nano USB.
  Connect external supply GND and Nano GND together.
*/

#include <AccelStepper.h>
#include <stdlib.h>

const uint8_t NUM_MOTORS = 4;
const uint32_t SERIAL_BAUD = 9600;

// The 4 constructors must match actual driver input wiring.
AccelStepper motors[NUM_MOTORS] = {
  AccelStepper(AccelStepper::FULL4WIRE, 6, 5, 3, 4),
  AccelStepper(AccelStepper::FULL4WIRE, 10, 8, 9, 7),
  AccelStepper(AccelStepper::FULL4WIRE, 18, 11, 19, 12),
  AccelStepper(AccelStepper::FULL4WIRE, 14, 16, 15, 17)
};

// Match Processing MAX_OPEN_STEPS and MOTOR_DIRECTION exactly.
const long MAX_OPEN_STEPS[NUM_MOTORS] = {512, 341, 683, 512};
const int8_t MOTOR_DIRECTION[NUM_MOTORS] = {-1, -1, +1, +1};

// Lower speeds than a one-motor test are deliberate: four motors run together.
const float MAX_SPEED_STEPS_PER_S = 280.0f;
const float ACCEL_STEPS_PER_S2 = 520.0f;

// On connection loss, drive every arm back towards closed (step zero).
const unsigned long COMMAND_TIMEOUT_MS = 1500;
unsigned long lastValidCommandAt = 0;

char serialBuffer[72];
uint8_t serialLength = 0;

long clampMotorTarget(uint8_t i, long target) {
  // Physical range, 0 to +/- configured open travel.
  if (MOTOR_DIRECTION[i] > 0) {
    if (target < 0) return 0;
    if (target > MAX_OPEN_STEPS[i]) return MAX_OPEN_STEPS[i];
  } else {
    if (target > 0) return 0;
    if (target < -MAX_OPEN_STEPS[i]) return -MAX_OPEN_STEPS[i];
  }
  return target;
}

void closeAllMotors() {
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    if (motors[i].targetPosition() != 0) motors[i].moveTo(0);
  }
}

void handleCommand(char *line) {
  if (line[0] == 'H' && line[1] == '\0') {
    closeAllMotors();
    lastValidCommandAt = millis();
    return;
  }
  if (line[0] != 'A' || line[1] != ',') return;

  long values[NUM_MOTORS];
  char *cursor = line + 2;
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    char *endPointer;
    long value = strtol(cursor, &endPointer, 10);
    if (cursor == endPointer) return; // not a number
    if (i < NUM_MOTORS - 1) {
      if (*endPointer != ',') return;
      cursor = endPointer + 1;
    } else {
      if (*endPointer != '\0') return;
    }
    values[i] = clampMotorTarget(i, value);
  }

  // Apply all four targets only after a COMPLETE valid packet.
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    if (motors[i].targetPosition() != values[i]) {
      motors[i].moveTo(values[i]);
    }
  }
  lastValidCommandAt = millis();
}

void readSerialLines() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      serialBuffer[serialLength] = '\0';
      handleCommand(serialBuffer);
      serialLength = 0;
    } else if (serialLength < sizeof(serialBuffer) - 1) {
      serialBuffer[serialLength++] = c;
    } else {
      // Drop an oversized packet rather than corrupting memory.
      serialLength = 0;
    }
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    motors[i].setMaxSpeed(MAX_SPEED_STEPS_PER_S);
    motors[i].setAcceleration(ACCEL_STEPS_PER_S2);
    motors[i].setCurrentPosition(0);  // See the warning: no physical homing!
    motors[i].moveTo(0);
    motors[i].enableOutputs();
  }
  lastValidCommandAt = millis();
  Serial.println(F("Shy Cat stepper controller READY; 4 motors; 9600 baud"));
}

void loop() {
  readSerialLines();

  // Fail closed on stopped/disconnected Processing commands.
  if (millis() - lastValidCommandAt > COMMAND_TIMEOUT_MS) {
    closeAllMotors();
  }

  // Do not use delay() or Stepper.step(): simultaneous movement requires run().
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    motors[i].run();
  }
}
