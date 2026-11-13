// IDE CPS Group 5 - "Shy Cat" - Arduino stepper actuation code:)

//This Arduino code was written by Josh Hill with guidance, technical help and debugging from ChatGPT.

#include <AccelStepper.h>

const int NUM_MOTORS = 4;
const int SERIAL_BAUD = 9600;

AccelStepper motors[NUM_MOTORS] = {
  AccelStepper(AccelStepper::FULL4WIRE, 6, 5, 3, 4),
  AccelStepper(AccelStepper::FULL4WIRE, 10, 8, 9, 7),
  AccelStepper(AccelStepper::FULL4WIRE, 18, 11, 19, 12),
  AccelStepper(AccelStepper::FULL4WIRE, 14, 16, 15, 17)
};

const long MAX_OPEN_STEPS[NUM_MOTORS] = {1000, 1000, 1000, 1000};
const int MOTOR_DIRECTION[NUM_MOTORS] = {-1, -1, 1, 1};

const float MOTOR_SPEED = 250;
const float MOTOR_ACCELERATION = 650;
const unsigned long TIMEOUT_MS = 1500;

unsigned long lastCommandTime = 0;

String incomingLine = "";
const int MAX_LINE_LENGTH = 71;
bool lineTooLong = false;

// Keep each stepper within its allowed opening range.
long limitMotorTarget(int motorNumber, long target) {
  if (MOTOR_DIRECTION[motorNumber] == 1) {
    if (target < 0) target = 0;
    if (target > MAX_OPEN_STEPS[motorNumber]) target = MAX_OPEN_STEPS[motorNumber];
  } else {
    if (target > 0) target = 0;
    if (target < -MAX_OPEN_STEPS[motorNumber]) target = -MAX_OPEN_STEPS[motorNumber];
  }
  return target;
}

void closeAllMotors() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].moveTo(0);
  }
}

// Check that a piece of text contains a whole number.
bool isWholeNumber(String text) {
  if (text.length() == 0) return false;

  int firstDigit = 0;
  if (text.charAt(0) == '-' || text.charAt(0) == '+') {
    firstDigit = 1;
  }
  if (firstDigit == text.length()) return false;

  for (int i = firstDigit; i < text.length(); i++) {
    if (text.charAt(i) < '0' || text.charAt(i) > '9') {
      return false;
    }
  }
  return true;
}

// Read the four requested motor positions and send them to the motors after checking that the command is valid.
void handleCommand(String command) {
  if (command == "H") {
    closeAllMotors();
    lastCommandTime = millis();
    return;
  }

  if (!command.startsWith("A,")) return;

  long targets[NUM_MOTORS];
  int start = 2;  // Skip the "A,"

  for (int i = 0; i < NUM_MOTORS; i++) {
    int comma = command.indexOf(',', start);
    String numberText;

    if (i < NUM_MOTORS - 1) {
      // The first three numbers must be followed by a comma.
      if (comma == -1) return;
      numberText = command.substring(start, comma);
      start = comma + 1;
    } else {
      // The fourth number must be the end of the command.
      if (comma != -1) return;
      numberText = command.substring(start);
    }

    if (!isWholeNumber(numberText)) return;

    // Limit to a safe number of digits before converting to long.
    // The motor targets only need values in the thousands.
    int digitCount = numberText.length();
    if (numberText.charAt(0) == '-' || numberText.charAt(0) == '+') {
      digitCount--;
    }
    if (digitCount > 9) return;

    long requestedTarget = numberText.toInt();
    targets[i] = limitMotorTarget(i, requestedTarget);
  }

  // Only update motors after all four numbers have been checked.
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].moveTo(targets[i]);
  }

  lastCommandTime = millis();
}

// Collect incoming characters until a complete command is received, then pass it to handleCommand().
void readCommands() {
  while (Serial.available() > 0) {
    char letter = Serial.read();

    if (letter == '\r') continue;

    if (letter == '\n') {
      if (!lineTooLong) handleCommand(incomingLine);
      incomingLine = "";
      lineTooLong = false;
    } else if (!lineTooLong) {
      if (incomingLine.length() < MAX_LINE_LENGTH) {
        incomingLine += letter;
      } else {
        // Ignore the rest of an oversized line until its newline.
        incomingLine = "";
        lineTooLong = true;
      }
    }
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);

  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].enableOutputs();
    motors[i].setCurrentPosition(0);
  }

  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].setMaxSpeed(MOTOR_SPEED);
    motors[i].setAcceleration(MOTOR_ACCELERATION);
    motors[i].moveTo(0);
  }

  incomingLine = "";
  lineTooLong = false;
  lastCommandTime = millis();
  Serial.println("SHY CAT READY");
}

void loop() {
  readCommands();

  if (millis() - lastCommandTime > TIMEOUT_MS) {
    closeAllMotors();
  }

  // run() must be called repeatedly for smooth stepper movement.
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].run();
  }
}
