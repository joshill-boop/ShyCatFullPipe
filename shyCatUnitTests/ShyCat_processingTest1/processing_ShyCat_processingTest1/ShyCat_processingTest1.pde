
import processing.serial.*;

Serial port;

final int num_steppers = 4;

// Keyboard controls for each motor
char[] forwardKeys = {'q', 'w', 'e', 'r'};
char[] backwardKeys = {'a', 's', 'd', 'f'};

// Track which keys are currently held
boolean[] keysDown = new boolean[8];

// Serial update interval
int sendInterval = 50;
int lastSent = 0;

void setup() {

  size(640, 360);

  // Show available serial ports
  println("Available serial ports:");
  printArray(Serial.list());

  // Connect to Arduino
  port = new Serial(this, "/dev/cu.usbmodem101", 9600);

  // Set text display
  textAlign(CENTER, CENTER);
}

void draw() {

  background(30);

  // Display title
  fill(255);
  textSize(24);
  text("Stepper Motor Control", width / 2, 40);

  // Display each motor's current state
  for (int i = 0; i < num_steppers; i++) {

    int x = 80 + i * 160;
    int y = 150;

    boolean forward = keysDown[i * 2];
    boolean backward = keysDown[i * 2 + 1];

    // Choose colour based on movement
    if (forward && !backward) {
      fill(50, 200, 100);
    }
    else if (backward && !forward) {
      fill(220, 100, 50);
    }
    else {
      fill(100);
    }

    // Draw motor indicator
    ellipse(x, y, 80, 80);

    // Display motor number
    fill(255);
    textSize(18);
    text("Motor " + (i + 1), x, y - 70);

    // Display movement direction
    textSize(14);

    if (forward && !backward) {
      text("FORWARD", x, y + 65);
    }
    else if (backward && !forward) {
      text("BACKWARD", x, y + 65);
    }
    else {
      text("STOPPED", x, y + 65);
    }

    // Display keyboard controls
    textSize(14);
    text(
      forwardKeys[i] + " / " + backwardKeys[i],
      x, y + 95
    );
  }

  // Send keyboard state to Arduino regularly
  if (millis() - lastSent >= sendInterval) {
    sendControls();
    lastSent = millis();
  }
}

// Detect when a key is pressed
void keyPressed() {

  updateKey(key, true);
}

// Detect when a key is released
void keyReleased() {

  updateKey(key, false);
}

// Update the stored keyboard state
void updateKey(char k, boolean pressed) {

  k = Character.toLowerCase(k);

  for (int i = 0; i < num_steppers; i++) {

    if (k == forwardKeys[i]) {
      keysDown[i * 2] = pressed;
    }

    if (k == backwardKeys[i]) {
      keysDown[i * 2 + 1] = pressed;
    }
  }

  // Send immediately when a key changes
  sendControls();
}

// Send all eight key states as one byte
void sendControls() {

  int command = 0;

  for (int i = 0; i < 8; i++) {

    if (keysDown[i]) {
      command |= (1 << i);
    }
  }

  port.write(command);
}
