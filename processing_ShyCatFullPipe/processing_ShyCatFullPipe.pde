import oscP5.*;
import netP5.*;
import processing.serial.*;

// communication ports
final int WEK_OSC_PORT = 12000;
final int PERSON_OSC_PORT = 12001;
final String ARDUINO_PORT = "/dev/cu.usbmodem1401";
final int ARDUINO_BAUD = 9600;
final int SERIAL_SEND_INTERVAL_MS = 50;


// change these values to fine tune the behaviour of the sculpture

// MUST MATCH ARDUINO - determines how many steps from fully closed to fully open (steppers 1, 2, 3, 4)
final int[] MAX_OPEN_STEPS = {512, 341, 683, 512};
final int[] MOTOR_DIRECTION = {-1, -1, +1, +1}; // -1 = anticlockwise | +1 = clockwise

final float TRUST_GAIN_PER_SECOND = 0.5;
final float TRUST_LOSS_PER_SECOND = 0.40;
final float TRUST_DECAY_UNOBSERVED = 0.04;
final float MIMIC_START_TRUST = 0.75;
final float MIMIC_FULL_TRUST = 0.95;
// Wrist Y normalisation: units are shoulder widths from shoulder centre.
// -1.4 = hand well below shoulder; +1.4 = hand well above shoulder.
final float MIMIC_VERTICAL_LOW = -1.4;
final float MIMIC_VERTICAL_HIGH = 1.4;
// Horizontal wrist reach, measured OUTWARD from each estimated shoulder.
// Units: shoulder widths. At MIN the wrist is close to its own shoulder;
// at MAX the arm is naturally extended sideways.
// Decrease MAX to reach full opening with less extension; increase for more.
final float MIMIC_HORIZONTAL_MIN = 0.0;
final float MIMIC_HORIZONTAL_MAX = 1.25;
// Set true when input image X is mirrored relative to anatomical left/right.
// If outward arm extension produces no reach, toggle this switch.
final boolean MIRRORED_WRIST_X = true;

final float STARTLE_DURATION_S = 1.3;
final float THREAT_STARTLE_THRESHOLD = 0.72;
final float IDLE_OPENNESS = 0.66;  // no person: timidly open, wandering
final float IDLE_WANDER_AMPLITUDE = 0.14;
final float PERSON_MIN_OPENNESS = 0.17;
final float PERSON_MAX_OPENNESS = 0.92;
final float INPUT_TIMEOUT_MS = 750;
final float WEK_TIMEOUT_MS = 1300;

OscP5 wekOsc;
OscP5 personOsc;
Serial arduino;
boolean serialEnabled = false; // hardware motion ARMED, not merely connected
boolean arduinoReady = false; // becomes true only after startup homing
int calibratingMotor = 0; // 0 until first Arduino status, then 1..4
String arduinoStatus = "NOT CONNECTED";
int lastSerialSendMs = -100000;

// communication variables to help wekinator and python talk to processing
final Object networkLock = new Object();
// First 12 features remain identical to Wekinator training inputs.
// Feature 13 (index 12) is shoulder-centre Y, sent ONLY to Processing.
float[] poseFeatures = new float[13];
boolean shoulderYAvailable = false;
boolean rawPersonPresent = false;
int lastPoseTrueMs = -100000;
int lastWekMs = -100000;
float rawThreat = 0;
float rawEngagement = 0;

// current state variables | DONT CHANGE!
float threat = 0;
float engagement = 0;
float trust = 0;
float fear = 0;
float mimicStrength = 0;
float startleRemaining = 0;
float startleCooldown = 0;
float previousThreat = 0;

boolean wasPresent = false;
boolean emergencyClosed = false;

float[] openness = {0, 0, 0, 0};
float[] targetOpenness = new float[4];
float[] visualAngle = new float[4];
int[] stepTargets = new int[4];

String behaviouralState = "IDLE";
int previousDrawMs = -1;

// ---------- HAND TESTING (without Python/Wekinator) ----------
boolean testMode = false;
boolean testPersonPresent = false;
float testThreat = 0.05;
float testEngagement = 0.85;
float[] testFeatures = {
  .75, .72, .25, .72, .50, .23,
  .12, .12, .10, .05, .05, .50,
  .36 // shoulder-centre Y (13th feature, for body-relative wrist height)
};

// ---------- DRAWING THE ORIGINAL FOUR-PIVOT DIGITAL TWIN ----------
final String[] PIVOT_LABEL = {"A", "B", "C", "D"};
final float[] PIVOT_X = {.19, .39, .61, .81};
final float[] CLOSED_ANGLE_DEG = {270, 270, 270, 270};
final float[] OPEN_ANGLE_DEG = {-48, -18, 18, 48};
final float[] ARM_LENGTH = {185, 160, 160, 185};

void settings() {
  size(1200, 800);
  smooth(4);
}

void setup() {
  frameRate(60);
  textFont(createFont("SansSerif", 15));

  wekOsc = new OscP5(this, WEK_OSC_PORT);
  personOsc = new OscP5(this, PERSON_OSC_PORT);

  println("Shy Cat listening: Wekinator OSC :" + WEK_OSC_PORT
    + ", Python OSC :" + PERSON_OSC_PORT);
  println("Available serial ports:");
  printArray(Serial.list());
  println("Hardware DISARMED. Press B to connect and calibrate; then S to arm.");
}

void oscEvent(OscMessage message) {
  synchronized (networkLock) {
    if (message.checkAddrPattern("/wek/outputs")) {
      if (message.arguments().length >= 2) {
        try {
          rawThreat = constrain(message.get(0).floatValue(), 0, 1);
          rawEngagement = constrain(message.get(1).floatValue(), 0, 1);
          lastWekMs = millis();
        } catch (Exception ex) {
          println("Wekinator outputs must be TWO floating-point values: Threat, Engagement.");
        }
      }
    } else if (message.checkAddrPattern("/person/present")) {
      try {
        int present = message.get(0).intValue();
        rawPersonPresent = present != 0;
        if (rawPersonPresent) lastPoseTrueMs = millis();
      } catch (Exception ex) {
        println("/person/present must be an integer 0 or 1.");
      }
    } else if (message.checkAddrPattern("/person/features")) {
      if (message.arguments().length >= 12) {
        for (int i = 0; i < 12; i++) {
          poseFeatures[i] = constrain(message.get(i).floatValue(), 0, 1);
        }
        // Old 12-value Python messages remain usable (camera-relative Y fallback).
        shoulderYAvailable = message.arguments().length >= 13;
        if (shoulderYAvailable) {
          poseFeatures[12] = constrain(message.get(12).floatValue(), 0, 1);
        }
      }
    }
  }
}

void draw() {
  int now = millis();
  float dt = previousDrawMs < 0 ? 1.0/60 : constrain((now - previousDrawMs) / 1000.0, .001, .05);
  previousDrawMs = now;

  float[] f = new float[13];
  boolean present;
  boolean wekFresh;
  float currentThreat, currentEngagement;

  synchronized (networkLock) {
    if (testMode) {
      arrayCopy(testFeatures, f);
      present = testPersonPresent;
      wekFresh = true;
      currentThreat = testThreat;
      currentEngagement = testEngagement;
    } else {
      arrayCopy(poseFeatures, f);
      // Short dropout tolerance, but no permanent latched detection.
      present = (now - lastPoseTrueMs < INPUT_TIMEOUT_MS);
      wekFresh = (now - lastWekMs < WEK_TIMEOUT_MS);
      currentThreat = wekFresh ? rawThreat : 1.0; // fail closed if Wekinator stops
      currentEngagement = wekFresh ? rawEngagement : 0.0;
    }
  }

  // Read Arduino's homing status on the existing serial connection.
  readArduinoStatus();
  updateBehaviour(dt, now, present, wekFresh, currentThreat, currentEngagement, f);
  drawTwin(present, wekFresh, f);
  if (serialEnabled && arduinoReady && now - lastSerialSendMs >= SERIAL_SEND_INTERVAL_MS) {
    sendToArduino();
    lastSerialSendMs = now;
  }
}

void updateBehaviour(float dt, int now, boolean present, boolean wekFresh,
                     float incomingThreat, float incomingEngagement, float[] f) {

  if (present && !wasPresent) {
    startleRemaining = STARTLE_DURATION_S;
    fear = 1.0;
  }
  wasPresent = present;
  startleRemaining = max(0, startleRemaining - dt);
  startleCooldown = max(0, startleCooldown - dt);

  // Raw rapid threat can startle even before filtered HUD numbers catch up.
  if (present && wekFresh && incomingThreat > THREAT_STARTLE_THRESHOLD
      && previousThreat <= THREAT_STARTLE_THRESHOLD && startleCooldown <= 0) {
    startleRemaining = max(startleRemaining, STARTLE_DURATION_S);
    startleCooldown = 1.5;
    trust = max(0, trust - .20);
  }
  previousThreat = incomingThreat;

  float inAlpha = 1.0 - exp(-dt * 5.5);
  threat += (incomingThreat - threat) * inAlpha;
  engagement += (incomingEngagement - engagement) * inAlpha;

  // Trust only increases via the Wekinator-learned engagement signal.
  if (!present) {
    trust = max(0, trust - TRUST_DECAY_UNOBSERVED * dt);
  } else if (!wekFresh) {
    trust = max(0, trust - 0.10 * dt);
  } else if (startleRemaining <= 0) {
    if (incomingThreat >= 0.42) {
      trust = max(0, trust - TRUST_LOSS_PER_SECOND * incomingThreat * dt);
    } else {
      float positiveEngagement = max(0, (incomingEngagement - .20) / .80);
      float gain = TRUST_GAIN_PER_SECOND * positiveEngagement * (1.0 - incomingThreat);
      float erosion = 0.015 * (1.0 - positiveEngagement);
      trust = constrain(trust + (gain - erosion) * dt, 0, 1);
    }
  }

  float wantedFear = 0;
  if (present) {
    if (!wekFresh || startleRemaining > 0) wantedFear = 1;
    else wantedFear = constrain(map(incomingThreat, .35, .85, 0, 1), 0, 1);
  }
  float fearRate = (wantedFear > fear) ? 6.8 : 0.90;
  fear += (wantedFear - fear) * (1 - exp(-dt * fearRate));

  // Mimicking smoothly starts at 75% trust; full strength at 95%.
  float desiredMimic = (present && wekFresh) ? smoothstep(MIMIC_START_TRUST, MIMIC_FULL_TRUST, trust) : 0;
  desiredMimic *= (1 - fear);
  mimicStrength += (desiredMimic - mimicStrength) * (1 - exp(-dt * 2.3));

  float[] mimic = calculateMimic(f);
  float engagedOpen = lerp(PERSON_MIN_OPENNESS, PERSON_MAX_OPENNESS, trust);

  for (int i = 0; i < 4; i++) {
    // Noise creates independent, nonrepeating, slowly varying wander.
    float wandering = (noise(10*i + now*.00029) - .5) * 2 * IDLE_WANDER_AMPLITUDE;
    float secondary = sin(now*.00064 + i*1.7) * .035;
    float idle = constrain(IDLE_OPENNESS + wandering + secondary, .28, .90);

    float base = present ? engagedOpen + wandering*.38 : idle;
    float combined = lerp(base, mimic[i], mimicStrength);
    if (present) combined = lerp(combined, 0.015, fear);
    if (emergencyClosed) combined = 0;
    targetOpenness[i] = constrain(combined, 0, 1);

    // Retract decisively; approach trust/open more slowly.
    float rate = targetOpenness[i] < openness[i] ? 5.8 : 1.55;
    openness[i] += (targetOpenness[i] - openness[i]) * (1.0 - exp(-dt * rate));
    openness[i] = constrain(openness[i], 0, 1);

    visualAngle[i] = lerp(CLOSED_ANGLE_DEG[i], OPEN_ANGLE_DEG[i], openness[i]);
    stepTargets[i] = round(MOTOR_DIRECTION[i] * MAX_OPEN_STEPS[i] * openness[i]);
  }

  if (emergencyClosed) behaviouralState = "MANUAL CLOSE";
  else if (!present) behaviouralState = "IDLE / WANDERING";
  else if (!wekFresh) behaviouralState = "ML SIGNAL LOST - CLOSING";
  else if (fear > .45 || startleRemaining > 0) behaviouralState = "FRIGHTENED / HIDING";
  else if (mimicStrength > .80) behaviouralState = "MIMICKING";
  else if (mimicStrength > .05) behaviouralState = "OPENING / LEARNING TO MIMIC";
  else if (trust < .20) behaviouralState = "CAUTIOUS";
  else behaviouralState = "ENGAGING / BUILDING TRUST";
}

float[] calculateMimic(float[] f) {
  // Features: left wrist x/y = 0/1, right wrist x/y = 2/3,
  // torso centre X = 4, shoulder width = 5.
  float torsoX = f[4];
  float shoulderW = max(.08, f[5]);
  // Estimate each shoulder from torso centre and apparent shoulder width.
  // Measure wrists OUTWARD from their own shoulders, not across the torso.
  // Dividing by shoulder width compensates for apparent body size in camera.
  // Camera feeds differ: a mirrored feed places the person's LEFT
  // shoulder to the right of screen centre. Keep anatomical wrist labels.
  float leftShoulderX = torsoX + (MIRRORED_WRIST_X ? +0.5 : -0.5) * shoulderW;
  float rightShoulderX = torsoX + (MIRRORED_WRIST_X ? -0.5 : +0.5) * shoulderW;
  float leftOutward = (MIRRORED_WRIST_X ? f[0] - leftShoulderX : leftShoulderX - f[0]) / shoulderW;
  float rightOutward = (MIRRORED_WRIST_X ? rightShoulderX - f[2] : f[2] - rightShoulderX) / shoulderW;
  float leftReach = constrain(map(leftOutward, MIMIC_HORIZONTAL_MIN, MIMIC_HORIZONTAL_MAX, 0, 1), 0, 1);
  float rightReach = constrain(map(rightOutward, MIMIC_HORIZONTAL_MIN, MIMIC_HORIZONTAL_MAX, 0, 1), 0, 1);
   // Height relative to the shoulder centre, divided by apparent shoulder width.
   // This compensates for movement up/down in frame and distance to camera.
   // Only activate for 13-value OSC messages (or hand-test mode).
   float leftHeight, rightHeight;
   if (testMode || shoulderYAvailable) {
     float shoulderY = f[12];
     float leftVertical = (shoulderY - f[1]) / shoulderW;
     float rightVertical = (shoulderY - f[3]) / shoulderW;
     leftHeight = constrain(map(leftVertical, MIMIC_VERTICAL_LOW, MIMIC_VERTICAL_HIGH, 0, 1), 0, 1);
     rightHeight = constrain(map(rightVertical, MIMIC_VERTICAL_LOW, MIMIC_VERTICAL_HIGH, 0, 1), 0, 1);
   } else {
     // Backwards-compatible fallback until Python sends shoulder-centre Y.
     leftHeight = constrain(map(f[1], .83, .13, 0, 1), 0, 1);
     rightHeight = constrain(map(f[3], .83, .13, 0, 1), 0, 1);
   }
  // A/B mirror the left hand's horizontal/vertical motion;
  // C/D mirror the right hand's vertical/horizontal motion.
  return new float[] {
    lerp(.10, .97, leftReach),
    lerp(.10, .97, leftHeight),
    lerp(.10, .97, rightHeight),
    lerp(.10, .97, rightReach)
  };
}

float smoothstep(float e0, float e1, float x) {
  float t = constrain((x - e0) / (e1 - e0), 0, 1);
  return t*t*(3 - 2*t);
}

void drawTwin(boolean present, boolean wekFresh, float[] f) {
  background(14, 20, 29);
  fill(240);
  textSize(27);
  text("SHY CAT  /  UNIFIED DIGITAL TWIN", 36, 49);
  textSize(16);
  fill(162, 190, 202);
  text(behaviouralState, 38, 78);
  textSize(13);
  text(testMode ? "MANUAL TEST MODE" : "LIVE CAMERA / WEKINATOR", 950, 42);
  fill(serialEnabled ? color(255, 170, 85) : color(138, 160, 170));
  String hwLabel = (arduino == null) ? "ARDUINO: DISCONNECTED"
      : (!arduinoReady ? "ARDUINO: CALIBRATION"
      : (serialEnabled ? "ARDUINO: ARMED" : "ARDUINO: DISARMED"));
  text(hwLabel, 950, 67);

  drawMeter("THREAT", threat, 38, 105, color(230, 101, 98));
  drawMeter("ENGAGEMENT", engagement, 331, 105, color(75, 173, 232));
  drawMeter("TRUST", trust, 624, 105, color(100, 213, 145));
  drawMeter("MIMIC", mimicStrength, 917, 105, color(239, 187, 88));

  // Faint floor and anchor baseline, representing the original pivot drawing.
  float pivotY = height * .64;
  stroke(49, 77, 91);
  strokeWeight(2);
  line(70, pivotY, width - 70, pivotY);
  noStroke();
  fill(27, 36, 49);
  rect(90, pivotY + 12, width - 180, 26, 9);

  for (int i = 0; i < 4; i++) {
    float px = PIVOT_X[i] * width;
    float py = pivotY;
    float a = radians(visualAngle[i]);
    float ex = px + sin(a) * ARM_LENGTH[i];
    float ey = py - cos(a) * ARM_LENGTH[i];
    stroke(74, 112, 130, 120);
    strokeWeight(13);
    line(px, py, ex, ey);
    stroke(137, 220, 221);
    strokeWeight(7);
    line(px, py, ex, ey);
    noStroke();
    fill(247, 215, 148);
    ellipse(px, py, 20, 20);
    fill(226, 247, 245);
    ellipse(ex, ey, 15, 15);
    textSize(16);
    fill(238);
    textAlign(CENTER);
    text(PIVOT_LABEL[i], px, py + 39);
    textSize(13);
    fill(151, 184, 191);
    text(nf(openness[i] * 100, 0, 0) + "%", px, py + 59);
    text(stepTargets[i] + " steps", px, py + 80);
  }
  textAlign(LEFT);

  // Optional wrist-position markers (camera coordinates simplified in a panel).
  fill(26, 39, 49);
  rect(36, 607, 250, 105, 12);
  fill(175, 198, 210);
  textSize(12);
  text("WRIST INPUTS  (camera X/Y)", 50, 630);
  drawWristDot(f[0], f[1], 52, 640, 98, 59, color(116, 226, 214));
  drawWristDot(f[2], f[3], 160, 640, 98, 59, color(242, 188, 103));

  fill(190, 203, 210);
  textSize(14);
  String status = "PERSON: " + (present ? "DETECTED" : "ABSENT")
      + "     WEKINATOR: " + (testMode ? "SIMULATED" : (wekFresh ? "LIVE" : "NO DATA"))
      + "     STARTLE: " + nf(startleRemaining, 1, 2) + " s";
  text(status, 314, 647);
  if (arduino != null && !arduinoReady) {
    fill(240, 190, 96);
    text("CALIBRATING MOTOR " + (calibratingMotor == 0 ? "..." : calibratingMotor)
      + ":  A backward  |  D forward  |  S STOP  |  N set zero / next", 36, 742);
    fill(182, 201, 213);
    text("Arduino: " + arduinoStatus + "     (B disconnects; stop motor first)", 36, 770);
  } else {
    text("B: connect/disconnect   S: arm/disarm after calibration   T: test mode   X: force closed", 36, 742);
    text("P: person   E/R: engagement   F/G: threat   Arrows, I/J/K/L: wrists   V/C: trust", 36, 770);
  }
}

void drawMeter(String label, float value, float x, float y, int colour) {
  noStroke();
  textSize(12);
  fill(194, 208, 215);
  text(label, x, y);
  fill(46, 61, 73);
  rect(x, y+8, 240, 11, 4);
  fill(colour);
  rect(x, y+8, 240 * constrain(value, 0, 1), 11, 4);
  fill(231);
  text(nf(value*100, 0, 0) + "%", x + 250, y+19);
}

void drawWristDot(float x, float y, float bx, float by, float w, float h, int colour) {
  noFill();
  stroke(65, 85, 97);
  rect(bx, by, w, h, 4);
  noStroke();
  fill(colour);
  ellipse(bx + constrain(x, 0, 1)*w, by + constrain(y, 0, 1)*h, 9, 9);
}

// Receive newline-delimited messages from the Arduino while keeping its
// serial connection open. READY is emitted only AFTER four motors are homed.
void readArduinoStatus() {
  if (arduino == null) return;
  try {
    while (arduino.available() > 0) {
      String line = arduino.readStringUntil('\n');
      if (line == null) break;
      line = trim(line);
      if (line.length() == 0) continue;
      println("Arduino: " + line);
      arduinoStatus = line;

      if (line.startsWith("CALIBRATING MOTOR ")) {
        try {
          calibratingMotor = Integer.parseInt(
              trim(line.substring("CALIBRATING MOTOR ".length())));
        } catch (Exception ignored) { }
      }

      if (line.equals("SHY CAT CONTROLLER READY") ||
          line.equals("NORMAL OPERATION ACTIVE")) {
        arduinoReady = true;
        serialEnabled = false; // manual safety arm required
        arduinoStatus = "CALIBRATED - PRESS S TO ARM";
        println("Calibration finished. Press S to arm physical motors.");
      }
    }
  } catch (Exception e) {
    println("Arduino serial read failed: " + e.getMessage());
    disconnectArduino();
  }
}

void sendToArduino() {
  if (arduino == null || !arduinoReady || !serialEnabled) return;
  String packet = "A," + stepTargets[0] + "," + stepTargets[1] + ","
      + stepTargets[2] + "," + stepTargets[3] + "\n";
  try {
    arduino.write(packet);
  } catch (Exception e) {
    println("Arduino serial disconnected: " + e.getMessage());
    disconnectArduino();
  }
}

void sendCalibrationKey(char k) {
  if (arduino == null || arduinoReady) return;
  try {
    arduino.write(k); // Arduino's homing loop reads individual characters
    arduinoStatus = "SENT " + Character.toUpperCase(k);
  } catch (Exception e) {
    println("Calibration command failed: " + e.getMessage());
    disconnectArduino();
  }
}

void disconnectArduino() {
  if (arduino != null) {
    try {
      if (!arduinoReady) arduino.write('s'); // stop calibration motor
      else arduino.write("H\n"); // request close before disconnect
    } catch (Exception ignored) { }
    try { arduino.stop(); } catch (Exception ignored) { }
  }
  arduino = null;
  arduinoReady = false;
  serialEnabled = false;
  calibratingMotor = 0;
  arduinoStatus = "NOT CONNECTED";
  println("Arduino disconnected. Reconnecting will reset Nano and restart homing.");
}

void toggleConnection() {
  if (arduino != null) {
    disconnectArduino();
    return;
  }
  try {
    arduino = new Serial(this, ARDUINO_PORT, ARDUINO_BAUD);
    arduino.clear();
    // Opening the port resets a typical Nano; don't send normal commands yet.
    arduinoReady = false;
    serialEnabled = false;
    calibratingMotor = 0;
    arduinoStatus = "WAITING FOR ARDUINO CALIBRATION";
    println("Connected: " + ARDUINO_PORT + ". Calibrate with A/D/S/N.");
  } catch (Exception e) {
    println("Cannot open " + ARDUINO_PORT + ": " + e.getMessage());
    println("Available ports:");
    printArray(Serial.list());
    arduino = null;
    arduinoReady = false;
    serialEnabled = false;
  }
}

void toggleArm() {
  if (arduino == null || !arduinoReady) {
    println("Connect with B and complete all four calibrations first.");
    return;
  }
  serialEnabled = !serialEnabled;
  if (!serialEnabled) {
    try { arduino.write("H\n"); } catch (Exception e) { disconnectArduino(); }
    println("Hardware DISARMED; commanded closed position.");
  } else {
    lastSerialSendMs = -100000;
    println("Hardware ARMED: sending live position targets.");
  }
}

void keyPressed() {
  if (key == CODED) {
    if (!testMode) return;
    if (keyCode == LEFT)  testFeatures[0] = constrain(testFeatures[0]-.04, 0, 1);
    if (keyCode == RIGHT) testFeatures[0] = constrain(testFeatures[0]+.04, 0, 1);
    if (keyCode == UP)    testFeatures[1] = constrain(testFeatures[1]-.04, 0, 1);
    if (keyCode == DOWN)  testFeatures[1] = constrain(testFeatures[1]+.04, 0, 1);
    return;
  }

  char k = Character.toLowerCase(key);

  // Connection key is always available. It never arms the motors.
  if (k == 'b') {
    toggleConnection();
    return;
  }

  // During calibration, A/D/S/N override every digital-twin shortcut.
  if (arduino != null && !arduinoReady) {
    if (k == 'a' || k == 'd' || k == 's' || k == 'n') {
      sendCalibrationKey(k);
    }
    return;
  }

  if (k == 't') testMode = !testMode;
  else if (k == 's') toggleArm();
  else if (k == 'x') emergencyClosed = !emergencyClosed;
  else if (k == 'c') trust = 0;
  else if (k == 'v') trust = .85;
  else if (testMode) {
    if (k == 'p') testPersonPresent = !testPersonPresent;
    else if (k == 'e') testEngagement = constrain(testEngagement+.10, 0, 1);
    else if (k == 'r') testEngagement = constrain(testEngagement-.10, 0, 1);
    else if (k == 'f') testThreat = constrain(testThreat+.10, 0, 1);
    else if (k == 'g') testThreat = constrain(testThreat-.10, 0, 1);
    else if (k == 'j') testFeatures[2] = constrain(testFeatures[2]-.04, 0, 1);
    else if (k == 'l') testFeatures[2] = constrain(testFeatures[2]+.04, 0, 1);
    else if (k == 'i') testFeatures[3] = constrain(testFeatures[3]-.04, 0, 1);
    else if (k == 'k') testFeatures[3] = constrain(testFeatures[3]+.04, 0, 1);
  }
}

void dispose() {
  // Stop a calibration jog if possible before quitting.
  disconnectArduino();
}
