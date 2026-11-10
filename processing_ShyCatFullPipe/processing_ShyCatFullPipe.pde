/*
  SHY CAT: SINGLE BEHAVIOUR ENGINE + DIGITAL TWIN + FOUR STEPPER OUTPUTS

  Wekinator -> /wek/outputs at OSC port 12000:
     output 1 = Threat [0..1], output 2 = Engagement [0..1]
  Python -> OSC port 12001:
     /person/present  [0 or 1]
     /person/features [12 floats: wrist X/Y first]
  Processing -> Arduino USB serial 9600 baud:
     A,stepsA,stepsB,stepsC,stepsD\n
  This is ONE .pde file. Only ONE settings() exists.
  First verify the digital twin; press S to arm serial output ONLY after
  checking wiring, power and physical motor limits.
*/

import oscP5.*;
import netP5.*;
import processing.serial.*;

// ---------- PORTS ----------
final int WEK_OSC_PORT = 12000;
final int PERSON_OSC_PORT = 12001;
final String ARDUINO_PORT = "/dev/cu.usbmodem1401";
final int ARDUINO_BAUD = 9600;
final int SERIAL_SEND_INTERVAL_MS = 50;

// ---------- PHYSICAL MOTION (must match Arduino) ----------
final int[] MAX_OPEN_STEPS = {512, 341, 683, 512};
// Set each sign to +1 or -1 depending on which direction OPENS its arm.
// Initial values assume left arms open counterclockwise, right clockwise.
final int[] MOTOR_DIRECTION = {-1, -1, +1, +1};

// ---------- BEHAVIOUR TUNING ----------
final float TRUST_GAIN_PER_SECOND = 0.12;
final float TRUST_LOSS_PER_SECOND = 0.40;
final float TRUST_DECAY_UNOBSERVED = 0.04;
final float MIMIC_START_TRUST = 0.75;
final float MIMIC_FULL_TRUST = 0.95;
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
boolean serialEnabled = false; // SAFE DEFAULT: DIGITAL TWIN ONLY
int lastSerialSendMs = -100000;

// ---------- NETWORK INPUT ----------
final Object networkLock = new Object();
float[] poseFeatures = new float[12];
boolean rawPersonPresent = false;
int lastPoseTrueMs = -100000;
int lastWekMs = -100000;
float rawThreat = 0;
float rawEngagement = 0;

// ---------- STATE ----------
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
  .25, .72, .75, .72, .50, .23,
  .12, .12, .10, .05, .05, .50
};

// ---------- DRAWING THE ORIGINAL FOUR-PIVOT DIGITAL TWIN ----------
final String[] PIVOT_LABEL = {"A", "B", "C", "D"};
final float[] PIVOT_X = {.19, .39, .61, .81};
final float[] CLOSED_ANGLE_DEG = {50, 30, -30, -50};
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
  println("Hardware DISARMED. Press S after verifying the digital twin and physical motor setup.");
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
      }
    }
  }
}

void draw() {
  int now = millis();
  float dt = previousDrawMs < 0 ? 1.0/60 : constrain((now - previousDrawMs) / 1000.0, .001, .05);
  previousDrawMs = now;

  float[] f = new float[12];
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

  updateBehaviour(dt, now, present, wekFresh, currentThreat, currentEngagement, f);
  drawTwin(present, wekFresh, f);
  if (serialEnabled && now - lastSerialSendMs >= SERIAL_SEND_INTERVAL_MS) {
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
  // Distances relative to torso for sideways reach, reducing effect of walking.
  float leftReach = constrain(.22 + (torsoX - f[0]) / (1.5 * shoulderW), 0, 1);
  float rightReach = constrain(.22 + (f[2] - torsoX) / (1.5 * shoulderW), 0, 1);
  // Height: 0 at image bottom, 1 at image top.
  // Note: shoulder Y is not in the 12 old features; this is camera-relative.
  float leftHeight = constrain(map(f[1], .83, .13, 0, 1), 0, 1);
  float rightHeight = constrain(map(f[3], .83, .13, 0, 1), 0, 1);
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
  text(serialEnabled ? "ARDUINO: ARMED" : "ARDUINO: DISARMED", 950, 67);

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
  text("T: test mode   P: toggle person   E/R: engagement +/-   F/G: threat +/-", 36, 742);
  text("Arrows: left wrist   I/J/K/L: right wrist   V: trust=85%   C: reset trust   X: force closed   S: serial", 36, 770);
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

void sendToArduino() {
  if (arduino == null) return;
  String packet = "A," + stepTargets[0] + "," + stepTargets[1] + ","
      + stepTargets[2] + "," + stepTargets[3] + "\n";
  try {
    arduino.write(packet);
  } catch (Exception e) {
    println("Arduino serial disconnected: " + e.getMessage());
    serialEnabled = false;
    try { arduino.stop(); } catch (Exception ignored) {}
    arduino = null;
  }
}

void toggleSerial() {
  if (serialEnabled) {
    serialEnabled = false;
    if (arduino != null) arduino.stop();
    arduino = null;
    println("Arduino output DISARMED");
    return;
  }
  try {
    arduino = new Serial(this, ARDUINO_PORT, ARDUINO_BAUD);
    delay(1800); // Nano normally resets when the serial port opens.
    arduino.clear();
    serialEnabled = true;
    lastSerialSendMs = -100000;
    println("Arduino output ARMED: " + ARDUINO_PORT);
  } catch (Exception e) {
    println("Cannot open " + ARDUINO_PORT + ": " + e.getMessage());
    println("Available ports:");
    printArray(Serial.list());
    serialEnabled = false;
    arduino = null;
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
  if (k == 't') testMode = !testMode;
  else if (k == 's') toggleSerial();
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
  // Nano has its own loss-of-serial fail-closed behaviour.
  if (arduino != null) arduino.stop();
}
