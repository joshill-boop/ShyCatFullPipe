import oscP5.*;
import processing.serial.*;

// Connections
final int WEK_OSC_PORT = 12000;
final int PERSON_OSC_PORT = 12001;
final String ARDUINO_PORT = "/dev/cu.usbmodem2101";
final int ARDUINO_BAUD = 9600;
final int SERIAL_SEND_INTERVAL_MS = 50;

// These settings must match Arduino.
final int[] MAX_OPEN_STEPS = {1000, 1000, 1000, 1000};
final int[] MOTOR_DIRECTION = {-1, -1, +1, +1};

// Behaviour settings
final float TRUST_GAIN_PER_SECOND = 0.1;
final float TRUST_LOSS_PER_SECOND = 0.20;
final float TRUST_DECAY_UNOBSERVED = 0.04;
final float MIMIC_START_TRUST = 0.75;
final float MIMIC_FULL_TRUST = 0.95;

final float MIMIC_VERTICAL_LOW = -1.4;
final float MIMIC_VERTICAL_HIGH = 1.4;
final float MIMIC_HORIZONTAL_MIN = 0.0;
final float MIMIC_HORIZONTAL_MAX = 1.25;
final boolean MIRRORED_WRIST_X = true;

final float STARTLE_DURATION_S = 5;
final float THREAT_STARTLE_THRESHOLD = 0.72;
final float IDLE_OPENNESS = 0.66;
final float IDLE_WANDER_AMPLITUDE = 0.5;
final float PERSON_MIN_OPENNESS = 0.17;
final float PERSON_MAX_OPENNESS = 0.92;
final float INPUT_TIMEOUT_MS = 750;
final float WEK_TIMEOUT_MS = 1300;

OscP5 wekOsc;
OscP5 personOsc;
Serial arduino;

boolean serialEnabled = false;
boolean arduinoReady = false;
int calibratingMotor = 0;
String arduinoStatus = "NOT CONNECTED";
int lastSerialSendMs = -100000;

// OSC callbacks and draw() run on separate threads.
final Object networkLock = new Object();

// Optional feature 13 is shoulder-centre Y, for Processing only.
float[] poseFeatures = new float[13];
boolean shoulderYAvailable = false;
int lastPoseTrueMs = -100000;
int lastWekMs = -100000;
float rawThreat = 0;
float rawEngagement = 0;

// Current behaviour state
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
int[] stepTargets = new int[4];

String behaviouralState = "IDLE";
int previousDrawMs = -1;

// Manual test inputs
boolean testMode = false;
boolean testPersonPresent = false;
float testThreat = 0.05;
float testEngagement = 0.85;
float[] testFeatures = {
  .75, .72, .25, .72, .50, .23,
  .12, .12, .10, .05, .05, .50,
  .36
};

void settings() {
  size(1000, 760);
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
    try {
      int count = message.arguments().length;

      if (message.checkAddrPattern("/wek/outputs") && count >= 2) {
        rawThreat = constrain(message.get(0).floatValue(), 0, 1);
        rawEngagement = constrain(message.get(1).floatValue(), 0, 1);
        lastWekMs = millis();

      } else if (message.checkAddrPattern("/person/present") && count >= 1) {
        if (message.get(0).intValue() != 0) lastPoseTrueMs = millis();

      } else if (message.checkAddrPattern("/person/features") && count >= 12) {
        for (int i = 0; i < min(count, 13); i++) {
          poseFeatures[i] = constrain(message.get(i).floatValue(), 0, 1);
        }
        shoulderYAvailable = count >= 13;
      }
    } catch (Exception ex) {
      println("Invalid OSC message: " + message.addrPattern());
    }
  }
}

void draw() {
  int now = millis();
  float dt = previousDrawMs < 0
    ? 1.0/60
    : constrain((now - previousDrawMs) / 1000.0, .001, .05);
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
      present = now - lastPoseTrueMs < INPUT_TIMEOUT_MS;
      wekFresh = now - lastWekMs < WEK_TIMEOUT_MS;
      currentThreat = wekFresh ? rawThreat : 1.0;
      currentEngagement = wekFresh ? rawEngagement : 0.0;
    }
  }

  readArduinoStatus();
  updateBehaviour(
    dt, now, present, wekFresh, currentThreat, currentEngagement, f
  );
  drawPanel(present, wekFresh, f);

  if (serialEnabled && arduinoReady
      && now - lastSerialSendMs >= SERIAL_SEND_INTERVAL_MS) {
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

  if (present && wekFresh && incomingThreat > THREAT_STARTLE_THRESHOLD
      && previousThreat <= THREAT_STARTLE_THRESHOLD && startleCooldown <= 0) {
    startleRemaining = max(startleRemaining, STARTLE_DURATION_S);
    startleCooldown = 1.5;
    trust = max(0, trust - .50);
  }
  previousThreat = incomingThreat;

  float inAlpha = 1.0 - exp(-dt * 5.5);
  threat += (incomingThreat - threat) * inAlpha;
  engagement += (incomingEngagement - engagement) * inAlpha;

  if (!present) {
    trust = max(0, trust - TRUST_DECAY_UNOBSERVED * dt);
  } else if (!wekFresh) {
    trust = max(0, trust - 0.10 * dt);
  } else if (startleRemaining <= 0) {
    if (incomingThreat >= 0.42) {
      trust = max(0, trust - TRUST_LOSS_PER_SECOND * incomingThreat * dt);
    } else {
      float positiveEngagement = max(0, (incomingEngagement - .20) / .80);
      float gain = TRUST_GAIN_PER_SECOND
        * positiveEngagement * (1.0 - incomingThreat);
      float erosion = 0.015 * (1.0 - positiveEngagement);
      trust = constrain(trust + (gain - erosion) * dt, 0, 1);
    }
  }

  float wantedFear = 0;
  if (present) {
    if (!wekFresh || startleRemaining > 0) wantedFear = 1;
    else wantedFear = constrain(map(incomingThreat, .35, .85, 0, 1), 0, 1);
  }

  float fearRate = wantedFear > fear ? 6.8 : 0.90;
  fear += (wantedFear - fear) * (1 - exp(-dt * fearRate));

  float desiredMimic = present && wekFresh
    ? smoothstep(MIMIC_START_TRUST, MIMIC_FULL_TRUST, trust)
    : 0;
  desiredMimic *= 1 - fear;
  mimicStrength += (desiredMimic - mimicStrength) * (1 - exp(-dt * 2.3));

  float[] mimic = calculateMimic(f);
  float engagedOpen = lerp(PERSON_MIN_OPENNESS, PERSON_MAX_OPENNESS, trust);

  for (int i = 0; i < 4; i++) {
    float wandering = (noise(10*i + now*.00029) - .5)
      * 2 * IDLE_WANDER_AMPLITUDE;
    float secondary = sin(now*.00064 + i*1.7) * .035;
    float idle = constrain(IDLE_OPENNESS + wandering + secondary, .28, .90);

    float base = present ? engagedOpen + wandering*.38 : idle;
    float combined = lerp(base, mimic[i], mimicStrength);
    if (present) combined = lerp(combined, 0.015, fear);
    if (emergencyClosed) combined = 0;
    float target = constrain(combined, 0, 1);

    float rate = target < openness[i] ? 5.8 : 1.55;
    openness[i] += (target - openness[i]) * (1.0 - exp(-dt * rate));
    openness[i] = constrain(openness[i], 0, 1);

    stepTargets[i] = round(
      MOTOR_DIRECTION[i] * MAX_OPEN_STEPS[i] * openness[i]
    );
  }

  if (emergencyClosed) behaviouralState = "MANUAL CLOSE";
  else if (!present) behaviouralState = "IDLE / WANDERING";
  else if (!wekFresh) behaviouralState = "ML SIGNAL LOST - CLOSING";
  else if (fear > .45 || startleRemaining > 0) {
    behaviouralState = "FRIGHTENED / HIDING";
  } else if (mimicStrength > .80) behaviouralState = "MIMICKING";
  else if (mimicStrength > .05) {
    behaviouralState = "OPENING / LEARNING TO MIMIC";
  } else if (trust < .20) behaviouralState = "CAUTIOUS";
  else behaviouralState = "ENGAGING / BUILDING TRUST";
}

float[] calculateMimic(float[] f) {
  // A/B follow left wrist reach/height; C/D follow right height/reach.
  float torsoX = f[4];
  float shoulderW = max(.08, f[5]);
  float leftShoulderX = torsoX
    + (MIRRORED_WRIST_X ? +0.5 : -0.5) * shoulderW;
  float rightShoulderX = torsoX
    + (MIRRORED_WRIST_X ? -0.5 : +0.5) * shoulderW;

  float leftOutward = (
    MIRRORED_WRIST_X ? f[0] - leftShoulderX : leftShoulderX - f[0]
  ) / shoulderW;
  float rightOutward = (
    MIRRORED_WRIST_X ? rightShoulderX - f[2] : f[2] - rightShoulderX
  ) / shoulderW;

  return new float[] {
    lerp(.10, .97, map01(
      leftOutward, MIMIC_HORIZONTAL_MIN, MIMIC_HORIZONTAL_MAX
    )),
    lerp(.10, .97, wristHeight(f[1], f[12], shoulderW)),
    lerp(.10, .97, wristHeight(f[3], f[12], shoulderW)),
    lerp(.10, .97, map01(
      rightOutward, MIMIC_HORIZONTAL_MIN, MIMIC_HORIZONTAL_MAX
    ))
  };
}

float wristHeight(float wristY, float shoulderY, float shoulderW) {
  if (testMode || shoulderYAvailable) {
    return map01(
      (shoulderY - wristY) / shoulderW,
      MIMIC_VERTICAL_LOW, MIMIC_VERTICAL_HIGH
    );
  }
  return map01(wristY, .83, .13);
}

float map01(float value, float low, float high) {
  return constrain(map(value, low, high, 0, 1), 0, 1);
}

float smoothstep(float e0, float e1, float x) {
  float t = constrain((x - e0) / (e1 - e0), 0, 1);
  return t*t*(3 - 2*t);
}

void drawPanel(boolean present, boolean wekFresh, float[] f) {
  background(14, 20, 29);
  fill(240); textSize(24);
  text("SHY CAT / CONTROL PANEL", 24, 38);
  textSize(14); fill(162, 190, 202);
  text(behaviouralState, 24, 64);
  text(testMode ? "TEST / SIMULATED INPUT" : "LIVE / CAMERA + WEKINATOR", 660, 38);

  String hardware = arduino == null ? "DISCONNECTED"
    : !arduinoReady ? "CALIBRATING"
    : serialEnabled ? "ARMED" : "DISARMED";
  fill(serialEnabled ? color(255, 170, 85) : color(162, 190, 202));
  text("ARDUINO: " + hardware, 660, 64);

  drawMeter("THREAT", threat, 24, 110, color(230, 101, 98));
  drawMeter("ENGAGEMENT", engagement, 520, 110, color(75, 173, 232));
  drawMeter("TRUST", trust, 24, 170, color(100, 213, 145));
  drawMeter("FEAR", fear, 520, 170, color(230, 101, 98));
  drawMeter("MIMIC", mimicStrength, 24, 230, color(239, 187, 88));

  float meanOpen = (openness[0] + openness[1] + openness[2] + openness[3]) / 4;
  drawMeter("OPENNESS / MEAN COMMAND", meanOpen, 520, 230, color(116, 226, 214));

  fill(190, 203, 210); textSize(14);
  text("COMMANDED OPENNESS / software values, not measured motor positions", 24, 302);

  String[] labels = {
    "A: LEFT REACH", "B: LEFT HEIGHT", "C: RIGHT HEIGHT", "D: RIGHT REACH"
  };
  for (int i = 0; i < 4; i++) {
    drawMeter(
      labels[i] + " / " + stepTargets[i] + " steps", openness[i],
      24 + (i % 2) * 496, 338 + (i / 2) * 60, color(116, 226, 214)
    );
  }

  fill(190, 203, 210); textSize(13);
  text("PERSON: " + (present ? "DETECTED" : "ABSENT")
    + "     WEKINATOR: " + (testMode ? "SIMULATED" : wekFresh ? "LIVE" : "NO DATA")
    + "     STARTLE: " + nf(startleRemaining, 1, 2) + " s"
    + "     CLOSED TARGETS: " + (emergencyClosed ? "ON" : "OFF"), 24, 470);

  text("LEFT WRIST X/Y: " + nf(f[0], 1, 2) + " / " + nf(f[1], 1, 2)
    + "     RIGHT WRIST X/Y: " + nf(f[2], 1, 2) + " / " + nf(f[3], 1, 2),
    24, 496);

  noStroke(); fill(26, 39, 49);
  rect(16, 524, width - 32, 216, 10);
  textSize(14);

  if (arduino != null && !arduinoReady) {
    fill(240, 190, 96);
    text("CALIBRATING MOTOR "
      + (calibratingMotor == 0 ? "..." : calibratingMotor), 32, 550);
    text("A: backward    D: forward    S: STOP    N: set closed zero / next motor",
      32, 578);
    fill(190, 203, 210);
    text("B: disconnect (stop the motor first)", 32, 606);
    text("Calibration has no travel limits. Stop before mechanical stops.", 32, 634);
    text("After all four zeros are set, press S to arm output.", 32, 662);
  } else {
    fill(240);
    text("KEYBOARD / click this window first", 32, 550);
    fill(190, 203, 210);
    text("B: connect/disconnect    S: arm/disarm    T: live/test    X: closed targets",
      32, 578);
    text("V: trust to 85%    C: reset trust", 32, 606);
    fill(testMode ? color(116, 226, 214) : color(138, 160, 170));
    text("TEST MODE: P: person    E/R: engagement +/-    F/G: threat +/-", 32, 634);
    text("Arrows: left wrist    J/L: right wrist X    I/K: right wrist Y", 32, 662);
  }

  fill(162, 190, 202); textSize(12);
  text("Record and train in Wekinator. Test keys simulate inputs here.", 32, 694);
  String outputStatus = arduinoReady
    ? (serialEnabled ? "MOTOR OUTPUT ARMED" : "CALIBRATED / press S to arm")
    : arduinoStatus;
  text("Arduino: " + outputStatus, 32, 720);
}

void drawMeter(String label, float value, float x, float y, int colour) {
  noStroke(); textSize(13); fill(194, 208, 215);
  text(label, x, y);
  fill(46, 61, 73); rect(x, y + 10, 360, 14, 4);
  fill(colour); rect(x, y + 10, 360 * constrain(value, 0, 1), 14, 4);
  fill(231); text(nf(value * 100, 0, 0) + "%", x + 374, y + 23);
}

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
            trim(line.substring("CALIBRATING MOTOR ".length()))
          );
        } catch (Exception ignored) { }
      }

      if (line.equals("SHY CAT CONTROLLER READY")
          || line.equals("NORMAL OPERATION ACTIVE")) {
        arduinoReady = true;
        serialEnabled = false;
        arduinoStatus = "CALIBRATED - PRESS S TO ARM";
        println("Calibration finished. Press S to arm physical motors.");
      }
    }
  } catch (Exception e) {
    println("Arduino serial read failed: " + e.getMessage());
    disconnectArduino();
  }
}

boolean writeArduino(String message) {
  if (arduino == null) return false;

  try {
    arduino.write(message);
    return true;
  } catch (Exception e) {
    println("Arduino write failed: " + e.getMessage());
    disconnectArduino();
    return false;
  }
}

void sendToArduino() {
  if (!arduinoReady || !serialEnabled) return;
  writeArduino("A," + stepTargets[0] + "," + stepTargets[1] + ","
    + stepTargets[2] + "," + stepTargets[3] + "\n");
}

void sendCalibrationKey(char k) {
  if (arduino == null || arduinoReady) return;
  if (writeArduino(str(k))) arduinoStatus = "SENT " + Character.toUpperCase(k);
}

void disconnectArduino() {
  if (arduino != null) {
    try {
      if (!arduinoReady) arduino.write('s');
      else arduino.write("H\n");
    } catch (Exception ignored) { }

    try {
      arduino.stop();
    } catch (Exception ignored) { }
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
    writeArduino("H\n");
    println("Hardware DISARMED; commanded closed position.");
  } else {
    lastSerialSendMs = -100000;
    println("Hardware ARMED: sending live position targets.");
  }
}

void moveTestWrist(int coordinate, float change) {
  testFeatures[coordinate] = constrain(testFeatures[coordinate] + change, 0, 1);
}

void keyPressed() {
  if (key == CODED) {
    if (!testMode) return;
    if (keyCode == LEFT)  moveTestWrist(0, -.04);
    if (keyCode == RIGHT) moveTestWrist(0, .04);
    if (keyCode == UP)    moveTestWrist(1, -.04);
    if (keyCode == DOWN)  moveTestWrist(1, .04);
    return;
  }

  char k = Character.toLowerCase(key);

  if (k == 'b') {
    toggleConnection();
    return;
  }

  // Calibration takes priority over normal letter shortcuts.
  if (arduino != null && !arduinoReady) {
    if (k == 'a' || k == 'd' || k == 's' || k == 'n') sendCalibrationKey(k);
    return;
  }

  switch (k) {
    case 't': testMode = !testMode; return;
    case 's': toggleArm(); return;
    case 'x': emergencyClosed = !emergencyClosed; return;
    case 'c': trust = 0; return;
    case 'v': trust = .85; return;
  }

  if (!testMode) return;

  switch (k) {
    case 'p': testPersonPresent = !testPersonPresent; break;
    case 'e': testEngagement = constrain(testEngagement + .10, 0, 1); break;
    case 'r': testEngagement = constrain(testEngagement - .10, 0, 1); break;
    case 'f': testThreat = constrain(testThreat + .10, 0, 1); break;
    case 'g': testThreat = constrain(testThreat - .10, 0, 1); break;
    case 'j': moveTestWrist(2, -.04); break;
    case 'l': moveTestWrist(2, .04); break;
    case 'i': moveTestWrist(3, -.04); break;
    case 'k': moveTestWrist(3, .04); break;
  }
}

void dispose() {
  disconnectArduino();
}
