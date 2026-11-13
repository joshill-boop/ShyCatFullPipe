# Shy Cat sculpture — unified four-stepper pipeline

This package contains the **complete three-script prototype**:

- `python/shy_cat_pose.py` — camera / MediaPipe, **12 OSC features** to Wekinator and Processing.
- `processing/ShyCatUnified/ShyCatUnified.pde` — state machine, trust, fear, idle wandering, progressive 4-arm mimic, full visual digital twin, serial commands.
- `arduino/ShyCatFourSteppers/ShyCatFourSteppers.ino` — four nonblocking AccelStepper motors with travel clamps and timeout return-to-closed.

## Signal and port specification

| From | To | OSC / message | Values |
|---|---|---|---|
| Python | Wekinator `127.0.0.1:6448` | `/wek/inputs` | **12 floats** |
| Wekinator | Processing `127.0.0.1:12000` | `/wek/outputs` | **2 floats**: output 1 = Threat, output 2 = Engagement |
| Python | Processing `127.0.0.1:12001` | `/person/features` | the **same 12 floats** |
| Python | Processing `127.0.0.1:12001` | `/person/present` | one integer: 0 or 1 |
| Processing | Arduino USB | `A,-255,-220,270,380\n` | 4 signed target steps; this is an **example only** |

**Wekinator project:** 12 numerical inputs, **two continuous (regression) outputs**, with the output destination `localhost:12000` and message `/wek/outputs`. Train/range outputs as `Threat 0–1` (safe to threatening) and `Engagement 0–1` (no invitation to calm, inviting movement). The model should run continuously during live operation.

### The twelve inputs (same order as previous project)

| # | Name | Encoding |
|--:|---|---|
| 1 | Left wrist X | smoothed camera X 0–1 |
| 2 | Left wrist Y | smoothed camera Y 0–1 |
| 3 | Right wrist X | smoothed camera X 0–1 |
| 4 | Right wrist Y | smoothed camera Y 0–1 |
| 5 | Torso X | shoulder midpoint camera X 0–1 |
| 6 | Torso scale | shoulder width in frame, clipped 0–1 |
| 7 | Left wrist movement speed | shoulder-relative, normalised 0–1 |
| 8 | Right wrist movement speed | shoulder-relative, normalised 0–1 |
| 9 | Body movement speed | six landmarks, scaled by shoulder width, 0–1 |
| 10 | Movement variability | standard deviation of body speed over ~0.6 s, 0–1 |
| 11 | Acceleration | change in body speed, 0–1 |
| 12 | Approach / recession | change in shoulder width (0.5 = stationary) |

**Training compatibility notice:** We reconstructed these from your project's known 12-feature definitions, but do **not** have your exact previously trained Python source or its filtering/normalisation constants. Matching input **count and order** does **not guarantee** an existing Wekinator model will make accurate predictions. Start by testing your saved model. If its outputs are inconsistent, keep the exact 12-feature extraction maths from your last trained Python script (and preserve `send_message("/person/features", features)` and `/person/present`) rather than automatically retraining. Retrain only if you decide to change the feature definitions. Mirroring itself never needs a separate ML model.

## 1. Prepare the Arduino first

The current **classic Arduino Nano** cannot use A6/A7 (digital pin numbers 20/21) as digital outputs. Rewire the *two affected Motor 3 driver inputs*:

| Motor | AccelStepper constructor pin order |
|---|---|
| A | `6,5,3,4` |
| B | `10,8,9,7` |
| C | **`18,11,19,12`** (changed from `18,20,19,21`) |
| D | `14,16,15,17` |

These numbers are the **Arduino digital pins connected to the ULN2003 input pins**, not the internal stepper motor wire colour order. Your prior AccelStepper order is retained. Arduino IDE → Library Manager → install **AccelStepper**. Select the correct Nano board and USB port and upload `ShyCatFourSteppers.ino`.

**Safety:** Your steppers have **no homing switches or absolute encoders**. On every restart, position all arms manually at the *closed/hidden* mechanical origin **before powering the motors**. The firmware then calls `setCurrentPosition(0)`. Wrong origins or travel limits can damage the sculpture; `MAX_OPEN_STEPS` are only software estimates until calibrated. Confirm the correct **sign of movement** separately for each motor by changing `MOTOR_DIRECTION` consistently in **both** Processing and Arduino. If motor direction is correct but it vibrates rather than rotates, inspect the ULN2003 input-to-pin sequence. Never force powered stepper arms by hand. Use an adequately rated independent **5V motor supply**, with its GND connected to Nano GND. Do not draw four motors' current from the USB/Nano regulator.

The Arduino moves all four motors concurrently with `run()`, caps each target to the configured open travel, and returns towards **0 = closed** if no serial packet arrives for 1.5 seconds. It cannot detect stalls, slipping, or physical collisions. Start with the linkage disconnected / safe travel and test one arm at a time.

## 2. Run Processing in digital-twin mode first

Open `processing/ShyCatUnified/ShyCatUnified.pde` in **Processing 4**. Install `oscP5` (Contribution Manager → Libraries). Start the sketch.

It starts with **no Arduino output**. Press `T` to enter manual test mode, then `P` to simulate a person entering frame. It will hide initially, then gradually trust and open; press `V` to set trust to 85% and move the wrist coordinates to test progressive mimicry. Raise simulated Threat via `F` until it flinches, then lower via `G`. Press `P` to simulate leaving and observe the slow return to wandering. Press `X` to force-close targets, and `C` to reset trust.

Keyboard controls:

| Key | Operation |
|---|---|
| `T` | toggle test / live mode |
| `P` | person present toggle (test mode) |
| `E` / `R` | increase/decrease Engagement (test mode) |
| `F` / `G` | increase/decrease Threat (test mode) |
| arrow keys | move left wrist X/Y (test mode) |
| `J` / `L`, `I` / `K` | move right wrist X, Y (test mode) |
| `V` | set trust to 85% (inspect mimic blend) |
| `C` | reset trust to zero |
| `X` | toggle forced closed targets |
| **`S`** | **arm/disarm actual serial motor output** |

Once happy with the simulation, set `ARDUINO_PORT` in the Processing sketch to the actual value from the `Serial.list()` printout. `S` is intentionally explicit so opening the digital twin doesn't move the hardware. If you close Processing or press `S` to disarm, Arduino times out and travels back towards zero.

## 3. Run MediaPipe + Wekinator

In your Python environment:

```bash
cd python
python -m pip install -r requirements.txt
python shy_cat_pose.py
```

Copy the already used `pose_landmarker_lite.task` file into the `python/` folder. Python uses the **MediaPipe Tasks VIDEO API**, so this is the same API family as your existing project. It assumes your macOS camera (`CAMERA_BACKEND_AVFOUNDATION = True`); change this constant to `False` on Windows/Linux. Keep `FLIP_CAMERA` consistent with the orientation used for Wekinator training. Press `Q` in the camera preview to stop.

Run your saved Wekinator project in its **Run** mode. Start the Processing sketch and press `T` again if currently in manual mode, returning it to live input. Initially keep Arduino output **disarmed** until the digital twin reacts correctly.

## 4. What the machine does

1. **No person:** four independent slowly wandering arms near a moderately open resting posture.
2. **First detection:** a ~1.3 s startle and immediate tendency towards the closed/fearful positions.
3. **Cautious engagement:** only a low Threat / high Engagement Wekinator output increases persistent trust. It opens gradually.
4. **Mimicking:** starts as trust crosses `0.75` and blends smoothly into complete wrist mimic by `0.95` (provided low fear). The four motors represent left wrist sideways reach, left wrist height, right wrist height, right wrist sideways reach.
5. **Threat:** rising Threat triggers the fearful close response, removes trust quickly and suppresses mirroring.
6. **Departure:** no person -> slow trust decay and independent idle wandering returns.
7. **Loss of Wekinator while person present:** fail-closed fear; no unverified ML prediction can raise trust.
8. **Loss of Processing serial:** Arduino independently drives targets back towards zero after timeout.

## Tune these values

`ShyCatUnified.pde`:
- `TRUST_GAIN_PER_SECOND` / `TRUST_LOSS_PER_SECOND` / `TRUST_DECAY_UNOBSERVED`
- `MIMIC_START_TRUST = 0.75` / `MIMIC_FULL_TRUST = 0.95`
- `IDLE_OPENNESS` / `IDLE_WANDER_AMPLITUDE`
- `STARTLE_DURATION_S` / `THREAT_STARTLE_THRESHOLD`
- `MAX_OPEN_STEPS`, `MOTOR_DIRECTION` (**match Arduino**)
- `CLOSED_ANGLE_DEG`, `OPEN_ANGLE_DEG`, `PIVOT_X`, `ARM_LENGTH` to match the actual geometry

`ShyCatFourSteppers.ino`:
- `MAX_OPEN_STEPS`, `MOTOR_DIRECTION` (**match Processing**)
- `MAX_SPEED_STEPS_PER_S`, `ACCEL_STEPS_PER_S2`

`shy_cat_pose.py`:
- `CAMERA_INDEX`, `CAMERA_BACKEND_AVFOUNDATION`, `FLIP_CAMERA`
- Feature normalisation/smoothing only when matching your original model or planning to retrain

## Verification status

Python **syntax** and a synthetic sequence of 12-feature inputs have been checked. The Processing and Arduino code were manually checked for matching packet/OSC interfaces and bounds but **could not be compiled on a Processing/Arduino installation or run on your actual hardware in this environment**. Expect to calibrate physical positions and verify the saved Wekinator predictions before enabling serial.
