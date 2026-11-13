# IDE CPS Group 5 - "Shy Cat" - MediaPipe body tracking and OSC output to Wekinator and Processing :)
# If "pose_landmarker_lite.task" isnt in the same folder as the code, it wont work. 
# You must also run the code from that folder or it wont find "pose_landmarker_lite.task".

# This Python code was written by Josh Hill with guidance, technical help and debugging from ChatGPT.

import math
import os
import statistics
import time
from collections import deque

# OpenCV handles the camera, MediaPipe finds the body landmarks, and pythonosc sends data to other programs.
import cv2
import mediapipe as mp
from pythonosc import udp_client

# Camera and tracking settings. True means on, False means off.
CAMERA_NUMBER = 0
USE_MAC_CAMERA = True
FLIP_CAMERA = False
SHOW_CAMERA = True
CONFIDENCE = 0.55

# This is the trained MediaPipe model file that detects body poses.
MODEL_FILE = "pose_landmarker_lite.task"

WEKINATOR_IP = "127.0.0.1"
WEKINATOR_PORT = 6448
PROCESSING_IP = "127.0.0.1"
PROCESSING_PORT = 12001

# Change these to adjust how Wekinator and Processing react to movement. It may cause some weird behaviour in the Wekinator models though.
MAX_WRIST_SPEED = 2.3
MAX_BODY_SPEED = 1.8
MAX_VARIABILITY = 0.9
MAX_ACCELERATION = 5.0
MAX_APPROACH = 1.0

SPEED_WINDOW = 0.10 # Seconds used to calculate movement speed
VARIABILITY_WINDOW = 0.60 # Seconds of recent speeds used to measure how uneven movement is
HISTORY_LENGTH = 0.90 # How long we keep old body positions (in seconds)

# MediaPipe landmarks - Dont change
LEFT_SHOULDER = 11
RIGHT_SHOULDER = 12
LEFT_WRIST = 15
RIGHT_WRIST = 16
LEFT_HIP = 23
RIGHT_HIP = 24

BODY_PARTS = [LEFT_SHOULDER, RIGHT_SHOULDER, LEFT_WRIST, RIGHT_WRIST, LEFT_HIP, RIGHT_HIP]

# Keeps a value between 0 and 1 so that Wekinator doesnt get confused with larger or smaller numbers.
def limit(value):
    # min stops the value going over 1, then max stops it going below 0.
    return max(0.0, min(1.0, value))

# Takes x and y of two points and calculates the distance between them.
def distance(point1, point2):
    # [0] means X and [1] means Y.
    change_x = point1[0] - point2[0]
    change_y = point1[1] - point2[1]
    # Pythagoras: square the X and Y changes, add them and take the square root.
    return math.sqrt(change_x ** 2 + change_y ** 2)

# Checks that enough points of a person are visible before sending data to Wekinator.
def person_detected(landmarks):
    # If MediaPipe found nobody, or returned too few landmarks, say no person.
    if landmarks is None or len(landmarks) < 25:
        return False
    # Check confidence for the shoulders and hips.
    for part in [LEFT_SHOULDER, RIGHT_SHOULDER, LEFT_HIP, RIGHT_HIP]:
        if landmarks[part].visibility < CONFIDENCE:
            return False
        return True


# Reduces jitter in the detected positions. It smooths more when movement is slow, but lets faster movements through more quickly.
class SmoothValue:
    def __init__(self):
        self.previous_value = None
        self.smoothed_speed = 0.0
        self.previous_time = None

    # Works out how strongly to mix a new reading into the smoothed value.
    # Larger cutoff means less smoothing / a quicker response.
    def smoothing_amount(self, cutoff, seconds):
        time_constant = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + time_constant / seconds)

    # Take a new position value and return a smoother version of it.
    def update(self, value, current_time):
        if self.previous_time is None:
            self.previous_time = current_time
            self.previous_value = value
            return value

        # Find how much time passed. The limits avoid dividing by zero or a huge gap.
        seconds = max(0.001, min(0.25, current_time - self.previous_time))
        # Estimate how quickly this X or Y coordinate has changed.
        speed = (value - self.previous_value) / seconds
        amount = self.smoothing_amount(1.0, seconds)
        # Gradually update the estimated speed instead of using the raw speed directly.
        self.smoothed_speed += amount * (speed - self.smoothed_speed)

        # Faster movement increases cutoff, so the filter follows it more closely.
        cutoff = 1.2 + 0.08 * abs(self.smoothed_speed)
        amount = self.smoothing_amount(cutoff, seconds)
        # Move the old smoothed value part of the way towards the new reading.
        self.previous_value += amount * (value - self.previous_value)
        self.previous_time = current_time
        return self.previous_value

# This class takes the MediaPipe landmarks and calculates the 12 Wekinator inputs; after applying the smoothing filters.
class BodyTracker:
    def __init__(self):
        # This runs when we create a BodyTracker, e.g. tracker = BodyTracker().
        # Make separate X and Y smoothing filters for each body part.
        # {} creates an empty dictionary, storing one filter for each named body part.
        self.x_filters = {}
        self.y_filters = {}
        for part in BODY_PARTS:
            self.x_filters[part] = SmoothValue()
            self.y_filters[part] = SmoothValue()

        # Store recent positions and body speeds for movement calculations.
        # These histories keep old readings so we can compare movement over time.
        self.position_history = deque()
        self.speed_history = deque()
        self.last_body_speed = 0.0
        self.last_body_time = None

    # Read all six body parts from MediaPipe and smooth their coordinates.
    def get_smoothed_positions(self, landmarks, current_time):
        positions = {} # Empty dictionary to store the results

        for part in BODY_PARTS:
            # MediaPipe gives X and Y as fractions of the image width and height.
            x = landmarks[part].x
            y = landmarks[part].y

            # A temporarily hidden wrist should not suddenly jump position.
            # Only use this special rule on wrists, not shoulders or hips.
            if part in [LEFT_WRIST, RIGHT_WRIST]:
                if landmarks[part].visibility < 0.45:
                    if self.x_filters[part].previous_value is not None:
                        x = self.x_filters[part].previous_value
                        y = self.y_filters[part].previous_value

            # Each body part has its own X and Y smoothing filters.
            x = self.x_filters[part].update(x, current_time)
            y = self.y_filters[part].update(y, current_time)
            positions[part] = (x, y) # Save the pair of smoothed coordinates

        return positions

    # Calculate the speed of one wrist using current and older frames.
    def wrist_speed(self, part, positions, old_positions,
                    shoulder_width, old_shoulder_width, seconds):
        """Measure wrist movement relative to the shoulders, not the camera."""
        # Find the midpoint between both shoulders in the current frame.
        # [0] selects X, [1] selects Y, and / 2 gives the average.
        centre_x = (positions[LEFT_SHOULDER][0] + positions[RIGHT_SHOULDER][0]) / 2
        centre_y = (positions[LEFT_SHOULDER][1] + positions[RIGHT_SHOULDER][1]) / 2
        # Do the same with the older frame.
        old_centre_x = (old_positions[LEFT_SHOULDER][0] + old_positions[RIGHT_SHOULDER][0]) / 2
        old_centre_y = (old_positions[LEFT_SHOULDER][1] + old_positions[RIGHT_SHOULDER][1]) / 2

        # Subtract the shoulder centre to remove sideways body movement.
        # Divide by shoulder width to reduce the effect of distance from the camera.
        wrist_now = ((positions[part][0] - centre_x) / shoulder_width,
                     (positions[part][1] - centre_y) / shoulder_width)
        wrist_before = ((old_positions[part][0] - old_centre_x) / old_shoulder_width,
                        (old_positions[part][1] - old_centre_y) / old_shoulder_width)

        # Compare the two wrist positions, then divide distance moved by time = speed.
        return distance(wrist_now, wrist_before) / seconds

    # Main method: turns MediaPipe landmarks into the 12 Wekinator numbers.
    def calculate_features(self, landmarks, current_time):
        # Call another method in this class to get smoothed X and Y values.
        positions = self.get_smoothed_positions(landmarks, current_time)

        left_shoulder = positions[LEFT_SHOULDER]
        right_shoulder = positions[RIGHT_SHOULDER]
        # Torso X = horizontal midpoint of the shoulders.
        torso_x = (left_shoulder[0] + right_shoulder[0]) / 2
        # 0.065 is a minimum to avoid division by very small numbers.
        shoulder_width = max(0.065, distance(left_shoulder, right_shoulder))

        # Save this frame. Keep no more than 0.9 seconds of history.
        self.position_history.append((current_time, positions, shoulder_width))
        # Remove readings older than HISTORY_LENGTH, but keep at least two.
        while len(self.position_history) > 2:
            if self.position_history[0][0] >= current_time - HISTORY_LENGTH:
                break
            self.position_history.popleft()

        # Find an older frame, ideally 0.1 seconds ago, to calculate speed.
        # [0] gets the oldest stored frame as a fallback.
        old_frame = self.position_history[0]
        # Search backwards to find a frame about SPEED_WINDOW seconds ago.
        for frame in reversed(self.position_history):
            if frame[0] <= current_time - SPEED_WINDOW:
                old_frame = frame
                break

        # Split the saved frame into its three stored values.
        old_time, old_positions, old_shoulder_width = old_frame
        # Use at least 0.025 seconds so speed does not become excessively large.
        seconds = max(0.025, current_time - old_time)
        average_width = max(0.065, (shoulder_width + old_shoulder_width) / 2)

        # Run the wrist speed method once for each wrist.
        left_speed = self.wrist_speed(LEFT_WRIST, positions, old_positions,
                                      shoulder_width, old_shoulder_width, seconds)
        right_speed = self.wrist_speed(RIGHT_WRIST, positions, old_positions,
                                       shoulder_width, old_shoulder_width, seconds)

        # Average speed of all six tracked body parts.
        # Start a running total, then add each body part's distance moved.
        total_movement = 0.0
        for part in BODY_PARTS:
            total_movement += distance(positions[part], old_positions[part])
        # Divide by number of parts, shoulder width, and time to get average relative speed.
        body_speed = total_movement / (len(BODY_PARTS) * average_width * seconds)

        # Variability measures how much the body speed changes recently.
        # Save the new speed with its timestamp for comparison with recent speeds.
        self.speed_history.append((current_time, body_speed))
        while len(self.speed_history) > 2:
            if self.speed_history[0][0] >= current_time - VARIABILITY_WINDOW:
                break
            self.speed_history.popleft()

        # Make a simple list containing only the speed values, not their timestamps.
        recent_speeds = []
        for sample_time, speed in self.speed_history:
            recent_speeds.append(speed)

        # Standard deviation describes how spread out the recent speeds are.
        # Constant speed = low variability; changing speed = higher variability.
        variability = 0.0
        if len(recent_speeds) >= 2:
            variability = statistics.pstdev(recent_speeds)

        # Acceleration measures how quickly body speed is changing.
        # Compare speed with the previous frame. abs() makes changes positive
        # whether the person speeds up or slows down.
        acceleration = 0.0
        if self.last_body_time is not None:
            elapsed = max(0.025, current_time - self.last_body_time)
            acceleration = abs(body_speed - self.last_body_speed) / elapsed
        # Remember this frame's speed and time for next time.
        self.last_body_speed = body_speed
        self.last_body_time = current_time

        # Larger shoulders on screen suggest the person is approaching.
        # If shoulders get bigger in the image, approach is positive; smaller = negative.
        approach = ((shoulder_width / old_shoulder_width) - 1.0) / seconds

        # Keep this exact order: Wekinator was trained with these 12 inputs.
        # limit() puts each feature into the 0 to 1 range expected by Wekinator.
        # Dividing by a MAX value scales speeds etc. into that range.
        features = [
            limit(positions[LEFT_WRIST][0]),         # 1. Left wrist X
            limit(positions[LEFT_WRIST][1]),         # 2. Left wrist Y
            limit(positions[RIGHT_WRIST][0]),        # 3. Right wrist X
            limit(positions[RIGHT_WRIST][1]),        # 4. Right wrist Y
            limit(torso_x),                          # 5. Torso X
            limit(shoulder_width),                   # 6. Shoulder width
            limit(left_speed / MAX_WRIST_SPEED),     # 7. Left wrist speed
            limit(right_speed / MAX_WRIST_SPEED),    # 8. Right wrist speed
            limit(body_speed / MAX_BODY_SPEED),      # 9. Body speed
            limit(variability / MAX_VARIABILITY),    # 10. Movement variability
            limit(acceleration / MAX_ACCELERATION),  # 11. Acceleration
            limit(0.5 + 0.5 * approach / MAX_APPROACH)  # 12. Approach / retreat
        ]

        # Return both the 12 inputs and the smoothed positions for other parts of the code.
        return features, positions


# -------------------- RUN THE CAMERA --------------------

# This function starts the camera, runs MediaPipe, and sends the results continuously.
def main():
    # Stop with a clear error if the model file cannot be found.
    if not os.path.isfile(MODEL_FILE):
        raise FileNotFoundError("pose_landmarker_lite.task not found in the current working directory")

    # Set up two OSC connections: one for Wekinator and one for Processing.
    wekinator = udp_client.SimpleUDPClient(WEKINATOR_IP, WEKINATOR_PORT)
    processing = udp_client.SimpleUDPClient(PROCESSING_IP, PROCESSING_PORT)

    # Use the Mac-specific camera connection if selected, otherwise the default.
    if USE_MAC_CAMERA:
        camera = cv2.VideoCapture(CAMERA_NUMBER, cv2.CAP_AVFOUNDATION)
    else:
        camera = cv2.VideoCapture(CAMERA_NUMBER)

    if not camera.isOpened():
        raise RuntimeError("Camera could not be opened")

    # Request a camera frame size of 960 by 540 pixels.
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 540)

    # Load the MediaPipe model. VIDEO means we process frames in time order.
    settings = mp.tasks.vision.PoseLandmarkerOptions(
        base_options=mp.tasks.BaseOptions(model_asset_path=str(MODEL_FILE)),
        running_mode=mp.tasks.vision.RunningMode.VIDEO,
        num_poses=1,
        min_pose_detection_confidence=CONFIDENCE,
        min_pose_presence_confidence=CONFIDENCE,
        min_tracking_confidence=CONFIDENCE
    )

    # Create a tracker that remembers previous movement between frames.
    tracker = BodyTracker()
    person_was_present = False # Remember whether a person appeared last frame
    start_time = time.monotonic() # Start a clock for measuring elapsed time
    last_timestamp = -1 # Used to keep MediaPipe frame times increasing

    print("Shy Cat running. Press Q in the camera window to quit.")

    # finally (below) will release the camera even if an error happens.
    try:
        with mp.tasks.vision.PoseLandmarker.create_from_options(settings) as pose_model:
            # Keep repeating until Q is pressed, the camera fails, or the program stops.
            while True:
                # Read an image from the camera. success tells us whether it worked.
                success, frame = camera.read()
                if not success:
                    print("Could not read camera frame")
                    break

                # Optionally mirror the image horizontally.
                if FLIP_CAMERA:
                    frame = cv2.flip(frame, 1)

                # MediaPipe needs a timestamp in milliseconds for each frame.
                current_time = time.monotonic() - start_time
                # Convert seconds to milliseconds. max() makes sure time always moves forward.
                timestamp = max(last_timestamp + 1, int(current_time * 1000))
                last_timestamp = timestamp

                # OpenCV uses BGR colours; MediaPipe expects RGB. Convert the format first.
                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)
                # Ask MediaPipe to detect the body landmarks in this camera frame.
                result = pose_model.detect_for_video(mp_image, timestamp)

                # If MediaPipe sees a pose, use the first one (we configured one person).
                landmarks = None
                if result.pose_landmarks:
                    landmarks = result.pose_landmarks[0]

                # Send Processing a 1 when present and a 0 when not present.
                present = person_detected(landmarks)
                processing.send_message("/person/present", int(present))

                if present:
                    # Start fresh when somebody re-enters the frame.
                    if not person_was_present:
                        tracker = BodyTracker()

                    # Calculate the 12 features, send them to Wekinator and Processing.
                    features, positions = tracker.calculate_features(landmarks, current_time)
                    wekinator.send_message("/wek/inputs", features)
                    processing.send_message("/person/features", features)
                    person_was_present = True

                    if SHOW_CAMERA:
                        # Get image size so normalised wrist positions can become pixel positions.
                        height, width = frame.shape[:2]
                        for part in [LEFT_WRIST, RIGHT_WRIST]:
                            x, y = positions[part]
                            # Draw a small coloured circle over each tracked wrist.
                            cv2.circle(frame, (int(x * width), int(y * height)),
                                                                  8, (0, 240, 170), -1)

                else:
                    person_was_present = False
                    # Don't send fake zero values to Wekinator.

                if SHOW_CAMERA:
                    # Display whether a person is present on the camera preview.
                    message = "Person: YES" if present else "Person: NO"
                    colour = (0, 220, 50) if present else (0, 70, 255)

                    cv2.putText(frame, message, (20, 36),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.9, colour, 2)

                    cv2.putText(frame, "ESC: quit", (20, 72),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                    cv2.imshow("Shy Cat - MediaPipe", frame)

                    # Stop the loop when Escape is pressed in the camera window.
                    if cv2.waitKey(1) & 0xFF == 27:
                        break

    # Close the camera and notify Processing that nobody is present.
    finally:
        processing.send_message("/person/present", 0)
        camera.release()

        if SHOW_CAMERA:
            cv2.destroyAllWindows()


# Only run main() when this file is executed directly, not when imported elsewhere.
if __name__ == "__main__":
    main()
