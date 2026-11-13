## IDE CPS Group 5 - "Shy Cat" - MediaPipe body tracking and OSC output to Wekinator and Processing :)

from __future__ import annotations

from collections import deque
from pathlib import Path
import math
import statistics
import time

import cv2
import mediapipe as mp
from pythonosc import udp_client

WEK_HOST, WEK_PORT = "127.0.0.1", 6448
PROCESSING_HOST, PROCESSING_PORT = "127.0.0.1", 12001
CAMERA_INDEX = 0
CAMERA_BACKEND_AVFOUNDATION = True  # macOS; set False for Windows/Linux
FLIP_CAMERA = False  # Match orientation used when training Wekinator!
MODEL_PATH = Path(__file__).with_name("pose_landmarker_lite.task")
PREVIEW = True
PRINT_FEATURES = False
POSE_CONFIDENCE = 0.55

# Motion normalisation: tune only if retraining / matching previous version.
WRIST_SPEED_AT_ONE = 2.3        # torso widths per second
BODY_SPEED_AT_ONE = 1.8         # torso widths per second
VARIABILITY_AT_ONE = 0.9        # torso widths per second (std dev)
ACCELERATION_AT_ONE = 5.0       # torso widths per second^2
APPROACH_AT_ONE = 1.0           # shoulder width changes per second
VELOCITY_WINDOW_S = 0.10
VARIABILITY_WINDOW_S = 0.60
HISTORY_S = 0.90

# MediaPipe landmark indices
LEFT_SHOULDER, RIGHT_SHOULDER = 11, 12
LEFT_WRIST, RIGHT_WRIST = 15, 16
LEFT_HIP, RIGHT_HIP = 23, 24
TRACKED = [LEFT_SHOULDER, RIGHT_SHOULDER, LEFT_WRIST, RIGHT_WRIST,
           LEFT_HIP, RIGHT_HIP]


def clip(x: float, lo: float = 0.0, hi: float = 1.0) -> float:
    return max(lo, min(hi, x))


def distance(p, q) -> float:
    return math.hypot(p[0] - q[0], p[1] - q[1])


class OneEuro:
    """Adaptive position smoothing; a fresh filter for each scalar coordinate."""

    def __init__(self, min_cutoff=1.2, beta=0.08, d_cutoff=1.0):
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self.prev = None
        self.dx = 0.0
        self.last_t = None

    @staticmethod
    def alpha(cutoff, dt):
        tau = 1.0 / (2 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def update(self, value, t):
        if self.last_t is None:
            self.last_t, self.prev = t, value
            return value
        dt = max(1e-3, min(0.25, t - self.last_t))
        raw_derivative = (value - self.prev) / dt
        self.dx += self.alpha(self.d_cutoff, dt) * (raw_derivative - self.dx)
        cutoff = self.min_cutoff + self.beta * abs(self.dx)
        self.prev += self.alpha(cutoff, dt) * (value - self.prev)
        self.last_t = t
        return self.prev


class FeatureExtractor:
    def __init__(self):
        self.filters = {index: (OneEuro(), OneEuro()) for index in TRACKED}
        self.samples = deque()  # (seconds, coords_by_landmark, shoulder_width)
        self.speed_samples = deque()  # (seconds, average_body_speed)
        self.previous_body_speed = 0.0
        self.previous_body_time = None

    def reset(self):
        self.__init__()

    def sample_at_or_before(self, t):
        # Use the closest earlier sample for stable finite-difference velocity.
        for sample in reversed(self.samples):
            if sample[0] <= t:
                return sample
        return self.samples[0] if self.samples else None

    def process(self, landmarks, t):
        coords = {}
        for idx in TRACKED:
            xfilter, yfilter = self.filters[idx]
            px, py = landmarks[idx].x, landmarks[idx].y
            # Keep the last reliable wrist when it becomes briefly occluded.
            # Otherwise a low-confidence wrist jump may be learned as a threat.
            if idx in (LEFT_WRIST, RIGHT_WRIST) and \
                    getattr(landmarks[idx], "visibility", 1.0) < 0.45 and \
                    xfilter.prev is not None:
                px, py = xfilter.prev, yfilter.prev
            coords[idx] = (xfilter.update(px, t), yfilter.update(py, t))

        sl, sr = coords[LEFT_SHOULDER], coords[RIGHT_SHOULDER]
        shoulder_center_x = (sl[0] + sr[0]) / 2.0
        shoulder_width = max(0.065, distance(sl, sr))

        self.samples.append((t, coords, shoulder_width))
        while len(self.samples) > 2 and self.samples[0][0] < t - HISTORY_S:
            self.samples.popleft()

        older = self.sample_at_or_before(t - VELOCITY_WINDOW_S)
        dt = max(0.025, t - older[0]) if older else 0.1
        old_coords = older[1] if older else coords
        old_width = older[2] if older else shoulder_width
        reference_width = max(0.065, (shoulder_width + old_width) / 2.0)

        def relative_motion(idx):
            # Translation compensation makes movement less sensitive to walking.
            centre_now = (coords[LEFT_SHOULDER][0] + coords[RIGHT_SHOULDER][0]) / 2
            centre_old = (old_coords[LEFT_SHOULDER][0] + old_coords[RIGHT_SHOULDER][0]) / 2
            cy_now = (coords[LEFT_SHOULDER][1] + coords[RIGHT_SHOULDER][1]) / 2
            cy_old = (old_coords[LEFT_SHOULDER][1] + old_coords[RIGHT_SHOULDER][1]) / 2
            p = ((coords[idx][0] - centre_now) / shoulder_width,
                 (coords[idx][1] - cy_now) / shoulder_width)
            q = ((old_coords[idx][0] - centre_old) / old_width,
                 (old_coords[idx][1] - cy_old) / old_width)
            return distance(p, q) / dt

        left_speed = relative_motion(LEFT_WRIST)
        right_speed = relative_motion(RIGHT_WRIST)

        # Body speed: six tracked points, compensating for camera distance.
        body_speed = sum(distance(coords[i], old_coords[i]) for i in TRACKED)
        body_speed /= len(TRACKED) * reference_width * dt

        self.speed_samples.append((t, body_speed))
        while len(self.speed_samples) > 2 and self.speed_samples[0][0] < t - VARIABILITY_WINDOW_S:
            self.speed_samples.popleft()
        variability = (statistics.pstdev(x[1] for x in self.speed_samples)
                       if len(self.speed_samples) >= 2 else 0.0)

        if self.previous_body_time is None:
            acceleration = 0.0
        else:
            adt = max(0.025, t - self.previous_body_time)
            acceleration = abs(body_speed - self.previous_body_speed) / adt
        self.previous_body_speed, self.previous_body_time = body_speed, t

        # Positive means moving closer; encoded 0.5 for stationary.
        approach = ((shoulder_width / old_width) - 1.0) / dt

        # EXACT FEATURE ORDER (all 12 in range [0, 1]):
        features = [
            clip(coords[LEFT_WRIST][0]),                         # 1 left wrist X
            clip(coords[LEFT_WRIST][1]),                         # 2 left wrist Y
            clip(coords[RIGHT_WRIST][0]),                        # 3 right wrist X
            clip(coords[RIGHT_WRIST][1]),                        # 4 right wrist Y
            clip(shoulder_center_x),                             # 5 torso X
            clip(shoulder_width),                                # 6 torso scale
            clip(left_speed / WRIST_SPEED_AT_ONE),               # 7 left wrist speed
            clip(right_speed / WRIST_SPEED_AT_ONE),              # 8 right wrist speed
            clip(body_speed / BODY_SPEED_AT_ONE),                # 9 body speed
            clip(variability / VARIABILITY_AT_ONE),              # 10 variability
            clip(acceleration / ACCELERATION_AT_ONE),            # 11 acceleration
            clip(0.5 + 0.5 * approach / APPROACH_AT_ONE),        # 12 approach/recession
        ]
        return features, coords


def pose_is_valid(landmarks):
    return (len(landmarks) >= 25 and
            all(getattr(landmarks[i], "visibility", 1.0) >= POSE_CONFIDENCE
                for i in (LEFT_SHOULDER, RIGHT_SHOULDER, LEFT_HIP, RIGHT_HIP)))


def main():
    if not MODEL_PATH.exists():
        raise FileNotFoundError(
            f"Missing {MODEL_PATH.name}. Copy pose_landmarker_lite.task into {MODEL_PATH.parent}"
        )

    wek_client = udp_client.SimpleUDPClient(WEK_HOST, WEK_PORT)
    proc_client = udp_client.SimpleUDPClient(PROCESSING_HOST, PROCESSING_PORT)
    backend = cv2.CAP_AVFOUNDATION if CAMERA_BACKEND_AVFOUNDATION else cv2.CAP_ANY
    cap = cv2.VideoCapture(CAMERA_INDEX, backend)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera index {CAMERA_INDEX}")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 540)
    extractor = FeatureExtractor()
    BaseOptions = mp.tasks.BaseOptions
    PoseLandmarker = mp.tasks.vision.PoseLandmarker
    PoseOptions = mp.tasks.vision.PoseLandmarkerOptions
    RunningMode = mp.tasks.vision.RunningMode
    options = PoseOptions(
        base_options=BaseOptions(model_asset_path=str(MODEL_PATH)),
        running_mode=RunningMode.VIDEO,
        num_poses=1,
        min_pose_detection_confidence=POSE_CONFIDENCE,
        min_pose_presence_confidence=POSE_CONFIDENCE,
        min_tracking_confidence=POSE_CONFIDENCE,
    )
    start_time = time.monotonic()
    last_timestamp_ms = -1
    had_pose = False
    last_print = 0.0
    print("Camera started. OSC -> Wekinator :6448 and Processing :12001. Press Q to quit.")

    try:
        with PoseLandmarker.create_from_options(options) as landmarker:
            while True:
                ok, frame = cap.read()
                if not ok:
                    proc_client.send_message("/person/present", 0)
                    print("Camera read failed; presence cleared.")
                    break
                if FLIP_CAMERA:
                    frame = cv2.flip(frame, 1)

                t = time.monotonic() - start_time
                timestamp_ms = max(last_timestamp_ms + 1, int(t * 1000))
                last_timestamp_ms = timestamp_ms
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
                result = landmarker.detect_for_video(image, timestamp_ms)
                lm = result.pose_landmarks[0] if result.pose_landmarks else None
                present = lm is not None and pose_is_valid(lm)
                proc_client.send_message("/person/present", int(present))

                if present:
                    if not had_pose:
                        extractor.reset()  # avoid explosive velocity on reacquisition
                    features, points = extractor.process(lm, t)
                    # Same 12 floats, both OSC destinations. No Wekinator changes needed.
                    wek_client.send_message("/wek/inputs", features)
                    proc_client.send_message("/person/features", features)
                    had_pose = True
                    if PREVIEW:
                        h, w = frame.shape[:2]
                        for i in (LEFT_WRIST, RIGHT_WRIST):
                            x, y = points[i]
                            cv2.circle(frame, (int(x * w), int(y * h)), 8,
                                       (0, 240, 170), -1)
                    if PRINT_FEATURES and t - last_print > 0.2:
                        print("12 features:", [round(v, 3) for v in features])
                        last_print = t
                else:
                    had_pose = False
                    # Do not send fake zero inputs to Wekinator.

                if PREVIEW:
                    cv2.putText(frame, f"Person: {'YES' if present else 'NO'}",
                                (20, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                                (0, 220, 50) if present else (0, 70, 255), 2)
                    cv2.putText(frame, "Q: quit", (20, 72),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    cv2.imshow("Shy Cat - MediaPipe", frame)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
    finally:
        try:
            proc_client.send_message("/person/present", 0)
        finally:
            cap.release()
            if PREVIEW:
                cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
