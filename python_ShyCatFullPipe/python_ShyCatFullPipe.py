"""Camera pose -> 12 unchanged Wekinator inputs + Processing presence/features."""

from collections import deque
from pathlib import Path
import math
import statistics
import time

import cv2
import mediapipe as mp
from pythonosc import udp_client

# Camera and OSC settings.
WEK_HOST, WEK_PORT = "127.0.0.1", 6448
PROCESSING_HOST, PROCESSING_PORT = "127.0.0.1", 12001
CAMERA_INDEX = 0
CAMERA_BACKEND_AVFOUNDATION = True  # macOS; False for Windows/Linux
FLIP_CAMERA = False  # Keep the orientation used for Wekinator training.
MODEL_PATH = Path(__file__).with_name("pose_landmarker_lite.task")
PREVIEW = True
PRINT_FEATURES = False
POSE_CONFIDENCE = 0.55

# Preserve these values to keep the same Wekinator input scale.
WRIST_SPEED_AT_ONE = 2.3
BODY_SPEED_AT_ONE = 1.8
VARIABILITY_AT_ONE = 0.9
ACCELERATION_AT_ONE = 5.0
APPROACH_AT_ONE = 1.0
VELOCITY_WINDOW_S = 0.10
VARIABILITY_WINDOW_S = 0.60
HISTORY_S = 0.90

LEFT_SHOULDER, RIGHT_SHOULDER = 11, 12
LEFT_WRIST, RIGHT_WRIST = 15, 16
LEFT_HIP, RIGHT_HIP = 23, 24
TRACKED = [
    LEFT_SHOULDER, RIGHT_SHOULDER, LEFT_WRIST, RIGHT_WRIST,
    LEFT_HIP, RIGHT_HIP,
]


def clip(x, lo=0.0, hi=1.0):
    return max(lo, min(hi, x))


def distance(p, q):
    return math.hypot(p[0] - q[0], p[1] - q[1])


class OneEuro:
    """Smooth one coordinate, responding faster to larger movements."""

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

        dt = clip(t - self.last_t, 1e-3, 0.25)
        raw_derivative = (value - self.prev) / dt
        self.dx += self.alpha(self.d_cutoff, dt) * (raw_derivative - self.dx)

        cutoff = self.min_cutoff + self.beta * abs(self.dx)
        self.prev += self.alpha(cutoff, dt) * (value - self.prev)
        self.last_t = t
        return self.prev


class FeatureExtractor:
    def __init__(self):
        self.filters = {index: (OneEuro(), OneEuro()) for index in TRACKED}
        self.samples = deque()        # (time, coordinates, shoulder width)
        self.speed_samples = deque()  # (time, body speed)
        self.previous_body_speed = 0.0
        self.previous_body_time = None

    def reset(self):
        self.__init__()

    def sample_at_or_before(self, t):
        for sample in reversed(self.samples):
            if sample[0] <= t:
                return sample
        return self.samples[0] if self.samples else None

    def process(self, landmarks, t):
        coords = {}
        for idx in TRACKED:
            xfilter, yfilter = self.filters[idx]
            px, py = landmarks[idx].x, landmarks[idx].y

            # Hold briefly occluded wrists instead of treating jumps as threats.
            if (idx in (LEFT_WRIST, RIGHT_WRIST)
                    and getattr(landmarks[idx], "visibility", 1.0) < 0.45
                    and xfilter.prev is not None):
                px, py = xfilter.prev, yfilter.prev

            coords[idx] = (xfilter.update(px, t), yfilter.update(py, t))

        sl, sr = coords[LEFT_SHOULDER], coords[RIGHT_SHOULDER]
        shoulder_center_x = (sl[0] + sr[0]) / 2.0
        shoulder_width = max(0.065, distance(sl, sr))

        self.samples.append((t, coords, shoulder_width))
        while len(self.samples) > 2 and self.samples[0][0] < t - HISTORY_S:
            self.samples.popleft()

        old_t, old_coords, old_width = self.sample_at_or_before(
            t - VELOCITY_WINDOW_S
        )
        dt = max(0.025, t - old_t)
        reference_width = max(0.065, (shoulder_width + old_width) / 2.0)

        centre_old = (
            old_coords[LEFT_SHOULDER][0] + old_coords[RIGHT_SHOULDER][0]
        ) / 2
        cy_now = (sl[1] + sr[1]) / 2
        cy_old = (
            old_coords[LEFT_SHOULDER][1] + old_coords[RIGHT_SHOULDER][1]
        ) / 2

        def relative_motion(idx):
            # Wrist speed relative to shoulders removes body translation.
            p = (
                (coords[idx][0] - shoulder_center_x) / shoulder_width,
                (coords[idx][1] - cy_now) / shoulder_width,
            )
            q = (
                (old_coords[idx][0] - centre_old) / old_width,
                (old_coords[idx][1] - cy_old) / old_width,
            )
            return distance(p, q) / dt

        left_speed = relative_motion(LEFT_WRIST)
        right_speed = relative_motion(RIGHT_WRIST)

        body_speed = sum(distance(coords[i], old_coords[i]) for i in TRACKED)
        body_speed /= len(TRACKED) * reference_width * dt

        self.speed_samples.append((t, body_speed))
        while (len(self.speed_samples) > 2
               and self.speed_samples[0][0] < t - VARIABILITY_WINDOW_S):
            self.speed_samples.popleft()

        variability = (
            statistics.pstdev(x[1] for x in self.speed_samples)
            if len(self.speed_samples) >= 2 else 0.0
        )

        acceleration = 0.0
        if self.previous_body_time is not None:
            adt = max(0.025, t - self.previous_body_time)
            acceleration = abs(body_speed - self.previous_body_speed) / adt

        self.previous_body_speed, self.previous_body_time = body_speed, t
        approach = ((shoulder_width / old_width) - 1.0) / dt

        # Exact original order. Approach: 0.5 = stationary.
        features = [
            clip(coords[LEFT_WRIST][0]),
            clip(coords[LEFT_WRIST][1]),
            clip(coords[RIGHT_WRIST][0]),
            clip(coords[RIGHT_WRIST][1]),
            clip(shoulder_center_x),
            clip(shoulder_width),
            clip(left_speed / WRIST_SPEED_AT_ONE),
            clip(right_speed / WRIST_SPEED_AT_ONE),
            clip(body_speed / BODY_SPEED_AT_ONE),
            clip(variability / VARIABILITY_AT_ONE),
            clip(acceleration / ACCELERATION_AT_ONE),
            clip(0.5 + 0.5 * approach / APPROACH_AT_ONE),
        ]
        return features, coords


def pose_is_valid(landmarks):
    return (
        len(landmarks) >= 25
        and all(
            getattr(landmarks[i], "visibility", 1.0) >= POSE_CONFIDENCE
            for i in (LEFT_SHOULDER, RIGHT_SHOULDER, LEFT_HIP, RIGHT_HIP)
        )
    )


def show_preview(frame, present, points):
    """Draw camera feedback; return True when Q is pressed."""
    h, w = frame.shape[:2]

    for i in (LEFT_WRIST, RIGHT_WRIST) if present else ():
        x, y = points[i]
        cv2.circle(frame, (int(x * w), int(y * h)), 8, (0, 240, 170), -1)

    cv2.putText(
        frame, f"Person: {'YES' if present else 'NO'}", (20, 36),
        cv2.FONT_HERSHEY_SIMPLEX, 0.9,
        (0, 220, 50) if present else (0, 70, 255), 2,
    )
    cv2.putText(
        frame, "Q: quit", (20, 72),
        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2,
    )
    cv2.imshow("Shy Cat - MediaPipe", frame)
    return cv2.waitKey(1) & 0xFF == ord("q")


def main():
    if not MODEL_PATH.exists():
        raise FileNotFoundError(f"Missing model: {MODEL_PATH}")

    wek_client = udp_client.SimpleUDPClient(WEK_HOST, WEK_PORT)
    proc_client = udp_client.SimpleUDPClient(PROCESSING_HOST, PROCESSING_PORT)
    backend = cv2.CAP_AVFOUNDATION if CAMERA_BACKEND_AVFOUNDATION else cv2.CAP_ANY
    cap = cv2.VideoCapture(CAMERA_INDEX, backend)

    try:
        if not cap.isOpened():
            raise RuntimeError(f"Could not open camera index {CAMERA_INDEX}")

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 540)

        options = mp.tasks.vision.PoseLandmarkerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path=str(MODEL_PATH)),
            running_mode=mp.tasks.vision.RunningMode.VIDEO,
            num_poses=1,
            min_pose_detection_confidence=POSE_CONFIDENCE,
            min_pose_presence_confidence=POSE_CONFIDENCE,
            min_tracking_confidence=POSE_CONFIDENCE,
        )

        extractor = FeatureExtractor()
        start_time = time.monotonic()
        timestamp_ms = -1
        had_pose = False
        last_print = 0.0

        print(
            "Camera started. OSC -> Wekinator :6448 and Processing :12001. "
            "Press Q to quit."
        )

        with mp.tasks.vision.PoseLandmarker.create_from_options(options) as landmarker:
            while True:
                ok, frame = cap.read()
                if not ok:
                    print("Camera read failed; presence cleared.")
                    break

                if FLIP_CAMERA:
                    frame = cv2.flip(frame, 1)

                t = time.monotonic() - start_time
                timestamp_ms = max(timestamp_ms + 1, int(t * 1000))

                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
                result = landmarker.detect_for_video(image, timestamp_ms)
                landmarks = (
                    result.pose_landmarks[0] if result.pose_landmarks else None
                )
                present = landmarks is not None and pose_is_valid(landmarks)
                proc_client.send_message("/person/present", int(present))

                points = {}
                if present:
                    if not had_pose:
                        extractor.reset()

                    features, points = extractor.process(landmarks, t)
                    wek_client.send_message("/wek/inputs", features)
                    proc_client.send_message("/person/features", features)

                    if PRINT_FEATURES and t - last_print > 0.2:
                        print("12 features:", [round(v, 3) for v in features])
                        last_print = t

                had_pose = present

                if PREVIEW and show_preview(frame, present, points):
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