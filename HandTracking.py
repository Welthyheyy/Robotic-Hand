import cv2
import mediapipe as mp
import math
import serial
import time

# python HandTracking.py

# Initialize MediaPipe
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    model_complexity = 0,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.7
)

mp_draw = mp.solutions.drawing_utils

# Open webcam
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Open Serial port
ser = serial.Serial('/dev/cu.usbmodem1301', 9600)
smooth_angle = 90
Alpha = 0.2

prev_hand_angle = None
STEP_SENSITIVITY = 0.05   # degrees of hand rotation → 1 step


#Calculates distance between landmarks
def distance(a, b):
    return math.sqrt(
        (a.x - b.x) ** 2 +
        (a.y - b.y) ** 2
    )

# Maps finger angle to servo angle
def map_value(x, in_min, in_max, out_min, out_max):
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert BGR → RGB
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    result = hands.process(rgb)

    if result.multi_hand_landmarks:
        for hand_landmarks in result.multi_hand_landmarks:
            mp_draw.draw_landmarks(
                frame,
                hand_landmarks,
                mp_hands.HAND_CONNECTIONS
            )

            landmarks = hand_landmarks.landmark

            #Wrist
            # Compute hand angle
            wrist = landmarks[0]
            index_base = landmarks[5]

            dx = index_base.x - wrist.x
            dy = index_base.y - wrist.y

            hand_angle = math.degrees(math.atan2(dy, dx))

            last_step_time = 0
            STEP_INTERVAL = 0.05  # seconds (20 Hz max)


            if prev_hand_angle is None:
                prev_hand_angle = hand_angle
            else:
                delta = (hand_angle - prev_hand_angle + 180) % 360 - 180

                # DEADZONE (ignore tiny motion)
                now = time.time()
                if abs(delta) > 2 and now - last_step_time > STEP_INTERVAL:
                    step_delta = int(delta / STEP_SENSITIVITY)
                    step_delta = max(-50, min(50, step_delta))

                    if step_delta != 0:
                        ser.write(f"W:{step_delta}\n".encode())
                        last_step_time = now

                prev_hand_angle = hand_angle


    # Index finger
            index_tip = landmarks[8]
            index_base = landmarks[5]

            index_dist = distance(index_tip, index_base)
            last_sent_servo = None

            # Debug print
            #print(f"Index finger distance: {index_dist:.3f}")

            # Clamp distance
            index_dist = max(0.035, min(0.11, index_dist))

            raw_angle = map_value(index_dist, 0.035, 0.11, 180, 0)
            raw_angle = max(0, min(180, raw_angle))

            # Deadzone at extremes
            if raw_angle < 5:
                raw_angle = 0
            elif raw_angle > 175:
                raw_angle = 180

            # Adaptive smoothing
            alpha = 0.05 if raw_angle in (0,180) else 0.2
            smooth_angle += alpha * (raw_angle - smooth_angle)
            smooth_angle = int(smooth_angle)

            if last_sent_servo is None or abs(smooth_angle - last_sent_servo) >= 2:
                ser.write(f"S:{smooth_angle}\n".encode())
                last_sent_servo = smooth_angle


    cv2.imshow("Hand Tracking", frame)

    if cv2.waitKey(1) & 0xFF == 27:  # ESC key
        break

cap.release()
ser.close()
cv2.destroyAllWindows()
