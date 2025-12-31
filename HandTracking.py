import cv2
import mediapipe as mp
import math
import serial

# python HandTracking.py

smooth_index_angle = 90
smooth_middle_angle = 90
smooth_thumb_angle = 90
smooth_ring_pinky_angle = 90

index_last_sent_servo = None
middle_last_sent_servo = None
thumb_last_sent_servo = None
ring_pinky_last_sent_servo = None

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
ser = serial.Serial('/dev/cu.usbmodem11301', 9600)

WRIST_SENSITIVITY = 2
initial_hand_angle = None
angle_offset = 0

#Calculates angle between landmarks
def angle(tip, mid,base):
    v1_x = mid.x - base.x
    v1_y = mid.y - base.y

    v2_x = tip.x - mid.x
    v2_y = tip.y - mid.y


    # Calculate angle between vectors
    dot = v1_x * v2_x + v1_y * v2_y
    mag1 = math.sqrt(v1_x**2 + v1_y**2)
    mag2 = math.sqrt(v2_x**2 + v2_y**2)

    if mag1 > 0 and mag2 > 0:
        cos_angle = dot / (mag1 * mag2)
        cos_angle = max(-1, min(1, cos_angle))  # Clamp to valid range
        angle_rad = math.acos(cos_angle)
        finger_angle = math.degrees(angle_rad)

    # Map angle to servo (straight ≈ 180°, bent ≈ 90°)
    raw_angle = map_value(finger_angle, 90, 180, 180, 0)
    raw_angle = max(0, min(180, raw_angle))

    return raw_angle

# Smoothes raw angle data
def smooth_angle(raw_angle, current_smooth):
    alpha = 0.15

    if raw_angle < 5:
        raw_angle = 0
    elif raw_angle > 175:
        raw_angle = 180

    new_smooth = current_smooth + alpha * (raw_angle - current_smooth)

    if abs(new_smooth - 0) < 3:
        new_smooth = 0
    elif abs(new_smooth - 180) < 3:
        new_smooth = 180

    return int(new_smooth)


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

            hand_angle = (hand_angle+360) % 360

            if initial_hand_angle is None:
                initial_hand_angle = hand_angle
                angle_offset = hand_angle
                ser.write(f"W:0\n".encode())

            relative_angle = (hand_angle - angle_offset + 360) % 360

            target_stepper_pos = int((relative_angle / 360.0) * 2048 * WRIST_SENSITIVITY) # 360 degrees = 2048 steps
            ser.write(f"W:{target_stepper_pos}\n".encode())


            # Index finger
            index_tip = landmarks[8] #fingertip
            index_mid = landmarks[6] #middle joint
            index_base = landmarks[5] #base joint

            raw_index_angle = angle(index_tip, index_mid,index_base)

            smooth_index_angle = smooth_angle(raw_index_angle, smooth_index_angle)

            if index_last_sent_servo is None or abs(smooth_index_angle - index_last_sent_servo) >= 2:
                ser.write(f"I:{smooth_index_angle}\n".encode())
                index_last_sent_servo = smooth_index_angle

            # Middle finger

            middle_tip = landmarks[12]
            middle_mid = landmarks[10]
            middle_base = landmarks[9]

            raw_middle_angle = angle(middle_tip, middle_mid,middle_base)
            smooth_middle_angle = smooth_angle(raw_middle_angle,smooth_middle_angle)

            if middle_last_sent_servo is None or abs(smooth_middle_angle - middle_last_sent_servo) >= 2:
                ser.write(f"M:{smooth_middle_angle}\n".encode())
                middle_last_sent_servo = smooth_middle_angle


            # Thumb

            thumb_tip = landmarks[4]
            thumb_mid = landmarks[3]
            thumb_base = landmarks[2]

            raw_thumb_angle = angle(thumb_tip, thumb_mid,thumb_base)
            smooth_thumb_angle = smooth_angle(raw_thumb_angle,smooth_thumb_angle)

            if thumb_last_sent_servo is None or abs(smooth_thumb_angle - thumb_last_sent_servo) >= 2:
                ser.write(f"T:{smooth_thumb_angle}\n".encode())
                thumb_last_sent_servo = smooth_thumb_angle

            #Ring/Pinky

            ring_tip = landmarks[16]
            ring_mid = landmarks[14]
            ring_base = landmarks[13]

            raw_ring_angle = angle(ring_tip, ring_mid,ring_base)

            pinky_tip = landmarks[20]
            pinky_mid = landmarks[18]
            pinky_base = landmarks[17]

            raw_pinky_angle = angle(pinky_tip, pinky_mid, pinky_base)

            raw_ring_pinky_angle = (raw_pinky_angle + raw_ring_angle)/2 #average the two angles

            smooth_ring_pinky_angle = smooth_angle(raw_ring_pinky_angle,smooth_ring_pinky_angle)

            if ring_pinky_last_sent_servo is None or abs(smooth_ring_pinky_angle - ring_pinky_last_sent_servo) >= 2:
                ser.write(f"RP:{smooth_ring_pinky_angle}\n".encode())
                ring_pinky_last_sent_servo = smooth_ring_pinky_angle



    cv2.imshow("Hand Tracking", frame)

    if cv2.waitKey(1) & 0xFF == 27:  # ESC key
        break

cap.release()
ser.close()
cv2.destroyAllWindows()
