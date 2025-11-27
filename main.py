import cv2
import mediapipe as mp
import numpy as np
import pyautogui
import time

webCam = cv2.VideoCapture(0)

mp_face_mesh=  mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(
    max_num_faces=1,
    refine_landmarks=True,
    static_image_mode=False,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5,
)

LEFT_PUPIL_CENTER = 468 
LEFT_EYE_TOP_LID = 159
LEFT_EYE_BOTTOM_LID = 145

RIGHT_PUPIL_CENTER = 473 
RIGHT_EYE_TOP_LID = 386
RIGHT_EYE_BOTTOM_LID = 374



BLINK_THRESHOLD = 12 
SWIPE_DELAY = 1.0

last_swipe_time = 0
last_blink_state = {"left": False, "right": False}

while True:
    # read data from web cam
    ret, frame = webCam.read()
    if not ret:
        break
    frame = cv2.flip(frame,1)
    # convert bgr to rgb
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = face_mesh.process(rgb_frame)

    # if face was detected
    if results.multi_face_landmarks:
        face_landmarks = results.multi_face_landmarks[0]
        h, w, c = frame.shape

        def get_cords(idx):
            lm = face_landmarks.landmark[idx]
            return int(lm.x*w), int(lm.y*h)
        
        left_pupil_center_xy = get_cords(LEFT_PUPIL_CENTER)
        left_top_lid_xy = get_cords(LEFT_EYE_TOP_LID)
        left_bottom_lid_xy = get_cords(LEFT_EYE_BOTTOM_LID)

        right_pupil_center_xy = get_cords(RIGHT_PUPIL_CENTER)
        right_top_lid_xy = get_cords(RIGHT_EYE_TOP_LID)
        right_bottom_lid_xy = get_cords(RIGHT_EYE_BOTTOM_LID)

        cv2.circle(frame, left_pupil_center_xy, 3, (0,0,255), -1)
        cv2.circle(frame, left_top_lid_xy, 3, (0,0,255), -1)
        cv2.circle(frame, left_bottom_lid_xy, 3, (0,0,255), -1)

        cv2.circle(frame, right_pupil_center_xy, 3, (0,0,255), -1)
        cv2.circle(frame, right_top_lid_xy, 3, (0,0,255), -1)
        cv2.circle(frame, right_bottom_lid_xy, 3, (0,0,255), -1)

        # diff between top and bottom vectors
        left_eye_vertical_dist = np.linalg.norm(np.array(left_top_lid_xy) - np.array(left_bottom_lid_xy))
        cv2.putText(frame, f"Left Eye Dist: {left_eye_vertical_dist:0.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)

        # diff between top and bottom vectors
        right_eye_vertical_dist = np.linalg.norm(np.array(right_top_lid_xy) - np.array(right_bottom_lid_xy))
        cv2.putText(frame, f"Right Eye Dist: {right_eye_vertical_dist:0.2f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)


        left_closed = left_eye_vertical_dist < BLINK_THRESHOLD
        right_closed = right_eye_vertical_dist < BLINK_THRESHOLD
        current_time = time.time()


        if current_time - last_swipe_time > SWIPE_DELAY:


            if right_eye_vertical_dist < BLINK_THRESHOLD and left_eye_vertical_dist<BLINK_THRESHOLD:
                cv2.putText(frame, "Both Eyes Closed", (w - 200, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
            elif left_eye_vertical_dist < BLINK_THRESHOLD:
                pyautogui.hotkey('ctrl', 'win', 'left')
                cv2.putText(frame, "SWIPE RIGHT TRIGGERED", (w - 400, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)
                last_swipe_time = current_time
            elif right_eye_vertical_dist < BLINK_THRESHOLD:
                pyautogui.hotkey('ctrl', 'win', 'right')
                cv2.putText(frame, "SWIPE RIGHT TRIGGERED", (w - 400, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)
                last_swipe_time = current_time
    cv2.imshow('Face Mesh Eye Tracker', frame)
    if cv2.waitKey(5) & 0XFF == ord('q'):
        break


webCam.release()
cv2.destroyAllWindows()