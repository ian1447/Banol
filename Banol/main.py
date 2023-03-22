import cv2
import mediapipe as mp
import numpy as np
import time
import os

mp_pose = mp.solutions.pose
body_pose = mp_pose.Pose(min_detection_confidence=0.3,min_tracking_confidence=0.3)

cap = cv2.VideoCapture(0)
codec = cv2.VideoWriter_fourcc(*'XVID')

recording_flag = False

while cap.isOpened():
    success, image = cap.read()

    start = time.time()

    # Flip the image horizontally for a later selfie-view display
    # Also convert the color space from BGR to RGB
    image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)

    # To improve performance
    image.flags.writeable = False

    # Get the result
    body_results = body_pose.process(image)

    # To improve performance
    image.flags.writeable = True

    # Convert the color space from RGB to BGR
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    if body_results.pose_landmarks:
        print("naa")
    else:
        print("wala")

    cv2.imshow('Head Pose Estimation', image)

    # if recording_flag:
    #     output.write(image)

    if cv2.waitKey(5) & 0xFF == ord('q'):
        break
