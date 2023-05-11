import cv2
import mediapipe as mp
import numpy as np
import time
import os
import serial
import RPi.GPIO as GPIO

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(27, GPIO.IN)         #Read output from PIR motion sensor
GPIO.setup(17, GPIO.IN)         #Read output from PIR motion sensor
GPIO.setup(22, GPIO.IN)         #Read output from PIR motion sensor
GPIO.setup(4, GPIO.IN)         #Read output from PIR motion sensor

# Enable Serial Communication
port = serial.Serial("/dev/ttyUSB0", baudrate=115200, timeout = 1)

gun_cascade = cv2.CascadeClassifier('cascade.xml')

mp_pose = mp.solutions.pose
body_pose = mp_pose.Pose(min_detection_confidence=0.3,min_tracking_confidence=0.3)

cap = cv2.VideoCapture(0)
codec = cv2.VideoWriter_fourcc(*'DIVX')

recording_flag = False
detection_is_true = False
weapon_detection = False
detection_reset = False

while True:
    #print("Reset")
    if ((GPIO.input(17) == 1 or GPIO.input(4) == 1 or GPIO.input(27) == 1 or GPIO.input(22) == 1)):
        if (detection_reset == False):
            detection_reset = True
            print("Motion Detected!")
            port.write(b'AT\r\n')
            time.sleep(0.5)

            port.write(b'AT+CFUN=1\r\n')
            time.sleep(0.5)

            port.write(b'AT+CMGF=1\r\n')
            time.sleep(0.5)

            port.write(b'AT+CNMI=2,1,0,0,0\r\n')
            time.sleep(0.5)

            port.write(b'AT+CMGS="09706504875"\r\n')
            time.sleep(0.5)

            msg = "Motion has been detected!!!"
            port.reset_output_buffer()
            time.sleep(0.5)
            port.write(str.encode(msg + chr(26)))
            time.sleep(0.5)
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
                
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

                if body_results.pose_landmarks:
                    print("BODY DETECTED")
                    detection_is_true = True
                    gun = gun_cascade.detectMultiScale(gray,
                                       1.3, 7,
                                       minSize=(100, 100))
                                
                    if len(gun) > 0:
                        weapon_detection = True
                    if recording_flag == False:
                        print("Recording")
                        isexist = True
                        number = 0
                        while(isexist):
                            number += 1
                            path = 'captures/' + str(number) + '.avi'
                            isexist = os.path.exists(path)
                        
                        output = cv2.VideoWriter(path, codec, 30, (640,480))
                        recording_flag = True
                else:
                    port.write(b'AT\r\n')
                    time.sleep(0.5)

                    port.write(b'AT+CFUN=1\r\n')
                    time.sleep(0.5)

                    port.write(b'AT+CMGF=1\r\n')
                    time.sleep(0.5)

                    port.write(b'AT+CNMI=2,1,0,0,0\r\n')
                    time.sleep(0.5)

                    port.write(b'AT+CMGS="09706504875"\r\n')
                    time.sleep(0.5)

                    if detection_is_true and weapon_detection:
                        msg = "Video has been saved with Human detections and Weapon Detections."
                    elif detection_is_true and weapon_detection == False:
                        msg = "Video has been saved with Human detections and without Weapon detections."
                    elif detection_is_true == False:
                        msg = "Video has been saved without detections detected."
                    port.reset_output_buffer()
                    time.sleep(0.5)
                    port.write(str.encode(msg + chr(26)))
                    time.sleep(0.5)
                    recording_flag = False
                    detection_is_true = False
                    print("End Recording")
                    break

                #cv2.imshow('Footage', image)

                if recording_flag:
                    output.write(image)
    else:
        print("NO DETECTION")
        detection_reset = False
