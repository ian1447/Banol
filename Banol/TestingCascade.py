import numpy as np
import cv2
import imutils

gun_cascade = cv2.CascadeClassifier('dosCascade.xml')
knife_cascade = cv2.CascadeClassifier('knifecascade.xml')
camera = cv2.VideoCapture(0)

firstFrame = None
gun_exist = False

while True:

    ret, frame = camera.read()

    frame = imutils.resize(frame, width=500)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    gun = gun_cascade.detectMultiScale(gray,
                                       1.8, 15,
                                       minSize=(100, 100))
    knife = knife_cascade.detectMultiScale(gray,
                                       1.3, 7,
                                       minSize=(100, 100))

    for (x, y, w, h) in gun:
        frame = cv2.rectangle(frame,
                              (x, y),
                              (x + w, y + h),
                              (255, 0, 0), 2)
        roi_gray = gray[y:y + h, x:x + w]
        roi_color = frame[y:y + h, x:x + w]

    for (x, y, w, h) in knife:
        frame = cv2.rectangle(frame,
                              (x, y),
                              (x + w, y + h),
                              (255, 255, 0), 2)
        roi_gray = gray[y:y + h, x:x + w]
        roi_color = frame[y:y + h, x:x + w]

    if firstFrame is None:
        firstFrame = gray
        continue


    cv2.imshow("Security Feed", frame)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'):
        break

camera.release()
cv2.destroyAllWindows()