# import the necessary packages
from __future__ import print_function

import gpiozero
from gpiozero import AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory
from imutils.video import VideoStream
import argparse
import imutils
import time
import cv2
import numpy as np
import os
import control_data

vs = cv2.VideoCapture("http://192.168.137.188:8080/?action=stream")
factory = PiGPIOFactory(host='192.168.137.188')
gpiozero.Device.pin_factory = factory
lowerServo = 2
upperServo = 3
keys = []
myCorrection = 0.45
maxPW = (1.5 + myCorrection) / 1000
minPW = (1.0 - myCorrection) / 1000

# define the lower and upper boundaries of the object
# to be detected in the HSV color space
colorLower = [24, 100, 100]
colorUpper = [44, 255, 255]

colorLower = np.array(colorLower, dtype="uint8")
colorUpper = np.array(colorUpper, dtype="uint8")

# define Servos GPIOs
panServo = AngularServo(2, min_angle=-180, max_angle=180, initial_angle=0, max_pulse_width=maxPW, min_pulse_width=minPW,
                        pin_factory=factory)
tiltServo = AngularServo(3, min_angle=180, max_angle=-180, initial_angle=0, max_pulse_width=maxPW,
                         min_pulse_width=minPW, pin_factory=factory)

# Initialize angle servos at 90-90 position




def mapObjectPosition(x, y):
    print("[INFO] Object Center coordenates at X0 = {0} and Y0 =  {1}".format(x, y))


# position servos to present object at center of the frame
def mapServoPosition(x, y):



    if (x > 285):
        print("servo right")

        if control_data.x < 180:
            control_data.x = control_data.x + 10
            setServoAngle(panServo, control_data.x)
            print("servo angle {}".format(panServo.angle))
        else:
            print("maximum angle ranched for pan servo")
            panServo.detach()





    if (x < 215):
        print("servo left")
        if control_data.x > -180:
            control_data.x = control_data.x - 10
            setServoAngle(panServo, control_data.x)
            print("servo angle {}".format(panServo.angle))
        else:
            print("maximum angle ranched for pan servo")
            panServo.detach()




    if (y < 140):
        print("servo up")
        if control_data.y < 140:
            control_data.y = control_data.y + 5
            setServoAngle(tiltServo, control_data.y)
            print("servo angle {}".format(tiltServo.angle))
        else:
            print("maximum angle ranched for tilt servo")
            tiltServo.detach()

    if (y > 210):
        print("servo down")
        if control_data.y > -140:
            control_data.y = control_data.y - 5
            setServoAngle(tiltServo, control_data.y)
            print("servo angle {}".format(tiltServo.angle))
        else:
            print("maximum angle ranched for tilt servo")
            tiltServo.detach()


def positionServo(servo, angle):
    os.system("python angleServoCtrl.py " + str(servo) + " " + str(angle))
    print("[INFO] Positioning servo at GPIO {0} to {1} degrees\n".format(servo, angle))


def setServoAngle(servo, angle):
    servo.angle = angle



# positioning Pan/Tilt servos at initial position
setServoAngle(panServo, control_data.x)
# print(panServo.angle)
setServoAngle(tiltServo, control_data.y)
# print(tiltServo.angle)


# loop over the frames from the video stream
while True:
    # grab the next frame from the video stream, Invert 180o, resize the
    # frame, and convert it to the HSV color space
    ret, frame = vs.read()
    frame = cv2.flip(frame, 180)
    frame = imutils.resize(frame, width=500)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # construct a mask for the obect color, then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, colorLower, colorUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # find contours in the mask and initialize the current
    # (x, y) center of the object

    cnts, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    center = None

    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)

        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # only proceed if the radius meets a minimum size
        if radius > 10:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(x), int(y)), int(radius),
                       (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)

            # print center of circle coordinates
            #mapObjectPosition(int(x), int(y))
            # position Servo at center of circle
            print(x)

            mapServoPosition(int(x), int(y))

    # show the frame to our screen
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF

    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        break

# do a bit of cleanup
# do a bit of cleanup
print("\n [INFO] Exiting Program and cleanup stuff \n")
positionServo(panServo, 0.0)
positionServo(tiltServo, 0.0)
cv2.destroyAllWindows()
vs.stop()
