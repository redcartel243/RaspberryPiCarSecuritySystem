from threading import Thread

import cv2 as cv
import numpy as np
import time as T
import object_detector
import distance_calculator
import speed_calculator
from datetime import datetime
import pandas
import math
from object_Data_files import object_data
import gpiozero
from gpiozero import AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory
import control_data
import RobotState_and_image_State

"""factory = PiGPIOFactory(host='192.168.137.188')
gpiozero.Device.pin_factory = factory
lowerServo = 2
upperServo = 3
keys = []
myCorrection = 0.45
maxPW = (1.5 + myCorrection) / 1000
minPW = (1.0 - myCorrection) / 1000"""

# define the lower and upper boundaries of the object
# to be detected in the HSV color space
colorLower = [24, 100, 100]
colorUpper = [44, 255, 255]

colorLower = np.array(colorLower, dtype="uint8")
colorUpper = np.array(colorUpper, dtype="uint8")

# define Servos GPIOs
"""panServo = AngularServo(2, min_angle=-180, max_angle=180, initial_angle=0, max_pulse_width=maxPW, min_pulse_width=minPW,
                        pin_factory=factory)
tiltServo = AngularServo(3, min_angle=180, max_angle=-180, initial_angle=0, max_pulse_width=maxPW,
                         min_pulse_width=minPW, pin_factory=factory)

def setServoAngle(servo, angle):
    servo.angle = angle

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
            tiltServo.detach()"""


# thread for face detection,CAM2
class berry(Thread):
    def __init__(self):
        Thread.__init__(self)

    def run(self):

        # positioning Pan/Tilt servos at initial position
        #setServoAngle(panServo, control_data.x)
        # print(panServo.angle)
        #setServoAngle(tiltServo, control_data.y)
        # print(tiltServo.angle)

        speedList = []
        DistanceList = []
        averageSpeed = 0
        intialDisntace = 0

        # colors for object detected
        GREEN = (0, 255, 0)
        RED = (0, 0, 255)
        BLACK = (0, 0, 0)
        YELLOW = (0, 255, 255)
        WHITE = (255, 255, 255)
        CYAN = (255, 255, 0)
        MAGENTA = (255, 0, 242)
        GOLDEN = (32, 218, 165)
        LIGHT_BLUE = (255, 9, 2)
        PURPLE = (128, 0, 128)
        CHOCOLATE = (30, 105, 210)
        PINK = (147, 20, 255)
        ORANGE = (0, 69, 255)
        # defining fonts
        FONTS = cv.FONT_HERSHEY_COMPLEX





        """http://192.168.137.188:8080/?action=stream"""
        cap = cv.VideoCapture(0)
        ret, frame1 = cap.read()
        ret, frame2 = cap.read()

        # Assigning our static_back to None
        static_back = None

        # List when any moving object appear
        motion_list = [None, None]

        # Time of movement
        time = []

        # Initializing DataFrame, one column is start
        # time and other column is end time
        df = pandas.DataFrame(columns=["Start", "End", "Type", "speed", "distance from camera"])

        while True:
            nbobjects = []
            coordObjectsx = []
            coordObjectsy = []
            coordObjects = []
            count = 0
            # the coordinate that we use for the servo mapping
            xservo = 0
            yservo = 0

            ret, frame = cap.read()

            frame = cv.flip(frame, 180)
            intialTime = T.time()

            data = object_detector.object_detector(frame)
            # Initializing motion = 0(no motion)
            motion = 0

            diff = cv.absdiff(frame1, frame2)
            diff_gray = cv.cvtColor(diff, cv.COLOR_BGR2GRAY)
            blur = cv.GaussianBlur(diff_gray, (5, 5), 0)
            _, thresh = cv.threshold(blur, 20, 255, cv.THRESH_BINARY)
            dilated = cv.dilate(thresh, None, iterations=3)
            contours, _ = cv.findContours(
                dilated, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

            # In first iteration we assign the value
            # of static_back to our first frame

            for d in data:
                x, y = d[2]

                if d[0] in object_data.chosenObject:
                    x, y = d[2]

                    count += 1
                    nbobjects.append(count)
                    coordObjects.append(d[2])
                    coordObjectsx.append(x)
                    coordObjectsy.append(y)
                    print("{0}:{1},{2}".format(d[0], coordObjectsx, coordObjectsy))
                    # Here we calculate the midpoint if there is more than one object
                    if max(nbobjects) > 1:
                        # print(max(coordObjects))
                        # print(max(nbobjects))
                        # print(min(coordObjects))
                        xm=sum(coordObjectsx)/len(coordObjectsx)
                        ym=sum(coordObjectsy)/len(coordObjectsy)
                        """xmin, ymin = min(coordObjects)
                        xmax, ymax = max(coordObjects)"""
                        xservo = math.ceil(xm)
                        yservo = math.ceil(ym)
                        print(xservo, yservo)
                    else:
                        xservo = x
                        yservo = y
                    # finding focal length
                    focal_object = distance_calculator.focal_length_finder(object_data.KNOWN_DISTANCE, object_data.chosenObject[d[0]]["width"],
                                                                           object_data.chosenObject[d[0]]["width_in_rf"])
                    distance = distance_calculator.distance_finder(focal_object, object_data.chosenObject[d[0]]["width"], d[1])
                    DistanceList.append(distance)
                    avergDistnce = speed_calculator.averageFinder(DistanceList, 6)
                    # print("Average distance : ",avergDistnce)
                    roundedDistance = round((avergDistnce * 0.0254), 2)
                    Distance_level = int(distance)

                    #mapServoPosition(xservo, yservo)
                    # print(x,y)
                    if intialDisntace != 0:
                        changeDistance = distance - intialDisntace
                        distanceInMeters = changeDistance * 0.0254

                        velocity = speed_calculator.speedFinder(distanceInMeters, changeInTime)

                        speedList.append(velocity)

                        averageSpeed = speed_calculator.averageFinder(speedList, 6)
                    # intial Distance
                    intialDisntace = avergDistnce

                    changeInTime = T.time() - intialTime
                    # print("Change in time : ",changeInTime)

                cv.line(frame, (25, 45), (180, 45), (ORANGE), 26)
                cv.line(frame, (25, 45), (180, 45), (GREEN), 20)
                # cv2.line(image, (x, y-11), (x+180, y-11), (YELLOW), 20)
                # cv2.line(image, (x, y-11), (x+Distance_level, y-11), (GREEN), 18)
                if averageSpeed < 0:
                    averageSpeed = averageSpeed * -1
                cv.putText(frame, f'Speed: {round(averageSpeed, 2)} m/s', (30, 50), FONTS, 0.48, BLACK, 1)
                cv.rectangle(frame, (x, y - 3), (x + 150, y + 23), BLACK, -1)
                cv.putText(frame, f'Dis: {round(distance, 2)} inch', (x + 5, y + 13), FONTS, 0.48, GREEN, 2)
                for contour in contours:
                    (x, y, w, h) = cv.boundingRect(contour)
                    if cv.contourArea(contour) < 900:
                        continue
                    motion = 1
                    # cv.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv.putText(frame, "Status: {}".format('Movement'), (10, 20), cv.FONT_HERSHEY_SIMPLEX,
                               1, (255, 0, 0), 3)
                # Appending status of motion
                motion_list.append(motion)

                motion_list = motion_list[-2:]

                if motion_list[-1] == 1 and motion_list[-2] == 0:
                    time.append(datetime.now())
                # Appending Start time of motion

                # Appending End time of motion, speed,distance and class (obecjt)
                if motion_list[-1] == 0 and motion_list[-2] == 1:
                    time.append(datetime.now())
                    for i in range(0, len(time), 2):
                        try:
                            if time[i + 1].second - time[i].second >= 3:
                                print("the difference is {}".format(time[i + 1].second - time[i].second))
                                df = df.append({"Start": time[i], "End": time[i + 1]}, ignore_index=True)
                            if d[0] == 'person':
                                df = df.append({"type": d[0]}, ignore_index=True)
                                df = df.append({"speed": f'Speed: {round(averageSpeed, 2)} m/s'}, ignore_index=True)
                                df = df.append({"distance from camera": f'Dis: {round(distance, 2)} inch'},
                                               ignore_index=True)
                            if d[0] == 'cell phone':
                                df = df.append({"type": d[0]}, ignore_index=True)
                                df = df.append({"speed": f'Speed: {round(averageSpeed, 2)} m/s'}, ignore_index=True)
                                df = df.append({"distance from camera": f'Dis: {round(distance, 2)} inch'},
                                               ignore_index=True)

                            time.pop(i)
                            time.pop(i + 1)

                        except Exception as e:
                            print(e)

                    # Creating a CSV file in which time of movements will be saved
                    df.to_csv("Time_of_movements.csv")

            # cv.imshow("Video", frame1)
            frame1 = frame2
            ret, frame2 = cap.read()

            cv.imshow('berry', frame)

            if cv.waitKey(1) == 27:
                # if something is movingthen it append the end time of movement
                if motion == 1:
                    time.append(datetime.now())
                break

        # Appending time of motion in DataFrame
        for i in range(0, len(time), 2):
            df = df.append({"Start": time[i], "End": time[i + 1]}, ignore_index=True)

        # Creating a CSV file in which time of movements will be saved
        df.to_csv("Time_of_movements.csv")

        cap.release()
        cv.destroyAllWindows()


thread1 = berry()

thread1.start()

thread1.join()
