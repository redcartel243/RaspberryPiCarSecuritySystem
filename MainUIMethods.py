import math
from datetime import datetime

import pandas
from PyQt5.QtWidgets import *
from PyQt5 import QtWidgets
import distance_calculator
import speed_calculator
from MainUI import Ui_MainWindow
import sys
from threading import Lock
import time
from object_Data_files import object_data
import RobotState_and_image_State
from Streaming import *
import cv2
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QImage, QPixmap
import time as T
import gpiozero
from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import Robot, DistanceSensor
from gpiozero import AngularServo
from time import sleep
import control_data
from pynput.keyboard import Key, Listener
from threading import Thread
import object_detector
import traceback
import SettingsUIMethods


"""class WebcamVideoStream:

    def __init__(self, src, width=640, height=480):
        self.stream = cv2.VideoCapture(src)
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.stream.set(cv2.CAP_PROP_BUFFERSIZE, 5)
        self.FPS = 1 / 30
        self.FPS_MS = int(self.FPS * 1000)
        (self.grabbed, self.frame) = self.stream.read()
        self.started = False
        self.read_lock = Lock()

    def start(self):
        if self.started:
            return None
        self.started = True
        self.thread = Thread(target=self.update, args=())
        self.thread.daemon = True
        self.thread.start()
        return self

    def update(self):
        while self.started:
            (grabbed, frame) = self.stream.read()
            time.sleep(self.FPS)
            self.read_lock.acquire()
            self.grabbed, self.frame = grabbed, frame
            self.read_lock.release()

    def read(self):
        self.read_lock.acquire()
        frame = self.frame.copy()
        self.read_lock.release()
        return frame

    def stop(self):
        self.started = False
        if self.thread.is_alive():
            self.thread.join()

    def pause(self):
        self.stream.release()

    def __exit__(self, exc_type, exc_value, traceback):
        self.stream.release()"""




class MainUIMethods(Ui_MainWindow, QMainWindow):
    # dont forget to reenable it
    factory = PiGPIOFactory(host='192.168.137.164')
    gpiozero.Device.pin_factory = factory
    robot = Robot(left=(18, 23), right=(24, 25), pin_factory=factory)
    distanceSensor = DistanceSensor(21, 20, pin_factory=factory)
    lowerServo = 2
    upperServo = 3
    keys = []
    myCorrection = 0.45
    maxPW = (1.5 + myCorrection) / 1000
    minPW = (1.0 - myCorrection) / 1000

    panServo = AngularServo(2, min_angle=-180, max_angle=180, initial_angle=0, max_pulse_width=maxPW,
                            min_pulse_width=minPW,
                            pin_factory=factory)
    tiltServo = AngularServo(3, min_angle=180, max_angle=-180, initial_angle=0, max_pulse_width=maxPW,
                             min_pulse_width=minPW, pin_factory=factory)

    #capture = WebcamVideoStream(src=0)
    speedList = []
    DistanceList = []
    averageSpeed = 0
    intialDisntace = 0
    changeInTime = 0

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
    FONTS = cv2.FONT_HERSHEY_COMPLEX




    # print(f"Person width in pixels : {person_width_in_rf} mobile width in pixel: {mobile_width_in_rf}")

    """http://192.168.137.188:8080/?action=stream"""


    # List when any moving object appear
    motion_list = [None, None]

    # Time of movement
    time = []

    # Initializing DataFrame, one column is start
    # time and other column is end time
    df = pandas.DataFrame(columns=["Start", "End", "Type", "speed", "distance from camera"])

    # Define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter('output.avi', fourcc, 20.0, (640, 480))

    #SettingWindow = QtWidgets.QMainWindow()

    def __init__(self, title=" "):
        QMainWindow.__init__(self)
        self.title = title
        self.left = 400
        self.top = 50

        self.SettingsWindow = SettingsUIMethods.Settings()



    def setupUi(self, MainWindow):
        super().setupUi(MainWindow)
        MainWindow.move(self.left, self.top)  # set location for window
        MainWindow.setWindowTitle(self.title)  # change title
        self.pushButtonStartStreaming.clicked.connect(self.start_cam)
        self.pushButtonStartRecording.clicked.connect(self.setImageRecording1)
        self.pushButtonStopRecording_2.clicked.connect(self.setImageRecording2)
        self.pushButtonStartRecording.setEnabled(False)
        self.pushButtonStopRecording_2.setEnabled(False)
        self.pushButtondetectButton.clicked.connect(self.detect_webcam)
        self.labelStatus1.setHidden(True)
        self.labelStatus2.setHidden(True)
        self.pushButtondetectButton.setEnabled(False)
        self.pushButtonSettings.clicked.connect(lambda checked: self.toggle_window(self.SettingsWindow))
        self.pushButtonStartAuto.clicked.connect(self.activate_auto)
        self.automode=False


        # dont forget to reenable it
        self.pushButtonLeft_2.clicked.connect(self.servoTurnLeft)
        self.pushButtonRight_2.clicked.connect(self.servoTurnRight)
        self.pushButtonDown_2.clicked.connect(self.servoDown)
        self.pushButtonUp_2.clicked.connect(self.servoUp)
        self.pushButtonLeft.clicked.connect(self.robotLeft)
        self.pushButtonRight.clicked.connect(self.robotRight)
        self.pushButtonDown.clicked.connect(self.robotBackward)
        self.pushButtonUp.clicked.connect(self.robotForward)
        self.ActivateKeyboardControl()
        self.detection = False
        self.streaming = False



    def start_cam(self):
        if self.streaming == False:
            self.pushButtonStartRecording.setEnabled(True)
            self.pushButtondetectButton.setEnabled(True)
            self.labelStatus1.setHidden(False)
            self.labelStatus1.setText('Streaming')
            self.pushButtonStartStreaming.setText('Stop Streaming')
            # self.radioButtonStreaming.setChecked()
            # self.radioButtonStreaming.setChecked(True)
            # self.capture.start()
            self.capture = cv2.VideoCapture("http://192.168.137.164:8080/?action=stream")
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 5)

            self.timer = QTimer(self)
            self.timer.timeout.connect(self.update_frame)
            self.timer.start(2)
            self.streaming=True
        else:
            self.pushButtonStopRecording_2.setEnabled(True)
            self.pushButtonStartRecording.setEnabled(True)
            self.pushButtondetectButton.setEnabled(False)
            self.labelStatus1.setHidden(True)
            self.pushButtonStartStreaming.setText('Start Streaming')
            self.stop_webcam()
            self.streaming = False

    def activate_auto(self):
        if self.automode==False:
            self.pushButtonStartAuto.setEnabled(False)
            #we stop the detection and disable the detection button then activate the auto mode
            self.automode = True
            self.detection=True
            self.labelStatus2.setHidden(True)
            self.pushButtondetectButton.setEnabled(False)

        else:
            self.pushButtonStartAuto.setEnabled(False)
            self.pushButtondetectButton.setEnabled(True)
            self.automode = False

    def detect_webcam(self):
            if self.detection==False:
                self.labelStatus2.setHidden(False)
                self.labelStatus2.setText('Detecting')
                self.pushButtondetectButton.setText('Stop Detection')
                self.detection=True
            else:
                self.labelStatus2.setHidden(True)
                self.pushButtondetectButton.setText('Start Detection')
                self.detection=False
    def stop_webcam(self):
        self.capture.release()
        self.timer.stop()

    def setImageRecording1(self):
        self.pushButtonStopRecording_2.setEnabled(True)
        RobotState_and_image_State.Recording = True

    def setImageRecording2(self):
        self.out.release()
        RobotState_and_image_State.Recording = False


    def update_frame(self):
        try:

            ret,self.frame = self.capture.read()
            ret, self.frame1 = self.capture.read()
            ret, self.frame2 = self.capture.read()
            ret, self.frame3 = self.capture.read()
            self.frame = cv2.flip(self.frame, 180)
            self.displayImage(self.frame)
            if (self.detection):
                red=self.RecognitionObject(self.frame,self.frame1,self.frame2,self.frame3)
                self.displayImage(red)
            else:
                self.displayImage(self.frame)

            if RobotState_and_image_State.Recording == True:
                self.out.write(self.frame)

        except Exception as e:
            # Get current system exception
            ex_type, ex_value, ex_traceback = sys.exc_info()

            # Extract unformatter stack traces as tuples
            trace_back = traceback.extract_tb(ex_traceback)

            # Format stacktrace
            stack_trace = list()

            for trace in trace_back:
                stack_trace.append(
                    "File : %s , Line : %d, Func.Name : %s, Message : %s" % (trace[0], trace[1], trace[2], trace[3]))

            print("Exception type : %s " % ex_type.__name__)
            print("Exception message : %s" % ex_value)
            print("Stack trace : %s" % stack_trace)

    def displayImage(self, img):
        qformat = QImage.Format_Indexed8
        if len(img.shape) == 3:
            if img.shape[2] == 4:
                qformat = QImage.Format_RGBA8888
            else:
                qformat = QImage.Format_RGB888

        outImage = QImage(img, img.shape[1], img.shape[0], img.strides[0], qformat)
        outImage = outImage.rgbSwapped()

        self.label.setPixmap(QPixmap.fromImage(outImage))
        self.label.setScaledContents(True)
        return outImage

    def recordImage(self, img):
        self.fourcc = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
        self.out = cv2.VideoWriter("Registered_videos/output.avi", self.fourcc, 10, (640, 480))
        self.out.write(img)

    def stopstreaming(self):
        self.radioButtonStreaming.setChecked(False)
        self.capture.stop()

    def robotForward(self):
        self.robot.forward(RobotState_and_image_State.robotspeed)
        self.robot.stop()

    def robotBackward(self):
        self.robot.backward(RobotState_and_image_State.robotspeed)
        self.robot.stop()

    def robotLeft(self):
        self.robot.left(RobotState_and_image_State.robotspeed)
        self.robot.stop()

    def robotRight(self):
        self.robot.right(RobotState_and_image_State.robotspeed)
        self.robot.stop()

    def setServoAngle(self, servo, angle):
        servo.angle = angle

    def mapServoPosition(self,x, y):
        if (x > 350):
            print("servo right")

            if control_data.x < 180:
                control_data.x = control_data.x + 10
                self.setServoAngle(self.panServo, control_data.x)
                self.labelServo2.setText("Servo 2:{0}".format(control_data.x))
            else:
                self.labelServo2.setText("Servo 2: maximum angle ranched")
                self.panServo.detach()

        if (x < 150):
            print("servo left")
            if control_data.x > -180:
                control_data.x = control_data.x - 10
                self.setServoAngle(self.panServo, control_data.x)
                self.labelServo2.setText("Servo 2:{0}".format(control_data.x))
            else:
                self.labelServo2.setText("Servo 2: maximum angle ranched")
                self.panServo.detach()

        if (y < 140):
            print("servo up")
            if control_data.y < 140:
                control_data.y = control_data.y + 5
                self.setServoAngle(self.tiltServo, control_data.y)
                self.labelServo1.setText("Servo 1:{0}".format(control_data.y))
            else:
                self.labelServo1.setText("Servo 1: maximum angle ranched")
                self.tiltServo.detach()

        if (y > 210):
            print("servo down")
            if control_data.y > -140:
                control_data.y = control_data.y - 5
                self.setServoAngle(self.tiltServo, control_data.y)
                self.labelServo1.setText("Servo 1:{0}".format(control_data.y))
            else:
                self.labelServo1.setText("Servo 1: maximum angle ranched")
                self.tiltServo.detach()
    def servoTurnRight(self):
        print("servo right")

        if control_data.x < 180:
            control_data.x = control_data.x + 10
            self.setServoAngle(self.panServo, control_data.x)
            self.labelServo2.setText("Servo 2:{0}".format(control_data.x))
        else:
            self.labelServo2.setText("Servo 2: maximum angle ranched")
            self.panServo.detach()

    def servoTurnLeft(self):
        print("servo left")
        if control_data.x > -180:
            control_data.x = control_data.x - 10
            self.setServoAngle(self.panServo, control_data.x)
            self.labelServo2.setText("Servo 2:{0}".format(control_data.x))
        else:

            self.labelServo2.setText("Servo 2: maximum angle ranched")
            self.panServo.detach()

    def servoDown(self):
        print("servo down")
        if control_data.y > -140:
            control_data.y = control_data.y - 5
            self.setServoAngle(self.tiltServo, control_data.y)
            self.labelServo1.setText("Servo 1:{0}".format(control_data.y))
        else:
            self.labelServo1.setText("Servo 1: maximum angle ranched")
            self.tiltServo.detach()

    def servoUp(self):
        print("servo up")
        if control_data.y < 140:
            control_data.y = control_data.y + 5
            self.setServoAngle(self.tiltServo, control_data.y)
            self.labelServo1.setText("Servo 1:{0}".format(control_data.y))
        else:
            self.labelServo1.setText("Servo 1: maximum angle ranched")
            self.tiltServo.detach()

    def on_press(self, key):
        try:
            if (key.char == 'w'):
                self.servoUp()
                self.pushButtonUp_2.setDown(True)
                sleep(0.1)
            elif (key.char == 'd'):
                self.servoTurnRight()
                self.pushButtonRight_2.setDown(True)
                sleep(0.1)
            elif (key.char == 's'):
                self.servoDown()
                self.pushButtonDown_2.setDown(True)
                sleep(0.1)
            elif (key.char == 'a'):
                self.servoTurnLeft()
                self.pushButtonLeft_2.setDown(True)
                sleep(0.1)
            else:
                print()

        except AttributeError:
            if (key == key.up):
                self.robot.forward(RobotState_and_image_State.robotspeed)
                self.pushButtonUp.setDown(True)
            elif (key == key.down):
                self.robot.backward(RobotState_and_image_State.robotspeed)
                self.pushButtonDown.setDown(True)
            elif (key == key.right):
                self.robot.right(RobotState_and_image_State.robotspeed)
                self.pushButtonRight.setDown(True)
            elif (key == key.left):
                self.robot.left(RobotState_and_image_State.robotspeed)
                self.pushButtonLeft.setDown(True)

    def on_release(self, key):
        try:
            if (key.char == 'w'):
                self.pushButtonUp_2.setDown(False)
                print()
            elif (key.char == 'd'):
                self.pushButtonRight_2.setDown(False)
                print()
            elif (key.char == 's'):
                self.pushButtonDown_2.setDown(False)
                print()
            elif (key.char == 'a'):
                self.pushButtonLeft_2.setDown(False)
                print()
            else:
                print()

        except AttributeError:
            if (key == key.up):
                self.pushButtonUp.setDown(False)
                self.robot.stop()
            elif (key == key.down):
                self.pushButtonDown.setDown(False)
                self.robot.stop()
            elif (key == key.right):
                self.pushButtonRight.setDown(False)
                self.robot.stop()
            elif (key == key.left):
                self.pushButtonLeft.setDown(False)
                self.robot.stop()

        if key == Key.esc:
            # Stop listener
            return False

    def KeyboardControl(self):
        with Listener(on_press=self.on_press,
                      on_release=self.on_release) as listener:
            listener.join()

    def ActivateKeyboardControl(self):
        self.thread = Thread(target=self.KeyboardControl, args=())
        self.thread.daemon = True
        self.thread.start()

    def toggle_window(self,window):
        if window.isVisible():
            window.hide()

        else:
            print(RobotState_and_image_State.objectischecked["person"])
            window.show()

    def RecognitionObject(self, frame, frame1, frame2,frame4):

        nbobjects = []
        coordObjectsx = []
        coordObjectsy = []
        coordObjects = []
        count = 0

        # the coordinate that we use for the servo mapping
        xservo = 0
        yservo = 0

        intialTime = T.time()

        data = object_detector.object_detector(frame)
        # Initializing motion = 0(no motion)
        motion = 0

        diff = cv2.absdiff(frame1, frame2)
        diff_gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(diff_gray, (5, 5), 0)
        _, thresh = cv2.threshold(blur, 20, 255, cv2.THRESH_BINARY)
        dilated = cv2.dilate(thresh, None, iterations=3)
        contours, _ = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


        for d in data:
            x, y = d[2]

            if RobotState_and_image_State.objectischecked[d[0]]==True:
                x, y = d[2]

                count += 1
                nbobjects.append(count)
                coordObjects.append(d[2])
                coordObjectsx.append(x)
                coordObjectsy.append(y)
                #print("{0}:{1},{2}".format(d[0], coordObjectsx, coordObjectsy))
                # Here we calculate the midpoint if there is more than one object
                if max(nbobjects) > 1:
                    # print(max(coordObjects))
                    # print(max(nbobjects))
                    # print(min(coordObjects))
                    xm = sum(coordObjectsx) / len(coordObjectsx)
                    ym = sum(coordObjectsy) / len(coordObjectsy)
                    """xmin, ymin = min(coordObjects)
                    xmax, ymax = max(coordObjects)"""
                    xservo = math.ceil(xm)
                    yservo = math.ceil(ym)
                    print(xservo, yservo)
                else:
                    xservo = x
                    yservo = y
                    #print("{0} {1}".format(x, y))
                # finding focal length
                focal_object = distance_calculator.focal_length_finder(object_data.KNOWN_DISTANCE,
                                                                       object_data.chosenObject[d[0]]["width"],
                                                                       object_data.chosenObject[d[0]]["width_in_rf"])

                distance = distance_calculator.distance_finder(focal_object, object_data.chosenObject[d[0]]["width"], d[1])

                self.DistanceList.append(distance)
                avergDistnce = speed_calculator.averageFinder(self.DistanceList, 6)
                # print("Average distance : ",avergDistnce)

                if self.automode==True:
                    self.mapServoPosition(xservo, yservo)
                # print(x,y)
                if self.intialDisntace != 0:
                    changeDistance = distance - self.intialDisntace
                    distanceInMeters = changeDistance * 0.0254

                    velocity = speed_calculator.speedFinder(distanceInMeters, self.changeInTime)

                    self.speedList.append(velocity)

                    self.averageSpeed = speed_calculator.averageFinder(self.speedList, 6)
                # intial Distance
                self.intialDisntace = avergDistnce
                self.changeInTime = T.time() - intialTime
                # print("Change in time : ",changeInTime)

            cv2.line(frame, (25, 45), (180, 45), (self.ORANGE), 26)
            cv2.line(frame, (25, 45), (180, 45), (self.GREEN), 20)
            # cv2.line(image, (x, y-11), (x+180, y-11), (YELLOW), 20)
            # cv2.line(image, (x, y-11), (x+Distance_level, y-11), (GREEN), 18)
            if self.averageSpeed < 0:
                self.averageSpeed = self.averageSpeed * -1
            cv2.putText(frame, f'Speed: {round(self.averageSpeed, 2)} m/s', (30, 50), self.FONTS, 0.48, self.BLACK,1)
            cv2.rectangle(frame, (x, y - 3), (x + 150, y + 23), self.BLACK, -1)
            cv2.putText(frame, f'Dis: {round(distance, 2)} inch', (x + 5, y + 13), self.FONTS, 0.48, self.GREEN, 2)

            for contour in contours:

                (x, y, w, h) = cv2.boundingRect(contour)
                if cv2.contourArea(contour) < 900:
                    continue
                motion = 1
                # cv.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, "Status: {}".format('Movement'), (10, 20), cv2.FONT_HERSHEY_SIMPLEX,1, (255, 0, 0), 3)
            # Appending status of motion
            self.motion_list.append(motion)

            self.motion_list = self.motion_list[-2:]

            if self.motion_list[-1] == 1 and self.motion_list[-2] == 0:
                self.time.append(datetime.now())
            # Appending Start time of motion

            # Appending End time of motion, speed,distance and class (obecjt)
            if self.motion_list[-1] == 0 and self.motion_list[-2] == 1:
                self.time.append(datetime.now())
                for i in range(0, len(self.time), 2):
                    try:
                        if self.time[i + 1].second - self.time[i].second >= 3:
                            print("the difference is {}".format(self.time[i + 1].second - self.time[i].second))
                            self.df = self.df.append({"Start": self.time[i], "End": self.time[i + 1]},
                                                     ignore_index=True)
                            movement = "<span style=\" font-size:8pt; font-weight:600; color:#ff0000;\" >"
                            movement += "Movement time: \n  Start:{0}   End{1}".format(self.time[i],self.time[i + 1])
                            movement += "</span>"
                            self.textEditInformation.append(movement)
                        if d[0] == 'person':

                            self.df = self.df.append({"type": d[0]}, ignore_index=True)
                            self.df = self.df.append({"speed": f'Speed: {round(self.averageSpeed, 2)} m/s'},
                                                     ignore_index=True)
                            self.df = self.df.append({"distance from camera": f'Dis: {round(distance, 2)} inch'},
                                                     ignore_index=True)
                            type="<span style=\" font-size:8pt; font-weight:600; color:#ff0000;\" >"
                            type+="type: {0}\n".format(d[0])
                            type+="</span>"
                            self.textEditInformation.append(type)
                            if RobotState_and_image_State.parameterischecked["speed"] == True:
                                self.textEditInformation.append("speed: {0} m/s\n".format(round(self.averageSpeed, 2)))
                            if RobotState_and_image_State.parameterischecked["distance"] == True:
                                self.textEditInformation.append("distance from camera {0} inch\n".format(round(distance, 2)))
                        if d[0] == 'cell phone':
                            self.df = self.df.append({"type": d[0]}, ignore_index=True)
                            self.df = self.df.append({"speed": f'Speed: {round(self.averageSpeed, 2)} m/s'},
                                                     ignore_index=True)
                            self.df = self.df.append({"distance from camera": f'Dis: {round(distance, 2)} inch'},
                                                     ignore_index=True)
                            type = "<span style=\" font-size:8pt; font-weight:600; color:#ff0000;\" >"
                            type += "type: {0}\n".format(d[0])
                            type += "</span>"
                            self.textEditInformation.append(type)
                            if RobotState_and_image_State.parameterischecked["speed"] == True:
                                self.textEditInformation.append("speed: {0} m/s\n".format(round(self.averageSpeed, 2)))
                            if RobotState_and_image_State.parameterischecked["distance"] == True:
                                self.textEditInformation.append(
                                    "distance from camera {0} inch\n".format(round(distance, 2)))
                        if d[0] == 'bottle':
                            self.df = self.df.append({"type": d[0]}, ignore_index=True)
                            self.df = self.df.append({"speed": f'Speed: {round(self.averageSpeed, 2)} m/s'},
                                                     ignore_index=True)
                            self.df = self.df.append({"distance from camera": f'Dis: {round(distance, 2)} inch'},
                                                     ignore_index=True)
                            type = "<span style=\" font-size:8pt; font-weight:600; color:#ff0000;\" >"
                            type += "type: {0}\n".format(d[0])
                            type += "</span>"
                            self.textEditInformation.append(type)
                            if RobotState_and_image_State.parameterischecked["speed"] == True:
                                self.textEditInformation.append("speed: {0} m/s\n".format(round(self.averageSpeed, 2)))
                            if RobotState_and_image_State.parameterischecked["distance"] == True:
                                self.textEditInformation.append(
                                    "distance from camera {0} inch\n".format(round(distance, 2)))

                        self.time.pop(i)
                        self.time.pop(i + 1)

                    except Exception as e:
                        print(e)

                # Creating a CSV file in which time of movements will be saved
                self.df.to_csv("Time_of_movements.csv")

        # cv.imshow("Video", frame1)
        frame1 = frame2
        frame2 = frame4

        # cv.imshow('berry', frame)

        return frame

        """if cv2.waitKey(1) == 27:
            # if something is movingthen it append the end time of movement
            if motion == 1:
                self.time.append(datetime.now())

        # Appending time of motion in DataFrame

        for i in range(0, len(self.time), 2):
            self.df = self.df.append({"Start": self.time[i], "End": self.time[i + 1]}, ignore_index=True)

            # Creating a CSV file in which time of movements will be saved
        self.df.to_csv("Time_of_movements.csv")"""

        """try:

            
        except Exception as e:
            # Get current system exception
            ex_type, ex_value, ex_traceback = sys.exc_info()

            # Extract unformatter stack traces as tuples
            trace_back = traceback.extract_tb(ex_traceback)

            # Format stacktrace
            stack_trace = list()

            for trace in trace_back:
                stack_trace.append(
                    "File : %s , Line : %d, Func.Name : %s, Message : %s" % (trace[0], trace[1], trace[2], trace[3]))

            print("Exception type : %s " % ex_type.__name__)
            print("Exception message : %s" % ex_value)
            print("Stack trace : %s" % stack_trace)"""

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = MainUIMethods("Control system")
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
