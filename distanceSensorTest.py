import gpiozero
from gpiozero.pins.pigpio import PiGPIOFactory
import time
import keyboard
from gpiozero import Robot,DistanceSensor
import cv2 as cv
from gpiozero import AngularServo
from time import sleep
from gpiozero import Servo
from time import sleep
from gpiozero import MotionSensor
"""how to use streaming camera :
video = cv2.VideoCapture("http://192.168.1.101:8080/?action=stream")
"""

factory = PiGPIOFactory(host='192.168.137.188')
gpiozero.Device.pin_factory=factory
robot = Robot(left=(18, 23), right=(24, 25),pin_factory=factory)
distanceSensor = DistanceSensor(21, 20,pin_factory=factory)



while True:
    distance_to_object = distanceSensor.distance * 100
    print(distance_to_object)
    if distance_to_object <= 30:
        robot.backward()
        sleep(0.3)
        robot.left()
        sleep(0.3)
    else:
        robot.stop()