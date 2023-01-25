import gpiozero
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import AngularServo
from time import sleep

factory = PiGPIOFactory(host='192.168.0.109')
gpiozero.Device.pin_factory=factory
lowerServo = 2
upperServo= 3
keys = []
myCorrection = 0.45
maxPW = (1.5 + myCorrection) / 1000
minPW = (1.0 - myCorrection) / 1000

s = AngularServo(2, min_angle=-180, max_angle=180,initial_angle=0,max_pulse_width=maxPW,min_pulse_width=minPW,pin_factory=factory)
for i in range (0,200,20):
    s.angle = i
    print(i)
    sleep(1.0)


s.detach()