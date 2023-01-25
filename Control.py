import gpiozero
from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import Robot,DistanceSensor
from gpiozero import AngularServo
from time import sleep
import control_data
from pynput.keyboard import Key, Listener
from threading import Thread

"""how to use streaming camera :
video = cv2.VideoCapture("http://192.168.137.88:8080/?action=stream")
"""
#k
factory = PiGPIOFactory(host='192.168.137.164')
gpiozero.Device.pin_factory=factory
robot = Robot(left=(18, 23), right=(24, 25),pin_factory=factory)
distanceSensor = DistanceSensor(21, 20,pin_factory=factory)
lowerServo = 2
upperServo= 3
keys = []
myCorrection = 0.45
maxPW = (1.5 + myCorrection) / 1000
minPW = (1.0 - myCorrection) / 1000

panServo = AngularServo(2, min_angle=-180, max_angle=180, initial_angle=0, max_pulse_width=maxPW, min_pulse_width=minPW,
                        pin_factory=factory)
tiltServo = AngularServo(3, min_angle=180, max_angle=-180, initial_angle=0, max_pulse_width=maxPW,
                         min_pulse_width=minPW, pin_factory=factory)

class RobotControl(Thread):
    def __init__(self):
        Thread.__init__(self)
    def run(self):
        with Listener(on_press=on_press,
                      on_release=on_release) as listener:
            listener.join()
class RobotAutoStop(Thread):
    def __init__(self):
        Thread.__init__(self)
    def run(self):
        while True:
            distance_to_object = distanceSensor.distance * 100
            print(distance_to_object)
            if distance_to_object <= 30:
                robot.backward()

                robot.stop()


def setServoAngle(servo, angle):
    servo.angle = angle

def servoTurnRight():
    print("servo right")

    if control_data.x < 180:
        control_data.x = control_data.x + 10
        setServoAngle(panServo, control_data.x)
        print("servo angle {}".format(panServo.angle))
    else:
        print("maximum angle ranched for pan servo")
        panServo.detach()
def servoTurnLeft():
    print("servo left")
    if control_data.x > -180:
        control_data.x = control_data.x - 10
        setServoAngle(panServo, control_data.x)
        print("servo angle {}".format(panServo.angle))
    else:
        print("maximum angle ranched for pan servo")
        panServo.detach()


def servoDown():
    print("servo down")
    if control_data.y > -140:
        control_data.y = control_data.y - 5
        setServoAngle(tiltServo, control_data.y)
        print("servo angle {}".format(tiltServo.angle))
    else:
        print("maximum angle ranched for tilt servo")
        tiltServo.detach()

def servoUp():
    print("servo up")
    if control_data.y < 140:
        control_data.y = control_data.y + 5
        setServoAngle(tiltServo, control_data.y)
        print("servo angle {}".format(tiltServo.angle))
    else:
        print("maximum angle ranched for tilt servo")
        tiltServo.detach()
#ws


#wsssssssssssssdddaaaaaasssssssssssaaasassawwddd


def on_press(key):
    try:
        if (key.char == 'w'):
            servoUp()
            sleep(0.1)
        elif (key.char == 'd'):
            servoTurnRight()
            sleep(0.1)
        elif (key.char == 's'):
            servoDown()
            sleep(0.1)
        elif (key.char == 'a'):
            servoTurnLeft()
            sleep(0.1)
        else:
            print()

    except AttributeError:
        if (key == key.up):
            robot.forward()
        elif (key == key.down):
            robot.backward()
        elif (key == key.right):
            robot.right()
        elif (key == key.left):
            robot.left()






def on_release(key):
    try:
        if (key.char == 'w'):
            print()
        elif (key.char == 'd'):
            print()
        elif (key.char == 's'):
            print()
        elif (key.char == 'a'):
            print()
        else:
            print()

    except AttributeError:
        if (key == key.up):
            robot.stop()
        elif (key == key.down):
            robot.stop()
        elif (key == key.right):
            robot.stop()
        elif (key == key.left):
            robot.stop()

    if key == Key.esc:
        # Stop listener
        return False



thread1 = RobotControl()
thread2= RobotAutoStop()
thread2.start()
thread1.start()
thread1.join()
thread2.join()






"""while True:
    # to control wheels from computer
    if keyboard.read_key() == "up":
        keyboard.
        keyboard.on_press_key("up", lambda _: robot.forward(), suppress=True)
        keyboard.on_release_key("up", lambda _: robot.stop(), suppress=True)
    elif keyboard.read_key() == "down":
        keyboard.on_press_key("down", lambda _: robot.backward(), suppress=True)
        keyboard.on_release_key("down", lambda _: robot.stop(), suppress=True)
    elif keyboard.read_key() == "left":
        keyboard.on_press_key("left", lambda _: robot.left(), suppress=True)
        keyboard.on_release_key("left", lambda _: robot.stop(), suppress=True)
    elif keyboard.read_key() == "right":
        keyboard.on_press_key("right", lambda _: robot.right(), suppress=True)
        keyboard.on_release_key("right", lambda _: robot.stop(), suppress=True)
    # control servo:
    elif keyboard.read_key() == "w":
        keyboard.on_press_key("w", lambda _: servo2.min(), suppress=True)
        keyboard.on_release_key("s", lambda _: servo2.mid(), suppress=True)
    elif keyboard.read_key() == "s":
        keyboard.on_press_key("s", lambda _: servo2.max(), suppress=True)
        keyboard.on_release_key("s", lambda _: servo2.mid(), suppress=True)
    elif keyboard.read_key() == "a":
        keyboard.on_press_key("a", lambda _: servo.max(), suppress=True)
        keyboard.on_release_key("a", lambda _: servo.mid(), suppress=True)
    elif keyboard.read_key() == "d":
        keyboard.on_press_key("d", lambda _: servo.min(), suppress=True)
        keyboard.on_release_key("d", lambda _: servo.mid(), suppress=True)
    else:
        continue"""

"""
    servo.mid()
    print("mid")
    sleep(0.5)
    servo.min()
    print("min")
    sleep(1)
    servo.mid()
    print("mid")
    sleep(0.5)
    servo.max()
    print("max")
    sleep(1)"""

