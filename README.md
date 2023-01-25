# RaspberryPiCarSecuritySystem
## **Object tracking**
We use data representing the bounding boxes positions provided by the object recognition model YOLOv4 to track a moving item and then transfer it to a servo to keep track of it. The accuracy of object tracking in our case strongly depends on the quality of the object recognition model and the algorithm efficiency. 

An object is in the middle of the camera when x=250 and y=175

**Representation of the two-dimensional coordinate plane on OpenCV**

**Servo and Object position representation**

**Field of view and Limit representation(The camera tracks objects moving outside the limit)**









**Midpoint of multiple object**

The single and the multiple(midpoint) object tracking require to get the X and Y coordinate from object recognition.
### **Distance calculation**
The distance is calculated using a single camera. 

**Triangle similarity representation**

Known width W. We then place this object to a known distance D away from the camera we then use image processing techniques from OpenCV to calculate the apparent width of the object in pixel (object recognition) P hence, we can find the focal length: 

`                                                               `F=(P•D)/W                                                                   

D’ can be found using F’=(P•D)/W|P•D=F•W|D=(W•F)/P hence:

`   `D’=(W•F)/P					                

This means that from the focal length we found at the beginning, we can continuously find the distance of the object D’:

Object recognition–>Object width in pixel P->F= (P•D known)/W known -> D’=(W known • F)/P->….


**Speed Calculation**

To perform the speed calculation of an object we need to know its distance on each frame and store the change in a list with:

changeDistance = distance - self.intialDisntace

then convert it in meter because the distance is in inch:

distanceInMeters = changeDistance \* 0.0254

**Main project Design**


\- Live feed image of video streaming image 

\- Sending instructions to the robot (Control)

Components:

-Raspberry Pi

\- Floor Chassis L298N motor diver chip

\- Ultrasonic sensor HC-SR04(Obstacles)

\- Upper and Lower servo motors

-Camera

-4 DC motors

-4 Wheels


The control system window is the principal windows; it contains the principal features implemented in the system:

`			                      `**Main window of user interface**

It is composed of 14 elements designed to perform different tasks, we will describe the function of each element and briefly show the implementation of important features.

1- Display the image from Raspberry pi camera

This QT widget is a label that has been converted from its original use to display images automatically. The method that is used to display video has been implemented as follows:

2- Display the two servos motor position

These two labels display the servo motor position in degrees. Here is an example of the implementation:

**Display servo position method**

3- Start/Stop streaming

**Start/Stop streaming**

This button is used to start/stop the video streaming on the raspberry pi. Here is the implementation of the button.

4- Start/Stop Object recognition

This button is used to start/stop the object recognition method introduced in chapters 2 and 3.

**Start/Stop Object recognition 1**

Then the Boolean will activate the recognition method in the method that updates the frames in the label:

**Start/Stop Object recognition 2**

5- Start video recording

This button is used to start a video recording.

6- Stop video recording

This button is used to stop a video recording.

7- Control raspberry pi wheels

These 4 control buttons are used to control raspberry pi motor drivers.

8- Control servo 

These 4 control buttons are used to control the servo motors.

9- Start auto mode

This button is used to start the auto mode introduced in chapter 2 as object tracking.

**Auto mode method 1**

**Auto mode method 2**

10- Stop auto mode

This button is used to stop the auto mode, it sets the Boolean in “automode” to False 

11- Start monitoring

This button is used to start the monitoring mode. The monitoring mode Turns off all the buttons to leave only the camera open.

12- Stop monitoring

This button is used to stop the monitoring mode.

14- Text field

This text field is used to display information from.

**Display Information method**
### Settings
Settings is a secondary window implemented to support the main window features and provide some options such as choosing what information to receive, which object to recognize, Robot speed and servo speed at runtime. 

**Settings window**
## Functional user interface
### Streaming

**Streaming feature 1**

The streaming feature turns the camera on (Raspberry pi camera) that is connected to the computer via Ip-based network. Turning On the camera enables all the other features such as Object recognition, Object tracking or Automode (See Fig.5-13). The streaming feature server implementation as been described in 

**Streaming feature 2**
### Detection
Detection button holds the motion and the object recognition feature. We are able to choose which object to detect and what information to display on the display panel from the settings:

**Selection of settings**

**Settings selected display**
### Automode
The auto mode has been introduced in chapter 3 as object tracking. It enables the automatic object tracking and disables the user servo control feature. 

` `**Automode** 
### ` `Robot control
From the moment the program launches, the robot can be controlled using the keyword up, down, right, left button or the direction buttons on the screen (can be used on touchscreen).

**Fig.5-17 Robot Control panel** 

### Monitoring
The monitoring mode can be explained as a stealth mode where the robot speed is decreased to the minimum (to prevent noise), the servo speed is decreased to the minimum and the object recognition is on. It is used in situations when the robot should not be detected by its surrounding.
