#DO NOT USE THIS FILE FOR NOW

import cv2 as cv
import numpy as np
import pandas
from matplotlib import pyplot as plt

from datetime import datetime

def motionDetection():
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
    df = pandas.DataFrame(columns=["Start", "End"])

    while cap.isOpened():
        # Initializing motion = 0(no motion)
        motion = 0

        diff = cv.absdiff(frame1, frame2)
        diff_gray = cv.cvtColor(diff, cv.COLOR_BGR2GRAY)
        blur = cv.GaussianBlur(diff_gray, (5, 5), 0)
        _, thresh = cv.threshold(blur, 20, 255, cv.THRESH_BINARY)
        dilated = cv.dilate(thresh, None, iterations=3)
        contours, _ = cv.findContours(
            dilated, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            (x, y, w, h) = cv.boundingRect(contour)
            if cv.contourArea(contour) < 900:
                continue
            motion = 1
            cv.rectangle(frame1, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv.putText(frame1, "Status: {}".format('Movement'), (10, 20), cv.FONT_HERSHEY_SIMPLEX,
                       1, (255, 0, 0), 3)
        # Appending status of motion
        motion_list.append(motion)

        motion_list = motion_list[-2:]

        # Appending Start time of motion
        if motion_list[-1] == 1 and motion_list[-2] == 0:
            time.append(datetime.now())

        # Appending End time of motion
        if motion_list[-1] == 0 and motion_list[-2] == 1:
            time.append(datetime.now())
            for i in range(0, len(time), 2):
                try:
                    if time[i+1].second-time[i].second>=3:
                        print("the difference is {}".format(time[i+1].second-time[i].second))
                        df = df.append({"Start": time[i], "End": time[i + 1]}, ignore_index=True)
                    time.pop(i)
                    time.pop(i + 1)

                except Exception as e:
                    print(e)




            # Creating a CSV file in which time of movements will be saved
            df.to_csv("Time_of_movements.csv")



        # cv.drawContours(frame1, contours, -1, (0, 255, 0), 2)

        cv.imshow("Video", frame1)
        frame1 = frame2
        ret, frame2 = cap.read()

        if cv.waitKey(50) == 27:
            break

    # Appending time of motion in DataFrame
    for i in range(0, len(time), 2):
        df = df.append({"Start": time[i], "End": time[i + 1]}, ignore_index=True)

    # Creating a CSV file in which time of movements will be saved
    df.to_csv("Time_of_movements.csv")

    cap.release()
    cv.destroyAllWindows()


if __name__ == "__main__":
    motionDetection()