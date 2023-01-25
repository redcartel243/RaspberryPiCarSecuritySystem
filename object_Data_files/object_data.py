import cv2 as cv
# reading the reference image from dir
import object_detector


#!!!!ADD OBJECT DATA HERE FOR DISTANCE AND SPEED CALCULATION
ref_person = cv.imread('ReferenceImages/person.png')
ref_mobile = cv.imread('ReferenceImages/cell phone.png')
ref_bottle = cv.imread('ReferenceImages/bottle.jpg')
# calling them seperately because on each image , there is a single class . If there was many class on each image we could have used them in a different way

person_data = object_detector.object_detector(ref_person)
person_width_in_rf = person_data[0][1]


mobile_data = object_detector.object_detector(ref_mobile)
mobile_width_in_rf = mobile_data[0][1]

bottle_data = object_detector.object_detector(ref_bottle)
bottle_width_in_rf = bottle_data[0][1]

# Distance constants
KNOWN_DISTANCE = 45  # INCHES

# print(f"Person width in pixels : {person_width_in_rf} mobile width in pixel: {mobile_width_in_rf}")

chosenObject = {"cell phone": {"width": 3.0, "width_in_rf": mobile_width_in_rf},
                    "bottle": {"width": 4.0, "width_in_rf": bottle_width_in_rf},
                    "person": {"width": 22.0, "width_in_rf": person_width_in_rf}}  # WIDTH IN INCHES