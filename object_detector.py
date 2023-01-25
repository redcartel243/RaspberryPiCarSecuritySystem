import cv2 as cv
import RobotState_and_image_State

# colors for object detected
COLORS = [(255, 0, 0), (255, 0, 255), (0, 255, 255), (255, 255, 0), (0, 255, 0), (255, 0, 0)]
# Object detector constant
CONFIDENCE_THRESHOLD = 0.4
NMS_THRESHOLD = 0.3
# defining fonts
FONTS = cv.FONT_HERSHEY_COMPLEX


# getting class names from classes.txt file
class_names = []
with open("object_Data_files/classes.txt", "r") as f:
    class_names = [cname.strip() for cname in f.readlines()]
#  setttng up opencv net
yoloNet = cv.dnn.readNet('yolov4-tiny.weights', 'yolov4-tiny.cfg')

yoloNet.setPreferableBackend(cv.dnn.DNN_BACKEND_CUDA)
yoloNet.setPreferableTarget(cv.dnn.DNN_TARGET_CUDA_FP16)

model = cv.dnn_DetectionModel(yoloNet)
model.setInputParams(size=(416, 416), scale=1 / 255, swapRB=True)
# object detector funciton /method
def object_detector(image):
    classes, scores, boxes = model.detect(image, CONFIDENCE_THRESHOLD, NMS_THRESHOLD)
    # creating empty list to add objects data
    data_list = []
    for (classid, score, box) in zip(classes, scores, boxes):
        # define color of each, object based on its class id
        color = COLORS[int(classid) % len(COLORS)]


        label = "%s : %f" % (class_names[classid[0]], score)
        """if(class_names[classid[0]]=="person"):
            print(class_names[classid[0]])"""


        #normaly its box[0] but we will do box[0]+box[0]/2 because we need a good position of x for the object tracking so we will return this x
        x=int(box[0]+box[0]/2)



        # getting the data
        # 1: class name  2: object width in pixels, 3: position where have to draw text(distance)
        #See this to understand where to add more class ID to be recognized
        if classid in RobotState_and_image_State.chosenObject.values():  # designated class id
            # draw rectangle on and label on object
            cv.putText(image, label, (box[0], box[1] - 14), FONTS, 0.5, color, 2)
            cv.rectangle(image, box, color, 2)
            data_list.append([class_names[classid[0]], box[2], (box[0], box[1] - 2)])

        # if you want inclulde more classes then you have to simply add more [elif] statements here
        # returning list containing the object data.
    return data_list