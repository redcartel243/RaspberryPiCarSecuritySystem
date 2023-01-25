from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QImage, QPixmap
import sys
import cv2
from threading import Thread, Lock
import time
from Streaming import *


class WebcamVideoStream :

    def __init__(self, src, width = 640, height = 480) :
        self.stream = cv2.VideoCapture(src)
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.stream.set(cv2.CAP_PROP_BUFFERSIZE, 5)
        self.FPS = 1/30
        self.FPS_MS = int(self.FPS * 1000)
        (self.grabbed, self.frame) = self.stream.read()
        self.started = False
        self.read_lock = Lock()

    def start(self) :
        if self.started :
            return None
        self.started = True
        self.thread = Thread(target=self.update, args=())
        self.thread.daemon = True
        self.thread.start()
        return self

    def update(self) :
        while self.started :
            (grabbed, frame) = self.stream.read()
            time.sleep(self.FPS)
            self.read_lock.acquire()
            self.grabbed, self.frame = grabbed, frame
            self.read_lock.release()

    def read(self) :
        self.read_lock.acquire()
        frame = self.frame.copy()
        self.read_lock.release()
        return frame

    def stop(self) :
        self.started = False
        if self.thread.is_alive():
            self.thread.join()

    def __exit__(self, exc_type, exc_value, traceback) :
        self.stream.release()


class MainWindow(QtWidgets.QDialog, Ui_Dialog):





    # colors for object detected
    COLORS = [(255, 0, 0), (255, 0, 255), (0, 255, 255), (255, 255, 0), (0, 255, 0), (255, 0, 0)]
    # Object detector constant
    CONFIDENCE_THRESHOLD = 0.4
    NMS_THRESHOLD = 0.3
    # defining fonts
    FONTS = cv2.FONT_HERSHEY_COMPLEX

    # getting class names from classes.txt file
    class_names = []
    with open("object_Data_files/classes.txt", "r") as f:
        class_names = [cname.strip() for cname in f.readlines()]
    #  setttng up opencv net
    yoloNet = cv2.dnn.readNet('yolov4-tiny.weights', 'yolov4-tiny.cfg')

    yoloNet.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
    yoloNet.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)

    model = cv2.dnn_DetectionModel(yoloNet)
    model.setInputParams(size=(416, 416), scale=1 / 255, swapRB=True)
    def __init__(self, *args, obj=None, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.setupUi(self)
        self.pushButton.clicked.connect(self.start_cam)
        self.pushButton_2.clicked.connect(self.recordImage)


    def start_cam(self):
        #self.setImageRecording(False)
        self.capture = WebcamVideoStream(src = 0).start()
        self.timer=QTimer(self)
        self.timer.setTimerType(QtCore.Qt.PreciseTimer)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(2)



    def update_frame(self):
        self.image=self.capture.read()
        self.RecognitionMethod(self.image)
        self.displayImage(self.image)


    def displayImage(self,img):
        qformat=QImage.Format_Indexed8
        if len(img.shape)==3:
            if img.shape[2]==4:
                qformat=QImage.Format_RGBA8888
            else:
                qformat=QImage.Format_RGB888

        outImage=QImage(img,img.shape[1],img.shape[0],img.strides[0],qformat)
        outImage=outImage.rgbSwapped()

        self.label.setPixmap(QPixmap.fromImage(outImage))
        self.label.setScaledContents(True)
        return outImage

    def recordImage(self, img):
        self.fourcc = cv2.VideoWriter_fourcc('M','J','P','G')
        self.out = cv2.VideoWriter('output.avi', self.fourcc, 10, (640,480))
        self.out.write(img)


    def RecognitionMethod(self,img):
        classes, scores, boxes = self.model.detect(img, self.CONFIDENCE_THRESHOLD, self.NMS_THRESHOLD)
        # creating empty list to add objects data
        data_list = []
        for (classid, score, box) in zip(classes, scores, boxes):
            # define color of each, object based on its class id
            color = self.COLORS[int(classid) % len(self.COLORS)]

            label = "%s : %f" % (self.class_names[classid[0]], score)
            """if(class_names[classid[0]]=="person"):
                print(class_names[classid[0]])"""

            # draw rectangle on and label on object
            cv2.rectangle(img, box, color, 2)
            # normaly its box[0] but we will do box[0]+box[0]/2 because we need a good position of x for the object tracking so we will return this x
            x = int(box[0] + box[0] / 2)

            cv2.putText(img, label, (box[0], box[1] - 14), self.FONTS, 0.5, color, 2)



if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    mainW = MainWindow()
    mainW.show()
    sys.exit(app.exec_())