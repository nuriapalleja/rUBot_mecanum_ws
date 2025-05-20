#!/usr/bin/env python3
import datetime
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from tensorflow.keras.models import load_model
import os
from time import time

# Constants
SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
MODEL_PATH = os.path.join(SCRIPT_DIR, "../models/keras_model.h5")
LABELS_PATH = os.path.join(SCRIPT_DIR, "../models/labels.txt")

class KerasImageClassifier:
    def __init__(self):
        # Load model and labels
        self.model = load_model(MODEL_PATH)
        self.labels = self.load_labels(LABELS_PATH)
        self.imgut_shape = self.model.imgut_shape[1:3]

        # Class to transform images from ROS messages to OpenCV images
        self.bridge = CvBridge()

        # Publisher to publish predicted classes
        self.pub_class = rospy.Publisher("/predicted_class", String, queue_size=1)

        # Subscriber to read raw images
        self.sub_img = rospy.Subscriber("/usb_cam/image_raw", Image,
                                        self.image_callback, queue_size=1)

        # Capturas
        self.capture_enabled = True
        self.capture_dir = os.path.expanduser("~/rUBot_captures")
        os.makedirs(self.capture_dir, exist_ok=True)
        self.make_class_dirs()
        self.capture_interval = 1.0
        self.last_capture_time = time.time()
        rospy.Subscriber("/capture_toggle", Bool, self.toggle_callback)


    def load_labels(self, path):
        '''
        Load model lables

        Params:
            path: str
                String with the labels file path
        Returns:
            lables: list[str]
                list with all model lables
        '''

        with open(path, 'r') as f:
            return [line.strip().split(' ',1)[1] for line in f]


    def make_class_dirs(self):
        '''
        Load model lables

        Params:
            path: str
                String with the labels file path
        Returns:
            lables: list[str]
                list with all model lables
        '''
        for class_name in self.labels:
            os.makedirs(os.path.join(self.capture_dir, class_name), exist_ok=True)


    def toggle_callback(self, msg):
        self.capture_enabled = msg.data
        state = "ON" if msg.data else "OFF"
        rospy.loginfo(f"[ImagePredictor] Captura automática {state}")


    def capture_image(self, img, class_name):
        '''
        Saves the curret image
        
        Params:
            img: numpy.ndarray
                Image in OpenCV format

            class_name: str
                String with the class name for the dir name
        '''

        current_time = time.time()
        if current_time - self.last_capture_time >= self.capture_interval:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"{class_name}_{timestamp}.jpg"
            class_folder = os.path.join(self.capture_dir, class_name)
            filepath = os.path.join(class_folder, filename)
            cv2.imwrite(filepath, img)
            rospy.loginfo(f"Imagen guardada: {filepath}")
            self.last_capture_time = current_time


    def image_callback(self, msg):
        '''
        Performs Image prediction and publish it

        Params:
            msg: sensor_msgs.msg.Image
                ROS image message from camera
        '''
        try:
            # Convert image to OpenCV
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            resized = cv2.resize(cv_img, tuple(self.imgut_shape))
            img = resized.astype(np.float32) / 255.0
            img = np.expand_dims(img, axis=0)

            # Prediction
            predictions = self.model.predict(img)
            class_index = np.argmax(predictions)
            class_name = self.labels[class_index]
            rospy.loginfo(f"Detectado {class_name}")

            # Publish
            self.pub_class.publish(class_name)

            # Save image
            if self.capture_enabled:
                self.capture_image(cv_img, class_name)

        except CvBridgeError as e:
            rospy.logerr(f"[ImagePredictor] CvBridgeError: {e}")

        except Exception as e:
            rospy.logerr(f"[ImagePredictor] Error: {e}")


if __name__ == "__main__":
    rospy.init_node("keras_detect_signs")
    KerasImageClassifier()
    rospy.spin()
