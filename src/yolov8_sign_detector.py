#!/usr/bin/python3

# Python
import copy

# Object detection
import cv2
import numpy as np
from ultralytics import YOLO
from ultralytics.yolo.engine.results import Results
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image as im

# ROS
import rospy
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import UInt8

class Sign_Detector():
    def __init__(self):

        self.image_res = 240, 320, 3 # Camera resolution: height, width
        self.image_np = np.zeros(self.image_res) # The numpy array to pour the image data into

        self.model: YOLO = YOLO('/home/rohamzn/Robotics-Course/catkin_ws/src/robotics_final_project/yolo/best.pt')
        # subscribes raw image
        self.sub_image_original = rospy.Subscriber('/detect/image_input', Image, self.camera_listener, queue_size = 1)
        # publishes traffic sign image in compressed type 
        self.pub_image_traffic_sign = rospy.Publisher('/detect/image_output/compressed', CompressedImage, queue_size = 1)
        self.results: Results = None

        # publishes
        self.pub_stop_sign = rospy.Publisher('/detect/traffic_sign/stop_sign', UInt8, queue_size=1)
        self.pub_pedestrian_crossing = rospy.Publisher('/detect/traffic_sign/pedestrian_crossing', UInt8, queue_size=1)
        #self.pub_crossing_sign = rospy.Publisher('/detect/traffic_sign/crossing', UInt8, queue_size=1)
        self.pub_beware_of_children = rospy.Publisher('/detect/traffic_sign/beware_of_children', UInt8, queue_size=1)
        #self.pub_tunnel_sign = rospy.Publisher('/detect/traffic_sign', UInt8, queue_size=1)

        self.cvBridge = CvBridge()
        self.counter = 1


    def camera_listener(self, msg: Image):
        if self.counter % 3 != 0:
            self.counter += 1
            return
        else:
            self.counter = 1

        # Convert binary image data to  cv image
        cv_image_input = self.cvBridge.imgmsg_to_cv2(msg, "bgr8")
        self.image_np = np.frombuffer(msg.data, dtype=np.uint8)
        self.image_np = self.image_np.reshape(self.image_res)

        # TODO our model yolo is still getting trained. But we expect good results. 

        # Predicting results
        self.results = self.model(self.image_np, verbose = False)
        res_plotted = self.results[0].plot()
        cv_image_output = cv2.cvtColor(res_plotted, cv2.COLOR_BGR2RGB)
        self.detect_sign()
        #frame = copy.deepcopy(self.image_np)
        

        # publishes traffic sign image in compressed type
        self.pub_image_traffic_sign.publish(self.cvBridge.cv2_to_compressed_imgmsg(cv_image_output, "jpg"))

    def detect_sign(self):
        result = self.results[0]
        top5_predictions = []
        if result.probs == None:
            return 
        for class_id in result.probs.top5:
            top5_predictions.append(result.names[class_id])
        if "Stop_Sign" in top5_predictions:
            msg_sign = UInt8()
            msg_sign.data = 1
            self.pub_stop_sign.publish(msg_sign)
            rospy.loginfo("stop_sign")
            return
        elif "Pedestrian_Crossing" in top5_predictions:
            msg_sign = UInt8()
            msg_sign.data = 1
            self.pub_pedestrian_crossing.publish(msg_sign)
            rospy.loginfo("perestrian_crossing")
            return
        elif "Beware of children" in top5_predictions:
            msg_sign = UInt8()
            msg_sign.data = 1
            self.pub_beware_of_children.publish(msg_sign)
            rospy.loginfo("beware of children")
            return
        # elif "turn_right" in top5_predictions:
        #     msg_sign = UInt8()
        #     msg_sign.data = 1
        #     self.pub_image_traffic_sign.publish(msg_sign)
        #     rospy.loginfo("turn_right")
        #     return
            
    def main(self):
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node("yolo_sing_detector")

    image_processor = Sign_Detector()

    image_processor.main()
