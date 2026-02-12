#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class PublishCameraView:
    def __init__(self):
        self.debug("INFO", "catbot_vision_node started.")
        ####### publishers ##########
        self.imagePub = rospy.Publisher("/catbot/camera", Image, queue_size=1)
        self.camera   = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
        self.bridge   = CvBridge()
        ####### subscribers ########
        # rospy.Subscriber("chatter", String, self.callback)


    """ ROS callbacks """
    # def callback(self, data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)



    """ Main loop for the node """
    def mainThread(self):
        ret, frame = self.camera.read()
        if frame is None:
            rospy.logerr('Camera device not found')
            return
        frame = cv2.resize(frame, None, fx = 0.25, fy = 0.25)
        frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        
        
        try:
            self.imagePub.publish(self.bridge.cv2_to_imgmsg(frame, encoding="passthrough"))
        except CvBridgeError as e:
            print(e)



    def debug(self, typ, msg):
        print typ + ": " + msg + "\n"


if __name__ == '__main__':
    try:
        rospy.init_node('simulated_imu_sensor_node', anonymous=True)
        rate = rospy.Rate(10)    # 10 Hz
        node = PublishCameraView()

        while not rospy.is_shutdown():
            node.mainThread()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
