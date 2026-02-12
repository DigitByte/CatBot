#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped
import time
from tf.transformations import quaternion_from_euler
import numpy as np

class SimulateIMU:
    def __init__(self, robot_name):
        self.debug("INFO", "simulate_imu_node started.")
        ####### publishers ##########
        self.IMUPub = rospy.Publisher('/' + robot_name + "/imu_data", Imu, queue_size=1)
        ####### subscribers ########
        rospy.Subscriber('/' + robot_name + "/vel_cmd", TwistStamped, self.velCmdCB)

        self.yaw_vel = 0
        self.yaw     = 0

        self.timer = time.time()

        self.imu_data = Imu()


    """ ROS callbacks """
    def velCmdCB(self, data):
        self.yaw_vel = np.clip(data.twist.angular.z, -0.5, 0.5)

    """ Main loop for the node """

    def mainThread(self):
        dt = time.time() - self.timer
        self.timer = time.time()

        print(self.yaw_vel)

        self.yaw += self.yaw_vel * dt

        q = quaternion_from_euler(0.0, 0., self.yaw)
        self.imu_data.angular_velocity.x = 0
        self.imu_data.angular_velocity.y = 0
        self.imu_data.angular_velocity.z = self.yaw_vel


        self.imu_data.header.stamp = rospy.Time.now()
        self.imu_data.orientation.x = q[0]
        self.imu_data.orientation.y = q[1]
        self.imu_data.orientation.z = q[2]
        self.imu_data.orientation.w = q[3]


        self.IMUPub.publish(self.imu_data)



    def debug(self, typ, msg):
        print(typ + ": " + msg + "\n")


if __name__ == '__main__':
    try:
        rospy.init_node('publish_camera_view_node', anonymous=True)
        rate = rospy.Rate(50)  # 10 Hz

        robot_name = "catbot"
        if rospy.has_param('/robot_name'):
            robot_name = rospy.get_param("/robot_name")

        node = SimulateIMU(robot_name)

        while not rospy.is_shutdown():
            node.mainThread()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
