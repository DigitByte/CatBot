#!/usr/bin/env python
import rospy
import rospkg

rospack = rospkg.RosPack()


import pybullet as pb
import time
import pybullet_data
import numpy as np
from catbot_sim import catbot_sim

from sensor_msgs.msg import JointState, Imu
from catbot_msgs.msg import contact_detection
from sensor_msgs.msg import LaserScan


class CatBotPybullet:
    def __init__(self, urdf_dir, robot_name):
        self.debug("INFO", "catbot_pybullet_node started.")



        # ## PyBullet configuration ###
        pb.setTimeStep(0.001)
        pb.setGravity(0, 0, -9.8)
        ground = pb.loadURDF("plane.urdf", [0,0,0])

        # maze_urdf_dir = rospack.get_path('catbot_pybullet') + '/worlds/maze.urdf'

        room_urdf_dir = rospack.get_path('catbot_pybullet') + '/worlds/room.urdf'
        rospy.logerr(room_urdf_dir)
        room          = pb.loadURDF(room_urdf_dir, [0.0,0.0,0])

        pb.changeDynamics(ground, -1, lateralFriction=1, spinningFriction=0.01, rollingFriction=0.01)


        #### CatBot model ####
        startpos = [0, 0, 0.26]  # meters
        startOri = [0, 0, 0]     # degrees
        self.catbot = catbot_sim(urdf_dir, startpos, startOri, ground)
        self.jointStateMSg = JointState()
        self.jointStateMSg.name = list(self.catbot.getRevoluteJointNames())
        self.feet_contact = np.zeros(4, dtype = 'bool')
        self.contact_det_msg = contact_detection()

        ####### publishers ##########
        self.jointStatePub = rospy.Publisher('/' + robot_name + '/joints_state', JointState, queue_size=10)
        self.robotImuPub = rospy.Publisher('/' + robot_name + '/imu_data',  Imu, queue_size=10)
        self.contactDetectionPub = rospy.Publisher('/' + robot_name + '/contact_detection',  contact_detection, queue_size=10)
        self.laserScanPub = rospy.Publisher('/scan',  LaserScan, queue_size=10)





        ####### subscribers ########
        rospy.Subscriber('/' + robot_name + '/joints_control', JointState, self.jointControlCB)




    """ ROS callbacks """
    def jointControlCB(self, data):
        self.catbot.setJointPosition(data.position, velocity = 1)


    """ Main loop for the simulation node """
    def mainThread(self):

        basePos, baseOrn = pb.getBasePositionAndOrientation(1)  # current base pose
        # pb.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=45, cameraPitch=-20,
        # cameraTargetPosition=basePos)  # lock the view to the robot

        pb.stepSimulation()

        self.feet_contact = self.catbot.getFeetContact()

        # self.publishJointState()
        self.publishIMUdata()
        self.publishLidarData()
        self.publishContactDetectorData()
    def publishJointState(self):
        pos, vel, torq = self.catbot.getMotorJointStates()
        self.jointStateMSg.position = pos
        self.jointStateMSg.velocity = vel
        self.jointStateMSg.effort   = torq
        self.jointStateMSg.header.stamp = rospy.Time.now()
        self.jointStatePub.publish(self.jointStateMSg)

    def publishContactDetectorData(self):
        self.contact_det_msg.header.stamp = rospy.Time.now()
        self.contact_det_msg.feet_stance = [self.feet_contact[0],self.feet_contact[1],self.feet_contact[2],self.feet_contact[3]]
        # self.contact_det_msg.feet_stance.append(self.feet_contact[0])
        self.contactDetectionPub.publish(self.contact_det_msg)

    def publishIMUdata(self):
        pos, vel, ang_pos, ang_vel, acc, ang_acc = self.catbot.getRobotState()

        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "base_link"

        imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w = ang_pos

        imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z = ang_vel
        imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z = acc


        self.robotImuPub.publish(imu_msg)


    def publishLidarData(self):
        max_dist = 8 # meters
        res_deg = 1

        all_angles, distance = self.catbot.getLidarData(max_dist, res_deg)

        if all_angles is None:
            return

        lidar_msg = LaserScan()

        lidar_msg.angle_min       = 0
        lidar_msg.angle_max       = 2*np.pi
        lidar_msg.angle_increment = res_deg*(np.pi/180)

        lidar_msg.time_increment = (0.1/360)/1000000000.0
        lidar_msg.scan_time      = (0.1)/1000000000.0

        lidar_msg.range_min       = 0
        lidar_msg.range_max       = max_dist

        lidar_msg.ranges      = distance
        lidar_msg.intensities = distance

        lidar_msg.header.stamp = rospy.Time.now()
        lidar_msg.header.frame_id = "lidar_link"

        self.laserScanPub.publish(lidar_msg)

    def debug(self, typ, msg):
        print(typ + ": " + msg + "\n")


if __name__ == '__main__':
    try:
        rospy.init_node('catbot_pybullet_node', anonymous=True)
        rate = rospy.Rate(1000)    # 10 Hz

        robot_name = "catbot"
        urdf_dir  = rospack.get_path('catbot_description') + '/urdf/catbot.urdf'
        if rospy.has_param('/robot_name'):
            robot_name = rospy.get_param("/robot_name")

        if rospy.has_param('/urdf_path'):
            urdf_dir = rospy.get_param("/urdf_path")

        client = pb.connect(pb.GUI)  # or p.DIRECT for non-graphical version
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally

        node = CatBotPybullet(urdf_dir, robot_name)

        while not rospy.is_shutdown():
            node.mainThread()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    pb.disconnect()
