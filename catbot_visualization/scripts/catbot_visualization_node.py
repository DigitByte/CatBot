#!/usr/bin/env python
import rospy
#!/usr/bin/env python
import rospy
import rospkg
import sys

rospack = rospkg.RosPack()
# sys.path.append(rospack.get_path('catbot_drivers') + '/src/')


from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion, PolygonStamped, Point32
from sensor_msgs.msg import JointState
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
from  catbot_msgs.msg import feet_pressure, robot_state, contact_detection
import numpy as np
import numpy as np
from sensor_msgs.msg import JointState



def transformPoint(pos, trans, quaternion):
    trans_mat = tf.transformations.translation_matrix(trans)
    rot_mat = tf.transformations.quaternion_matrix(quaternion)
    tfMatrix = np.dot(rot_mat, trans_mat)


class CatBotVisualization:
    def __init__(self, robot_name):
        self.debug("INFO", "catbot_visualization_node started.")

        ####### publishers ##########
        # self.pub = rospy.Publisher('chatter', String, queue_size=10)

        self.feetPressPub     = rospy.Publisher('/' + robot_name + '/visualization/feet_pressure', Marker, queue_size=10)
        self.feetPressCntrPub = rospy.Publisher('/' + robot_name + '/visualization/feet_pressure_control', Marker,  queue_size=10)

        self.feetposPub = rospy.Publisher('/' + robot_name + '/visualization/feet_pos', Marker, queue_size=10)

        self.robotStateMarkerPub    = rospy.Publisher('/' + robot_name + '/visualization/robot_state', Marker, queue_size=10)
        self.robotStateMarkerPub = rospy.Publisher('/' + robot_name + '/visualization/joints_torques',  Marker, queue_size=10)
        self.joint_state = JointState()

        ####### subscribers ########
        rospy.Subscriber('/' + robot_name + '/feet_pressure',         feet_pressure, self.feetPressCB)
        rospy.Subscriber('/' + robot_name + '/feet_pressure_control', feet_pressure, self.feetPressCntCB)
        rospy.Subscriber('/' + robot_name + '/state_estimation',     robot_state, self.robotStateCB)
        rospy.Subscriber('/' + robot_name + '/contact_detection',     contact_detection, self.contactDetectionCB)

        # rospy.Subscriber('/catbot/imu_data',    Imu,           self.imuDataCallBack)
        # rospy.Subscriber('/catbot/zmp_state',    zmp,          self.zmpStateCallBack)

        self.tfListener = tf.TransformListener()

        self.curr_robot_state    = robot_state()
        self.curr_feet_press     = feet_pressure()
        self.curr_feet_press_cnt = feet_pressure()
        self.curr_contact_detection = np.ones(4, dtype='bool')



    """ ROS callbacks """
    def feetPressCB(self, data):
        self.curr_feet_press = data


    def feetPressCntCB(self, data):
        self.curr_feet_press_cnt = data


    def robotStateCB(self, data):
        self.curr_robot_state = data

    def contactDetectionCB(self, data):
        self.curr_contact_detection = np.array(data.feet_stance, dtype = 'bool')

    def publishFeetPressure(self, data, color, publisher):
        # build pressure markers for visualization

        for i in range(len(data.leg_name)):
            name     = data.leg_name[i]
            pressure = data.pressure_vector[i]

            try:
                (trans, rot) = self.tfListener.lookupTransform('odom', 'feet_' + name, rospy.Time(0))

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print('Transform unavailable for feet_' + name)
                continue



            m = Marker()
            m.action = Marker.ADD
            m.header.frame_id = 'odom'
            m.header.stamp = rospy.Time.now()
            m.ns = 'pressure_{:s}'.format( name )
            m.id = 0
            m.type = Marker.LINE_STRIP


            startPoint = Point()
            startPoint.x = trans[0]
            startPoint.y = trans[1]
            startPoint.z = trans[2]

            endPoints = Point()
            endPoints.x = trans[0] + pressure.x
            endPoints.y = trans[1] + pressure.y
            endPoints.z = trans[2] + pressure.z



            m.points.append(startPoint)
            m.points.append(endPoints)



            m.scale.x = 0.01
            m.scale.y = 0.01
            m.scale.z = 0.01

            m.color.r = color[0]
            m.color.g = color[1]
            m.color.b = color[2]
            m.color.a = color[3]


            publisher.publish(m)






    def publishRobotState(self, color):
        # build velocity markers for visualization

        m = Marker()
        m.action = Marker.ADD
        m.header.frame_id = 'ground'
        m.header.stamp = rospy.Time.now()
        m.ns = 'base_velocity'
        m.id = 0
        m.type = Marker.LINE_STRIP

        startPoint = Point()
        startPoint.x = self.curretRobotState.pose.position.x
        startPoint.y = self.curretRobotState.pose.position.y
        startPoint.z = self.curretRobotState.pose.position.z

        endPoints = Point()
        endPoints.x = startPoint.x + 1*self.curretRobotState.linear_velocity.x
        endPoints.y = startPoint.y + 1*self.curretRobotState.linear_velocity.y
        endPoints.z = startPoint.z + 1*self.curretRobotState.linear_velocity.z

        m.points.append(startPoint)
        m.points.append(endPoints)

        m.scale.x = 0.01
        m.scale.y = 0.01
        m.scale.z = 0.01

        m.color.r = color[0]
        m.color.g = color[1]
        m.color.b = color[2]
        m.color.a = color[3]

        self.robotStateMarkerPub.publish(m)


    def publishFeetPos(self, data):
        # build foot position markers for visualization
        for i in range(len(data.leg_name)):
            name     = data.leg_name[i]
            feet_pos = data.feet_pos[i]

            color = [1,0,0,1]
            if self.curr_contact_detection[i] == 0:
                color = [0,1,0,1]

            print(self.curr_contact_detection[i])

            m = Marker()
            m.action = Marker.ADD
            m.header.frame_id = 'odom'
            m.header.stamp = rospy.Time.now()
            m.ns = 'feet_{:s}'.format(name)
            m.id = 0
            m.type = Marker.SPHERE

            m.pose.position.x = feet_pos.x
            m.pose.position.y = feet_pos.y
            m.pose.position.z = feet_pos.z

            m.pose.orientation.w = 0
            m.pose.orientation.x = 0
            m.pose.orientation.y = 0
            m.pose.orientation.z = 1

            m.scale.x = 0.02
            m.scale.y = 0.02
            m.scale.z = 0.02

            m.color.r = color[0]
            m.color.g = color[1]
            m.color.b = color[2]
            m.color.a = color[3]
            self.feetposPub.publish(m)

    '''
    




    def publishStableRegion(self):


        polygon = PolygonStamped()

        for p in self.currentZMPstate.stable_region.points:
            polygon.polygon.points.append(p)


        polygon.header.stamp = rospy.Time.now()
        polygon.header.frame_id = 'ground'
        self.stableRegPub.publish(polygon)


    def publishGravity(self):
        try:
            (trans, rot) = self.tfListener.lookupTransform('ground', 'torso', rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print('Transform unavailable for gravity')
            return

        m = Marker()
        m.action = Marker.ADD
        m.header.frame_id = 'ground'
        m.header.stamp = rospy.Time.now()
        m.ns = 'gravity'
        m.id = 0
        m.type = Marker.LINE_STRIP

        pStart = Point()
        pStart.x = trans[0]
        pStart.y = trans[1]
        pStart.z = trans[2]
        m.points.append(pStart)

        pEnd = Point()

        pEnd.x = self.currentZMPstate.zmp_pos.x
        pEnd.y = self.currentZMPstate.zmp_pos.y
        pEnd.z = self.currentZMPstate.zmp_pos.z

        # pEnd.x = trans[0] - self.imu_data.linear_acceleration.x*0.04
        # pEnd.y = trans[1] - self.imu_data.linear_acceleration.y*0.04
        # pEnd.z = trans[2] - self.imu_data.linear_acceleration.z*0.04

        m.points.append(pEnd)

        m.scale.x = 0.005
        m.scale.y = 0.005
        m.scale.z = 0.005

        m.color.r = 0.75
        m.color.g = 0.75
        m.color.b = 0.75
        m.color.a = 1

        self.CoMgroundPub.publish(m)

    '''

    """ Main loop for the node """
    def mainThread(self):

        self.publishFeetPressure(self.curr_feet_press,[0, 1, 0, 1],self.feetPressPub)
        self.publishFeetPressure(self.curr_feet_press_cnt,[1, 0, 0, 1],self.feetPressCntrPub)
        self.publishFeetPos(self.curr_feet_press_cnt)
        # self.publishRobotState([1,0,0,1])
        # self.publishCoM()
        # self.publishStableRegion()
        # self.publishGravity()

    def debug(self, typ, msg):
        print(typ + ": " + msg + "\n")


if __name__ == '__main__':
    try:
        rospy.init_node('catbot_visualization_node', anonymous=True)
        rate = rospy.Rate(30)    # 10 Hz

        robot_name = "catbot"
        if rospy.has_param('/robot_name'):
            robot_name = rospy.get_param("/robot_name")

        node = CatBotVisualization(robot_name)


        while not rospy.is_shutdown():
            node.mainThread()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
