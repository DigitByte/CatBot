#!/usr/bin/env python

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point, PoseStamped
import numpy as np
import tf
import time


class ManualControl:
    def __init__(self):
        time.sleep(10)
        self.server = InteractiveMarkerServer("catbot/visualization/manual_control")

        self.tfListener = tf.TransformListener()
        while True:
            try:
                (trans, rot) = self.tfListener.lookupTransform('ground', 'base_link', rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
            # print('transform unavailable for gravity')

        position = Point(trans[0], trans[1], trans[2])



        self.make6DofMarker('ground', position)
        self.server.applyChanges()

        self.torsoControlPub = rospy.Publisher('/catbot/torso_pose_control', PoseStamped, queue_size=10)
        self.previous_pos = np.array([position.x, position.y, position.z])

    def processFeedback(self, feedback):
        # print feedback.pose.position, feedback.pose.orientation


        pos = np.array([feedback.pose.position.x, feedback.pose.position.y, feedback.pose.position.z])
        d = np.linalg.norm(pos - self.previous_pos)
        p = PoseStamped()
        p.header.frame_id = 'ground'
        p.pose = feedback.pose
        self.torsoControlPub.publish(p)
        self.previous_pos = pos

    def makeBox(self, msg):
        marker = Marker()
        marker.type = Marker.CUBE
        marker.scale.x = msg.scale * 0.5
        marker.scale.y = msg.scale * 0.5
        marker.scale.z = msg.scale * 0.5
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 0
        return marker

    def makeBoxControl(self, msg):
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(self.makeBox(msg))
        msg.controls.append(control)
        return control



    def make6DofMarker(self, frame_id, position):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = frame_id
        int_marker.pose.position = position
        int_marker.scale = 0.2

        int_marker.name = "control_torso"
        int_marker.description = "6-DOF torso control"

        # insert a box
        self.makeBoxControl(int_marker)
        int_marker.controls[0].interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS

        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS

        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS

        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

        int_marker.controls.append(control)
        self.server.insert(int_marker, self.processFeedback)



    """ Main loop for the node """
    def mainThread(self):
        a = 1





if __name__ == '__main__':
    try:
        rospy.init_node('catbot_control_node', anonymous=True)
        rate = rospy.Rate(5)  # 10 Hz
        node = ManualControl()

        while not rospy.is_shutdown():
            node.mainThread()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
