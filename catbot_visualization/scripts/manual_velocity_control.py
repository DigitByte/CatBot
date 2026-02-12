#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import pygame
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped
import numpy as np
import pygame
from tf import transformations
from catbot_msgs.srv import connect_servos, connect_servosRequest, connect_servosResponse
from catbot_msgs.srv import set_gait, set_gaitRequest, set_gaitResponse

class ManualControlNode:
    def __init__(self, robot_name):
        self.debug("INFO", "manual control node started.")
        pygame.init()

        pygame.font.init()  # you have to call this at the start,
        textFont = pygame.font.SysFont('Comic Sans MS', 30)
        self.screen = pygame.display.set_mode((640, 480))
        self.screen.fill((0, 0, 0))
        text = []
        text.append(textFont.render('Use "w", "s", "a", "d" to move', True, (255, 255, 255)) )
        text.append(textFont.render('forward, back, left, and right,', True, (255, 255, 255)) )
        text.append(textFont.render('and "q" / "e" for yaw', True, (255, 255, 255)) )
        for i in range(len(text)):
            self.screen.blit(text[i], (100, 200 + i*30))
        pygame.display.update()

        self.keys = [pygame.K_w, pygame.K_s,
                     pygame.K_q, pygame.K_e,
                     pygame.K_a, pygame.K_d,
                     pygame.K_r, pygame.K_y,
                     pygame.K_f, pygame.K_h,
                     pygame.K_t, pygame.K_g,
                     pygame.K_u, pygame.K_o,
                     pygame.K_j, pygame.K_l,
                     pygame.K_i, pygame.K_k,
                     pygame.K_u, pygame.K_j,
                     pygame.K_m, pygame.K_n,
                     pygame.K_o, pygame.K_p]

        self.pressed_key = [False] * len(self.keys)


        self.torsoCmd = PoseStamped()
        self.torsoCmd.pose.orientation.w = 1
        self.torsoCmd.pose.position.z = 0.15

        ####### publishers ##########
        self.velPub   = rospy.Publisher('/' + robot_name + "/vel_cmd", Twist, queue_size=10)
        self.torsoPub = rospy.Publisher('/' + robot_name + "/torso_pose_control", PoseStamped, queue_size=10)

        self.connectServosClient = rospy.ServiceProxy('/' + robot_name + '/connect_servos', connect_servos)
        self.setGaitClient       = rospy.ServiceProxy('/' + robot_name + '/set_gait', set_gait)



    """ Main loop for the node """
    def mainThread(self):
        for event in pygame.event.get():
            for i in range(len(self.keys)):
                if event.type == pygame.KEYDOWN:  # check for key presses
                    if event.key == self.keys[i]:
                        self.pressed_key[i] = True

                elif event.type == pygame.KEYUP:  # check for key presses
                    if event.key == self.keys[i]:
                        self.pressed_key[i] = False

        velCmd = Twist()
        velCmd.linear.x = 0
        velCmd.linear.y = 0
        velCmd.linear.z = 0
        velCmd.angular.z = 0

        velxy = 0.5  # m/s
        velz  = 0.05  # m/s
        velth = 2*np.pi # rad/s


        if self.pressed_key[0]: # w
            velCmd.linear.x = velxy

        if self.pressed_key[1]: #s
            velCmd.linear.x = -velxy
        if self.pressed_key[2]: #a
            velCmd.linear.y = velxy
        if self.pressed_key[3]: #d
            velCmd.linear.y = -velxy

        if self.pressed_key[4]: #q
            velCmd.angular.z = - velth
        if self.pressed_key[5]: #e
            velCmd.angular.z = velth


        vel_pos_body = 0.0001
        vel_ang_body = 0.001

        if self.pressed_key[6]: #r
            velCmd.angular.x += velth
        if self.pressed_key[7]: #y
            velCmd.angular.x += -velth
        if self.pressed_key[8]: #f
            velCmd.angular.y += velth
        if self.pressed_key[9]: #h
            velCmd.angular.y += -velth


        if self.pressed_key[10]: #t
            velCmd.angular.y += velxy
        if self.pressed_key[11]: #g
            velCmd.angular.y += -velxy

        if self.pressed_key[18]: #u
            velCmd.linear.z += velz
        if self.pressed_key[19]: #j
            velCmd.linear.z += - velz


        if self.pressed_key[20]:
            rospy.loginfo("connecting servos")

            req = connect_servosRequest()
            req.connect = True
            try:
                self.connectServosClient(req)
            except rospy.ServiceException as e:
                rospy.logwarn("Service call failed: %s" % e)


        if self.pressed_key[21]:
            rospy.loginfo("disconnecting servos")

            req = connect_servosRequest()
            req.connect = False
            try:
                self.connectServosClient(req)
            except rospy.ServiceException as e:
                rospy.logwarn("Service call failed: %s" % e)


        if self.pressed_key[22]:
            rospy.loginfo("setting gait")
            req = set_gaitRequest()
            req.gait_filename = "diagonal_fast"
            try:
                self.setGaitClient(req)
            except rospy.ServiceException as e:
                rospy.logwarn("Service call failed: %s" % e)

        if self.pressed_key[23]:
            rospy.loginfo("stopping gait")
            req = set_gaitRequest()
            req.gait_filename = "stop"
            try:
                self.setGaitClient(req)
            except rospy.ServiceException as e:
                rospy.logwarn("Service call failed: %s" % e)


        self.velPub.publish(velCmd)

        # print(velCmd.twist.linear.x, velCmd.twist.linear.y, velCmd.twist.angular.z)

    def debug(self, typ, msg):
        print(typ + ": " + msg + "\n")


if __name__ == '__main__':
    try:
        rospy.init_node('manual_control_node', anonymous=True)
        rate = rospy.Rate(10)    # 10 Hz
        robot_name = "catbot"
        if rospy.has_param('/robot_name'):
            robot_name = rospy.get_param("/robot_name")


        node = ManualControlNode(robot_name)

        while not rospy.is_shutdown():
            node.mainThread()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
