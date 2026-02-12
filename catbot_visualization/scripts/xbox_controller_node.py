#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from evdev import InputDevice, categorize, ecodes
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped
import numpy as np
from tf import transformations
from settings import *
from select import select
from catbot_msgs.srv import set_gait, set_gaitRequest, set_gaitResponse
from catbot_msgs.srv import connect_servos, connect_servosRequest, connect_servosResponse
import time

class Axis():
    def __init__(self, min_val, max_val, rest_val, out_min = -1, out_max = 1, threshold = 0):
        self.min_val = min_val
        self.max_val = max_val
        self.rest_val = rest_val
        self.out_min = out_min
        self.out_max = out_max
        self.threshold = threshold
        
    def mapValue(self, val):
        in_min = self.min_val - self.rest_val
        in_max = self.max_val - self.rest_val
        mapped_val = (val - in_min) * float(self.out_max - self.out_min) / (in_max - in_min) + self.out_min;
        
        
        if abs(mapped_val) < self.threshold:
            return 0
        return mapped_val
        
        
    def getValue(self, event):
        val = event.value
        val = val - self.rest_val
        val = self.mapValue(val)
        if np.abs(val) < 0.1:
            val = 0
        return val
    
class Button():
    def __init__(self):
            self.state = 0
        
    def getPressed(self, event):
        if event.value == 1:
            self.state = 1
            return True
        else:
            self.state = 0
            return False
        
        
class DoubleButton():
    def __init__(self, out_min = -1, out_max = 1):
        self.state = 0
        self.out_min = out_min
        self.out_max = out_max
        
    def getPressed(self, event):
        if event.value == 1:
            self.state = self.out_max
        elif event.value == -1:
            self.state = self.out_min
        else:
            self.state = 0
        
    def getState(self):
        return self.state


class FilterSmooth():
    def __init__(self, factor = 0):
        self.factor = factor
        self.input  = 0
        self.dxdt   = 0
        self.x      = 0 

    def set(self, val):
        self.input = val
        
    def update(self):
        self.dxdt = self.factor*(-self.x + self.input)
        self.x = self.x + self.dxdt
        
        
    def get(self):
        return self.x


class ManualControlNode:
    def __init__(self, robot_name, bluetooth_port = '/dev/input/event12'):

        self.debug("INFO", "Gamepad controller node started.")
        connected = False
        
        while not connected:
            try:
                self.controller = InputDevice(bluetooth_port)
                connected = True
            except:
                rospy.logerr('Gamepad not detected')
                time.sleep(2)
                pass
        rospy.logerr('Gamepad connected')

        self.forwardAxis = self._make_axis(CODE_FORWARD_AXIS, MIN_FORWARD_VEL, MAX_FORWARD_VEL)
        self.thAxis      = self._make_axis(CODE_TH_AXIS, MIN_TH_VEL, MAX_TH_VEL)
        self.rollAxis    = self._make_axis(CODE_ROLL_AXIS, MIN_ROLL_VEL, MAX_ROLL_VEL, threshold = 0.5)

        self.sideLeftAxis  = self._make_axis(CODE_SIDE_LEFT_AXIS, MIN_SIDE_LEFT_VEL, MAX_SIDE_LEFT_VEL)
        self.sideRightAxis = self._make_axis(CODE_SIDE_RIGHT_AXIS, MIN_SIDE_RIGHT_VEL, MAX_SIDE_RIGHT_VEL)
        
        self.zAxis       = DoubleButton(MIN_Z_VEL, MAX_Z_VEL)
        self.pitchAxis   = DoubleButton(PITCH_AXIS_MIN_VALUE, PITCH_AXIS_MAX_VALUE)

            
        self.connectServoButton    = Button()
        self.disconnectServoButton = Button()
        
        
        self.setStop         = Button()
        self.setDiagonal     = Button()
        self.setDiagonalFast = Button()
        self.setTrot         = Button()
        self.setInplace      = Button()

        self.forward_vel    = FilterSmooth(0.2)
        self.th_vel         = FilterSmooth(0.2)
        self.roll_vel       = FilterSmooth(0.2)
        self.pitch_vel      = FilterSmooth(0.2)
        self.side_left_vel  = FilterSmooth(0.2)
        self.side_right_vel = FilterSmooth(0.2)
        self.z_vel          = FilterSmooth(0.2)

        
        ####### publishers ##########
        self.velPub               = rospy.Publisher("/" + robot_name + "/vel_cmd", Twist, queue_size=10)
        
        ####### clients ############
        self.connectServosClient  = rospy.ServiceProxy('/' + robot_name + '/connect_servos', connect_servos)
        self.setGaitClient        = rospy.ServiceProxy('/' + robot_name + '/set_gait', set_gait)

    def _absinfo_or_default(self, code):
        try:
            return self.controller.absinfo(code)
        except Exception:
            return None

    def _make_axis(self, code, out_min, out_max, threshold = 0.0):
        info = self._absinfo_or_default(code)
        if info is None:
            # fallback range for unknown devices
            return Axis(-32768, 32767, 0, out_min, out_max, threshold = threshold)
        return Axis(info.min, info.max, info.value, out_min, out_max, threshold = threshold)


    def setGait(self, gait_filename):
        req = set_gaitRequest()
        req.gait_filename = gait_filename
        try:
            self.setGaitClient(req)
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s" % e)




    """ Main loop for the node """
    def mainThread(self):
        # select([self.controller], [], [])
        
        try:
            for event in self.controller.read():
                # print(event.code)
                if event.type == ecodes.EV_KEY:
                    tmp_var = 0
                    # print(event, event.value)
    
                if event.code == CODE_FORWARD_AXIS:
                    self.forward_vel.set(  self.forwardAxis.getValue(event)  )
                    
                if event.code == CODE_TH_AXIS: 
                    # print(event.value)
                    self.th_vel.set(  self.thAxis.getValue(event)  )

                if event.code == CODE_SIDE_LEFT_AXIS:
                    self.side_left_vel.set(  self.sideLeftAxis.getValue(event)  )
                    
                if event.code == CODE_SIDE_RIGHT_AXIS:
                    self.side_right_vel.set( self.sideRightAxis.getValue(event) )
                    
                if event.code == CODE_Z_AXIS:
                    self.zAxis.getPressed(event)
                    
                if event.code == CODE_ROLL_AXIS:
                    self.roll_vel.set(  self.rollAxis.getValue(event)  )
                    
                if event.code == CODE_PITCH_AXIS:
                    self.pitchAxis.getPressed(event)
                    self.pitch_vel.set( MAX_PITCH_VEL * self.pitchAxis.getState() )
                    
                    
                    
                if event.code == CODE_CONNECT_SERVOS_BUTTON:
                    if self.connectServoButton.getPressed(event):
                        rospy.loginfo("connecting servos")
    
                        req = connect_servosRequest()
                        req.connect = True
                        try:
                            self.connectServosClient(req)
                        except rospy.ServiceException as e:
                            rospy.logwarn("Service call failed: %s" % e)
                        
                if event.code == CODE_DISCONNECT_SERVOS_BUTTON:
                    if self.disconnectServoButton.getPressed(event):
                        rospy.loginfo("disconnecting servos")
                        req = connect_servosRequest()
                        req.connect = False
                        try:
                            self.connectServosClient(req)
                        except rospy.ServiceException as e:
                            rospy.logwarn("Service call failed: %s" % e)
                        
                
                
                if event.code == CODE_STOP_SERVOS_BUTTON:
                    if self.setStop.getPressed(event):
                        self.setGait("stop")
                
                
                
                if event.code == CODE_DIAGONAL_SERVOS_BUTTON:
                    if self.setDiagonal.getPressed(event):
                        self.setGait("diagonal")
                        
                        
                if event.code == CODE_DIAGONAL_FAST_SERVOS_BUTTON:
                    if self.setDiagonalFast.getPressed(event):
                        self.setGait("diagonal_fast")
                        
                        
                if event.code == CODE_TROT_SERVOS_BUTTON:
                    if self.setTrot.getPressed(event):
                        self.setGait("trot_slow")
                        
                if event.code == CODE_INPLACE_SERVOS_BUTTON:
                    if self.setInplace.getPressed(event):
                        self.setGait("inplace")
                        
        except:
             pass
                                        
                
        self.forward_vel.update()
        self.th_vel.update()
        self.side_left_vel.update()
        self.side_right_vel.update()
        self.z_vel.update()
        self.pitch_vel.update()
        self.roll_vel.update()


                
        side_velocity = self.side_left_vel.get() + self.side_right_vel.get()
        # print(self.forward_vel, self.th_vel, side_velocity, self.z_vel)
            
        self.z_vel.set( self.zAxis.getState() )
            
        velCmd = Twist()
        velCmd.linear.x = self.forward_vel.get()
        velCmd.linear.y = side_velocity
        velCmd.linear.z = self.z_vel.get()
        velCmd.angular.z = self.th_vel.get()
        velCmd.angular.y = self.roll_vel.get()
        velCmd.angular.x = self.pitch_vel.get()

        self.velPub.publish(velCmd)
        
            
    def debug(self, typ, msg):
        print(typ + ": " + msg + "\n")


if __name__ == '__main__':
    try:
        rospy.init_node('gamepad_controller_node', anonymous=True)
        rate = rospy.Rate(10)    # 10 Hz

        robot_name = "catbot"
        if rospy.has_param('/robot_name'):
            robot_name = rospy.get_param("/robot_name")

        controller_port = rospy.get_param('~controller_port', "/dev/input/event12")

        node = ManualControlNode(robot_name, bluetooth_port = controller_port)

        while not rospy.is_shutdown():
            node.mainThread()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
        
        
