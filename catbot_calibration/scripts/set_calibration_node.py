#!/usr/bin/env python
import rospy
from settings import *
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import time
from catbot_msgs.srv import set_motor_calibration, set_motor_calibrationRequest, set_motor_calibrationResponse
from catbot_msgs.srv import connect_servos, connect_servosRequest, connect_servosResponse
import numpy as np

import rospkg
rospack = rospkg.RosPack()
from urdf_parser_py.urdf import URDF

class setCalibration:
    def __init__(self):
        self.debug("INFO", "set_calibration_node started.")
        urdf_path = rospack.get_path('catbot_description') + '/urdf/catbot.urdf'
        urdf_str = open(urdf_path, 'r').read()
        self.robot_urdf = URDF.from_xml_string(urdf_str)
        self.joints = ['hip1', 'hip2', 'knee']

        self.loadLimitAnglesFromURDF()

        ####### services ########
        self.setMotorCalClient    = rospy.ServiceProxy('catbot/set_motor_calibration', set_motor_calibration)
        self.connectServosClient  = rospy.ServiceProxy('catbot/connect_servos', connect_servos)

        self.setMotorCalClient.wait_for_service()
        self.connectServosClient.wait_for_service()
        time.sleep(1)




    def loadLimitAnglesFromURDF(self):
        self.limAngles = {}

        for joint in self.robot_urdf.joints:
            for joint_name in self.joints:
                if joint_name in joint.name:
                    # leg = joint.name.split('_')[1]
                    self.limAngles[joint_name] = [joint.limit.lower, joint.limit.upper]


    def getJointIndex(self, leg, joint):
        ix_offset = -1

        if leg == 'fl':
            ix_offset = 0
        if leg == 'fr':
            ix_offset = 3
        elif leg == 'bl':
            ix_offset = 6
        elif leg == 'br':
            ix_offset = 9

        if ix_offset == -1:
            print('Unknown leg: {:s}'.format(leg))
            return

        if joint == 'hip1':
            ix_offset = ix_offset + 0
        elif joint == 'hip2':
            ix_offset = ix_offset + 1
        elif joint == 'knee':
            ix_offset = ix_offset + 2
        else:
            print('Unknown joint: {:s}'.format(joint))
            return
        return ix_offset





    def setCalibrationLeg(self, leg, set_servo_cal = True):

        calibration_path = rospack.get_path('catbot_calibration') + '/calibration/'
        filename_servo  = calibration_path + FILENAME_SERVO + "_{:s}.cal".format(leg)

        cal_angles = np.zeros([3, 2])
        servo_pos = np.zeros([3, 2])
        ang_vals = np.zeros([3, 2])


        if set_servo_cal:
            try:
                rospy.loginfo("Loading " + filename_servo)
                f = open(filename_servo, 'r')
            except:
                rospy.logerr("Missing calibration file: " + filename_servo)
                return

            ix = 0
            for line in f.readlines():
                if not "#" in line:
                    if "calAngles" in line:
                        vals = line.split(':')[1]
                        cal_angles[ix][0] = float(vals.split(',')[0])
                        cal_angles[ix][1] = float(vals.split(',')[1])

                    elif "servoPositions" in line:
                        vals = line.split(':')[1]
                        servo_pos[ix][0] = float(vals.split(',')[0])
                        servo_pos[ix][1] = float(vals.split(',')[1])

                    elif "angleValues" in line:
                        vals = line.split(':')[1]
                        ang_vals[ix][0] = float(vals.split(',')[0])
                        ang_vals[ix][1] = float(vals.split(',')[1])
                        ix += 1





        i = 0
        for joint in self.joints:

            req = set_motor_calibrationRequest()

            req.setCalibration_servo  = set_servo_cal

            req.servo_index  = self.getJointIndex(leg, joint)


            if joint == 'hip1':
                if 'r' in leg:
                    req.min_calAngle = -np.radians(cal_angles[i][0])
                    req.max_calAngle = -np.radians(cal_angles[i][1])
                else:
                    req.min_calAngle = np.radians(cal_angles[i][0])
                    req.max_calAngle = np.radians(cal_angles[i][1])

            else:
                req.min_calAngle = np.radians(cal_angles[i][0])
                req.max_calAngle = np.radians(cal_angles[i][1])

            req.min_posServo = servo_pos[i][0]
            req.max_posServo = servo_pos[i][1]

            req.min_posAngle = ang_vals[i][0]
            req.max_posAngle = ang_vals[i][1]


            req.min_lim_angle = self.limAngles[joint][0]
            req.max_lim_angle = self.limAngles[joint][1]


            try:
                self.setMotorCalClient(req)
                rospy.loginfo("Calibration sent to driver")
            except rospy.ServiceException as e:
                rospy.logwarn("Service call failed: %s" % e)

            i += 1
            time.sleep(0.2)

        rospy.loginfo('Calibration applied for leg {:s}'.format(leg))




    """ Main loop for the node """
    def mainThread(self, set_servo_cal = True, set_torque_cal = True):


        cm_req = connect_servosRequest()
        cm_req.connect = False

        try:
            self.connectServosClient(cm_req)
        except:
            pass

        time.sleep(0.5)

        self.setCalibrationLeg('fl', set_servo_cal = set_servo_cal)
        self.setCalibrationLeg('fr', set_servo_cal = set_servo_cal)
        self.setCalibrationLeg('bl', set_servo_cal = set_servo_cal)
        self.setCalibrationLeg('br', set_servo_cal = set_servo_cal)


        time.sleep(1)
        cm_req.connect = True
        try:
            self.connectServosClient(cm_req)
        except:
            pass

    def debug(self, typ, msg):
        print(typ + ": " + msg + "\n")



if __name__ == '__main__':
    try:
        rospy.init_node('set_calibration_node', anonymous=True)
        rate = rospy.Rate(10)    # 10 Hz
        node = setCalibration()
        node.mainThread()

    except rospy.ROSInterruptException:
        pass
