#!/usr/bin/env python
import rospy
from settings import *
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import time
from catbot_msgs.srv import set_motor_calibration, set_motor_calibrationRequest, set_motor_calibrationResponse

import rospkg
rospack = rospkg.RosPack()

class ServoCalibration:
    def __init__(self):


        self.debug("INFO", "servo_calibration_node started.")
        ####### publishers ##########
        self.jointControlPub = rospy.Publisher("/catbot/joints_control", JointState, queue_size=10)

        ####### services ########
        self.setMotorCalClient  = rospy.ServiceProxy('catbot/set_motor_calibration', set_motor_calibration)
        rospy.loginfo('Waiting for calibration service...')
        self.setMotorCalClient.wait_for_service()
        self.uncalibrateMotors()
        rospy.loginfo('Calibration service ready.')



        self.joints_control = JointState()

        self.joints_control.position = [0] * 12
        self.joints_control.velocity = [0] * 12
        self.joints_control.effort = [-1] * 12
        self.initial_joint_angles = []


        for leg in ['fl', 'fr', 'bl', 'br']:
            h, servo_pos = self.usePrevCalibration(leg)


            if h:
                leg_pos = [servo_pos[0][1], servo_pos[1][0], servo_pos[2][0]]

            else:
                leg_pos = [1500, 1500, 1500]

            self.initial_joint_angles += leg_pos
            self.setLegPose(leg, leg_pos)


        self.joints_control.position = self.initial_joint_angles
        self.joints_control.velocity = [0] * 12
        self.joints_control.effort = [-1] * 12

    """ ROS callbacks """

    def usePrevCalibration(self, leg):
        calibration_path = rospack.get_path('catbot_calibration') + '/calibration/'
        filename = calibration_path + FILENAME_SERVO + "_{:s}.cal".format(leg)

        try:
            rospy.loginfo("Loading " + filename)
            f = open(filename, 'r')
        except:
            rospy.logerr("Calibration file not found: " + filename)
            return False, None

        ix = 0

        cal_angles = np.zeros([3, 2])
        servo_pos = np.zeros([3, 2])
        ang_vals = np.zeros([3, 2])


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

        return True, servo_pos

    def uncalibrateMotors(self):
        '''
        offset_ix = 0
        if leg == 'fr':
            offset_ix = 3
        elif leg == 'bl':
            offset_ix = 6
        elif leg == 'br':
            offset_ix = 9
        '''
        for i in range(12):
            req = set_motor_calibrationRequest()
            req.servo_index = i
            req.setCalibration_servo = False

            try:
                self.setMotorCalClient(req)
            except rospy.ServiceException as e:
                rospy.logwarn("Service call failed: %s" % e)
            time.sleep(0.1)





    def defineData(self):
        self.CAL_ANGLES = {}
        self.CAL_ANGLES['hip1'] = HIP1_CAL_ANGLES
        self.CAL_ANGLES['hip2'] = HIP2_CAL_ANGLES
        self.CAL_ANGLES['knee'] = KNEE_CAL_ANGLES

        self.SERVO_POS = {}
        self.ANGLES_VAL = {}

        for joint in ['hip1', 'hip2', 'knee']:
            self.SERVO_POS[joint] = [0, 0]
            self.ANGLES_VAL[joint] = [0, 0]




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

    def setJointAngle(self, leg, joint, angle):
        self.joints_control.position[self.getJointIndex(leg, joint)] = angle
        self.jointControlPub.publish(self.joints_control)


    def setLegPose(self, leg, joint_angles):
        self.setJointAngle(leg, "hip1", joint_angles[0])
        self.setJointAngle(leg, "hip2", joint_angles[1])
        self.setJointAngle(leg, "knee", joint_angles[2])



    def moveJoint(self, leg, joint):
        cmd = input("")


        if cmd == 'w':
            self.servo_pos += 10
        elif cmd == 's':
            self.servo_pos -= 10
        elif cmd == 'e':
            self.servo_pos += 50
        elif cmd == 'd':
            self.servo_pos -= 50
        else:
            return cmd

        # self.servo_pos = np.clip(self.servo_pos, 0, 180)
        self.setJointAngle(leg, joint, self.servo_pos)
        return cmd



    """ Main loop for the node """
    def mainThread(self):
        print("#################### CALIBRATION ##################")
        print("")

        leg_choice = input("Choose leg to calibrate {'fr','fl','br','bl'} or press Enter to quit: ")

        if leg_choice not in ['fl', 'fr', 'bl', 'br']:
            print("############## CALIBRATION COMPLETE ###################")
            return

        self.defineData()

        print("############## CALIBRATING LEG " + leg_choice + " ########################")
        input("Leg {:s} servos will move to the zero pose. Press Enter to continue ...".format(leg_choice))


        for joint in ['hip1', 'hip2', 'knee']:
            self.servo_pos = self.initial_joint_angles[self.getJointIndex(leg_choice, joint)]

            for i in range(len(self.CAL_ANGLES[joint])):

                while True:

                    print("******** Calibrating joint {:s} **********".format(joint))
                    print("Use w,s,e,d to move to target {:.2f}".format(self.CAL_ANGLES[joint][i]))
                    print("Press 'y' when the target is reached.\n")

                    print("Joint:          {:s}".format(joint))
                    print("Angle:          {:.2f}".format( self.joints_control.position[self.getJointIndex(leg_choice, joint)] ))
                    print("Torque:         {:.2f}".format( self.joints_control.effort[self.getJointIndex(leg_choice, joint)] ))
                    print("Servo position: {:.2f}\n".format(self.servo_pos))

                    if self.moveJoint(leg_choice, joint) == 'y':
                        break
                    time.sleep(0.1)

                self.SERVO_POS[joint][i] = self.servo_pos
                self.ANGLES_VAL[joint][i] =  self.joints_control.position[self.getJointIndex(leg_choice, joint)]

                print("Recorded servo {:.2f} and angle {:.2f} for target {:.2f}".format(
                    self.SERVO_POS[joint][i],
                    self.ANGLES_VAL[joint][i],
                    self.CAL_ANGLES[joint][i]))


            self.setJointAngle(  leg_choice, joint,  self.SERVO_POS[joint][np.argmin(self.CAL_ANGLES[joint])]    )

            '''
            if joint != 'hip1':
                print("Calibrate Force sensor:")
                self.TORQUE_VAL[joint][0] = self.joints_control.effort[self.getJointIndex(leg_choice, joint)]
                print("Force pot value at rest: ", self.TORQUE_VAL[joint][0])
                input("Force the joint to its limit position of the spring and press enter:")

                self.TORQUE_VAL[joint][1] = self.joints_control.effort[self.getJointIndex(leg_choice, joint)]
                print("Force pot value at limit: ", self.TORQUE_VAL[joint][1])
            '''

        self.running = False
        fn = self.saveCalFile(leg_choice)
        print("Saved calibration to {:s}".format(fn))

        print("############## LEG {:s} CALIBRATION FINISHED *************".format(leg_choice))



    def saveCalFile(self, leg):
        calibration_path = rospack.get_path('catbot_calibration') + '/calibration/'

        filen = calibration_path + FILENAME_SERVO + "_{:s}.cal".format(leg)

        f = open(filen, 'w')
        f.write("# Servo calibration data for CatBot \n\n")
        f.write("# leg {:s}\n".format(leg))

        for joint in ['hip1', 'hip2', 'knee']:
            f.write("# {:s}\n".format(joint))
            f.write("     calAngles:      {:.2f}, {:.2f}\n".format(self.CAL_ANGLES[joint][0], self.CAL_ANGLES[joint][1]))
            f.write("     servoPositions: {:.2f}, {:.2f}\n".format(self.SERVO_POS[joint][0],  self.SERVO_POS[joint][1]))
            f.write("     angleValues:    {:.2f}, {:.2f}\n".format(self.ANGLES_VAL[joint][0], self.ANGLES_VAL[joint][1]))


        f.close()
        return filen


    def debug(self, typ, msg):
        print(typ + ": " + msg + "\n")


if __name__ == '__main__':
    try:
        rospy.init_node('catbot_calibration_node', anonymous=True)
        rate = rospy.Rate(10)    # 10 Hz
        node = ServoCalibration()

        while not rospy.is_shutdown():
            node.mainThread()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
