import numpy as np
import time
import rospkg
rospack = rospkg.RosPack()

from body import Body
from settings import *
import threading



class CalibrateServos():
    def __init__(self):
        self.body = Body(COM_PORT, None)
        self.running = True
        threading.Thread(target=self.runBodyThread).start()

    def runBodyThread(self):
        while self.running:
            self.body.mainLoop()
            time.sleep(0.1)


    def defineData(self):
        self.CAL_ANGLES = {}
        self.CAL_ANGLES['hip1'] = HIP1_CAL_ANGLES
        self.CAL_ANGLES['hip2'] = HIP2_CAL_ANGLES
        self.CAL_ANGLES['knee'] = KNEE_CAL_ANGLES

        self.TORQUE_CAL = {}
        self.TORQUE_CAL['hip1'] = HIP1_CAL_TORQUE
        self.TORQUE_CAL['hip2'] = HIP2_CAL_TORQUE
        self.TORQUE_CAL['knee'] = KNEE_CAL_TORQUE

        self.SERVO_POS   = {}
        self.ANGLES_VAL  = {}
        self.TORQUE_VAL  = {}

        for joint in ['hip1', 'hip2', 'knee']:
            self.SERVO_POS[joint]  = [0,0]
            self.ANGLES_VAL[joint] = [0,0]
            self.TORQUE_VAL[joint] = [0,0]



    def run(self):
        while True:
            print "#################### CALIBRATION ##################"
            print ""

            leg_choice = raw_input("Choose leg to calibrate {'fr','fl','br','bl'} or press Enter to quit: ")

            if leg_choice == '':
                print "############## CALIBRATION COMPLETE ###################"
                return

            leg = self.body.legs[leg_choice]
            self.defineData()



            print "############## CALIBRATING LEG " + leg.name + " ########################"
            raw_input("Leg {:s} servos will move to the zero pose. Press Enter to continue ...".format(leg.name))


            self.initialPose(leg)

            for J in ['hip1', 'hip2', 'knee']:
                joint = leg.joints[J]

                for i in range(len(self.CAL_ANGLES[J])):

                    while True:

                        print "******** Calibrating joint {:s} **********".format(joint.name)
                        print "Use w,s,e,d to move to the target angle {:.2f}".format(self.CAL_ANGLES[J][i])
                        print "Press 'y' when you reach the target.\n"

                        print "Joint:          {:s}"    .format(joint.name)
                        print "Angle:          {:.2f}"  .format(joint.getAngle())
                        print "Torque:         {:.2f}"  .format(joint.getTorque())
                        print "Servo position: {:.2f}\n".format(joint.getServoPos())

                        if self.moveJoint(joint) == 'y':
                            break
                        time.sleep(0.1)

                    self.SERVO_POS[J][i]   = joint.getServoPos()
                    self.ANGLES_VAL[J][i]  = joint.getAngle()

                    print "Recorded servo {:.2f} and angle {:.2f} for target {:.2f}".format(self.SERVO_POS[J][i],
                                                                                                             self.ANGLES_VAL[J][i],
                                                                                                             self.CAL_ANGLES[J][i])

                self.movingJointToZero(joint, self.SERVO_POS[J][np.argmin(self.CAL_ANGLES[J])])

                if J != 'hip1':
                    print "Calibrating force sensor:"
                    self.TORQUE_VAL[J][0] = joint.getTorque()
                    print "Force value at rest: ", self.TORQUE_VAL[J][0]
                    raw_input("Move the joint to its spring limit and press Enter:")

                    self.TORQUE_VAL[J][1] = joint.getTorque()
                    print "Force value at limit: ", self.TORQUE_VAL[J][1]


            self.running = False
            fn = self.saveCalFile(leg)
            print "Saved calibration to {:s}".format(fn)

            print "############## LEG {:s} CALIBRATION FINISHED *************".format(leg.name)


    def initialPose(self, leg):
        angles = {}
        angles['hip1'] = 100
        angles['hip2'] = 100
        angles['knee'] = 100
        leg.setAngles(angles)

    def saveCalFile(self, leg):
        calibration_path = rospack.get_path('catbot_drivers_com') + '/calibration/'

        filen = calibration_path + FILENAME + "_{:s}.cal".format(leg.name)

        f = open(filen, 'w')
        f.write("# Servo calibration data for CatBot \n\n")
        f.write("# leg {:s}\n".format(leg.name))

        for J in ['hip1', 'hip2', 'knee']:
            joint = leg.joints[J]
            f.write("# {:s}\n".format(joint.name))
            f.write("     calAngles:      {:.2f}, {:.2f}\n".format(self.CAL_ANGLES[J][0], self.CAL_ANGLES[J][1]))
            f.write("     servoPositions: {:.2f}, {:.2f}\n".format(self.SERVO_POS[J][0],  self.SERVO_POS[J][1]))
            f.write("     angleValues:    {:.2f}, {:.2f}\n".format(self.ANGLES_VAL[J][0], self.ANGLES_VAL[J][1]))
            f.write("     calTorques:     {:.2f}, {:.2f}\n".format(self.TORQUE_CAL[J][0], self.TORQUE_CAL[J][1]))
            f.write("     torqueValues:   {:.2f}, {:.2f}\n".format(self.TORQUE_VAL[J][0], self.TORQUE_VAL[J][1]))

        f.close()
        return filen

    def moveJoint(self, joint):
        cmd = raw_input("")
        pos = joint.getServoPos()
        max_pos = 200
        if cmd == 'w':
            pos += 1
        elif cmd == 's':
            pos -= 1
        elif cmd == 'e':
            pos += 10
        elif cmd == 'd':
            pos -= 10
        else:
            return cmd

        pos = np.clip(pos, 0, max_pos)
        joint.setAngle(pos)
        return cmd

    def movingJointToZero(self, joint, pos):
        joint.setAngle(100)


cal = CalibrateServos()
cal.run()

print "FINISHED!"






