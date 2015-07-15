#!/usr/bin/env python
#
# RightHand Robotics code for interfacing with a ReFlex SF hand
#

from os.path import join
import sys
import yaml

from dynamixel_msgs.msg import JointState
import rospy
import rospkg
from std_msgs.msg import Float64

from gripper.msg import Pose
from motor import Motor


class ReflexSFHand(object):
    def __init__(self,name):
        self.name = name
        nodeName = 'gripper_' + self.name
                
        rospy.init_node(nodeName)
        rospy.loginfo('Starting up the ReFlex SF Hand \'%s\''%self.name)

        self.fingers = ['f1', 'f2', 'f3', 'preshape']
        
        motorNS = ['/%s_%s'%(nodeName,finger) for finger in self._fingers]
        self.motors = { motor: Motor(motor) for motor in motorNS}
        
        rospy.Subscriber('/%s/command'%(nodeName), Pose, self.receiveCmdCb)
        rospy.loginfo('ReFlex SF Hand \'%s\' has started, waiting for commands...'%self.name)

    def receiveCmdCb(self, data):
        self.motors['/gripper_'+self.name+'_f1'].setMotorPosition(data.f1)
        self.motors['/gripper_'+self.name+'_f2'].setMotorPosition(data.f2)
        self.motors['/gripper_'+self.name+'_f3'].setMotorPosition(data.f3)
        self.motors['/gripper_'+self.name+'_preshape'].setMotorPosition(data.preshape)

    def printMotorPositions(self):
        print ""  # visually separates following print messages in the flow
        for motor in sorted(self.motors):
            print motor, " position: ", self.motors[motor].getCurrentPosition()

    def getMotorPositions(self):
        return [self.motors[motor].getCurrentPosition() for motor in sorted(self.motors)]

    def calibrate(self):
        for motor in sorted(self.motors):
            rospy.loginfo("Calibrating motor " + motor)
            command = raw_input("Type 't' to tighten motor, 'l' to loosen \
motor, or 'q' to indicate that the zero point has been reached\n")
            while not command.lower() == 'q':
                if command.lower() == 't':
                    print "Tightening motor " + motor
                    self.motors[motor].tighten()
                elif command.lower() == 'l':
                    print "Loosening motor " + motor
                    self.motors[motor].loosen()
                else:
                    print "Didn't recognize that command, use 't', 'l', or 'q'"
                command = raw_input("Tighten: 't'\tLoosen: 'l'\tDone: 'q'\n")
            rospy.loginfo("Saving current position for %s as the zero point",
                          motor)
            self.motors[motor].setMotorZeroPoint()
        print "Calibration complete, writing data to file"
        self.writeCurrentPositionsToZero()

    def writeZeroPointDataToFile(self, filename, data):
        rospack = rospkg.RosPack()
        gripper_path = rospack.get_path("gripper")
        yaml_path = "yaml"
        file_path = join(gripper_path, yaml_path, filename)
        with open(file_path, "w") as outfile:
            outfile.write(yaml.dump(data))

    def writeCurrentPositionsToZero(self):
        keys = [('gripper_'+self.name+'_f1'),
                ('gripper_'+self.name+'_f2'),
                ('gripper_'+self.name+'_f3'),
                ('gripper_'+self.name+'_preshape')]
        data = { key : dict(
            zero_point = self.motors['/'+key].getRawCurrentPosition() )
            for key in keys}
        
        self.writeZeroPointDataToFile('gripper_'+self.name+'_zero_points.yaml', data)

    def disableTorque(self):
        for motor in self.motors:
            self.motors[motor].disableTorque()

    def enableTorque(self):
        for motor in self.motors:
            self.motors[motor].enableTorque()


def main(argv):
    name = argv[0]

    if (name.find("/")!=-1 or name.find("\\")!=-1):
        print "Invalid name: " + name
        exit(1)

    hand = ReflexSFHand(name)
    rospy.on_shutdown(hand.disableTorque)
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv[1:])
