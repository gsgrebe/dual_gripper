#!/usr/bin/env python
#
# RightHand Robotics code for interfacing with a ReFlex SF hand
#

from os.path import join
import yaml

from dynamixel_msgs.msg import JointState
import rospy
import rospkg
from std_msgs.msg import Float64

from dual_gripper.msg import Pose
from motor import Motor


class ReflexSFHand(object):
    def __init__(self,whichHand):
        self.id = whichHand
        nodeName = 'dual_gripper_' + self.id
        rospy.init_node(nodeName)
        rospy.loginfo('Starting up the ReFlex SF ' + self.id +' hand')
        self.motors = {'/dual_gripper_'+self.id+'_f1': Motor('/dual_gripper_'+self.id+'_f1'),
                       '/dual_gripper_'+self.id+'_f2': Motor('/dual_gripper_'+self.id+'_f2'),
                       '/dual_gripper_'+self.id+'_f3': Motor('/dual_gripper_'+self.id+'_f3'),
                       '/dual_gripper_'+self.id+'_preshape': Motor('/dual_gripper_'+self.id+'_preshape')}
        rospy.Subscriber('/dual_gripper_'+self.id+'/command', Pose, self.receiveCmdCb)
        rospy.loginfo('ReFlex SF '+self.id+' hand has started, waiting for commands...')

    def receiveCmdCb(self, data):
        self.motors['/dual_gripper_'+self.id+'_f1'].setMotorPosition(data.f1)
        self.motors['/dual_gripper_'+self.id+'_f2'].setMotorPosition(data.f2)
        self.motors['/dual_gripper_'+self.id+'_f3'].setMotorPosition(data.f3)
        self.motors['/dual_gripper_'+self.id+'_preshape'].setMotorPosition(data.preshape)

    def printMotorPositions(self):
        print ""  # visually separates following print messages in the flow
        for motor in sorted(self.motors):
            print motor, " position: ", self.motors[motor].getCurrentPosition()

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
        dual_gripper_path = rospack.get_path("dual_gripper")
        yaml_path = "yaml"
        file_path = join(dual_gripper_path, yaml_path, filename)
        with open(file_path, "w") as outfile:
            outfile.write(yaml.dump(data))

    def writeCurrentPositionsToZero(self):
        keys = [('dual_gripper_'+self.id+'_f1'),
                ('dual_gripper_'+self.id+'_f2'),
                ('dual_gripper_'+self.id+'_f3'),
                ('dual_gripper_'+self.id+'_preshape')]
        data = {}
        for key in keys:
            data[key] = dict(
                zero_point = self.motors['/'+key].getRawCurrentPosition()
            )
        self.writeZeroPointDataToFile('dual_gripper_'+self.id+'_zero_points.yaml', data)

    def disableTorque(self):
        for motor in self.motors:
            self.motors[motor].disableTorque()

    def enableTorque(self):
        for motor in self.motors:
            self.motors[motor].enableTorque()


def main():
    args = rospy.myargv()
    if len(args) > 2:
        print "Too Many Arguments Specified"
    if len(args) == 2:
        name = args[1]
    else:
        name = args[0]

    if (name.find("/")!=-1 or name.find("\\")!=-1):
        print "Invalid name: " + name
        exit(1)

    hand = ReflexSFHand(name)
    rospy.on_shutdown(hand.disableTorque)
    rospy.spin()


if __name__ == '__main__':
    main()
