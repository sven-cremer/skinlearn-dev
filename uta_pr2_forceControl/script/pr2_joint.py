#!/usr/bin/env python
#test client for joint_states_listener

import roslib
roslib.load_manifest('uta_pr2_forceControl')
import rospy
from sensor_msgs.msg import JointState
import time
import sys

#pretty-print list to string
def pplist(list):
    return ' '.join(['%2.3f'%x for x in list])

joint_names = ["r_shoulder_pan_joint",
           "r_shoulder_lift_joint",
            "r_upper_arm_roll_joint",
            "r_elbow_flex_joint",
            "r_forearm_roll_joint",
            "r_wrist_flex_joint",
            "r_wrist_roll_joint"]

def callback(data):
    resp = data(joint_names)
    for (ind, joint_name) in enumerate(joint_names):
        if(not resp.found[ind]):
            print "joint %s not found!"%joint_name

    print "position:", pplist(position), pplist(joint_names)
#    print "velocity:", pplist(velocity)
#    print "effort:", pplist(effort)


def listener():
    rospy.init_node('joint_listener', anonymous=True)
    rospy.Subscriber("joint_states", JointState, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
