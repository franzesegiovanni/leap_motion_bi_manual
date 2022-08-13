#!/usr/bin/env python

""" For backwards compatibility with the old driver files
                Will be DELETED in the future               """

__author__ = 'flier'

import argparse

import rospy
import leap_interface
import leap_interface_bimanual
from leap_motion.msg import leap
from leap_motion.msg import leapros

import sys
sys.path.append("/usr/lib/Leap")
sys.path.append("/home/zheyu/LeapSDK/lib/x86")
sys.path.append("/home/zheyu/LeapSDK/lib")

FREQUENCY_ROSTOPIC_DEFAULT = 0.01
NODENAME = 'leap_pub'
PARAMNAME_FREQ = 'freq'
PARAMNAME_FREQ_ENTIRE = '/' + NODENAME + '/' + PARAMNAME_FREQ

def sender():
    '''
    This method publishes the data defined in leapros.msg to /leapmotion/data
    '''
    rospy.loginfo("Parameter set on server: PARAMNAME_FREQ={}".format(rospy.get_param(PARAMNAME_FREQ_ENTIRE, FREQUENCY_ROSTOPIC_DEFAULT)))

    li = leap_interface_bimanual.Runner()
    li.setDaemon(True)
    li.start()
    # pub     = rospy.Publisher('leapmotion/raw',leap)
    pub_ros   = rospy.Publisher('leapmotion/data',leapros, queue_size=2)
    pub_left_ros   = rospy.Publisher('leapmotion_left/data',leapros, queue_size=2)
    pub_right_ros   = rospy.Publisher('leapmotion_right/data',leapros, queue_size=2)
    rospy.init_node(NODENAME)

    while not rospy.is_shutdown():
        hand_direction_         = li.get_hand_direction()
        hand_normal_            = li.get_hand_normal()
        hand_palm_pos_          = li.get_hand_palmpos()
        hand_angle_             = li.get_hand_ang()

        msg_left = leapros()
        msg_left.direction.x = hand_direction_[0,0]
        msg_left.direction.y = hand_direction_[0,1]
        msg_left.direction.z = hand_direction_[0,2]
        msg_left.normal.x = hand_normal_[0,0]
        msg_left.normal.y = hand_normal_[0,1]
        msg_left.normal.z = hand_normal_[0,2]
        msg_left.palmpos.x = hand_palm_pos_[0,0]
        msg_left.palmpos.y = hand_palm_pos_[0,1]
        msg_left.palmpos.z = hand_palm_pos_[0,2]
        msg_left.ypr.x = hand_angle_[0,1]
        msg_left.ypr.y = hand_angle_[0,0]
        msg_left.ypr.z = hand_angle_[0,2]

        msg_right = leapros()
        msg_right.direction.x = hand_direction_[1,0]
        msg_right.direction.y = hand_direction_[1,1]
        msg_right.direction.z = hand_direction_[1,2]
        msg_right.normal.x = hand_normal_[1,0]
        msg_right.normal.y = hand_normal_[1,1]
        msg_right.normal.z = hand_normal_[1,2]
        msg_right.palmpos.x = hand_palm_pos_[1,0]
        msg_right.palmpos.y = hand_palm_pos_[1,1]
        msg_right.palmpos.z = hand_palm_pos_[1,2]
        msg_right.ypr.x = hand_angle_[1,1]
        msg_right.ypr.y = hand_angle_[1,0]
        msg_right.ypr.z = hand_angle_[1,2]

        fingerPointNames = ['metacarpal', 'proximal',
                            'intermediate', 'distal', 'tip']
        fingerNames = ['thumb', 'index', 'middle', 'ring', 'pinky']       

        for fingerName in fingerNames:
            for fingerPointName in fingerPointNames:
                pos_left = li.get_finger_point((fingerName+'_left'), fingerPointName)
                pos_right = li.get_finger_point((fingerName+'_right'), fingerPointName)
                for iDim, dimName in enumerate(['x', 'y', 'z']):
                    setattr(getattr(msg_left, '%s_%s' % (fingerName, fingerPointName)),
                            dimName, pos_left[iDim])
                    setattr(getattr(msg_right, '%s_%s' % (fingerName, fingerPointName)),
                            dimName, pos_right[iDim])

        # We don't publish native data types, see ROS best practices
        # pub.publish(hand_direction=hand_direction_,hand_normal = hand_normal_, hand_palm_pos = hand_palm_pos_, hand_pitch = hand_pitch_, hand_roll = hand_roll_, hand_yaw = hand_yaw_)
        pub_left_ros.publish(msg_left)
        pub_right_ros.publish(msg_right)
        pub_ros.publish(msg_left)
        rospy.sleep(rospy.get_param(PARAMNAME_FREQ_ENTIRE, FREQUENCY_ROSTOPIC_DEFAULT))


if __name__ == '__main__':
    try:
        sender()
    except rospy.ROSInterruptException:
        pass
