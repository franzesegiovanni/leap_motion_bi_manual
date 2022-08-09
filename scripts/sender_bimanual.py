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
        hand_direction_, hand_left_direction_, hand_right_direction_   = li.get_hand_direction()
        hand_normal_, hand_left_normal_, hand_right_normal_            = li.get_hand_normal()
        hand_palm_pos_, hand_left_palm_pos_, hand_right_palm_pos_      = li.get_hand_palmpos()
        hand_pitch_, hand_left_pitch_, hand_right_pitch_               = li.get_hand_pitch()
        hand_roll_, hand_left_roll_, hand_right_roll_                  = li.get_hand_roll()
        hand_yaw_, hand_left_yaw_, hand_right_yaw_                     = li.get_hand_yaw()

        msg = leapros()
        msg.direction.x = hand_direction_[0]
        msg.direction.y = hand_direction_[1]
        msg.direction.z = hand_direction_[2]
        msg.normal.x = hand_normal_[0]
        msg.normal.y = hand_normal_[1]
        msg.normal.z = hand_normal_[2]
        msg.palmpos.x = hand_palm_pos_[0]
        msg.palmpos.y = hand_palm_pos_[1]
        msg.palmpos.z = hand_palm_pos_[2]
        msg.ypr.x = hand_yaw_
        msg.ypr.y = hand_pitch_
        msg.ypr.z = hand_roll_

        msg_left = leapros()
        msg_left.direction.x = hand_left_direction_[0]
        msg_left.direction.y = hand_left_direction_[1]
        msg_left.direction.z = hand_left_direction_[2]
        msg_left.normal.x = hand_left_normal_[0]
        msg_left.normal.y = hand_left_normal_[1]
        msg_left.normal.z = hand_left_normal_[2]
        msg_left.palmpos.x = hand_left_palm_pos_[0]
        msg_left.palmpos.y = hand_left_palm_pos_[1]
        msg_left.palmpos.z = hand_left_palm_pos_[2]
        msg_left.ypr.x = hand_left_yaw_
        msg_left.ypr.y = hand_left_pitch_
        msg_left.ypr.z = hand_left_roll_

        msg_right = leapros()
        msg_right.direction.x = hand_right_direction_[0]
        msg_right.direction.y = hand_right_direction_[1]
        msg_right.direction.z = hand_right_direction_[2]
        msg_right.normal.x = hand_right_normal_[0]
        msg_right.normal.y = hand_right_normal_[1]
        msg_right.normal.z = hand_right_normal_[2]
        msg_right.palmpos.x = hand_right_palm_pos_[0]
        msg_right.palmpos.y = hand_right_palm_pos_[1]
        msg_right.palmpos.z = hand_right_palm_pos_[2]
        msg_right.ypr.x = hand_right_yaw_
        msg_right.ypr.y = hand_right_pitch_
        msg_right.ypr.z = hand_right_roll_

        fingerPointNames = ['metacarpal', 'proximal',
                            'intermediate', 'distal', 'tip']
        fingerNames = ['thumb', 'index', 'middle', 'ring', 'pinky']
        # fingerNames_left = ['thumb_left', 'index_left', 'middle_left', 'ring_left', 'pinky_left']
        # fingerNames_right = ['thumb_right', 'index_right', 'middle_right', 'ring_right', 'pinky_right']        

        if li.listener.left_hand and li.listener.right_hand:
            for fingerName in fingerNames:
                for fingerPointName in fingerPointNames:
                    pos_left = li.get_finger_point((fingerName+'_left'), fingerPointName)
                    for iDim, dimName in enumerate(['x', 'y', 'z']):
                        setattr(getattr(msg_left, '%s_%s' % (fingerName, fingerPointName)),
                                dimName, pos_left[iDim])
            for fingerName in fingerNames:
                for fingerPointName in fingerPointNames:
                    pos_right = li.get_finger_point((fingerName+'_right'), fingerPointName)
                    for iDim, dimName in enumerate(['x', 'y', 'z']):
                        setattr(getattr(msg_right, '%s_%s' % (fingerName, fingerPointName)),
                                dimName, pos_right[iDim])
        else:
            for fingerName in fingerNames:
                for fingerPointName in fingerPointNames:
                    pos = li.get_finger_point(fingerName, fingerPointName)
                    for iDim, dimName in enumerate(['x', 'y', 'z']):
                        setattr(getattr(msg, '%s_%s' % (fingerName, fingerPointName)),
                                dimName, pos[iDim])
		

        # We don't publish native data types, see ROS best practices
        # pub.publish(hand_direction=hand_direction_,hand_normal = hand_normal_, hand_palm_pos = hand_palm_pos_, hand_pitch = hand_pitch_, hand_roll = hand_roll_, hand_yaw = hand_yaw_)
        pub_left_ros.publish(msg_left)
        pub_right_ros.publish(msg_right)
        pub_ros.publish(msg)
        rospy.sleep(rospy.get_param(PARAMNAME_FREQ_ENTIRE, FREQUENCY_ROSTOPIC_DEFAULT))


if __name__ == '__main__':
    try:
        sender()
    except rospy.ROSInterruptException:
        pass
