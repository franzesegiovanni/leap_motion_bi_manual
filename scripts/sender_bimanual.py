#!/usr/bin/env python

""" For backwards compatibility with the old driver files
                Will be DELETED in the future               """

__author__ = 'flier'

import argparse

import rospy
import leap_interface_bimanual
from leap_motion.msg import leap
from leap_motion.msg import leapros

FREQUENCY_ROSTOPIC_DEFAULT = 0.01
NODENAME = 'leap_pub'
PARAMNAME_FREQ = 'freq'
PARAMNAME_FREQ_ENTIRE = '/' + NODENAME + '/' + PARAMNAME_FREQ

def sender():
    '''
    This method publishes the data defined in leapros.msg to /leapmotion/data
    '''
    rospy.loginfo("Parameter set on server: PARAMNAME_FREQ={}".format(rospy.get_param(PARAMNAME_FREQ_ENTIRE, FREQUENCY_ROSTOPIC_DEFAULT)))

    li_right = leap_interface_bimanual.Runner()
    li_right.setDaemon(True)
    li_right.start()

    li_left = leap_interface_bimanual.Runner()
    li_left.listener.hand_selected=1
    li_left.setDaemon(True)
    li_left.start()
    # pub     = rospy.Publisher('leapmotion/raw',leap)
    pub_right   = rospy.Publisher('leapmotion_right/data',leapros, queue_size=2)
    pub_left   =  rospy.Publisher('leapmotion_left/data',leapros, queue_size=2)
    rospy.init_node(NODENAME)
    fingerNames = ['thumb', 'index', 'middle', 'ring', 'pinky']
    fingerPointNames = ['metacarpal', 'proximal',
                            'intermediate', 'distal', 'tip']
    while not rospy.is_shutdown():

        hand_direction_right   = li_right.get_hand_direction()
        hand_normal_right      = li_right.get_hand_normal()
        hand_palm_pos_right    = li_right.get_hand_palmpos()
        hand_pitch_right       = li_right.get_hand_pitch()
        hand_roll_right        = li_right.get_hand_roll()
        hand_yaw_right         = li_right.get_hand_yaw()

        msg_right = leapros()
        msg_right.direction.x = hand_direction_right[0]
        msg_right.direction.y = hand_direction_right[1]
        msg_right.direction.z = hand_direction_right[2]
        msg_right.normal.x = hand_normal_right[0]
        msg_right.normal.y = hand_normal_right[1]
        msg_right.normal.z = hand_normal_right[2]
        msg_right.palmpos.x = hand_palm_pos_right[0]
        msg_right.palmpos.y = hand_palm_pos_right[1]
        msg_right.palmpos.z = hand_palm_pos_right[2]
        msg_right.ypr.x = hand_yaw_right
        msg_right.ypr.y = hand_pitch_right
        msg_right.ypr.z = hand_roll_right

        for fingerName in fingerNames:
            for fingerPointName in fingerPointNames:
                pos = li_right.get_finger_point(fingerName, fingerPointName)
                for iDim, dimName in enumerate(['x', 'y', 'z']):
                    setattr(getattr(msg_right, '%s_%s' % (fingerName, fingerPointName)),
                            dimName, pos[iDim])

        pub_right.publish(msg_right)

        hand_direction_left   = li_left.get_hand_direction()
        hand_normal_left    = li_left.get_hand_normal()
        hand_palm_pos_left    = li_left.get_hand_palmpos()
        hand_pitch_left       = li_left.get_hand_pitch()
        hand_roll_left       = li_left.get_hand_roll()
        hand_yaw_left         = li_left.get_hand_yaw()

        msg_left = leapros()
        msg_left.direction.x = hand_direction_left[0]
        msg_left.direction.y = hand_direction_left[1]
        msg_left.direction.z = hand_direction_left[2]
        msg_left.normal.x = hand_normal_left[0]
        msg_left.normal.y = hand_normal_left[1]
        msg_left.normal.z = hand_normal_left[2]
        msg_left.palmpos.x = hand_palm_pos_left[0]
        msg_left.palmpos.y = hand_palm_pos_left[1]
        msg_left.palmpos.z = hand_palm_posleft_[2]
        msg_left.ypr.x = hand_yaw_left
        msg_left.ypr.y = hand_pitch_left
        msg_left.ypr.z = hand_roll_left

        for fingerName in fingerNames:
            for fingerPointName in fingerPointNames:
                pos = li_left.get_finger_point(fingerName, fingerPointName)
                for iDim, dimName in enumerate(['x', 'y', 'z']):
                    setattr(getattr(msg_left, '%s_%s' % (fingerName, fingerPointName)),
                            dimName, pos[iDim])

        pub_left.publish(msg_left)


        rospy.sleep(rospy.get_param(PARAMNAME_FREQ_ENTIRE, FREQUENCY_ROSTOPIC_DEFAULT))



if __name__ == '__main__':
    try:
        sender()
    except rospy.ROSInterruptException:
        pass
