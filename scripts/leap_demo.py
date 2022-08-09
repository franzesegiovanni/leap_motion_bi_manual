#!/usr/bin/env python

""" For backwards compatibility with the old driver files
                Will be DELETED in the future               """

__author__ = 'flier'

import rospy
import math
import numpy as np
import pickle
from leap_motion.msg import leap
from leap_motion.msg import leapros

# Native datatypes, I've heard this is bad practice, use the geometry messages instead.
# def callback(data):
#    rospy.loginfo(rospy.get_name() + ": Leap Raw Data %s" % data)

def cal_angle(point_a, point_b, point_c):
    a_x, b_x, c_x = point_a[0], point_b[0], point_c[0]  
    a_y, b_y, c_y = point_a[1], point_b[1], point_c[1]  
    a_z, b_z, c_z = point_a[2], point_b[2], point_c[2] 

    x1,y1,z1 = (a_x-b_x),(a_y-b_y),(a_z-b_z)
    x2,y2,z2 = (c_x-b_x),(c_y-b_y),(c_z-b_z)

    if all(i==0 for i in point_a) and all(i==0 for i in point_b) and all(i==0 for i in point_c):
        B = 0.0
    else:
        cos_b = (x1*x2 + y1*y2 + z1*z2) / (math.sqrt(x1**2 + y1**2 + z1**2) *(math.sqrt(x2**2 + y2**2 + z2**2))) 
        B = float(math.acos(cos_b))
    return B

def cal_angle_four(point_a, point_b, point_c, point_d):
    a_x, b_x, c_x, d_x = point_a[0], point_b[0], point_c[0], point_d[0]
    a_y, b_y, c_y, d_y = point_a[1], point_b[1], point_c[1], point_d[1]  
    a_z, b_z, c_z, d_z = point_a[2], point_b[2], point_c[2], point_d[2] 

    x1,y1,z1 = (a_x-b_x),(a_y-b_y),(a_z-b_z)
    x2,y2,z2 = (d_x-c_x),(d_y-c_y),(d_z-c_z)

    if all(i==0 for i in point_a) and all(i==0 for i in point_b) and all(i==0 for i in point_c) and all(i==0 for i in point_d):
        B = 0.0
    else:
        cos_b = (x1*x2 + y1*y2 + z1*z2) / (math.sqrt(x1**2 + y1**2 + z1**2) *(math.sqrt(x2**2 + y2**2 + z2**2))) 
        B = float(math.acos(cos_b))
    return B

# Callback of the ROS subscriber, just print the received data.
def callback_ros(data):
    thumb_tip = [data.thumb_tip.x, data.thumb_tip.y, data.thumb_tip.z]
    thumb_distal = [data.thumb_distal.x, data.thumb_distal.y, data.thumb_distal.z]
    thumb_intermediate = [data.thumb_intermediate.x, data.thumb_intermediate.y, data.thumb_intermediate.z]
    thumb_proximal = [data.thumb_proximal.x, data.thumb_proximal.y, data.thumb_proximal.z]

    # index_tip = [data.index_tip.x, data.index_tip.y, data.index_tip.z]
    index_distal = [data.index_distal.x, data.index_distal.y, data.index_distal.z]
    index_intermediate = [data.index_intermediate.x, data.index_intermediate.y, data.index_intermediate.z]
    index_proximal = [data.index_proximal.x, data.index_proximal.y, data.index_proximal.z]
    index_metacarpal = [data.index_metacarpal.x, data.index_metacarpal.y, data.index_metacarpal.z]

    # middle_tip = [data.middle_tip.x, data.middle_tip.y, data.middle_tip.z]
    middle_distal = [data.middle_distal.x, data.middle_distal.y, data.middle_distal.z]
    middle_intermediate = [data.middle_intermediate.x, data.middle_intermediate.y, data.middle_intermediate.z]
    middle_proximal = [data.middle_proximal.x, data.middle_proximal.y, data.middle_proximal.z]
    middle_metacarpal = [data.middle_metacarpal.x, data.middle_metacarpal.y, data.middle_metacarpal.z]

    # ring_tip = [data.ring_tip.x, data.ring_tip.y, data.ring_tip.z]
    ring_distal = [data.ring_distal.x, data.ring_distal.y, data.ring_distal.z]
    ring_intermediate = [data.ring_intermediate.x, data.ring_intermediate.y, data.ring_intermediate.z]
    ring_proximal = [data.ring_proximal.x, data.ring_proximal.y, data.ring_proximal.z]
    ring_metacarpal = [data.ring_metacarpal.x, data.ring_metacarpal.y, data.ring_metacarpal.z]

    # pinky_tip = [data.pinky_tip.x, data.pinky_tip.y, data.pinky_tip.z]
    pinky_distal = [data.pinky_distal.x, data.pinky_distal.y, data.pinky_distal.z]
    pinky_intermediate = [data.pinky_intermediate.x, data.pinky_intermediate.y, data.pinky_intermediate.z]
    pinky_proximal = [data.pinky_proximal.x, data.pinky_proximal.y, data.pinky_proximal.z]
    pinky_metacarpal = [data.pinky_metacarpal.x, data.pinky_metacarpal.y, data.pinky_metacarpal.z]

    palm_pos = [data.palmpos.x, data.palmpos.y, data.palmpos.z]
    palm_normal = [data.normal.x, data.normal.y, data.normal.z]
    palm_rot = [data.ypr.x, data.ypr.y, data.ypr.z]

    # Thumb (THJ)
    #THJ0
    range_THJ0 = [-1.571, 0]
    dof_pos_THJ0 = np.clip((cal_angle(thumb_tip,thumb_distal,thumb_intermediate) - math.pi)*1.5, range_THJ0[0], range_THJ0[1])
    # dof_pos_THJ0 = cal_angle(thumb_tip,thumb_distal,thumb_intermediate) - math.pi
    #THJ1
    range_THJ1 = [-0.524, 0.524]
    dof_pos_THJ1 = np.clip((cal_angle(thumb_distal,thumb_intermediate,thumb_proximal) - math.pi)*1.5, range_THJ1[0], range_THJ1[1])
    # dof_pos_THJ1 = cal_angle(thumb_distal,thumb_intermediate,thumb_proximal) - math.pi
    #THJ2
    range_THJ2 = [-0.209, 0.209]
    # dof_pos_THJ2 = np.clip((cal_angle(thumb_distal,thumb_intermediate,thumb_proximal) - math.pi), range_THJ1[0], range_THJ1[1])
    dof_pos_THJ2 = 0.0
    #THJ3
    range_THJ3 = [0, 1.222]
    # dof_pos_THJ3 = np.clip((cal_angle(thumb_distal,thumb_intermediate,thumb_proximal) - math.pi), range_THJ1[0], range_THJ1[1])
    dof_pos_THJ3 = 1.1
    #THJ4
    range_THJ4 = [-1.047, 1.047]
    dof_pos_THJ4 = np.clip((-cal_angle(thumb_intermediate,thumb_proximal,palm_normal) + 1.1)*1.5, range_THJ4[0], range_THJ4[1])
    # dof_pos_THJ4 = cal_angle(thumb_intermediate,palm_pos,palm_normal) - math.pi/2

    # Index (FFJ)
    #FFJ1
    range_FFJ1 = [0, 1.571]
    dof_pos_FFJ1 = np.clip(-(cal_angle(index_distal,index_intermediate,index_proximal) - math.pi)*1.5, range_FFJ1[0], range_FFJ1[1])
    # dof_pos_FFJ1 = -(cal_angle(index_distal,index_intermediate,index_proximal) - math.pi)
    #FFJ2
    range_FFJ2 = [0, 1.571]
    dof_pos_FFJ2 = np.clip(-(cal_angle(index_intermediate,index_proximal,index_metacarpal) - math.pi)*1.5, range_FFJ2[0], range_FFJ2[1])
    # dof_pos_FFJ2 = -(cal_angle(index_intermediate,index_proximal,index_metacarpal) - math.pi)
    #FFJ3
    range_FFJ3 = [-0.349, 0.349]
    dof_pos_FFJ3 = np.clip((cal_angle(index_intermediate,index_proximal,middle_proximal)-1.71)*1.5, range_FFJ3[0], range_FFJ3[1])
    # dof_pos_FFJ3 = cal_angle(index_intermediate,index_proximal,middle_proximal) - 1.71

    # Middle (MFJ)
    #MFJ1
    range_MFJ1 = [0, 1.571]
    dof_pos_MFJ1 = np.clip(-(cal_angle(middle_distal,middle_intermediate,middle_proximal) - math.pi)*1.5, range_MFJ1[0], range_MFJ1[1])
    # dof_pos_MFJ1 = -(cal_angle(middle_distal,middle_intermediate,middle_proximal) - math.pi)
    #MFJ2
    range_MFJ2 = [0, 1.571]
    dof_pos_MFJ2 = np.clip(-(cal_angle(middle_intermediate,middle_proximal,middle_metacarpal) - math.pi)*1.5, range_MFJ2[0], range_MFJ2[1])
    # dof_pos_MFJ2 = -(cal_angle(middle_intermediate,middle_proximal,middle_metacarpal) - math.pi)
    #MFJ3
    range_MFJ3 = [-0.349, 0.349]
    dof_pos_MFJ3 = np.clip((-cal_angle(middle_intermediate,middle_proximal,index_proximal) + 1.57), range_MFJ3[0], range_MFJ3[1])
    # dof_pos_MFJ3 = cal_angle(middle_intermediate,middle_proximal,index_proximal) - math.pi/2


    # Ring (RFJ)
    #RFJ1
    range_RFJ1 = [0, 1.571]
    dof_pos_RFJ1 = np.clip(-(cal_angle(ring_distal,ring_intermediate,ring_proximal) - math.pi)*1.5, range_RFJ1[0], range_RFJ1[1])
    # dof_pos_RFJ1 = -(cal_angle(ring_distal,ring_intermediate,ring_proximal) - math.pi)
    #RFJ2
    range_RFJ2 = [0, 1.571]
    dof_pos_RFJ2 = np.clip(-(cal_angle(ring_intermediate,ring_proximal,ring_metacarpal) - math.pi)*1.5, range_RFJ2[0], range_RFJ2[1])
    # dof_pos_RFJ2 = -(cal_angle(ring_intermediate,ring_proximal,ring_metacarpal) - math.pi)
    #RFJ3
    range_RFJ3 = [-0.349, 0.349]
    dof_pos_RFJ3 = np.clip((-cal_angle(ring_intermediate,ring_proximal,middle_proximal) + 1.3)*1.5, range_RFJ3[0], range_RFJ3[1])
    # dof_pos_RFJ3 = cal_angle(ring_intermediate,ring_proximal,middle_proximal) - 1.41

    # Pinky (LFJ)
    #LFJ1
    range_LFJ1 = [0, 1.571]
    dof_pos_LFJ1 = np.clip(-(cal_angle(pinky_distal,pinky_intermediate,pinky_proximal) - math.pi)*1.5, range_LFJ1[0], range_LFJ1[1])
    # dof_pos_LFJ1 = -(cal_angle(pinky_distal,pinky_intermediate,pinky_proximal) - math.pi)
    #LFJ2
    range_LFJ2 = [0, 1.571]
    dof_pos_LFJ2 = np.clip(-(cal_angle(pinky_intermediate,pinky_proximal,pinky_metacarpal) - math.pi)*1.5, range_LFJ2[0], range_LFJ2[1])
    # dof_pos_LFJ2 = -(cal_angle(pinky_intermediate,pinky_proximal,pinky_metacarpal) - math.pi)
    #LFJ3
    range_LFJ3 = [-0.349, 0.349]
    dof_pos_LFJ3 = np.clip((-cal_angle(pinky_intermediate,pinky_proximal,middle_proximal)+1.3)*1.5, range_LFJ3[0], range_LFJ3[1])
    # dof_pos_LFJ3 = cal_angle(pinky_intermediate,pinky_proximal,middle_proximal) - math.pi/2
    #LFJ4
    range_LFJ4 = [0, 0.785]
    dof_pos_LFJ4 = np.clip((-cal_angle(pinky_proximal,palm_pos,palm_normal)+1.0)*1.5, range_LFJ4[0], range_LFJ4[1])
    # dof_pos_LFJ4 = cal_angle_four(pinky_proximal,pinky_metacarpal,palm_pos,palm_normal) + 0.35
    # dof_pos_LFJ4 = 0.0

    # Hand
    #WRJ0
    range_WRJ0 = [-0.698, 0.489]
    # dof_pos_WRJ0 = np.clip((cal_angle(pinky_distal,pinky_intermediate,pinky_proximal) - math.pi), range_WRJ0[0], range_WRJ0[1])
    dof_pos_WRJ0 = np.clip(-palm_rot[1]*1.5, range_WRJ0[0], range_WRJ0[1])
    #WRJ1
    range_WRJ1 = [-0.489, 0.14]
    dof_pos_WRJ1 = np.clip((-palm_rot[0])*0.5, range_WRJ1[0], range_WRJ1[1])

    # file = open("/home/zheyu/angles.txt", "w")   
    # if dof_pos_THJ0 != 100:
    #     # rospy.loginfo(dof_pos_THJ0)
    #     file.write(str(dof_pos_THJ0))
    # file.close()

    angles = [dof_pos_WRJ1, dof_pos_WRJ0, dof_pos_FFJ3, dof_pos_FFJ2, dof_pos_FFJ1,
                dof_pos_MFJ3, dof_pos_MFJ2, dof_pos_MFJ1, dof_pos_RFJ3, dof_pos_RFJ2, dof_pos_RFJ1,
                dof_pos_LFJ4, dof_pos_LFJ3, dof_pos_LFJ2, dof_pos_LFJ1,
                dof_pos_THJ4, dof_pos_THJ3, dof_pos_THJ2, dof_pos_THJ1, dof_pos_THJ0]
    print(angles)
    filename_save = "/home/zheyu/angles_pickle.p"
    outfile = open(filename_save,'wb')
    pickle.dump(angles,outfile)
    outfile.close()
    # pickle.dump(angles,checkfile)

    # infile = open(filename_save,'rb')
    # b = pickle.load(infile)
    # print(b)

# Yes, a listener aka subscriber ;) obviously. Listens to: leapmotion/data
def listener():
    rospy.init_node('leap_sub', anonymous=True)
    # rospy.Subscriber("leapmotion/raw", leap, callback)
    rospy.Subscriber("leapmotion/data", leapros, callback_ros)
    rospy.spin()

# def myhook():
#   print("shutdown time!")
#   checkfile.close()

if __name__ == '__main__':
    # filename_check = "/home/zheyu/angles_check.p"
    # checkfile = open(filename_check,'wb')
    # rospy.on_shutdown(myhook)
    listener()
