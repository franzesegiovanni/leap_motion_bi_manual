#################################################################################
# Copyright (C) 2012-2013 Leap Motion, Inc. All rights reserved.                #
# Leap Motion proprietary and confidential. Not for distribution.               #
# Use subject to the terms of the Leap Motion SDK Agreement available at        #
# https://developer.leapmotion.com/sdk_agreement, or another agreement          #
# between Leap Motion and you, your company or other organization.              #
#################################################################################

#################################################################################
# Altered LEAP example by Florian Lier, you need to have the LEAP SDK installed #
# for this to work properly ;)                                                  #
# This interface provides access to the LEAP MOTION hardware, you will need to  #
# have the official LEAP MOTION SDK installed in order to load the shared       #
# provided with the SDK.                                                        #
#################################################################################

""" For backwards compatibility with the old driver files
                Will be DELETED in the future               """

import sys
import time
# Set (append) your PYTHONPATH properly, or just fill in the location of your LEAP
# SDK folder, e.g., $HOME/LeapSDK/lib where the Leap.py lives and /LeapSDK/lib/x64 or
# x86 where the *.so files reside.

# Below, you can see the "dirty" version - NOT RECOMMENDED!

# sys.path.append("/home/YOUR_NAME/path/to/Leap_Developer/LeapSDK/lib")
# sys.path.append("/home/YOUR_NAME/path/to/Leap_Developer/Leap_Developer/LeapSDK/lib/x64")
import threading
import Leap
from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture

class LeapFinger():
    def __init__(self, finger=None):
        self.boneNames = ['metacarpal',
                          'proximal',
                          'intermediate',
                          'distal']
        for boneName in self.boneNames:
            setattr(self, boneName, [0.0, 0.0, 0.0])
        self.tip = [0.0, 0.0, 0.0]

        self.leapBoneNames = [Leap.Bone.TYPE_METACARPAL,
                              Leap.Bone.TYPE_PROXIMAL,
                              Leap.Bone.TYPE_INTERMEDIATE,
                              Leap.Bone.TYPE_DISTAL]

        if finger is not None:
            self.importFinger(finger)

    def importFinger(self, finger):
        for boneName in self.boneNames:
            # Get the base of each bone
            bone = finger.bone(getattr(Leap.Bone, 'TYPE_%s' % boneName.upper()))
            setattr(self, boneName, bone.prev_joint.to_float_array())
        # For the tip, get the end of the distal bone
        self.tip = finger.bone(Leap.Bone.TYPE_DISTAL).next_joint.to_float_array()



class LeapInterface(Leap.Listener):
    def on_init(self, controller):
        # These variables as probably not thread safe
        # TODO: Make thread safe ;)
        self.hand_selected=0
        self.hand           = [0,0,0]
        self.right_hand = False
        self.left_hand = False
        self.hand_direction = [0,0,0]
        self.hand_normal    = [0,0,0]
        self.hand_palm_pos  = [0,0,0]
        self.hand_pitch     = 0.0
        self.hand_yaw       = 0.0
        self.hand_roll      = 0.0
        self.fingerNames = ['thumb', 'index', 'middle', 'ring', 'pinky']
        for fingerName in self.fingerNames:
            setattr(self, fingerName, LeapFinger())
        print "Initialized Leap Motion Device"

    def on_connect(self, controller):
        print "Connected to Leap Motion Controller"

        # Enable gestures
        controller.enable_gesture(Leap.Gesture.TYPE_CIRCLE);
        controller.enable_gesture(Leap.Gesture.TYPE_KEY_TAP);
        controller.enable_gesture(Leap.Gesture.TYPE_SCREEN_TAP);
        controller.enable_gesture(Leap.Gesture.TYPE_SWIPE);

    def on_disconnect(self, controller):
        # Note: not dispatched when running in a debugger.
        print "Disconnected Leap Motion"

    def on_exit(self, controller):
        print "Exited Leap Motion Controller"

    def on_frame(self, controller):
        # Get the most recent frame and report some basic information
        frame = controller.frame()

        print "Frame id: %d, timestamp: %d, hands: %d, fingers: %d, tools: %d, gestures: %d" % (
              frame.id, frame.timestamp, len(frame.hands), len(frame.fingers), len(frame.tools), len(frame.gestures()))

        if not frame.hands[self.hand_selected]: #recently changed in API

            self.hand = frame.hands[self.hand_selected]

            # Check if the hand has any fingers
            fingers = self.hand.fingers
            if not fingers.is_empty:
                for fingerName in self.fingerNames:
                    #finger = fingers.finger_type(Leap.Finger.TYPE_THUMB)[0]
                    #self.thumb.importFinger(finger)
                    finger = fingers.finger_type(getattr(Leap.Finger, 'TYPE_%s' % fingerName.upper()))[0]
                    getattr(self, fingerName).importFinger(finger)

            # Get the hand's normal vector and direction
            normal = self.hand.palm_normal
            direction = self.hand.direction
            pos = self.hand.palm_position

            self.hand_direction[0] = direction.x
            self.hand_direction[1] = direction.y
            self.hand_direction[2] = direction.z
            self.hand_normal[0]    = normal.x
            self.hand_normal[1]    = normal.y
            self.hand_normal[2]    = normal.z
            self.hand_palm_pos[0]  = pos.x
            self.hand_palm_pos[1]  = pos.y
            self.hand_palm_pos[2]  = pos.z
            self.hand_pitch        = direction.pitch * Leap.RAD_TO_DEG
            self.hand_yaw          = normal.yaw * Leap.RAD_TO_DEG
            self.hand_roll         = direction.roll * Leap.RAD_TO_DEG

            # Calculate the hand's pitch, roll, and yaw angles
            print "Hand pitch: %f degrees, roll: %f degrees, yaw: %f degrees" % (self.hand_pitch, self.hand_roll, self.hand_yaw)

    def get_hand_direction(self):
        return self.hand_direction

    def get_hand_normal(self):
        return self.hand_normal

    def get_hand_palmpos(self):
        return self.hand_palm_pos

    def get_hand_yaw(self):
        return self.hand_yaw

    def get_hand_pitch(self):
        return self.hand_pitch

    def get_hand_roll(self):
        return self.hand_roll

    def get_finger_point(self, fingerName, fingerPointName):
        return getattr(getattr(self, get_finger_point), fingerPointName)


class Runner(threading.Thread):

    def __init__(self,arg=None):
        threading.Thread.__init__(self)
        self.arg=arg
        self.listener = LeapInterface()
        self.controller = Leap.Controller()
        self.controller.add_listener(self.listener)

    def __del__(self):
        self.controller.remove_listener(self.listener)

    def get_hand_direction(self):
        return self.listener.get_hand_direction()

    def get_hand_normal(self):
        return self.listener.get_hand_normal()

    def get_hand_palmpos(self):
        return self.listener.get_hand_palmpos()

    def get_hand_roll(self):
        return self.listener.get_hand_roll()

    def get_hand_pitch(self):
        return self.listener.get_hand_pitch()

    def get_hand_yaw(self):
        return self.listener.get_hand_yaw()

    def get_finger_point(self, fingerName, fingerPointName):
        return self.listener.get_finger_point(fingerName, fingerPointName)

    def run (self):
        while True:
            # Save some CPU time
            time.sleep(0.001)

