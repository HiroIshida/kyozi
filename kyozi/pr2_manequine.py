#!/usr/bin/env python
import rospkg
import argparse
import os
import sys
import time
import skrobot
import rospy
import threading
from pr2_mechanism_msgs.srv import SwitchController

from kyozi.utils import Config, construct_config

def switch_controller(start=True):
    controller_name = ["l_arm_controller"]
    loose_controller_name = ["l_arm_controller_loose"]
    sp = rospy.ServiceProxy('/pr2_controller_manager/switch_controller', SwitchController)
    if start:
        resp = sp(start_controllers=loose_controller_name, stop_controllers=controller_name)
    else:
        resp = sp(start_controllers=controller_name, stop_controllers=loose_controller_name)
    print('controller service response: {}'.format(resp))
    return resp

def switch_controller_whole(start=True):
    controller_name = ["l_arm_controller", "r_arm_controller"]
    loose_controller_name = ["l_arm_controller_loose", "r_arm_controller_loose"]
    sp = rospy.ServiceProxy('/pr2_controller_manager/switch_controller', SwitchController)
    if start:
        resp = sp(start_controllers=loose_controller_name, stop_controllers=controller_name)
    else:
        resp = sp(start_controllers=controller_name, stop_controllers=loose_controller_name)
    print('controller service response: {}'.format(resp))
    return resp

def start_mannequin(is_whole):
    if is_whole:
        switch_controller_whole(start=True)
    else:
        switch_controller(start=True)

def stop_mannequin(is_whole):
    if is_whole:
        switch_controller_whole(start=False)
    else:
        switch_controller(start=False)

class Mannequin(object):
    def __init__(self, config, is_whole=True):
        robot = skrobot.models.PR2()
        self.robot = robot
        self.ri = skrobot.interfaces.ros.PR2ROSRobotInterface(robot)
        self.config = config

        time.sleep(3)
        lnames = [
                "l_shoulder_pan_joint",
                "l_shoulder_lift_joint",
                "l_upper_arm_roll_joint",
                "l_elbow_flex_joint", 
                "l_forearm_roll_joint",
                "l_wrist_flex_joint",
                "l_wrist_roll_joint"
                ]
        rnames = [
                "r_shoulder_pan_joint",
                "r_shoulder_lift_joint",
                "r_upper_arm_roll_joint",
                "r_elbow_flex_joint", 
                "r_forearm_roll_joint",
                "r_wrist_flex_joint",
                "r_wrist_roll_joint"
                ]
        self.ljoints = [robot.__dict__[n] for n in lnames]
        self.rjoints = [robot.__dict__[n] for n in rnames]
        self.is_whole = is_whole

        self.is_thread_active = False
        self.thread = None

        self.reset_mannequin()

    def reset_mannequin(self):
        stop_mannequin(False)
        self.is_thread_active = False
        if self.thread:
            self.thread.join()
        for jn, angle in zip(self.config.init_joint_names, self.config.init_joint_angles):
            self.robot.__dict__[jn].joint_angle(angle)
        self.ri.angle_vector(self.robot.angle_vector(), time=2.0, time_scale=1.0)
        print("resetting pose...")
        time.sleep(3.5)

    def mirror(self, time):
        if self.is_whole:
            return

        self.robot.angle_vector(self.ri.angle_vector())
        langles = [j.joint_angle() for j in self.ljoints]
        for j, a in zip(self.rjoints, langles):
            reverse_joints = ["r_upper_arm_roll_joint", "r_forearm_roll_joint", "r_wrist_roll_joint"]
            if j.name in reverse_joints:
                j.joint_angle(-a)
            elif j.name=="r_shoulder_pan_joint":
                j.joint_angle(-a + 1.0)
            else:
                j.joint_angle(a)
        self.ri.angle_vector(self.robot.angle_vector(), time=time, time_scale=1.0)

    def start(self):
        start_mannequin(is_whole)
        self.is_thread_active = True
        def mirror_while():
            r = rospy.Rate(10)
            self.mirror(3.0)
            while self.is_thread_active:
                self.mirror(0.5)
                r.sleep()

        self.thread = threading.Thread(target=mirror_while)
        self.thread.daemon = True
        self.thread.start()

    def terminate(self):
        self.is_thread_active = False
        print("deactivate...")
        self.thread.join()
        stop_mannequin(self.is_whole)
        print("mannequin stopped")

if __name__=='__main__':
    config_path = os.path.join(rospkg.RosPack().get_path('kyozi'), 'config')

    parser = argparse.ArgumentParser()
    parser.add_argument('--whole', action='store_true')
    parser.add_argument('-config', type=str, default=os.path.join(config_path, 'pr2_rarm.yaml'))
    args = parser.parse_args()

    config_file = args.config
    config = construct_config(config_file)

    is_whole = args.whole

    mq = Mannequin(config, is_whole=is_whole)
    mq.start()

    while True:
        print('e: end mannequin, r: reset pose')
        if sys.version_info.major == 3:
            key = input()
        else:
            key = raw_input()
        if key=='e':
            mq.terminate()
            break
        if key=='r':
            mq.reset_mannequin()
            mq.start()
