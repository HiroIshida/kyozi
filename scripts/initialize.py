#!/usr/bin/env python
import os
import argparse
import skrobot
import rospkg
from utils import Config, construct_config

if __name__=='__main__':
    parser = argparse.ArgumentParser()
    config_path = os.path.join(rospkg.RosPack().get_path('kyozi'), 'config')
    parser.add_argument('-config', type=str, default=os.path.join(config_path, 'pr2_rarm.yaml'))
    args = parser.parse_args()
    config_file = args.config
    config = construct_config(config_file)

    robot = skrobot.models.PR2()

    for jn, angle in zip(config.init_joint_names, config.init_joint_angles):
        robot.__dict__[jn].joint_angle(angle)

    ri = skrobot.interfaces.ros.PR2ROSRobotInterface(robot)
    ri.angle_vector(robot.angle_vector(), time=1.0, time_scale=1.0)
