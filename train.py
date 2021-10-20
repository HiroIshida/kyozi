#!/usr/bin/env python
import argparse
import os
import sys
import uuid
import pickle
import yaml
import json
import rospy
import rospkg
import message_filters
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState

import cv2
from cv_bridge import CvBridge

def get_cache_directory(): 
    dirname = os.path.expanduser('~/.ros_data_collection')
    if not os.path.exists(dirname):
        os.makedirs(dirname)
    return dirname

class DataChunk:
    def __init__(self):
        self.python_major_version = sys.version_info.major
        self.img_seq = []
        self.cmd_seq = []
    def push(self, img, cmd):
        self.img_seq.append(img)
        self.cmd_seq.append(cmd)
    def dump(self):
        postfix = str(uuid.uuid4())
        filename = os.path.join(
                get_cache_directory(), 'datachunk{0}.cache'.format(postfix))
        with open(filename, 'wb') as f:
            pickle.dump(self, f)

class DataManager:
    def __init__(self, config):
        sub_img = message_filters.Subscriber(config['image_topic'], Image)
        sub_joint_state = message_filters.Subscriber(config['joint_states_topic'], JointState)
        ts = message_filters.ApproximateTimeSynchronizer([sub_img, sub_joint_state], 2, 0.2)
        ts.registerCallback(self.callback)

        self.subs = [sub_img, sub_joint_state]
        self.joint_names = config['joint_names']
        self.joint_name_idx_table = None
        self.data_chunk = DataChunk()
        self.flag_start = False

    def callback(self, img_msg, joint_states_msg):
        if not self.flag_start:
            return 
        if not self.joint_name_idx_table:
            data_dict = {name: i for (i, name) in enumerate(joint_states_msg.name)}
        rospy.loginfo('inside callback') 

        cmd = [data_dict[jname] for jname in self.joint_names]
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
        self.data_chunk.push(image, cmd)

    def start_session(self):
        self.flag_start = True

    def end_session(self):
        self.data_chunk.dump()
        self.flag_start = False
        for sub in self.subs:
            sub.unregister()

if __name__=='__main__':
    rospy.init_node('data_collector', disable_signals=True)
    config_path = os.path.join(rospkg.RosPack().get_path('ros_data_collection'), 'config')

    parser = argparse.ArgumentParser()
    parser.add_argument('-config', type=str, default=os.path.join(config_path, 'pr2_rarm.yaml'))
    args = parser.parse_args()
    config_file = args.config

    with open(config_file) as f:
        config = yaml.safe_load(f)
    dm = DataManager(config)
    dm.start_session()
    try:
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
    except KeyboardInterrupt: 
        rospy.loginfo('finish')
        dm.end_session()
        sys.exit(0)
