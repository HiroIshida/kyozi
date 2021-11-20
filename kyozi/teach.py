#!/usr/bin/env python
import argparse
import os
import sys
import yaml
import json
import rospy
import rospkg
import message_filters
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from cv_bridge import CvBridge
import numpy as np

from utils import Config, construct_config
from utils import DataChunk

class DataManager:
    def __init__(self, config):
        sub_img = message_filters.Subscriber(config.image_topic, Image)
        if config.depth_image_topic is not None:
            sub_depth_img = message_filters.Subscriber(config.depth_image_topic, Image)
        else:
            sub_depth_img = None
        sub_joint_state = message_filters.Subscriber(config.joint_states_topic, JointState)

        if sub_depth_img is None:
            ts = message_filters.ApproximateTimeSynchronizer([sub_img, sub_joint_state], 2, 0.2)
            ts.registerCallback(self.callback)
        else:
            ts = message_filters.ApproximateTimeSynchronizer([sub_img, sub_depth_img, sub_joint_state], 2, 0.2)
            ts.registerCallback(self.callback_with_depth)

        self.subs = [sub_img, sub_joint_state]
        self.control_joint_names = config.control_joint_names
        self.joint_idxes = None
        self.data_chunk = DataChunk(self.control_joint_names)
        self.flag_start = False
        self.config = config

    def callback_with_depth(self, img_msg, depth_image_msg, joint_states_msg):
        self._callback(img_msg, depth_image_msg, joint_states_msg)

    def callback(self, img_msg, joint_states_msg):
        self._callback(img_msg, None, joint_states_msg)

    def _callback(self, img_msg, depth_image_msg, joint_states_msg):
        if not self.flag_start:
            return 
        if not self.joint_idxes:
            data_dict = {name: i for (i, name) in enumerate(joint_states_msg.name)}
            self.joint_idxes = [data_dict[name] for name in self.control_joint_names]
        rospy.loginfo('save data') 

        cmd = [joint_states_msg.position[idx] for idx in self.joint_idxes]
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
        t = img_msg.header.stamp
        time = t.secs + 1e-9 * t.nsecs

        if depth_image_msg is not None:
            tmp = bridge.imgmsg_to_cv2(depth_image_msg, desired_encoding='passthrough')
            depth_image = np.expand_dims(tmp, axis=2)
            image = np.concatenate((image, depth_image), axis=2)

        self.data_chunk.push(time, image, cmd)

    def start_session(self):
        self.flag_start = True

    def end_session(self):
        self.flag_start = False
        self.data_chunk.dump(self.config)
        for sub in self.subs:
            sub.unregister()

if __name__=='__main__':
    rospy.init_node('data_collector', disable_signals=True)
    config_path = os.path.join(rospkg.RosPack().get_path('kyozi'), 'config')

    parser = argparse.ArgumentParser()
    parser.add_argument('-config', type=str, default=os.path.join(config_path, 'pr2_rarm.yaml'))
    args = parser.parse_args()
    config_file = args.config

    config = construct_config(config_file)
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
