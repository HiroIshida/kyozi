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

from utils import DataChunk
from utils import get_cache_directory

class DataManager:
    def __init__(self, config):
        sub_img = message_filters.Subscriber(config['image_topic'], Image)
        sub_joint_state = message_filters.Subscriber(config['joint_states_topic'], JointState)
        ts = message_filters.ApproximateTimeSynchronizer([sub_img, sub_joint_state], 2, 0.2)
        ts.registerCallback(self.callback)

        self.subs = [sub_img, sub_joint_state]
        self.joint_names = config['joint_names']
        self.joint_idxes = None
        self.data_chunk = DataChunk(self.joint_names)
        self.flag_start = False

    def callback(self, img_msg, joint_states_msg):
        if not self.flag_start:
            return 
        if not self.joint_idxes:
            data_dict = {name: i for (i, name) in enumerate(joint_states_msg.name)}
            self.joint_idxes = [data_dict[name] for name in self.joint_names]
        rospy.loginfo('inside callback') 

        cmd = [joint_states_msg.position[idx] for idx in self.joint_idxes]
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
        t = img_msg.header.stamp
        time = t.secs + 1e-9 * t.nsecs

        self.data_chunk.push(time, image, cmd)

    def start_session(self):
        self.flag_start = True

    def end_session(self):
        self.flag_start = False
        self.data_chunk.dump()
        for sub in self.subs:
            sub.unregister()

if __name__=='__main__':
    rospy.init_node('data_collector', disable_signals=True)
    config_path = os.path.join(rospkg.RosPack().get_path('kyozi'), 'config')

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
