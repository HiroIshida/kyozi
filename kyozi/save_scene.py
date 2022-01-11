#!/usr/bin/env python3
import argparse
import os
import time
import pickle
import sys
import yaml
import json
import rospy
import rospkg
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from utils import Config, construct_config, Resizer, DepthResizer, load_depth
from utils import get_cache_directory

glb_vals = {'is_rec': False, 'image': None, 'depth': None}
def callback(img_msg, depth_msg):
    if glb_vals['is_rec']:
        return
    glb_vals['image'] = img_msg
    glb_vals['depth'] = depth_msg
    glb_vals['is_rec'] = True

def postprocess(config):
    img_msg = glb_vals['image']
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
    image_resized = Resizer(config.image_config)(image)

    depth_msg = glb_vals['depth']
    depth = load_depth(depth_msg)
    depth_resized = Resizer(config.image_config)(depth)

    postfix = time.strftime("%Y%m%d%H%M%S")
    filename = os.path.join(
            get_cache_directory(config, 'scene'), 'scene-{0}.pkl'.format(postfix))
    with open(filename, 'wb') as f:
        pickle.dump((image_resized, depth_resized), f)

if __name__=='__main__':
    rospy.init_node('scene_saver', disable_signals=True)
    config_path = os.path.join(rospkg.RosPack().get_path('kyozi'), 'config')
    parser = argparse.ArgumentParser()
    parser.add_argument('-config', type=str, default=os.path.join(config_path, 'pr2_rarm.yaml'))
    args = parser.parse_args()
    config_file = args.config
    config = construct_config(config_file)

    sub_img = message_filters.Subscriber(config.image_topic, Image)
    assert config.depth_image_topic is not None
    sub_depth_img = message_filters.Subscriber(config.depth_image_topic, Image)
    ts = message_filters.ApproximateTimeSynchronizer([sub_img, sub_depth_img], 2, 0.2)
    ts.registerCallback(callback)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
        if glb_vals['is_rec']:
            postprocess(config)
            rospy.loginfo('finish')
            sys.exit(0)
