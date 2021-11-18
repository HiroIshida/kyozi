#!/usr/bin/env python
import sys
import argparse
import os
import rospy
import rospkg
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from cv_bridge import CvBridge
import numpy as np

from kyozi.utils import Config
from kyozi.utils import Resizer
from kyozi.utils import construct_config

assert sys.version_info.major==3, 'python2.x under construction'

class Controller:
    def __init__(self, predictor, config: Config, hz=5):

        rospy.Subscriber(config.image_topic, Image, self.on_image)
        rospy.Subscriber(config.joint_states_topic, JointState, self.on_joint_state)
        rospy.Timer(rospy.Duration(1.0/hz), self.on_timer)

        self.config = config
        self.hz = hz
        self.predictor = predictor
        self.resizer = Resizer(config.image_config)

        self._is_active = False
        self._image_msg = None
        self._joint_state_msg = None
        self._joint_index_table = {}

    def activate(self): self._is_active = True
    def on_image(self, image_msg): self._image_msg = image_msg

    def on_joint_state(self, joint_state_msg): 
        self._joint_state_msg = joint_state_msg

    def obtain_angle_vector(self, joint_state_msg):
        if self._joint_index_table == {}: # create cache
            for idx, name in enumerate(joint_state_msg.name):
                self._joint_index_table[name] = idx
        ctrl_idxes = [self._joint_index_table[name] for name in self.config.control_joint_names]
        angle_vector = np.array([joint_state_msg.position[idx] for idx in ctrl_idxes])
        return angle_vector

    def check_msgs_validity(self):
        ts_now = rospy.get_rostime().to_sec()
        ts_image = self._image_msg.header.stamp.to_sec()
        ts_jointstate = self._joint_state_msg.header.stamp.to_sec()

        delay_image = ts_now - ts_image
        delay_joint_state = ts_now - ts_jointstate
        threshold_delay = 1.0/self.hz * 0.5

        rospy.loginfo('delay img: {0}, joint: {1}, threshold: {2}'.format(delay_image, delay_joint_state, threshold_delay))
        if delay_image > 0.5 * threshold_delay or delay_joint_state > 0.5 * threshold_delay:
            rospy.logwarn('thogh acceptable, delay is bit large.')
            if delay_image > threshold_delay or delay_joint_state > threshold_delay:
                rospy.logerr('critical delay detected. Please set hz lower.')

    def on_timer(self, event):
        if not (self._is_active and self._image_msg and self._joint_state_msg):
            return
        self.check_msgs_validity()

        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(self._image_msg, desired_encoding='passthrough')
        image_resized = self.resizer(image)
        angle_vector = self.obtain_angle_vector(self._joint_state_msg)

if __name__=='__main__':
    rospy.init_node('visuo_motor_controller', anonymous=True)
    config_path = os.path.join(rospkg.RosPack().get_path('kyozi'), 'config')

    parser = argparse.ArgumentParser()
    parser.add_argument('-config', type=str, default=os.path.join(config_path, 'pr2_rarm.yaml'))
    args = parser.parse_args()
    config_file = args.config
    config = construct_config(config_file)

    cont = Controller(None, config, hz=3)
    cont.activate()
    rospy.spin()
