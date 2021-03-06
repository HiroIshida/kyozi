#!/usr/bin/env python
import sys
assert sys.version_info.major==3, 'python2.x under construction'
import time
import argparse
import os
import rospy
import rospkg
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from cv_bridge import CvBridge
import numpy as np
import skrobot

from kyozi.utils import Config
from kyozi.utils import Resizer
from kyozi.utils import construct_config
from kyozi.utils import get_cache_directory

from mimic.predictor import ImageCommandPredictor

try:
    import moviepy
    from moviepy.editor import ImageSequenceClip
    _has_moviepy = True
except:
    _has_moviepy = False

class Controller:
    def __init__(self, predictor: ImageCommandPredictor, config: Config, hz=5, n_bootstrap=10):

        self.robot = skrobot.models.PR2() # TODO(HiroIshida) currently only PR2
        self.ri = skrobot.interfaces.ros.PR2ROSRobotInterface(self.robot)
        self.robot.angle_vector(self.ri.angle_vector())
        self.config = config
        self.hz = hz
        self.predictor = predictor
        self.resizer = Resizer(config.image_config)
        self.n_bootstrap = n_bootstrap

        self._feed_count = 0
        self._is_active = False
        self._image_msg = None
        self._joint_state_msg = None
        self._joint_index_table = {}

        self._debug_fed_images = []
        self._debug_pred_images = []

        rospy.Subscriber(config.image_topic, Image, self.on_image)
        rospy.Subscriber(config.joint_states_topic, JointState, self.on_joint_state)
        rospy.Timer(rospy.Duration(1.0/hz), self.on_timer)

    def activate(self): self._is_active = True
    def deactivate(self): self._is_active = False
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
        current_angle_vector = self.obtain_angle_vector(self._joint_state_msg)
        if self._feed_count==0:
            for i in range(self.n_bootstrap):
                self.predictor.feed((image_resized, current_angle_vector))
                self._debug_fed_images.append(image_resized)
        else:
            self.predictor.feed((image_resized, current_angle_vector))
            self._debug_fed_images.append(image_resized)

        self._feed_count += 1
        image_next, command_next =  self.predictor.predict(n_horizon=1)[0]

        if image_next is not None:
            self._debug_pred_images.append(image_next)

        assert len(command_next) == len(self.config.control_joint_names)

        current_angle_vector + (command_next - current_angle_vector) * self.hz
        for jn, angle in zip(self.config.control_joint_names, command_next):
            self.robot.__dict__[jn].joint_angle(angle)
        self.ri.angle_vector(self.robot.angle_vector(), time=1.0, time_scale=1.0)

    def end_replay(self):
        print("finishing the session...")
        self.deactivate()
        if not _has_moviepy:
            print("gif is not saved because moviepy not found")
            return
        directory = get_cache_directory(self.config, cache_name='replay_cache')
        postfix = time.strftime("%Y%m%d%H%M%S")
        filename_fed = os.path.join(directory, 'fed_images-{}.gif'.format(postfix))
        filename_pred = os.path.join(directory, 'pred_images-{}.gif'.format(postfix))

        clip = ImageSequenceClip([img for img in self._debug_fed_images], fps=20)
        clip.write_gif(filename_fed, fps=20)

        clip = ImageSequenceClip([img for img in self._debug_pred_images], fps=20)
        clip.write_gif(filename_pred, fps=20)

if __name__=='__main__':
    rospy.init_node('visuo_motor_controller', anonymous=True)
    config_path = os.path.join(rospkg.RosPack().get_path('kyozi'), 'config')

    parser = argparse.ArgumentParser()
    parser.add_argument('-config', type=str, default=os.path.join(config_path, 'pr2_rarm.yaml'))
    args = parser.parse_args()
    config_file = args.config
    config = construct_config(config_file)

    cont = Controller(None, config, hz=1)
    cont.activate()
    try:
        rospy.spin()
    except KeyboardInterrupt: 
        rospy.loginfo('finish')
        cont.end_replay()
        sys.exit(0)
