import os
import sys
import time
import yaml
import cv2

if sys.version_info.major==3:
    import pickle
else:
    import cPickle as pickle

class Resizer:
    def __init__(self, image_config):
        self.x_bound = slice(image_config.x_min, image_config.x_max)
        self.y_bound = slice(image_config.y_min, image_config.y_max)
        self.resol = image_config.resolution
    def __call__(self, cv_img):
        cv_img = cv_img[self.x_bound, self.y_bound]
        resized = cv2.resize(cv_img, (self.resol, self.resol), interpolation = cv2.INTER_AREA)
        return resized

class ImageConfig(object):
    def __init__(self, img_dict):
        self.x_min = img_dict['x_min']
        self.x_max = img_dict['x_max']
        self.y_min = img_dict['y_min']
        self.y_max = img_dict['y_max']
        self.resolution = img_dict['resolution']

class Config(object):
    def __init__(self, 
            project_path, 
            image_topic, 
            depth_image_topic,
            joint_states_topic, 
            control_joint_names,
            init_joint_names,
            init_joint_angles,
            image_config,
            ):
        self.project_path = project_path
        self.image_topic = image_topic
        self.depth_image_topic = depth_image_topic
        self.joint_states_topic = joint_states_topic
        self.control_joint_names = control_joint_names
        self.init_joint_names = init_joint_names
        self.init_joint_angles = init_joint_angles
        self.image_config = ImageConfig(image_config)

def construct_config(config_file):
    with open(config_file) as f:
        dic = yaml.safe_load(f)
    config = Config(
            project_path = dic['project_path'], 
            image_topic = dic['image_topic'],
            depth_image_topic= dic['depth_image_topic'],
            joint_states_topic = dic['joint_states_topic'],
            control_joint_names = dic['control_joint_names'],
            init_joint_names = dic['init_joint_names'],
            init_joint_angles = dic['init_joint_angles'],
            image_config = dic['image_config']
            )
    return config

def load_pickle_6compat(filename):
    try:
        with open(filename, 'rb') as f:
            obj = pickle.load(f)
    except UnicodeDecodeError:
        print('probably cache file was created by 2.x but attempt to load by 3.x')
        with open(filename, 'rb') as f:
            obj = pickle.load(f, encoding='latin1')
    return obj

def get_project_directory(config):
    dirname = os.path.expanduser(config.project_path)
    if not os.path.exists(dirname):
        os.makedirs(dirname)
    return dirname

def get_cache_directory(config): 
    project_dir = get_project_directory(config)
    cache_dir = os.path.join(project_dir, 'kyozi_cache')
    if not os.path.exists(cache_dir):
        os.makedirs(cache_dir)
    return cache_dir

class DataChunk:
    def __init__(self, control_joint_names):
        self.python_major_version = sys.version_info.major
        self.time_seq = []
        self.img_seq = []
        self.cmd_seq = []
        self.control_joint_names = control_joint_names
    def push(self, time, img, cmd):
        self.time_seq.append(time)
        self.img_seq.append(img)
        self.cmd_seq.append(cmd)
    def dump(self, config):
        postfix = time.strftime("%Y%m%d%H%M%S")
        filename = os.path.join(
                get_cache_directory(config), 'sequence-{0}.cache'.format(postfix))
        with open(filename, 'wb') as f:
            pickle.dump(self, f)

