import os
import sys
import time
import pickle
import yaml

class Config(object):
    def __init__(self, project_path, image_topic, joint_states_topic, joint_names):
        self.project_path = project_path
        self.image_topic = image_topic
        self.joint_states_topic = joint_states_topic
        self.joint_names = joint_names

def construct_config(config_file):
    with open(config_file) as f:
        dic = yaml.safe_load(f)
    config = Config(
            project_path = dic['project_path'], 
            image_topic = dic['image_topic'],
            joint_states_topic = dic['joint_states_topic'],
            joint_names = dic['joint_names'])
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
    def __init__(self, joint_names):
        self.python_major_version = sys.version_info.major
        self.time_seq = []
        self.img_seq = []
        self.cmd_seq = []
        self.joint_names = joint_names
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

