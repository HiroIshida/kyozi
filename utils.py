import os
import sys
import time
import pickle

def load_pickle_6compat(filename):
    try:
        with open(filename, 'rb') as f:
            obj = pickle.load(f)
    except UnicodeDecodeError:
        print('probably cache file was created by 2.x but attempt to load by 3.x')
        with open(filename, 'rb') as f:
            obj = pickle.load(f, encoding='latin1')
    return obj

def get_cache_directory(): 
    dirname = os.path.expanduser('~/.kyozi')
    if not os.path.exists(dirname):
        os.makedirs(dirname)
    return dirname

class DataChunk:
    def __init__(self):
        self.python_major_version = sys.version_info.major
        self.time_seq = []
        self.img_seq = []
        self.cmd_seq = []
    def push(self, time, img, cmd):
        self.time_seq.append(time)
        self.img_seq.append(img)
        self.cmd_seq.append(cmd)
    def dump(self):
        postfix = time.strftime("%Y%m%d%H%M%S")
        filename = os.path.join(
                get_cache_directory(), 'sequence-{0}.cache'.format(postfix))
        with open(filename, 'wb') as f:
            pickle.dump(self, f)

