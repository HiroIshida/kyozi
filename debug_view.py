import os
import time
import skrobot
import pickle

from utils import get_cache_directory
from utils import load_pickle_6compat

cache_dir = get_cache_directory()
files = [os.path.join(cache_dir, fname) for fname in os.listdir(cache_dir)]
cache_files = [fn for fn in files if os.path.splitext(fn)[1]=='.cache']
chunk_files = [fn for fn in files if os.path.splitext(fn)[1]=='.pickle']
assert len(chunk_files) == 1

sequence = load_pickle_6compat(cache_files[0])
chunk = load_pickle_6compat(chunk_files[0])

joint_names = sequence.joint_names
cmd_seq = chunk['cmd_seqs'][0]

robot = skrobot.models.PR2()
viewer = skrobot.viewers.TrimeshSceneViewer(resolution=(640, 480))
viewer.add(robot)
viewer.show()
joint_names = sequence.joint_names

for cmd in cmd_seq:
    for jn, angle in zip(joint_names, cmd):
        joint = robot.__dict__[jn]
        joint.joint_angle(angle)
    viewer.redraw()
    time.sleep(0.2)
