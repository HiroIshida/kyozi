#!/usr/bin/env python
import rospkg
import argparse
import sys
import os
import time
import pickle
import numpy as np
try:
    import moviepy
    from moviepy.editor import ImageSequenceClip
    _has_moviepy = True
except:
    _has_moviepy = False
try:
    import skrobot
    _has_skrobot = True
except:
    _has_skrobot = False

from utils import get_project_directory, get_cache_directory
from utils import load_pickle_6compat
from utils import construct_config

def get_sample_data(config):
    project_dir = get_project_directory(config)
    files = [os.path.join(project_dir, fname) for fname in os.listdir(project_dir)]
    chunk_files = [fn for fn in files if os.path.splitext(fn)[1]=='.pickle']
    assert len(chunk_files) == 1

    chunk = load_pickle_6compat(chunk_files[0])
    cmd_seq = chunk['cmd_seqs'][0]
    img_seq = chunk['img_seqs'][0]

    assert isinstance(cmd_seq, np.ndarray)
    assert cmd_seq.ndim == 2 # (n_seq, n_state)

    assert isinstance(img_seq, np.ndarray)
    assert img_seq.ndim == 4 # (n_seq, pix, pix, channel)
    return cmd_seq, img_seq

if __name__=='__main__':
    config_path = os.path.join(rospkg.RosPack().get_path('kyozi'), 'config')

    parser = argparse.ArgumentParser()
    parser.add_argument('-config', type=str, default=os.path.join(config_path, 'pr2_rarm.yaml'))
    args = parser.parse_args()
    config_file = args.config
    config = construct_config(config_file)

    cmd_seq, img_seq = get_sample_data(config)

    if _has_moviepy:
        cache_dir = get_project_directory(config)
        filename_gif = os.path.join(cache_dir, "sample.gif")
        clip = ImageSequenceClip([img for img in img_seq], fps=50)
        clip.write_gif(filename_gif, fps=50)

    if _has_skrobot:
        robot = skrobot.models.PR2()
        viewer = skrobot.viewers.TrimeshSceneViewer(resolution=(640, 480))
        viewer.add(robot)
        viewer.show()
        time.sleep(2)
        for cmd in cmd_seq:
            for jn, angle in zip(config.joint_names, cmd):
                joint = robot.__dict__[jn]
                joint.joint_angle(angle)
            viewer.redraw()
            time.sleep(0.2)
