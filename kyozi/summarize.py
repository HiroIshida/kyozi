#!/usr/bin/env python
import argparse
import os
import sys
import pickle
import tqdm
import numpy as np
import math
import rospkg

from utils import DataChunk
from utils import get_cache_directory, get_project_directory
from utils import load_pickle_6compat
from utils import Config, construct_config, Resizer

def clamp_to_s1(something):
    lower_side = -math.pi
    return ((something - lower_side) % (2 * math.pi)) + lower_side

def nearest_time_sampling(sequence, sampling_hz, resizer=None):
    coef = 1.2
    time_seq = sequence.time_seq
    average_hz = 1.0/((time_seq[-1] - time_seq[0])/(len(time_seq) - 1))
    assert sampling_hz * coef < average_hz, 'your data is too sparse for specified sampling hz. Please set smaller hz.'

    time_seq_normalized = np.array([e - time_seq[0] for e in time_seq])
    interval = 1.0/sampling_hz
    cmdseq = []
    imgseq = []
    for i in range(int(time_seq_normalized[-1] * sampling_hz)):
        t = interval * i
        idx = np.argmin(np.abs(time_seq_normalized - t))
        if resizer is not None:
            img = resizer(sequence.img_seq[idx])
        else:
            img = sequence.img_seq[idx]
        imgseq.append(img)
        cmdseq.append(sequence.cmd_seq[idx])
    return np.array(imgseq), np.array(cmdseq)

def summarize(config, sampling_hz=24):
    cache_dir = get_cache_directory(config)
    files = [os.path.join(cache_dir, fname) for fname in os.listdir(cache_dir)]
    cache_files = [fn for fn in files if os.path.splitext(fn)[1]=='.cache']
    resizer = Resizer(config.image_config)

    img_seqs = []
    cmd_seqs = []
    for fn in tqdm.tqdm(cache_files):
        sequence = load_pickle_6compat(fn)
        img_seq, cmd_seq = nearest_time_sampling(sequence, sampling_hz, resizer)
        img_seqs.append(img_seq)
        cmd_seqs.append(clamp_to_s1(cmd_seq))
        print("summarized into {0} points.".format(len(img_seq)))
    return img_seqs, cmd_seqs

if __name__=='__main__':
    config_path = os.path.join(rospkg.RosPack().get_path('kyozi'), 'config')

    parser = argparse.ArgumentParser()
    parser.add_argument('-config', type=str, default=os.path.join(config_path, 'pr2_rarm.yaml'))
    parser.add_argument('-hz', type=int, default=10)
    args = parser.parse_args()
    config_file = args.config
    sampling_hz = args.hz
    
    config = construct_config(config_file)
    img_seqs, cmd_seqs = summarize(config, sampling_hz)
    chunk = {'img_seqs': img_seqs, 'cmd_seqs': cmd_seqs, 'python_major_version': sys.version_info.major}
    filename = os.path.join(get_project_directory(config), 'summary_chunk.pickle')
    with open(filename, 'wb') as f:
        pickle.dump(chunk, f)
