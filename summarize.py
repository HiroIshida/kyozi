#!/usr/bin/env python
import argparse
import os
import sys
import pickle
import tqdm
import numpy as np
import math
import cv2

from utils import DataChunk
from utils import get_cache_directory

def clamp_to_s1(something):
    lower_side = -math.pi
    return ((something - lower_side) % (2 * math.pi)) + lower_side

def nearest_time_sampling(sequence, sampling_hz, resize_shape=None):
    coef = 1.2
    time_seq = sequence.time_seq
    average_hz = 1.0/((time_seq[-1] - time_seq[0])/(len(time_seq) - 1))
    assert sampling_hz * coef < average_hz, 'your data is too scarse for specified sampling hz'

    time_seq_normalized = np.array([e - time_seq[0] for e in time_seq])
    interval = 1.0/sampling_hz
    cmdseq = []
    imgseq = []
    for i in range(int(time_seq_normalized[-1] * sampling_hz)):
        t = interval * i
        idx = np.argmin(np.abs(time_seq_normalized - t))
        if resize_shape is not None:
            img = cv2.resize(sequence.img_seq[idx], resize_shape)
        else:
            img = sequence.img_seq[idx]
        imgseq.append(img)
        cmdseq.append(sequence.cmd_seq[idx])
    return np.array(imgseq), np.array(cmdseq)

def summarize(sampling_hz=24, resolution=None):
    cache_dir = get_cache_directory()
    files = [os.path.join(cache_dir, fname) for fname in os.listdir(cache_dir)]
    cache_files = [fn for fn in files if os.path.splitext(fn)[1]=='.cache']

    img_seqs = []
    cmd_seqs = []
    for fn in tqdm.tqdm(cache_files):
        for filename in tqdm.tqdm(cache_files):
            with open(filename, 'rb') as f:
                sequence = pickle.load(f)
            assert sequence.python_major_version == sys.version_info.major
            img_seq, cmd_seq = nearest_time_sampling(sequence, sampling_hz, resolution)
            img_seqs.append(img_seq)
            cmd_seqs.append(clamp_to_s1(cmd_seq))
            print("summarized into {0} points.".format(len(img_seqs[0])))

if __name__=='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-hz', type=int, default=10)
    args = parser.parse_args()
    sampling_hz = args.hz
    summarize(sampling_hz)
