#!/usr/bin/env python3
import argparse
import os
import subprocess
import signal
import time
import rospkg
from utils import Config, construct_config, get_cache_directory

def create_rosbag_command(config):
    postfix = time.strftime("%Y%m%d%H%M%S")
    filename = os.path.join(
            get_cache_directory(config), 'sequence-{0}'.format(postfix))
    cmd_rosbag = ['rosbag', 'record']
    cmd_rosbag.extend(config.rosbag_topics)
    cmd_rosbag.extend(['--output-name', filename])
    print('subprocess cmd: {}'.format(cmd_rosbag))
    return cmd_rosbag

if __name__=='__main__':
    config_path = os.path.join(rospkg.RosPack().get_path('kyozi'), 'config')
    parser = argparse.ArgumentParser()
    parser.add_argument('-config', type=str, default=os.path.join(config_path, 'pr2_rarm.yaml'))
    args = parser.parse_args()
    config_file = args.config

    config = construct_config(config_file)

    cmd = create_rosbag_command(config)
    p = subprocess.Popen(cmd)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt: 
        os.kill(p.pid, signal.SIGKILL)
