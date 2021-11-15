import os
import argparse
import rospkg
import pickle
from utils import construct_config
from mimic.datatype import ImageCommandDataChunk

# HiroIshida's personal project

if __name__=='__main__':
    config_path = os.path.join(rospkg.RosPack().get_path('kyozi'), 'config')

    parser = argparse.ArgumentParser()
    parser.add_argument('-config', type=str, default=os.path.join(config_path, 'pr2_rarm.yaml'))
    parser.add_argument('-pn', type=str, default='project name')
    args = parser.parse_args()
    config_file = args.config
    project_name = args.pn

    config = construct_config(config_file)
    filename = os.path.expanduser(os.path.join(config.project_path, 'summary_chunk.pickle'))
    with open(filename, 'rb') as f:
        data = pickle.load(f)

    chunk = ImageCommandDataChunk()
    for img_seq, cmd_seq in zip(data['img_seqs'], data['cmd_seqs']):
        chunk.push_epoch((img_seq, cmd_seq))
    chunk.dump(project_name)
