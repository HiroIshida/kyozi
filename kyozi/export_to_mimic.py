import os
import argparse
import rospkg
import pickle
from utils import construct_config
from utils import load_dill_6compat
from mimic.datatype import ImageCommandDataChunk

# HiroIshida's personal project

if __name__=='__main__':
    config_path = os.path.join(rospkg.RosPack().get_path('kyozi'), 'config')

    parser = argparse.ArgumentParser()
    parser.add_argument('-config', type=str, default=os.path.join(config_path, 'pr2_rarm.yaml'))
    parser.add_argument('-pn', type=str, default='real_robot')
    parser.add_argument('--strip-depth', action='store_true', help='strip depth if depth info included')
    args = parser.parse_args()
    config_file = args.config
    project_name = args.pn
    strip_depth = args.strip_depth

    config = construct_config(config_file)
    filename = os.path.expanduser(os.path.join(config.project_path, 'summary_chunk.pickle'))
    data = load_dill_6compat(filename)

    chunk = ImageCommandDataChunk()
    for img_seq, cmd_seq in zip(data['img_seqs'], data['cmd_seqs']):
        chunk.push_epoch((img_seq, cmd_seq))
    if strip_depth:
        assert chunk.with_depth
        chunk = chunk.to_depth_stripped()
        assert not chunk.with_depth
        print("successfully stripped")
    chunk.dump(project_name)
