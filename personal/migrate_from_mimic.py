import os
import pickle
import tqdm
from mimic.datatype import ImageDataChunk
from mimic.datatype import ImageCommandDataChunk
from mimic.dataset import ReconstructionDataset
project_name = 'robot'
chunk = ImageCommandDataChunk.load(project_name)
chunk_export = {'img_seqs': [], 'cmd_seqs': []}
for seqs in tqdm.tqdm(chunk.seqs_list):
    img_seq, cmd_seq = seqs
    chunk_export['img_seqs'].append(img_seq.data)
    chunk_export['cmd_seqs'].append(cmd_seq.data)
filename = os.path.join(os.path.expanduser('~/.kyozi/summary_chunk.pickle'))
with open(filename, 'wb') as f:
    pickle.dump(chunk_export, f)
