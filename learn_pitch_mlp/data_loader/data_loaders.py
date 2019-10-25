from torchvision import datasets, transforms
from base import BaseDataLoader
from torch.utils.data import Dataset
import numpy as np
from pyquaternion import Quaternion
import torch
import os

def get_ids_in_path(path, prefix):
    files = os.listdir(path)
    ids = [int(file.split('.')[0][len(prefix):])
           for file in files]
    ids.sort()
    return ids

def load_bounding_boxes(bb_file_path):
    bb_file = open(bb_file_path, 'r')
    lines = bb_file.readlines()
    # Has to handle the inconsistency with spaces and commas in our files
    bounding_boxes = [line.split(' ')[3:] for line in lines]
    bounding_boxes = [[float(i[:-1]) for i in line] for line in bounding_boxes]
    return bounding_boxes


def load_pitches(pose_file_path):
    poses_file = open(pose_file_path, 'r')
    lines = poses_file.readlines()
    # Has to handle the inconsistency with spaces and commas in our files
    poses = [line.split(' ')[3:] for line in lines]
    poses = [[float(i[:-1]) for i in line] for line in poses]
    pitches = []
    for pose in poses:
        q = Quaternion(pose[3:7])
        (y,p,r) = q.yaw_pitch_roll
        pitches.append(p)
    return pitches


def preprocess_inputs(boxes):
    # Bring between 0 and 1
    img_height = 480
    img_width = 640
    np_boxes = np.array(boxes)
    x_pos = [0, 2, 4, 6]
    y_pos = [1, 3, 5, 7]
    np_boxes[:, x_pos] = np_boxes[:, x_pos]/img_width
    np_boxes[:, y_pos] = np_boxes[:, y_pos]/img_height
    return list(np_boxes)

# From https://stackoverflow.com/questions/312443/how-do-you-split-a-list-into-evenly-sized-chunks
def chunks(l, n):
    """Yield successive n-sized chunks from l."""
    for i in range(0, len(l)-n):
        yield l[i:i + n]

class BoundingBoxPitchDataset(Dataset):
    """Bounding Box - Pitch dataset."""

    def __init__(self, bounding_boxes_path, pose_files_path, poses_files_prefix, bbs_files_prefix, sequence_length, img_height=480, img_width = 640):

        # Load boxes and pitch
        bb_ids = get_ids_in_path(bounding_boxes_path,bbs_files_prefix)
        pose_ids = get_ids_in_path(pose_files_path,poses_files_prefix)
        assert bb_ids == pose_ids, print("Ids of pose and bounding boxes don't match")

        self.bounding_boxes = []
        self.pitches = []

        for id in bb_ids:
            bb_file = bounding_boxes_path+bbs_files_prefix+str(id)+'.txt'
            poses_file = pose_files_path+poses_files_prefix+str(id)+'.txt'
            bounding_boxes = load_bounding_boxes(bb_file)
            bounding_boxes = preprocess_inputs(bounding_boxes)
            pitches = load_pitches(poses_file)

            assert len(bounding_boxes) == len(pitches), print("Not as many bounding boxes as poses for id "+str(id))

            # Pad with 0s at the beginning
            nb_to_add = sequence_length - len(bounding_boxes)%sequence_length

            for i in np.arange(nb_to_add):
                bounding_boxes.insert(0,np.array([0.,0.,0.,0.,0.,0.,0.,0.]))
                pitches.insert(0,0.0)

            # Gather into blocks of sequences
            self.bounding_boxes.extend(list(chunks(bounding_boxes, sequence_length)))
            self.pitches.extend(list(chunks(pitches, sequence_length)))

        self.bounding_boxes = torch.FloatTensor(self.bounding_boxes).reshape(-1,8*sequence_length)
        self.pitches = torch.FloatTensor(self.pitches).reshape(-1,sequence_length)


    def __len__(self):
        return len(self.bounding_boxes)

    def __getitem__(self, idx):
        if torch.is_tensor(idx):
            idx = idx.tolist()
        # sample = {'bounding_box': np.array(self.bounding_boxes[idx]), 'pitch': np.array(self.pitches[idx])}
        # return sample
        return self.bounding_boxes[idx], self.pitches[idx][-1]


class MnistDataLoader(BaseDataLoader):
    """
    MNIST data loading demo using BaseDataLoader
    """
    def __init__(self, data_dir, batch_size, shuffle=True, validation_split=0.0, num_workers=1, training=True):
        trsfm = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize((0.1307,), (0.3081,))
        ])
        self.data_dir = data_dir
        self.dataset = datasets.MNIST(self.data_dir, train=training, download=True, transform=trsfm)
        print(self.dataset)
        super().__init__(self.dataset, batch_size, shuffle, validation_split, num_workers)


class BoundingBoxPitchDataLoader(BaseDataLoader):
    """
    MNIST data loading demo using BaseDataLoader
    """

    def __init__(self, bounding_boxes_path, pose_files_path, poses_files_prefix, bbs_files_prefix, batch_size, sequence_length, shuffle=True, validation_split=0.0, num_workers=1, training=True):
        self.dataset = BoundingBoxPitchDataset(
            bounding_boxes_path, pose_files_path, poses_files_prefix, bbs_files_prefix, sequence_length)
        print(self.dataset)
        super().__init__(self.dataset, batch_size, shuffle, validation_split, num_workers)
