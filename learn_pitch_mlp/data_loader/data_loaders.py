from torchvision import datasets, transforms
from base import BaseDataLoader
from torch.utils.data import Dataset
import numpy as np
from pyquaternion import Quaternion
import torch
import os

def load_bounding_boxes(bb_path):
    bb_file = open(bb_path, 'r')
    lines = bb_file.readlines()
    bounding_boxes = [line.split()[2:] for line in lines]
    bounding_boxes = [[float(i) for i in line] for line in bounding_boxes]
    return bounding_boxes


def load_pitches(poses_path, file_name_prefix):
    pose_files = os.listdir(poses_path)
    # pose_files.sort()
    ids = [int(pose_file.split('.')[0][len(file_name_prefix):])
           for pose_file in pose_files]
    ids.sort()
    pitches = []
    for id in ids:
        filepath = poses_path+file_name_prefix+str(id)+'.txt'
        pose_file = open(filepath, 'r')
        q = Quaternion(pose_file.readlines()[0].split()[3:7])
        (y, p, r) = q.yaw_pitch_roll
        pitches.append(p)
    return pitches  # inputs


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
    for i in range(0, len(l), n):
        yield l[i:i + n]

class BoundingBoxPitchDataset(Dataset):
    """Bounding Box - Pitch dataset."""

    def __init__(self, bounding_boxes_path, pose_files_path, poses_files_prefix, sequence_length, img_height=480, img_width = 640):

        # Load boxes and pitch
        self.bounding_boxes = load_bounding_boxes(bounding_boxes_path)
        self.bounding_boxes = preprocess_inputs(self.bounding_boxes)
        self.pitches = load_pitches(pose_files_path, poses_files_prefix)

        # Pad with 0s at the beginning
        nb_to_add = sequence_length - len(self.bounding_boxes)%sequence_length

        for i in np.arange(nb_to_add):
            self.bounding_boxes.insert(0,np.array([0.,0.,0.,0.,0.,0.,0.,0.]))
            self.pitches.insert(0,0.0)
        # Gather into blocks of sequences
        self.bounding_boxes = torch.FloatTensor(list(
            chunks(self.bounding_boxes, sequence_length))).reshape(-1,8*sequence_length)
        self.pitches = torch.FloatTensor(list(
            chunks(self.pitches, sequence_length)))

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

    def __init__(self, bounding_boxes_path, pose_files_path, poses_files_prefix, batch_size, sequence_length, shuffle=True, validation_split=0.0, num_workers=1, training=True):
        self.dataset = BoundingBoxPitchDataset(
            bounding_boxes_path, pose_files_path, poses_files_prefix, sequence_length)
        print(self.dataset)
        super().__init__(self.dataset, batch_size, shuffle, validation_split, num_workers)
