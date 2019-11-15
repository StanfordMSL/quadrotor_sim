from torchvision import datasets, transforms
from base import BaseDataLoader
from torch.utils.data import Dataset
import numpy as np
from pyquaternion import Quaternion
import torch
import os
from convert_bb_corners_to_angle import bb_corners_to_angle
def reorder_sim_bb(bb):
    tmp_bb = np.array(bb)
    reordered_bb = np.zeros(tmp_bb.shape)
    order = [2, 3, 0, 1]
    for i in range(len(order)):
        reordered_bb[:, 2*i] = tmp_bb[:, 2*order[i]]
        reordered_bb[:, 2*i+1] = tmp_bb[:, 2*order[i]+1]

    return list(reordered_bb)

def downsample_list(list, rate):
    indexes_to_keep = np.arange(0,len(list),rate)
    return [list[i] for i in indexes_to_keep]


# def filter_data(bounding_boxes, poses, pitch_thresh):
#     ret_poses = poses.copy()
#     nb_removed = 0
#     for i, pose in enumerate(poses):
#         if np.abs(pitch[-1]) < pitch_thresh:
#             bounding_boxes.pop(i-nb_removed)
#             ret_pitches.pop(i-nb_removed)
#             nb_removed += 1
#     return bounding_boxes, ret_pitches


def get_ids_in_path(path, prefix):
    files = os.listdir(path)
    ids = [int(file.split('.')[0][len(prefix):]) for file in files if file[:len(prefix)] == prefix]
    ids.sort()
    return ids


def load_real_bounding_boxes(bb_file_path):
    input_file = open(bb_file_path, 'r')
    lines = input_file.readlines()
    inputs = [line.split()[2:] for line in lines]
    inputs = [[float(i) for i in line] for line in inputs]
    return inputs


def load_real_poses(poses_path, file_name_prefix):
    pose_files = os.listdir(poses_path)
    # pose_files.sort()
    ids = [int(pose_file.split('.')[0][len(file_name_prefix):])
           for pose_file in pose_files]
    ids.sort()
    poses = []
    for id in ids:
        filepath = poses_path+file_name_prefix+str(id)+'.txt'
        pose_file = open(filepath, 'r')
        (x,y,z,q0,q1,q2,q3) = [float(i) for i in pose_file.readlines()[0].split()[2:9]]
        if q0 < 0:
            (q0,q1,q2,q3) = (-q0,-q1,-q2,-q3)
        # q = Quaternion([q0,q1,q2,q3])
        # (y, p, r) = q.yaw_pitch_roll
        poses.append([x,y,z,q0,q1,q2,q3])
    return poses  # input


def load_sim_bounding_boxes(bb_file_path):
    bb_file = open(bb_file_path, 'r')
    lines = bb_file.readlines()
    # Has to handle the inconsistency with spaces and commas in our files
    bounding_boxes = [line.split(' ')[3:] for line in lines]
    bounding_boxes = [[float(i[:-1]) for i in line] for line in bounding_boxes]
    return bounding_boxes


def load_sim_poses(pose_file_path):
    poses_file = open(pose_file_path, 'r')
    lines = poses_file.readlines()
    # Has to handle the inconsistency with spaces and commas in our files
    poses = [line.split(' ')[3:] for line in lines]
    poses = [[float(i[:-1]) for i in line] for line in poses]
    # pose  s = []
    # for pose in poses:
        # (x,y,z) = pose[:3]
        # q = Quaternion(pose[3:7])
        # (y, p, r) = q.yaw_pitch_roll
        # poses.append([float(x),float(y),float(z),float(y),float(p),float(r)])
    return poses


def preprocess_inputs(boxes,use_width_height):
    # Bring between 0 and 1
    img_height = 480
    img_width = 640
    if use_width_height:
        np_boxes = np.array(boxes)
        np_boxes[:,[0,3]] = np_boxes[:,[0,3]]/img_height
        np_boxes[:,[1,2]] = np_boxes[:,[0,3]]/img_width
    else:
        np_boxes = np.array(boxes)
        x_pos = [0, 2, 4, 6]
        y_pos = [1, 3, 5, 7]
        np_boxes[:, x_pos] = np_boxes[:, x_pos]/img_width
        np_boxes[:, y_pos] = np_boxes[:, y_pos]/img_height
    return list(np_boxes)

# From https://stackoverflow.com/questions/312443/how-do-you-split-a-list-into-evenly-sized-chunks


def chunks(l, n):
    """Yield successive overlapping n-sized chunks from l."""
    for i in range(0, len(l)-n+1):
        yield l[i:i + n]


class SimBoundingBoxPitchDataset(Dataset):
    """Bounding Box - Pitch dataset."""

    def __init__(self, bounding_boxes_path, pose_files_path, poses_files_prefix, bbs_files_prefix, sequence_length, downsample_rate=1, filter_pitch=False, pitch_threshold=3.14159, img_height=480, img_width=640, use_width_height=True):

        # Load boxes and pitch
        bb_ids = get_ids_in_path(bounding_boxes_path, bbs_files_prefix)
        pose_ids = get_ids_in_path(pose_files_path, poses_files_prefix)
        assert bb_ids == pose_ids, print(
            "Ids of pose and bounding boxes don't match")

        self.bounding_boxes = []
        self.poses = []
        output_size = 7
        if use_width_height:
            input_size = 5
        else:
            input_size = 8

        for id in bb_ids:
            bb_file = bounding_boxes_path+bbs_files_prefix+str(id)+'.txt'
            poses_file = pose_files_path+poses_files_prefix+str(id)+'.txt'
            bounding_boxes = load_sim_bounding_boxes(bb_file)
            bounding_boxes = downsample_list(bounding_boxes, downsample_rate)
            bounding_boxes = reorder_sim_bb(bounding_boxes)
            if use_width_height:
                bounding_boxes = bb_corners_to_angle(bounding_boxes)
            bounding_boxes = preprocess_inputs(bounding_boxes, use_width_height)

            poses = load_sim_poses(poses_file)
            poses = downsample_list(poses, downsample_rate)
            assert len(bounding_boxes) == len(poses), print(
                "Not as many bounding boxes as poses for id "+str(id))

            # Gather into blocks of sequences

            self.bounding_boxes.extend(
                list(chunks(bounding_boxes, sequence_length)))
            self.poses.extend(list(chunks(poses, sequence_length)))

        # if filter_pitch:
        #     (self.bounding_boxes, self.poses) = filter_data(
        #         self.bounding_boxes, self.poses, pitch_threshold)


        self.bounding_boxes = torch.FloatTensor(
            self.bounding_boxes).reshape(-1, input_size*sequence_length)
        self.poses = torch.FloatTensor(
            self.poses).reshape(-1, output_size*sequence_length)
    def __len__(self):
        return len(self.bounding_boxes)

    def __getitem__(self, idx):
        if torch.is_tensor(idx):
            idx = idx.tolist()
        return self.bounding_boxes[idx], self.poses[idx][-7:]


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
        self.dataset = datasets.MNIST(
            self.data_dir, train=training, download=True, transform=trsfm)
        print(self.dataset)
        super().__init__(self.dataset, batch_size, shuffle, validation_split, num_workers)


class SimBoundingBoxPitchDataLoader(BaseDataLoader):
    """
    MNIST data loading demo using BaseDataLoader
    """

    def __init__(self, bounding_boxes_path, pose_files_path, poses_files_prefix, bbs_files_prefix, batch_size, sequence_length, downsample_rate, filter_pitch, pitch_threshold, shuffle=True, validation_split=0.0, num_workers=1, training=True, use_width_height=True):
        self.dataset = SimBoundingBoxPitchDataset(
            bounding_boxes_path, pose_files_path, poses_files_prefix, bbs_files_prefix, sequence_length, downsample_rate, filter_pitch, pitch_threshold, use_width_height)
        print(self.dataset)
        super().__init__(self.dataset, batch_size, shuffle, validation_split, num_workers)


class RealBoundingBoxPitchDataset(Dataset):
    """Bounding Box - Pitch dataset."""

    def __init__(self, bounding_boxes_path, pose_files_path, poses_files_prefix, bbs_files_prefix, sequence_length, downsample_rate=1, filter_pitch=False, pitch_threshold=3.14159, img_height=480, img_width=640, use_width_height=True):
        output_size = 7
        if use_width_height:
            input_size = 5
        else:
            input_size = 8
        self.bounding_boxes = load_real_bounding_boxes(bounding_boxes_path)
        if use_width_height:
                self.bounding_boxes = bb_corners_to_angle(self.bounding_boxes)
        self.bounding_boxes = preprocess_inputs(self.bounding_boxes, use_width_height)

        self.poses = load_real_poses(pose_files_path, poses_files_prefix)

        # Gather into blocks of sequences

        self.bounding_boxes = list(
            chunks(self.bounding_boxes, sequence_length))
        self.poses = list(chunks(self.poses, sequence_length))
        self.bounding_boxes = torch.FloatTensor(
            self.bounding_boxes).reshape(-1, input_size*sequence_length)
        self.poses = torch.FloatTensor(
            self.poses).reshape(-1, output_size*sequence_length)
        

    def __len__(self):
        return len(self.bounding_boxes)

    def __getitem__(self, idx):
        if torch.is_tensor(idx):
            idx = idx.tolist()
        return self.bounding_boxes[idx], self.poses[idx][-7:]


class RealBoundingBoxPitchDataLoader(BaseDataLoader):
    """
    MNIST data loading demo using BaseDataLoader
    """

    def __init__(self, bounding_boxes_path, pose_files_path, poses_files_prefix, bbs_files_prefix, batch_size, sequence_length, shuffle=True, validation_split=0.0, num_workers=1, training=False, use_width_height=True):
        self.dataset = RealBoundingBoxPitchDataset(
            bounding_boxes_path, pose_files_path, poses_files_prefix, bbs_files_prefix, sequence_length, use_width_height)
        print(self.dataset)
        super().__init__(self.dataset, batch_size, shuffle, validation_split, num_workers)
