import os
import numpy as np
from pyquaternion import Quaternion

def load_inputs(input_path):
    input_file = open(input_path,'r')
    lines = input_file.readlines()
    inputs = [line.split()[2:] for line in lines]
    inputs = [[float(i) for i in line] for line in inputs]
    return inputs


def load_targets(target_path,file_name_prefix):
    pose_files = os.listdir(target_path)
    # pose_files.sort()
    ids = [int(pose_file.split('.')[0][len(file_name_prefix):]) for pose_file in pose_files]
    ids.sort()
    pitches = []
    for id in ids:
        filepath = target_path+file_name_prefix+str(id)+'.txt'
        pose_file = open(filepath,'r')
        q = Quaternion(pose_file.readlines()[0].split()[3:7])
        (y,p,r) = q.yaw_pitch_roll
        pitches.append(p)
    # input_file = open(input_path,'r')
    # lines = input_file.readlines()
    # inputs = [line.split()[2:] for line in lines]
    return pitches  # inputs


def preprocess_inputs(boxes):
    # Bring between 0 and 1
    img_height = 480
    img_width = 640
    np_boxes = np.array(boxes)
    x_pos = [0,2,4,6]
    y_pos = [1,3,5,7]
    np_boxes[:, x_pos] = np_boxes[:, x_pos]/img_width
    np_boxes[:, y_pos] = np_boxes[:, y_pos]/img_height

    return list(np_boxes)
