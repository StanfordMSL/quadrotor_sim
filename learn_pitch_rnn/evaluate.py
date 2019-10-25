import matplotlib.ticker as ticker
import matplotlib.pyplot as plt
import math
import time
import torch
import torch.nn as nn
from data import load_inputs, load_targets, preprocess_inputs
from model import RNN
import random
import numpy as np

########### Loading
# Load data
target_path = '../adams_stuff/preprocessed_data/run4/results_2019-09-10-16-58-50/pose_gtboxes_and_time/'
pose_files_prefix = 'pose_gtboxes_and_time_'
input_path = '../adams_stuff/preprocessed_data/run4/results_2019-09-10-16-58-50/results_2019-09-10-16-58-50_angle.txt'

inputs = load_inputs(input_path)
inputs = preprocess_inputs(inputs)
targets = load_targets(target_path, pose_files_prefix)
# Load model
n_hidden = 64
rnn = RNN(len(inputs[0]), n_hidden, 1)

if torch.cuda.is_available():
    device = torch.device("cuda")
    rnn.to(device)
