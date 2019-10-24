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
    inputs = inputs.to_device()
    targets = targets.to_device()
    rnn.to_device()




####### Training
criterion = nn.MSELoss()

# If you set this too high, it might explode. If too low, it might not learn
learning_rate = 0.005


def train(pitches, boxes):
    hidden = rnn.initHidden()

    rnn.zero_grad()
    loss = 0
    outputs_saved = []
    for i in range(boxes.size()[0]):
        output, hidden = rnn(boxes[i].unsqueeze(0), hidden)
        loss += criterion(output, pitches[i])
        outputs_saved.append(output)
    loss.backward()

    # Add parameters' gradients to their values, multiplied by learning rate
    for p in rnn.parameters():
        p.data.add_(-learning_rate, p.grad.data)

    return outputs_saved, loss.item()


n_iters = 100000
print_every = 5000
plot_every = 1000


# Keep track of losses for plotting
current_loss = 0
all_losses = []


def timeSince(since):
    now = time.time()
    s = now - since
    m = math.floor(s / 60)
    s -= m * 60
    return '%dm %ds' % (m, s)


start = time.time()
length_sequences = 20
for iter in range(1, n_iters + 1):
    first_rand_id = random.randint(0, len(targets)-length_sequences)
    random_ids = np.arange(first_rand_id, first_rand_id+length_sequences)
    (pitches_samples, boxes_samples) = (np.array(targets)[random_ids], np.array(inputs)[random_ids])
    pitches_samples = torch.FloatTensor(pitches_samples)
    boxes_samples = torch.FloatTensor(boxes_samples)
    output, loss = train(pitches_samples, boxes_samples)
    current_loss += loss

    # Print iter number, loss, name and guess
    if iter % print_every == 0:
        print('%d %d%% (%s) %.4f %s / %s' % (iter, iter / n_iters *
                                                100, timeSince(start), loss, pitches_samples[-1], output[-1] ))

    # Add current loss avg to list of losses
    if iter % plot_every == 0:
        all_losses.append(current_loss / plot_every)
        current_loss = 0


######################################################################
# Plotting the Results
# --------------------
#
# Plotting the historical loss from ``all_losses`` shows the network
# learning:
#


plt.figure()
plt.plot(all_losses)
