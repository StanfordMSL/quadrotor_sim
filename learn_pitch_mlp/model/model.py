import torch.nn as nn
import torch.nn.functional as F
from base import BaseModel
from torch.nn.utils import weight_norm


class Chomp1d(nn.Module):
    def __init__(self, chomp_size):
        super(Chomp1d, self).__init__()
        self.chomp_size = chomp_size

    def forward(self, x):
        return x[:, :, :-self.chomp_size].contiguous()


class TemporalBlock(nn.Module):
    def __init__(self, n_inputs, n_outputs, kernel_size, stride, dilation, padding, dropout=0.2):
        super(TemporalBlock, self).__init__()
        self.conv1 = weight_norm(nn.Conv1d(n_inputs, n_outputs, kernel_size,
                                           stride=stride, padding=padding, dilation=dilation))
        self.chomp1 = Chomp1d(padding)
        self.relu1 = nn.ReLU()
        self.dropout1 = nn.Dropout(dropout)

        self.conv2 = weight_norm(nn.Conv1d(n_outputs, n_outputs, kernel_size,
                                           stride=stride, padding=padding, dilation=dilation))
        self.chomp2 = Chomp1d(padding)
        self.relu2 = nn.ReLU()
        self.dropout2 = nn.Dropout(dropout)

        self.net = nn.Sequential(self.conv1, self.chomp1, self.relu1, self.dropout1,
                                 self.conv2, self.chomp2, self.relu2, self.dropout2)
        self.downsample = nn.Conv1d(
            n_inputs, n_outputs, 1) if n_inputs != n_outputs else None
        self.relu = nn.ReLU()
        self.init_weights()

    def init_weights(self):
        self.conv1.weight.data.normal_(0, 0.01)
        self.conv2.weight.data.normal_(0, 0.01)
        if self.downsample is not None:
            self.downsample.weight.data.normal_(0, 0.01)

    def forward(self, x):
        out = self.net(x)
        res = x if self.downsample is None else self.downsample(x)
        return self.relu(out + res)


class TemporalConvNet(nn.Module):
    def __init__(self, num_inputs, num_channels, kernel_size=2, dropout=0.2, dilation_factor = 2):
        super(TemporalConvNet, self).__init__()
        layers = []
        num_levels = len(num_channels)
        for i in range(num_levels):
            dilation_size = dilation_factor ** i
            in_channels = num_inputs if i == 0 else num_channels[i-1]
            out_channels = num_channels[i]
            layers += [TemporalBlock(in_channels, out_channels, kernel_size, stride=1, dilation=dilation_size,
                                     padding=(kernel_size-1) * dilation_size, dropout=dropout)]

        self.network = nn.Sequential(*layers)

    def forward(self, x):
        return self.network(x)


# class PitchModel(BaseModel):
#     def __init__(self, sequence_length=20):
#         super().__init__()
#         self.conv1 = nn.Conv1d(8,10,30,2,0,1)
#         self.fc1 = nn.Linear(136, 50)
#         self.fc2 = nn.Linear(50, 1)
#         self.sequence_length = sequence_length

#     def forward(self, x):
#         x = F.relu(self.conv1(x.reshape(-1,8,self.sequence_length)))
#         x = F.dropout(x, training=self.training)
#         x = F.relu(self.fc2(self.fc1(x)))
#         return x

class PitchModel(BaseModel):
    def __init__(self, sequence_length, hidden_units_per_layer, levels, kernel_size, dropout_value, dilation_factor):
        super().__init__()
        self.input_dim = 8 # 8 values to describe a bounding box
        self.channel_sizes = [hidden_units_per_layer] * levels
        self.tcn = TemporalConvNet(
            self.input_dim, self.channel_sizes, kernel_size, dropout_value, dilation_factor)
        self.linear = nn.Linear(self.channel_sizes[-1], sequence_length)
        self.sequence_length = sequence_length

    def forward(self, x):
        # input should have dimension (N, C, L)
        y1 = self.tcn(x.reshape(-1,  self.input_dim, self.sequence_length))
        o = self.linear(y1[:, :, -1])
        return o

