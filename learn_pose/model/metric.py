import torch
import math
from pyquaternion import quaternion

def mse_quat(output,target):
    output = output[:, 3:]/(output[:,3:].norm(dim=1).view(-1, 1))
    target = target[:, 3:]
    return torch.mean((output-target)*(output-target))


def mse_pos(output,target):
    output = output[:, :3]
    target = target[:, :3]
    return torch.mean((output-target)*(output-target))
