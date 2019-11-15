import torch
import math
from pyquaternion import quaternion

def mse_quat(output,target):
    output = output[:, 3:]/(output[:,3:].norm(dim=1).view(-1, 1))
    target = target[:, 3:]
    return torch.mean(torch.sum((output-target)**2,axis=1))


def mse_pos(output,target):
    output = output[:, :3]
    target = target[:, :3]
    return torch.mean(torch.sum((output-target)**2, axis=1))
