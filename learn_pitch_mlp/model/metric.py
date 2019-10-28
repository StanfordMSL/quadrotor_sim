import torch
import math

def mse_deg(output,target):
    output = output*180/math.pi
    target = target*180/math.pi
    return torch.mean((output-target)*(output-target))

def std_deg(output,target):
    output = output*180/math.pi
    target = target*180/math.pi
    return torch.std(output-target)
