import torch.nn.functional as F
import math

def mse_loss_deg(output,target):
    output = output*180/math.pi
    target = target*180/math.pi
    return F.mse_loss(output,target)
