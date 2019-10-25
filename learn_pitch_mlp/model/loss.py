import torch.nn.functional as F
import math

def nll_loss(output, target):
    return F.nll_loss(output, target)


def mse_loss_deg(output,target):
    output = output*180/math.pi
    target = target*180/math.pi
    return F.mse_loss(output,target)
