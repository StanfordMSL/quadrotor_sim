import torch.nn.functional as F
import math

def mse_loss_deg(output,target):
    output = output*180/math.pi
    target = target*180/math.pi
    return F.mse_loss(output,target)

def mse_loss(output,target):
    return F.mse_loss(output,target)

def l1_loss(output,target):
    return F.l1_loss(output, target)


def l1_loss_deg(output, target):
    output = output*180/math.pi
    target = target*180/math.pi
    return F.l1_loss(output, target)

def smooth_l1_loss(output, target):
    return F.smooth_l1_loss(output, target)


def smooth_l1_loss_deg(output, target):
    output = output*180/math.pi
    target = target*180/math.pi
    return F.smooth_l1_loss(output, target)
