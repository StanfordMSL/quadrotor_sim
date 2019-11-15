import torch
import math

def mse_deg(output,target):
    output = output[:,-1]*180/math.pi
    target = target[:,-1]*180/math.pi
    # for i in range(len(output)):
    #     print(str(output[i]/180*math.pi)+' '+str(target[i]*math.pi/180))
    return torch.mean((output-target)*(output-target))

def std_deg(output,target):
    output = output[:, -1]*180/math.pi
    target = target[:, -1]*180/math.pi
    return torch.std(output-target)
