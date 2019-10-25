import torch
import math

def accuracy(output, target):
    with torch.no_grad():
        pred = torch.argmax(output, dim=1)
        assert pred.shape[0] == len(target)
        correct = 0
        correct += torch.sum(pred == target).item()
    return correct / len(target)


def top_k_acc(output, target, k=3):
    with torch.no_grad():
        pred = torch.topk(output, k, dim=1)[1]
        assert pred.shape[0] == len(target)
        correct = 0
        for i in range(k):
            correct += torch.sum(pred[:, i] == target).item()
    return correct / len(target)

def mse_deg(output,target):
    output = output*180/math.pi
    target = target*180/math.pi
    return torch.mean((output-target)*(output-target))

def std_deg(output,target):
    output = output*180/math.pi
    target = target*180/math.pi
    return torch.std(output-target)
