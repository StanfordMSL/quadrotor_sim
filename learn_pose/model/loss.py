import torch.nn.functional as F
import math

# See https://zpascal.net/cvpr2017/Kendall_Geometric_Loss_Functions_CVPR_2017_paper.pdf
def mse_loss(output,target):
    L_orientation = 4*F.mse_loss(output[:, 3:]/(output[:,3:].norm(dim=1).view(-1,1)), target[:, 3:])
    L_pos = 3*F.mse_loss(output[:, :3], target[:, :3])
    return L_pos+512*L_orientation
