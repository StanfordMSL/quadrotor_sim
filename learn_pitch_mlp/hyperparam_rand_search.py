import argparse
from train import main
import torch
import collections
from parse_config import ConfigParser
import numpy as np
import random

# fix random seeds for reproducibility
SEED = 123
torch.manual_seed(SEED)
torch.backends.cudnn.deterministic = True
torch.backends.cudnn.benchmark = False
np.random.seed(SEED)

with open('hyperparam_search_results.txt', 'w') as f:
    f.write("# seq_len hidden_units levels kernel_size lr batch_size optimizer mse_deg std_deg val_mse_deg val_std_deg\n")

if __name__ == '__main__':
    args = argparse.ArgumentParser(description='PyTorch Template')
    args.add_argument('-c', '--config', default=None, type=str,
                      help='config file path (default: None)')
    args.add_argument('-r', '--resume', default=None, type=str,
                      help='path to latest checkpoint (default: None)')
    args.add_argument('-d', '--device', default=None, type=str,
                      help='indices of GPUs to enable (default: all)')

    # custom cli options to modify configuration from default values given in json file.
    CustomArgs = collections.namedtuple('CustomArgs', 'flags type target')
    options = [
        CustomArgs(['--lr', '--learning_rate'],
                   type=float, target='optimizer;args;lr'),
        CustomArgs(['--bs', '--batch_size'], type=int,
                   target='data_loader;args;batch_size')
    ]
    config = ConfigParser.from_args(args, options)

    seq_len = [15, 30, 60, 150,300]
    hidden_units = [5, 15, 30,50,100]
    levels = [1, 3,5,7,10]
    kernel_size = [3,5,7,10]
    lr = [0.1,0.01,0.001,0.0001,0.00001]
    batch_size = [128,64,32,16,8]
    optimizer = [0,1] # 0 for Adam, 1 for SGD

    nb_iters = 1000
    for i in range(nb_iters):
        seq_len_i = random.choice(seq_len)
        hidden_units_i = random.choice(hidden_units)
        levels_i = random.choice(levels)
        kernel_size_i = random.choice(kernel_size)
        batch_size_i = random.choice(batch_size)
        lr_i = random.choice(lr)
        optimizer_i = random.choice(optimizer)
        config['arch']['args']['sequence_length'] = seq_len_i
        config['arch']['args']['hidden_units_per_layer'] = hidden_units_i
        config['arch']['args']['levels'] = levels_i
        config['arch']['args']['kernel_size'] = kernel_size_i
        config['data_loader']['args']['batch_size'] = batch_size_i
        config['data_loader']['args']['sequence_length'] = seq_len_i
        config['optimizer']['args']['lr'] = lr_i
        if optimizer_i == 0:
            config['optimizer']['type'] = 'Adam'
        elif optimizer_i == 1:
            config['optimizer']['type'] = 'SGD'
        else:
            print('Unknown optimizer')
            exit

        # config['trainer']['save_dir'] = 'saved_seq'+str(seq_len_i)+'_h'+str(
        #     hidden_units_i)+'_lev'+str(levels_i)+'_k'+str(kernel_size_i)+'/'
        res = main(config)
        print(res)
        with open('hyperparam_rand_search_results.txt', 'a') as f:
            f.write(str(seq_len_i)+' '+str(hidden_units_i)+' '+str(levels_i)+' '+str(kernel_size_i)+' '+str(lr_i)+' '+str(batch_size_i)+' '+str(optimizer_i)+' '+str(
                res['mse_deg'].item())+' '+str(res['std_deg'].item())+' '+str(res['val_mse_deg'].item())+' '+str(res['val_std_deg'].item())+'\n')
