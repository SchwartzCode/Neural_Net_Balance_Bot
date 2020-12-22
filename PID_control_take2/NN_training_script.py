import numpy as np
import os

import torch
import torch.nn as nn

from NN_util_funcs import *
import torch.nn.functional as F

import random

use_cuda = torch.cuda.is_available()
# random unsigned 32 bit integer, cast to uint64 because manual_seed() wants 64 bits
#   note: it's fine that the front 32 bits will all be zeros, although could be good
#         to try expanding random seeds to fill all 64 bits
seed = np.uint64( random.randint(0,2147483647) )
torch.manual_seed(seed)
device = torch.device("cuda" if use_cuda else "cpu")

# fpath = "./logs/arduino_data_21_162141.log"
# fpath = "./logs/arduino_data_22_164251.log"
fpath = "./logs/arduino_data_22_172659.log"

print("Loading data from:\t", fpath)

# fileData = np.loadtxt(fpath)

fileData = open(fpath, "r")
dataBegun = False
preDataLineCount = 0

dataLen = len(fileData.readlines())


fileData = open(fpath, "r")

for index, line in enumerate(fileData):
    if dataBegun:
        # print(line[:-2], '=-=-=-=- ayo!')
        inputData = line.split(' ')
        inputData = [float(i) for i in inputData] # convert list of strings to floats
        numpyData = np.array(inputData) # convert list of doubles to numpy array


        network_input[(index-preDataLineCount-1)] = numpyData[:-1]
        network_output[index-preDataLineCount-1] = numpyData[-1]

    elif( len(line.split(' ')) == 9):
        dataBegun = True
        preDataLineCount = index

        # initialize arrays to store all data
        network_input = np.zeros((dataLen-preDataLineCount,4)) # Nx4 numpy array
        network_output = np.zeros(dataLen-preDataLineCount) # N length numpy array

train_input = network_input[:-1000,:]
fileData_input = network_input[-1000:,:]

train_output = network_output[:-1000]
fileData_output = network_output[-1000:]

print('Data all sorted!')

network = nn.Sequential(
    nn.Linear(4, 25),
    nn.ReLU(),
    nn.Linear(25, 10),
    nn.ReLU(),
    # nn.Linear(10, 4),
    # nn.ReLU(),
    nn.Linear(10, 1),
    nn.ReLU()
).double()





# HYPER-PARAMETERS
batch_size = 200
learning_rate = 15e-3
max_iters =500

# initialize arrays, split data into batches, prepare torch Optimizer
batches = get_random_batches(train_input,train_output,batch_size)
batch_num = len(batches)
epoch = max_iters
log_interval = 5

optimizer = torch.optim.Adam(network.parameters(), lr=learning_rate)
training_loss = np.zeros(max_iters)
training_acc = np.zeros(max_iters)

valid_x = torch.tensor(fileData_input)

# time to train
for ep in range(epoch):
    total_loss = 0
    counter = 0
    for xb,yb in batches:
        xb = torch.tensor(xb)

        yb_numpy = yb.copy()
        yb = torch.tensor(yb)

        y_pred = torch.flatten(network(xb))

        # loss = F.binary_cross_entropy(y_pred, yb)
        loss = F.mse_loss(y_pred, yb)
        total_loss += loss.item()

        loss_calc, acc = compute_loss_and_acc(yb_numpy, y_pred.detach().numpy())
        training_acc[ep] += acc / batch_num

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

    print(ep, 'Loss:', total_loss)
    print('\tAcc:', training_acc[ep])
    training_loss[ep] = total_loss

    # y_pred = torch.flatten(network( torch.tensor(fileData_input) ))
    # valid_loss, valid_acc = compute_loss_and_acc(fileData_output, y_pred.detach().numpy())
    #
    # print(ep, '\tValid Acc:\t', valid_acc)
    # print('\tValid Loss:\t', valid_loss, '\n')

print('\ndone training\n')
probs = network(valid_x)
probs_np = probs.detach().numpy()

y = torch.tensor( fileData_output )
fileData_est_output = network( torch.tensor(fileData_input) )

y_pred = torch.flatten(network( torch.tensor(fileData_input) ))
valid_loss, valid_acc = compute_loss_and_acc(fileData_output, y_pred.detach().numpy())

print(ep, '\tValid Acc:\t', valid_acc)
print('\tValid Loss:\t', valid_loss, '\n')

print("Seed Used:\t", seed)
