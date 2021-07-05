from NN_util_funcs import *
import torch
import torch.nn as nn
import torch.nn.functional as F

import random

import numpy as np
import matplotlib.pyplot as plt

import os
os.environ['KMP_DUPLICATE_LIB_OK']='True'
os.environ["CUDA_VISIBLE_DEVICES"] = "0"

#
use_cuda = torch.cuda.is_available()
# random unsigned 32 bit integer, cast to uint64 because manual_seed() wants 64 bits
#   note: it's fine that the front 32 bits will all be zeros, although could be good
#         to try expanding random seeds to fill all 64 bits
seed = np.uint64( random.randint(0,2147483647) )
torch.manual_seed(seed)
device = torch.device("cuda" if use_cuda else "cpu")

# generate training & validation data
train_in = np.random.rand(10000)
train_in = train_in * np.pi/2 - 0.5 * np.pi/2
train_out = np.arctan(train_in)

valid_in = np.random.rand(500)
valid_in = valid_in * np.pi/2 - 0.5 * np.pi/2
valid_in.sort()
valid_out = np.arctan(valid_in)

# HYPER-PARAMETERS
batch_size = 250
learning_rate = 7e-4
max_iters = 100

# initialize arrays, split data into batches, prepare torch Optimizer
batches = get_random_batches(train_in,train_out,batch_size)
batch_num = len(batches)
epoch = max_iters
log_interval = 5

# define network structure
network = nn.Sequential(
    nn.Linear(1, 1)
).double()

optimizer = torch.optim.Adam(network.parameters(), lr=learning_rate)
training_loss = np.zeros(max_iters)
training_acc = np.zeros(max_iters)


# time to train
for ep in range(epoch):
    total_loss = 0
    counter = 0
    for xb,yb in batches:
        xb = torch.tensor(xb.reshape(len(xb),1))
        # xb = torch.reshape(xb, (len(xb),1))

        yb_numpy = yb.copy()
        yb = torch.tensor(yb)

        y_pred = torch.flatten(network(xb))

        # loss = F.binary_cross_entropy(y_pred, yb)
        loss = F.mse_loss(y_pred, yb)
        total_loss += loss.item()

        loss_calc, acc = compute_loss_and_acc(yb_numpy, y_pred.detach().numpy(),
                                acceptable_diff=0.1)
        training_acc[ep] += acc / batch_num

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

    print(ep, 'Loss:', total_loss)
    print('\tAcc:', training_acc[ep])
    training_loss[ep] = total_loss

print("Done training")

plt.plot(np.arange(0,epoch), training_acc)
plt.xlabel("Epoch")
plt.ylabel("Training accuracy")
plt.show()

# estimate validation data with network
x_valid = torch.tensor(valid_in.reshape(len(valid_in),1))
y_valid = torch.flatten(network(x_valid))

# sort data to plot
# sorted_train = sorted(zip(x_valid.detach().numpy(), y_valid.detach().numpy()))
# x_valid = [x for x, y in sorted_train]
# y_valid = [y for x, y in sorted_train]

plt.plot(valid_in, valid_out, label='Tangent Function')
plt.plot(x_valid.detach().numpy(), y_valid.detach().numpy(), color='orange', label="NN")
plt.xlabel("Input")
plt.ylabel("Output")
plt.legend()
plt.show()
