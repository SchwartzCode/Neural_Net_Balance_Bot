from NN_util_funcs import *
from NN_data_generator import data_generator
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
# seed = np.uint64(1810914062)
torch.manual_seed(seed)
device = torch.device("cuda" if use_cuda else "cpu")

# generate training & validation data
PID_data_gen = data_generator()
angle_train_in, train_in, angle_train_out, train_out \
    = PID_data_gen.gen_PID_data(num_points=20000)
angle_valid_in, valid_in, angle_valid_out, valid_out \
    = PID_data_gen.gen_PID_data(num_points=1000)

# HYPER-PARAMETERS
batch_size = 1000
learning_rate = 5e-3
max_iters = 400
# initialize arrays, split data into batches, prepare torch Optimizer
angle_batches = get_random_batches(angle_train_in, angle_train_out, batch_size)
batch_num = len(angle_batches)
epoch = max_iters
log_interval = 5

# define network structure
network = nn.Sequential(
    nn.Linear(4, 4),
    nn.ReLU(),
    nn.Linear(4, 1)
).double()


optimizer = torch.optim.Adam(network.parameters(), lr=learning_rate)
training_loss = np.zeros(max_iters)
training_acc = np.zeros(max_iters)

# time to train
for ep in range(epoch):
    total_loss = 0
    counter = 0
    for xb,yb in angle_batches:
    # for i in range(angle_train_out.size):
    #     xb = angle_train_in[i]
    #     yb = np.array([ angle_train_out[i] ])

        xb = torch.tensor(xb)
        # xb = torch.reshape(xb, (len(xb),1
        yb_numpy = yb.copy()
        yb = torch.tensor(yb)

        y_pred = torch.flatten(network(xb))


        # loss = F.binary_cross_entropy(y_pred, yb)
        loss = F.mse_loss(y_pred, yb)
        total_loss += loss.item()

        loss_calc, acc = compute_loss_and_acc(yb_numpy, y_pred.detach().numpy(),
                                acceptable_diff=0.02)
        training_acc[ep] += acc / batch_num

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

    if ep % 10 == 0:
        print(ep, 'Loss:', total_loss)
        print('\tAcc:', training_acc[ep])
    training_loss[ep] = total_loss

print("Done training")

plt.plot(np.arange(0,epoch), training_loss)
plt.xlabel("Epoch")
plt.ylabel("Training loss")
plt.ylim(0.0, 0.1)
plt.show()

# estimate outputs for validation dataset
y_est = network(torch.tensor(angle_valid_in))
y_est = y_est.detach().numpy()

# min_val = max(np.min(valid_out), np.min(y_est))
# max_val = min(np.max(valid_out), np.max(y_est))
#
# plt.scatter(valid_out, y_est)
# plt.plot([min_val, max_val], [min_val, max_val], color='navajowhite', label='Ideal')
# plt.xlabel("Actual PWM Output")
# plt.ylabel("NN PWM Estimate")
# plt.legend()
# plt.show()

# plotting angle estimates vs actual
min_val = max(np.min(angle_valid_out), np.min(y_est))
max_val = min(np.max(angle_valid_out), np.max(y_est))

plt.scatter(angle_valid_out, y_est)
plt.plot([min_val, max_val], [min_val, max_val], color='navajowhite', label='Ideal')
plt.xlabel("Actual Angle")
plt.ylabel("NN Angle Estimate")
plt.title("Angle output")
plt.legend()
plt.show()

print("Seed used:", seed)
