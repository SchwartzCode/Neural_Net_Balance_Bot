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

controller_network_path = './controller_network.pt'

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
batch_size = 500
learning_rate = 1e-3
max_iters = 1000

# initialize arrays, split data into batches, prepare torch Optimizer
batches = get_random_batches(train_in, train_out, batch_size)
batch_num = len(batches)
epochs = max_iters
log_interval = 5

# define network structure
controller_network = nn.Sequential(
    nn.Linear(4, 10),
    nn.ReLU(),
    nn.Linear(10, 1)
).double()

controller_network.load_state_dict(torch.load(controller_network_path))

optimizer = torch.optim.Adam(controller_network.parameters(), lr=learning_rate)
training_loss = np.zeros(max_iters)
training_acc = np.zeros(max_iters)

# time to train
for ep in range(epochs):
    total_loss = 0
    counter = 0
    for xb,yb in batches:

        xb = torch.tensor(xb)
        # xb = torch.reshape(xb, (len(xb),1
        yb_numpy = yb.copy()
        yb = torch.tensor(yb)

        y_pred = torch.flatten(controller_network(xb))

        # loss = F.binary_cross_entropy(y_pred, yb)
        loss = F.mse_loss(y_pred, yb)
        total_loss += loss.item()

        loss_calc, acc = compute_loss_and_acc(yb_numpy, y_pred.detach().numpy(),
                                acceptable_diff=10)
        training_acc[ep] += acc / batch_num

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

    if ep % 10 == 0:
        print(ep, 'Loss:', total_loss)
        print('\tAcc:', training_acc[ep])
    training_loss[ep] = total_loss

print("Done training")

torch.save(controller_network.state_dict(), controller_network_path)

plt.plot(np.arange(0,epochs), training_loss)
plt.xlabel("Epoch")
plt.ylabel("Training loss")
# plt.ylim(0.0, 0.1)
plt.show()

# estimate outputs for validation dataset
y_est = controller_network(torch.tensor(valid_in))
y_est = y_est.detach().numpy()

# plotting angle estimates vs actual
min_val = max(np.min(valid_out), np.min(y_est))
max_val = min(np.max(valid_out), np.max(y_est))

plt.scatter(valid_out, y_est)
plt.plot([min_val, max_val], [min_val, max_val], color='navajowhite', label='Ideal')
plt.xlabel("Actual PWM Output")
plt.ylabel("NN PWM Estimate")
plt.legend()
plt.show()

print("Seed used:", seed)
