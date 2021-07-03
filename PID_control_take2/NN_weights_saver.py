import os
import torch
import numpy as np

angle_net = torch.load('./good_angle_network')
controller_net = torch.load('controller_network')

ang_w0 = angle_net['0.weight']
ang_b0 = angle_net['0.bias']
ang_w2 = angle_net['2.weight']
ang_b2 = angle_net['2.bias']

np.savetxt('network_weights/ang_w0.txt', ang_w0.numpy())
np.savetxt('network_weights/ang_b0.txt', ang_b0.numpy())
np.savetxt('network_weights/ang_w2.txt', ang_w2.numpy())
np.savetxt('network_weights/ang_b2.txt', ang_b2.numpy())

controller_w0 = angle_net['0.weight']
controller_b0 = angle_net['0.bias']
controller_w2 = angle_net['2.weight']
controller_b2 = angle_net['2.bias']

np.savetxt('network_weights/controller_w0.txt', controller_w0.numpy())
np.savetxt('network_weights/controller_b0.txt', controller_b0.numpy())
np.savetxt('network_weights/controller_w2.txt', controller_w2.numpy())
np.savetxt('network_weights/controller_b2.txt', controller_b2.numpy())
