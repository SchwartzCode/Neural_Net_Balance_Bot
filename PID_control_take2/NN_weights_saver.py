import os
import torch
import numpy as np

angle_net = torch.load('./good_angle_network')
# angle_net.eval()
print(angle_net)
print(angle_net.keys())

# ['0.weight', '0.bias', '2.weight', '2.bias']
w0 = angle_net['0.weight']
b0 = angle_net['0.bias']
w2 = angle_net['2.weight']
b2 = angle_net['2.bias']
print("woo")

np.savetxt('network_weights/w0.txt', w0.numpy())
np.savetxt('network_weights/b0.txt', b0.numpy())
np.savetxt('network_weights/w2.txt', w2.numpy())
np.savetxt('network_weights/b2.txt', b2.numpy())
