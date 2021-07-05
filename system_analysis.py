import numpy as np
from matplotlib import pyplot as plt

g = 9.81 #m/s^2
mass = 1 #kg

theta = np.linspace(-0.5, 0.5, num=51)

F = g*mass*np.sin(theta)
test  = np.sin(theta)
#plt.plot(theta, test)
squareRoot = np.sqrt(theta)
squared = theta**2

plt.plot(theta, squareRoot, label='Root')
plt.plot(theta, 5*squared, label='Squared')
plt.plot(theta, theta)
plt.legend()
plt.xlabel('Theta [rad]')
plt.ylabel('Perpendicular Force [N]')
plt.show()
