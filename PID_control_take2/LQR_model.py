import numpy as np

class pendulum_bot(object):
    def __init__(self, initial_state=np.zeros(4)):
        # physics constants
        self.g = 9.81 # [m/s^2] acceleration due to gravity
        # constants for robot's physics (measured empirically)
        self.mass = 0.585 # [grams]
        self.wheel_rad = 0.033 # [m]
        self.CoM_dist = 0.045 # [m]  ditance from center of mass to wheel axis
        self.MoI = 0.00118462 #[kg*m^2]  moment of inertia about wheel axis

        # linearized state space matrices, derived by hand
        # where state vector = x and dx/dt =~ Ax * Bu
        self.A = np.zeros((4,4))
        self.A[0,1] = 1
        self.A[2,3] = 1
        self.A[3,2] = -self.mass * self.g * self.CoM_dist / self.MoI

        self.B = np.zeros(4)
        self.B[1] = 1 / (self.mass * self.wheel_rad)
        self.B[3] = 1 / self.MoI

        # initialize state vector (with provided values if given)
        self.state = initial_state

    def state_deriv(self, input):
        # input is torque applied to wheels in [Nm]
        # max possible torque is ~0.2kgf.cm (from part specs: https://www.dfrobot.com/product-1457.html)
        #   1 kgf = 9.81 N, so 0.2kgf.cm = 0.01962 Nm
        if input > 0.01962:
            input = 0.01962
        return self.A @ self.state + self.B * input

testeroni = pendulum_bot()
print(testeroni.A)
print(testeroni.state_deriv(10))
