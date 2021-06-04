import numpy as np
import matplotlib.pyplot as plt
import control

import copy

class pendulum_bot(object):
    def __init__(self, initial_state=np.zeros((4,1))):
        # physics constants
        self.g = 9.81 # [m/s^2] acceleration due to gravity
        # constants for robot's physics (measured empirically)
        self.mass = 0.585 # [grams]
        self.wheel_rad = 0.033 # [m]
        self.CoM_dist = 0.045 # [m]  ditance from center of mass to wheel axis
        self.MoI = 0.00118462 #[kg*m^2]  moment of inertia about wheel axis
        self.friction_coefficient = 0.1 # [unitless] friction from wheel
                                                   # driving on floor

        # linearized state space matrices, derived by hand
        # where state vector = x and dx/dt =~ Ax * Bu
        self.A = np.zeros((4,4))
        self.A[0,1] = 1
        self.A[2,1] = 0.5
        self.A[2,3] = 1
        self.A[3,2] = self.mass * self.g * self.CoM_dist / self.MoI

        self.B = np.zeros((4,1))
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

        # calculate derivative of state
        state_dot = self.A @ self.state + self.B * input

        # check if robot has hit the floor (faceplant)
        if abs(self.state[2]) >= np.pi/2:
            state_dot[2] = 0  # angular position can't change
            state_dot[3] = 0  # angular velocity can't change
            self.state[3] = 0  # angular velocity is 0
        return state_dot

    def get_LQR_gains(self, Q, R):
        '''
        Get the gains needed to obtain performance outlined by Q and R
        Inputs:
            Q - [4x4 numpy array of floats]
                4x4 diagonal matrix specifying importance of keeping each state
                variable near its desired value (or quickness of convergence to
                the desired value)
            R - [float]
                specifies how conservative we want to be with applying inputs to
                the system (relative to Q matrix values)
        Returns:
            K - [4x1 numpy vector/array of floats]
                gains to control the system with, where:
                    u = -K(x - x_des) for state vector x
        '''
        K,S,E = control.lqr(self.A, self.B, Q, R)
        # S is solution to Riccatti equation
        # E is eigen values of closed loop system

        # reshape K to make dot products work
        # K = np.array(K).reshape(1,4)
        return K

    def update_state(self, dt, u):
        '''
        Calculate system dynamics forward in time
        Inputs:
            dt - [float]
                time step between current state and next state
            u - [float]
                input to be applied at current state
        Returns:
            None, although self.state is updated
        '''
        self.state += dt*self.state_deriv(u)

    def sim_dynamics(self, K, desired_state, timesteps=100, dt=0.01):
        '''
        Simulate robot's dynamics for a set period of time
        Inputs:
            K - [4x1 numpy vector of floats]
                gains with which inputs will be calculated
            desired_state - [4x1 numpy array of floats]
                state you want to drive the robot toward
            timesteps - [integer]
                number of times update_state will be called, simulation will
                cover timesteps*dt seconds
            dt - [float]
                time in seconds each time step will cover
        Returns:
            states - [timesteps x 4 numpy array]
                state history of robot over time
        '''
        self.dt = dt
        states = self.state

        for i in range(timesteps):
            u = -K @ (self.state - desired_state)
            self.update_state(dt, u)
            states = np.hstack([states, self.state])

        return states.T

    def plot_states(self, states):
        '''
        Plot the state history of the robot
        Inputs:
            states - [Nx4 numpy array of floats]
                state history of a simulation of the robot's dynamics
        Returns:
            None, but shows plots of robot's behavior
        '''

        t = np.arange(0,len(states)*self.dt, self.dt)

        fig, (ax1, ax2) = plt.subplots(2,1,figsize=(8,6))

        # makes space for titles and axis labels
        plt.subplots_adjust(left=0.2, hspace=0.5)

        ax1.set_title("Translational Position")
        ax1.set_ylabel("Position [m]")
        ax1.plot(t, states[:,0])

        ax2.set_title("Angular Position")
        ax2.set_xlabel("Time [sec]")
        ax2.set_ylabel("Position [rad]")
        ax2.plot(t,states[:,2])

        plt.show()



# ===========================================================
# driver script
if __name__ == '__main__':
    testeroni = pendulum_bot(initial_state=np.array([0.,0.01,0.,0.]).reshape(4,1))
    # define controller parameters and obtain gains matrix
    Q = np.diag([1,1,1,1])
    R = 1e5  # larger = system more inclined to drive motors
    K = testeroni.get_LQR_gains(Q, R)
    state_des = np.array([1.0,0.,0.,0.]).reshape(4,1)
    states = testeroni.sim_dynamics(K, state_des, timesteps=1000)
    testeroni.plot_states(states)
