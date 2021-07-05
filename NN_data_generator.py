import numpy as np
import math

class data_generator(object):

    def __init__(self, desired_state=np.zeros(6)):

        # state = [last_angle, y_accel, z_accel, x_gyro, encoder_ticks, last_encoder_ticks]
        self.state_des = desired_state
        self.state = np.zeros(6)
        self.angle = 0.0

        # constants
        self.dt = 0.0033
        self.alpha = 0.05  # for comp filter to estimate angle w gyro & accel data

    def gen_PID_data(self, num_points=1000):

        '''
        values to generate
         - last angle
         - y_accel
         - z_accel
         - x_gyro
         - encoder_ticks
         - wheel_vel (from encoder ticks)
        '''

        # -45 -> 45 [deg]
        last_angle_vals = np.random.normal(0.0, 0.2, num_points)
        # TODO: do these cover normal range of values?
        y_accel_vals = np.random.normal(0, 1.0, num_points)
        z_accel_vals = np.random.normal(0, 1.0, num_points)
        x_gyro_vals = np.random.normal(0, 1.0, num_points)
        encoder_vals = np.random.normal(0, 1000, num_points)
        last_encoder_vals = np.random.normal(0, 1000, num_points)

        inputs = np.vstack([last_angle_vals, y_accel_vals, z_accel_vals,
                            x_gyro_vals, encoder_vals, last_encoder_vals]).T

        angle_inputs = np.vstack([last_angle_vals, y_accel_vals, z_accel_vals,
                                  x_gyro_vals]).T

        # 0 will be replaced by angle prediction output from angle network
        controller_inputs = np.vstack([np.zeros(num_points), x_gyro_vals,
                                       encoder_vals, last_encoder_vals]).T

        outputs = []
        angles = []
        for i, datum in enumerate(inputs):
            output = self.calculate_PID_output(datum)
            controller_inputs[i,0] = self.angle
            angles.append(self.angle)
            outputs.append(output)

        return angle_inputs, controller_inputs, np.asarray(angles), np.asarray(outputs)


    def calculate_PID_output(self, input):
        '''
        calculate PWM output from sensor inputs
        Parameters
            input [6 value np array] - array of input values, takes form:
                    [last_angle, y_accel, z_accel, x_gyro, encoder_ticks, last_encoder_ticks]
        '''

        self.state = input

        return self.position_PID() + self.attitude_PID()

    '''
    =========================================================
          Functions to approximate Arduino controller
    =========================================================
    '''

    def update_angle_est(self):
        angle_accel = math.atan(self.state[1] / self.state[2])
        angle_gyro = self.state[0] + self.state[3] * self.dt

        self.angle = (1 - self.alpha) * angle_gyro + self.alpha * angle_accel


    def position_PID(self):
        # calculate PWM output desired to get to proper position. If pos drifts
        #   too far from desired pos, change vel setpoint from 0 to 500 ticks/sec
        #   in desired direction
        P = self.state_des[4] - self.state[4]
        D = 0

        if abs(P) > 250:
            wheel_vel = (self.state[4] - self.state[5]) / self.dt
            D = -(500 * np.sign(P) - wheel_vel)

        # clamp pos PWM to +/- 100
        if abs(D) > 100.0:
            D = np.sign(D) * 100.0

        return D

    def attitude_PID(self):
        # update self.angle based on self.state
        self.update_angle_est()

        D_error = -self.state[3]
        error = -(self.angle-0.03)

        P = -3000 * error
        D = -300 * D_error

        return P + D

'''
IDEAS
 - one network to estimate angle (RNN), and another to determine PMW output (MLP)
'''

if __name__ == '__main__':
    data_gen = data_generator()

    inputs, outputs = data_gen.gen_PID_data()

    print(inputs.shape, outputs.shape)
