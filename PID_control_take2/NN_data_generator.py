import numpy as np

class data_generator(object):

    def __init__(self):
        print('init')

    def gen_PID_data(self, num_points=1000):

        # values to generate
        # -

        # to train NN you need sensor inputs & corresponding output
