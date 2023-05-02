import numpy as np
from .controller import Controller
from models.manipulator_model import ManiuplatorModel

class MMAController(Controller):
    def __init__(self, Tp):
        # TODO: Fill the list self.models with 3 models of 2DOF manipulators with different m3 and r3
        # I:   m3=0.1,  r3=0.05
        # II:  m3=0.01, r3=0.01
        # III: m3=1.0,  r3=0.3
        Model_1 = ManiuplatorModel(Tp=Tp)
        Model_1.m3=0.1
        Model_1.r3=0.05
        Model_2 = ManiuplatorModel(Tp=Tp)
        Model_2.m3=0.01
        Model_2.r3=0.01
        Model_3 = ManiuplatorModel(Tp=Tp)
        Model_3.m3=1.
        Model_3.r3=0.3
        self.models = [Model_1, Model_2, Model_3]
        self.i = 0
        self.Kd = 10.
        self.Kp = 10.
        self.u = np.zeros((2, 1))

    def choose_model(self, x):
        # TODO: Implement procedure of choosing the best fitting model from self.models (by setting self.i)
        x_p = np.zeros((2, 3))
        e = float('inf')
        chosen_model_index = 0

        for j in range(len(self.models)):
            y = self.models[j].M(x) @ self.u + self.models[j].C(x) @ np.reshape(x[2:], (2, 1))
            x_p[0][j] = y[0]
            x_p[1][j] = y[1]
        
        for j in range(len(self.models)):
            new_e = np.sum(abs(x[:2] - x_p[:, j]))
            if e > new_e:
                e = new_e
                chosen_model_index = j

        self.i = chosen_model_index

    def calculate_control(self, x, q_r, q_r_dot, q_r_ddot):
        self.choose_model(x)
        q = x[:2]
        q_dot = x[2:]
        v = q_r_ddot - self.Kd * (q_dot - q_r_dot) - self.Kp * (q - q_r) # TODO: add feedback
        M = self.models[self.i].M(x)
        C = self.models[self.i].C(x)
        u = M @ v[:, np.newaxis] + C @ q_dot[:, np.newaxis]
        return u
