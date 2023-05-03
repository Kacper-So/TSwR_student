import numpy as np
from observers.eso import ESO
from .controller import Controller
from models.manipulator_model import ManiuplatorModel

class ADRCJointController(Controller):
    def __init__(self, b, kp, kd, p, q0, Tp):
        self.b = b
        self.kp = kp
        self.kd = kd
        self.model = ManiuplatorModel(Tp)

        A = np.array([[0, 1, 0], [0, 0, 1], [0, 0, 0]])
        B = np.array([[0], [b], [0]])
        L = np.array([[3 * p], [3 * np.power(p, 2)], [np.power(p, 3)]])
        W = np.array([[1, 0, 0]])
        self.eso = ESO(A, B, W, L, q0, Tp)

    def set_b(self, b):
        self.b = b
        B = np.array([[0], [b], [0]])
        self.eso.set_B(B)

    def calculate_control(self, x, q_d, q_d_dot, q_d_ddot, index):
        ### TODO implement ADRC
        est = self.eso.get_state()
        q_dot_est = est[1]
        f = 0
        e = q_d - x[0]
        e_dot = q_d_dot - q_dot_est
        v = q_d_ddot + self.kd * e_dot + self.kp * e
        u = (v - f) / self.b
        self.eso.update(x[0], u)
        x_copy = x
        x_copy.append(0.)
        x_copy.append(0.)
        M = self.model.M(x_copy)
        M = np.linalg.inv(M)
        self.set_b(M[index, index])
        print(u)
        return u