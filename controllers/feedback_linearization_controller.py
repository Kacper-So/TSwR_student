import numpy as np
from models.manipulator_model import ManiuplatorModel
from .controller import Controller


class FeedbackLinearizationController(Controller):
    def __init__(self, Tp):
        self.model = ManiuplatorModel(Tp)
        self.Kd = 10.0
        self.Kp = 10.0

    def calculate_control(self, x, q_r, q_r_dot, q_r_ddot):
        """
        Please implement the feedback linearization using self.model (which you have to implement also),
        robot state x and desired control v.
        """        
        M = self.model.M(x)
        C = self.model.C(x)
        q = x[:2].T
        q_dot = x[2:].T
        q_r_ddot = q_r_ddot.T
        q_r_dot = q_r_dot.T
        q_r = q_r.T

        v = q_r_ddot - self.Kd * (q_dot - q_r_dot) - self.Kp * (q - q_r)
        
        tau = M.dot(v) + C.dot(q_r_dot) 
        return tau
