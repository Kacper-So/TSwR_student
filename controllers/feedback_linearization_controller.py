import numpy as np
from models.manipulator_model import ManiuplatorModel
from .controller import Controller


class FeedbackLinearizationController(Controller):
    def __init__(self, Tp):
        self.model = ManiuplatorModel(Tp)

    def calculate_control(self, x, q_r, q_r_dot, q_r_ddot):
        """
        Please implement the feedback linearization using self.model (which you have to implement also),
        robot state x and desired control v.
        """        
        v = q_r_ddot

        M = self.model.M(x)
        C = self.model.C(x)
        
        tau = M.dot(v) + C.dot(q_r_dot) 
        return tau
