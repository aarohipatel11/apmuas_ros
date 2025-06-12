import numpy as np
import matplotlib.pyplot as plt

import casadi as ca
from optitraj.models.casadi_model import CasadiModel
from optitraj.mpc.optimization import OptimalControlProblem
from optitraj.close_loop import CloseLoopSim
from optitraj.utils.report import Report
from optitraj.utils.data_container import MPCParams
from dataclasses import dataclass

from typing import List, Tuple, Dict


class Plane(CasadiModel):
    def __init__(self,
                 dt_val: float = 0.1,
                 airspeed_tau: float = 0.05,
                 pitch_tau: float = 0.02) -> None: 
        super().__init__()
        self.dt_val = dt_val
        self.airspeed_tau = airspeed_tau
        self.pitch_tau = pitch_tau
        self.define_states()
        self.define_controls()
        self.define_state_space()

    def define_states(self) -> None:
        pass

    def define_controls(self) -> None: 
        pass 

    def define_state_space(self) -> None:
        pass


@dataclass
class Obstacle:
    center: Tuple[float, float]
    radius: float


class PlaneOptControl(OptimalControlProblem):
    def __init__(self, 
                mpc_params: MPCParams,
                casadi_model: CasadiModel,
                obs_params: List[Obstacle]) -> None:
        super().__init__(mpc_params,
                        casadi_model)
        self.obs_params: List[Obstacle] = obs_params
        self.robot_radius: float = 3.0
        self.set_obstacle_avoidance_constraints()

    def set_obstacle_avoidance_constraints(self) -> None: 
        # set the avoidance contraints, in this case, the loiter points
        x_position = self.X[0, :]
        y_position = self.X[1, :]

        for i, obs in enumerate(self.obs_params):
            obs_center: Tuple[float] = ca.DM(obs.center)
            obs_radius: float = obs.radius
            distance = -ca.sqrt((x_position - obs_center[0])**2 +
                                (y_position - obs_center[1])**2)
            diff = distance + obs_radius + self.robot_radius
            self.g = ca.vertcat(self.g, diff[:-1].T)
