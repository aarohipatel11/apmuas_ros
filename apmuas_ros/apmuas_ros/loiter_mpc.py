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
        """define the states of your system"""
        # positions ofrom world
        self.x_f = ca.MX.sym('x_f')
        self.y_f = ca.MX.sym('y_f')
        self.z_f = ca.MX.sym('z_f')

        # attitude
        self.phi_f = ca.MX.sym('phi_f')
        self.theta_f = ca.MX.sym('theta_f')
        self.psi_f = ca.MX.sym('psi_f')
        self.v = ca.MX.sym('v')

        self.states = ca.vertcat(
            self.x_f,
            self.y_f,
            self.z_f,
            self.phi_f,
            self.theta_f,
            self.psi_f,
            self.v
        )

        self.n_states = self.states.size()[0]  # is a column vector

    def define_controls(self) -> None: 
        """controls for your system"""
        self.u_phi = ca.MX.sym('u_phi')
        self.u_theta = ca.MX.sym('u_theta')
        self.u_psi = ca.MX.sym('u_psi')
        self.v_cmd = ca.MX.sym('v_cmd')

        self.controls = ca.vertcat(
            self.u_phi,
            self.u_theta,
            self.u_psi,
            self.v_cmd
        )
        self.n_controls = self.controls.size()[0] 

    def define_state_space(self) -> None:
        """define the state space of your system"""
        self.g = 9.81  # m/s^2
        # #body to inertia frame
        self.x_fdot = self.v_cmd * ca.cos(self.theta_f) * ca.cos(self.psi_f)
        self.y_fdot = self.v_cmd * ca.cos(self.theta_f) * ca.sin(self.psi_f)
        self.z_fdot = -self.v_cmd * ca.sin(self.theta_f)
        #self.z_fdot = self.dz

        self.phi_fdot = -self.u_phi * (1/self.pitch_tau) - self.phi_f
        self.theta_fdot = -self.u_theta * 1/0.5 - self.theta_f
        self.v_dot = ca.sqrt(self.x_fdot**2 + self.y_fdot **
                             2 + self.z_fdot**2)
        self.psi_fdot = self.u_psi

        self.z_dot = ca.vertcat(
            self.x_fdot,
            self.y_fdot,
            self.z_fdot,
            self.phi_fdot,
            self.theta_fdot,
            self.psi_fdot,
            self.v_dot
        )

        # ODE function
        name = 'dynamics'
        self.function = ca.Function(name,
                                    [self.states, self.controls],
                                    [self.z_dot])


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


    def solve(self, x0: np.ndarray, xF: np.ndarray, u0: np.ndarray) -> np.ndarray:
            """
            Solve the optimal control problem for the given initial state and control

            """
            state_init = ca.DM(x0)
            state_final = ca.DM(xF)

            X0 = ca.repmat(state_init, 1, self.N+1)
            U0 = ca.repmat(u0, 1, self.N)

            n_states = self.casadi_model.n_states
            n_controls = self.casadi_model.n_controls
            # self.compute_obstacle_avoidance_cost()

            # set the obstacle avoidance constraints
            num_obstacles = len(self.obs_params)  # + 1
            num_obstacle_constraints = num_obstacles * (self.N)
            # Constraints for lower and upp bound for state constraints
            # First handle state constraints
            lbg_states = ca.DM.zeros((n_states*(self.N+1), 1))
            ubg_states = ca.DM.zeros((n_states*(self.N+1), 1))

            # Now handle the obstacle avoidance constraints and add them at the bottom
            # Obstacles' lower bound constraints (-inf)
            # this is set up where -distance + radius <= 0
            lbg_obs = ca.DM.zeros((num_obstacle_constraints, 1))
            lbg_obs[:] = -ca.inf
            ubg_obs = ca.DM.zeros((num_obstacle_constraints, 1))
            # Concatenate state constraints and obstacle constraints (state constraints come first)
            # Concatenate state constraints and then obstacle constraints
            lbg = ca.vertcat(lbg_states, lbg_obs)
            ubg = ca.vertcat(ubg_states, ubg_obs)  # Same for the upper bounds

            args = {
                'lbg': lbg, #dynamic contraints and path constraints
                'ubg': ubg,
                'lbx': self.pack_variables_fn(**self.lbx)['flat'],#state and control contraints
                'ubx': self.pack_variables_fn(**self.ubx)['flat'],
            }
            args['p'] = ca.vertcat(
                state_init,    # current state
                state_final   # target state
            )

            args['x0'] = ca.vertcat(
                ca.reshape(X0, n_states*(self.N+1), 1),
                ca.reshape(U0, n_controls*self.N, 1)
            )
            # init_time = time.time()
            solution = self.solver(
                x0=args['x0'],
                lbx=args['lbx'],
                ubx=args['ubx'],
                lbg=args['lbg'],
                ubg=args['ubg'],
                p=args['p']
            )

            return solution
    
    
    def compute_dynamics_cost(self) -> ca.MX:
        """
        Compute the dynamics cost for the optimal control problem
        """
        # initialize the cost
        cost: float = 0.0
        Q = self.mpc_params.Q
        R = self.mpc_params.R

        x_final = self.P[self.casadi_model.n_states:]

        for k in range(self.N):
            states = self.X[:, k]
            controls = self.U[:, k]
            cost += cost \
                + (states - x_final).T @ Q @ (states - x_final) \
                + controls.T @ R @ controls

        return cost
    
    
    def compute_total_cost(self) -> ca.MX:
        cost = self.compute_dynamics_cost()
        return cost
    


def main() -> None: 
    #create an object of the plane class. This is the plane that'll be worked on
    plane: Plane = Plane()
    




if __name__ == '__main__':
    main()