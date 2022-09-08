import casadi as ca
import numpy as np
# import time
from os import system

G = 9.8
w_max_xy =  6
w_max_yaw = 6

class QuadrotorMPC(object):

    def __init__(self, T, dt):
        # Time constant
        self._T = T
        self._dt = dt
        self._N = int(self._T / self._dt)

        # state dimension
        # [px, py, pz,       # quadrotor position
        #  vx, vy, vz,       # quadrotor linear velocity
        #  qw, qx, qy, qz]   # quadrotor quaternion
        self._x_dim = 10
        # control input dimension [az, wx, wy, wz]
        self._u_dim = 4

        # cost matrix for tracking the path point position
        self._Q_track_pos = np.diag([100, 100, 100])
        # cost matrix for tracking the path point velocity
        self._Q_track_vel = np.diag([10, 10, 10])
        # cost matrix for tracking the path point yaw
        self._Q_track_yaw = np.diag([20, 20])

        # cost matrix for the control input
        self._Q_u = np.diag([0.2, 0.6, 0.6, 0.6])

        self._quad_x0 = [0, 0, 0, 0, 0, 0, 1, 0, 0, 0]
        self._quad_u0 = [-G, 0, 0, 0]

        self._ipopt_options = {
            'verbose': False,
            'ipopt.tol': 1e-4,
            # 'ipopt.acceptable_tol': 1e-4,
            'ipopt.max_iter': 1000,
            'ipopt.warm_start_init_point': 'yes',
            'ipopt.print_level': 0,
            'print_time': False
        }
        self.nlp_x0 = []  # initial guess of nlp variables
        self.nlp_lbx = []  # lower bound of the variables, lbx <= nlp_x
        self.nlp_ubx = []  # upper bound of the variables, nlp_x <= ubx
        self.nlp_lbg = []  # lower bound of constraint functions, lbg <= g
        self.nlp_ubg = []  # upper bound of constraint functions, g <= lbg

        x_bound = ca.inf
        x_min = [-x_bound for _ in range(self._x_dim)]
        x_max = [+x_bound for _ in range(self._x_dim)]
        u_min = [-20, -w_max_xy, -w_max_xy, -w_max_yaw]
        u_max = [0, w_max_xy, w_max_xy, w_max_yaw]
        g_min = [0 for _ in range(self._x_dim)]
        g_max = [0 for _ in range(self._x_dim)]

        self.nlp_x0 += self._quad_x0
        self.nlp_lbx += x_min
        self.nlp_ubx += x_max
        self.nlp_lbg += g_min
        self.nlp_ubg += g_max
        for i in range(self._N):
            self.nlp_x0 += self._quad_u0
            self.nlp_x0 += self._quad_x0
            self.nlp_lbx += u_min
            self.nlp_ubx += u_max
            self.nlp_lbx += x_min
            self.nlp_ubx += x_max
            self.nlp_lbg += g_min
            self.nlp_ubg += g_max

        self.sol = []

    def define_solver_python(self):
        # states
        px, py, pz = ca.SX.sym('px'), ca.SX.sym('py'), ca.SX.sym('pz')
        vx, vy, vz = ca.SX.sym('vx'), ca.SX.sym('vy'), ca.SX.sym('vz')
        qw, qx, qy, qz = ca.SX.sym('qw'), ca.SX.sym('qx'), ca.SX.sym('qy'), ca.SX.sym('qz')
        # control input
        az, wx, wy, wz = ca.SX.sym('az'), ca.SX.sym('wx'), ca.SX.sym('wy'), ca.SX.sym('wz')

        self._x = ca.vertcat(px, py, pz,
                             vx, vy, vz,
                             qw, qx, qy, qz)
        self._u = ca.vertcat(az, wx, wy, wz)

        x_dot = ca.vertcat(
            vx,
            vy,
            vz,
            2 * (qw * qy + qx * qz) * az,
            2 * (qy * qz - qw * qx) * az,
            (qw * qw - qx * qx - qy * qy + qz * qz) * az + G,
            0.5 * (-wx * qx - wy * qy - wz * qz),
            0.5 * (wx * qw + wz * qy - wy * qz),
            0.5 * (wy * qw - wz * qx + wx * qz),
            0.5 * (wz * qw + wy * qx - wx * qy)
        )
        self.f = ca.Function('f', [self._x, self._u], [x_dot], ['x', 'u'], ['ode'])
        F = self.sys_dynamics(self._dt)
        fMap = F.map(self._N, 'openmp')

        #
        Delta_pos = ca.SX.sym('Delta_pos', 3)
        Delta_vel = ca.SX.sym('Delta_vel', 3)
        Delta_yaw = ca.SX.sym('Delta_yaw', 2)
        Delta_u = ca.SX.sym('Delta_u', self._u_dim)

        cost_pos = Delta_pos.T @ self._Q_track_pos @ Delta_pos
        cost_vel = Delta_vel.T @ self._Q_track_vel @ Delta_vel
        cost_yaw = Delta_yaw.T @ self._Q_track_yaw @ Delta_yaw
        cost_u = Delta_u.T @ self._Q_u @ Delta_u
        f_cost_pos = ca.Function('cost_pos', [Delta_pos], [cost_pos])
        f_cost_vel = ca.Function('cost_vel', [Delta_vel], [cost_vel])
        f_cost_yaw = ca.Function('cost_yaw', [Delta_yaw], [cost_yaw])
        f_cost_u = ca.Function('cost_u', [Delta_u], [cost_u])

        self.nlp_x = []  # nlp variables

        self.mpc_obj = 0  # objective
        self.nlp_g = []  # constraint functions
        

        P = ca.SX.sym('P', self._x_dim + (7+3)*self._N)
        X = ca.SX.sym('X', self._x_dim, self._N + 1)
        U = ca.SX.sym('U', self._u_dim, self._N)
        X_next = fMap(X[:, :self._N], U)

        # starting point
        self.nlp_x += [X[:, 0]]
        
        self.nlp_g += [X[:, 0] - P[0: self._x_dim]]
        

        for k in range(self._N):
            # add control input variables
            self.nlp_x += [U[:, k]]            

            p_index = self._x_dim + (7+3)*k
            delta_pos_k = (X[:3, k + 1] - P[p_index: p_index + 3])
            delta_vel_k = (X[3:6, k + 1] - P[p_index + 3: p_index + 6])
            
            yaw_k = P[p_index+6]
            c_yaw_k = ca.cos(yaw_k/2)
            s_yaw_k = ca.sin(yaw_k/2)
            _qw = X[6, k+1]
            _qz = X[9, k+1]
            _q_sqrt = ca.sqrt(_qw*_qw + _qz*_qz)
            delta_yaw_k = ca.vertcat(_qw-c_yaw_k*_q_sqrt, _qz-s_yaw_k*_q_sqrt)

            w_pos_k = P[p_index + 7]
            w_vel_k = P[p_index + 8]
            w_yaw_k = P[p_index + 9]

            cost_track_k = f_cost_pos(delta_pos_k) * w_pos_k + f_cost_vel(delta_vel_k) * w_vel_k + f_cost_yaw(delta_yaw_k) * w_yaw_k

            delta_u_k = U[:, k] - [-G, 0, 0, 0]
            cost_u_k = f_cost_u(delta_u_k)

            self.mpc_obj = self.mpc_obj + cost_track_k + cost_u_k
            #
            self.nlp_x += [X[:, k + 1]]

            self.nlp_g += [X_next[:, k] - X[:, k + 1]]

        nlp_dict = {
            'f': self.mpc_obj,
            'x': ca.vertcat(*self.nlp_x),
            'p': P,
            'g': ca.vertcat(*self.nlp_g)
        }

        self.solver = ca.nlpsol('solver', 'ipopt', nlp_dict, self._ipopt_options)

    def generate_solver_c(self, cname):
        self.solver.generate_dependencies(cname)
        ## system('gcc -fPIC -shared -O3' + cname + ' -o ' + './casadi_gen/mpc.so')


    def load_solver(self, path):
        self.solver = ca.nlpsol('solver', 'ipopt', path, self._ipopt_options)


    # def solve(self, path):

    #     self.sol = self.solver(
    #         x0=self.nlp_x0,
    #         lbx=self.nlp_lbx,
    #         ubx=self.nlp_ubx,
    #         p=path,
    #         lbg=self.nlp_lbg,
    #         ubg=self.nlp_ubg
    #     )

    #     sol_x0 = self.sol['x'].full()
    #     opt_u = sol_x0[self._x_dim: self._x_dim + self._u_dim]

    #     # self.nlp_x0 = sol_x0
    #     self.nlp_x0 = list(sol_x0[self._x_dim + self._u_dim: 2 * (self._x_dim + self._u_dim)]) + \
    #                   list(sol_x0[self._x_dim + self._u_dim:])

    #     x0_array = np.reshape(sol_x0[:-self._x_dim], newshape=(-1, self._x_dim + self._u_dim))

    #     return opt_u, x0_array

    def sys_dynamics(self, dt):
        M = 4
        DT = dt / M
        X0 = ca.SX.sym('X', self._x_dim)
        U = ca.SX.sym('U', self._u_dim)

        X = X0
        for _ in range(M):
            k1 = DT * self.f(X, U)
            k2 = DT * self.f(X + 0.5 * k1, U)
            k3 = DT * self.f(X + 0.5 * k2, U)
            k4 = DT * self.f(X + k3, U)
            X = X + (k1 + 2 * k2 + 2 * k3 + k4) / 6

        F = ca.Function('F', [X0, U], [X])
        return F


if __name__ == "__main__":
    mpc = QuadrotorMPC(0.5, 0.1)
    mpc.define_solver_python()
    mpc.generate_solver_c("mpc_T05_dt01.c")
