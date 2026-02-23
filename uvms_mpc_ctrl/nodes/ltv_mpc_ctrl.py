import numpy as np
import casadi as ca

import utils_sym
import utils_math
import time

import bluerov_dynamics_symbolic as sym_brv
import manipulator_kinematics_symbolic as sym_manip_kin
import manipulator_dynamics_symbolic as sym_manip_dyn
from thruster_b2_inversepoly import ThrusterInversePoly



def unit_quat(q):
    # q: 4x1 MX or SX
    n = ca.norm_2(q)
    n_safe = n + 1e-8
    return q / n_safe


class CFTOCSolver:
    def __init__(
        self,
        *,
        dt: float,
        solver: str,
        solver_opts: dict,
        weights: dict,
        cost_scaling: dict,
        n_horizon: int,
        v_bat: float,
        bluerov_params: dict,
        manipulator_dh_params: np.ndarray,
        alpha_params: dict,
        path_thruster_model_params: str,
        joint_pos_lim: list,
        joint_vel_lim: list,
        tank_bounds: dict, 
        thrust_limits: float,
        mpc_limits: dict,
        ctrl_pwm: bool,
    ):
        self.dt = float(dt)
        self.weights = dict(weights)
        self.cost_scaling = dict(cost_scaling)
        self.n_horizon = int(n_horizon)
        self.v_bat = float(v_bat)
        self.ctrl_pwm = bool(ctrl_pwm)

        self.joint_pos_lim = joint_pos_lim
        self.joint_vel_lim = joint_vel_lim
        self.tank_bounds = tank_bounds
        self.thrust_limits = thrust_limits
        self.mpc_limits = mpc_limits

        self.eef_pose = None 
        self.algebraic_constraint = None
        self.lin_disc = None
        self.h_linear = None
        self.thruster_model = None

        self.mixer = None

        self.n_joints = None
        self.n_dof = None
        self.state_dim = None
        self.ctrl_dim = None

        self.init_model(
            bluerov_params=bluerov_params,
            manipulator_dh_params=manipulator_dh_params,
            alpha_params=alpha_params,
            path_thruster_model_params=path_thruster_model_params,
        )

        self.handles = self.build_ocp_template(solver=solver, solver_opts=solver_opts, weights=self.weights)

        # store the warm start values
        self.x_warm = np.zeros((self.state_dim + self.n_joints, self.n_horizon + 1), dtype=float)
        self.u_warm = np.zeros((self.ctrl_dim, self.n_horizon), dtype=float)
        self.a_warm = np.zeros((self.n_dof, self.n_horizon), dtype=float)
        self.lam_g_warm = None
        self.first_run = True
        self.test = False

    def get_ctrl_cmd(self, x0, ref_eef_pos, ref_eef_att):
        # augment the measurement state
        x_measured = np.asarray(x0, dtype=float).copy()
        if self.first_run:
            # Fill ALL stages with the measured state (measured part only)
            self.x_warm[0:self.state_dim, :] = x_measured.reshape(-1, 1)
            self.first_run = False

        if self.test:
            x_star = self.x_warm[:, 0].copy() # MPC will run with its own prediction
        else:
            x_star = np.concatenate([x_measured, self.x_warm[self.state_dim:self.state_dim + self.n_joints, 1].copy()])



        u_star = self.u_warm[:, 0].copy()
        a_star = self.a_warm[:, 0].copy()
        a_guess = a_star.copy()
        for _ in range(3):
            a_guess = self.algebraic_constraint(x_star, u_star, a_guess, np.zeros((3, 1)), np.zeros((3, 1)))
        a_star = np.array(a_guess.full()).reshape(-1).astype(float)

        # warm-starts for next iteration
        X_guess = np.hstack([self.x_warm[:, 1:], self.x_warm[:, -1][:, None]])
        X_guess[:, 0] = x_star

        U_guess = np.hstack([self.u_warm[:, 1:], self.u_warm[:, -1][:, None]]) # der Arbeitspunkt ist der aktuell gültige Wert, im MPC ist dann aber schon der nächste Schritt

        A_guess = np.hstack([self.a_warm[:, 1:], self.a_warm[:, -1][:, None]])
        A_guess[:, 0] = a_star


        # solve OCP
        X_opt, U_opt, A_opt, J_opt, lam_g_last, constraint_flags, solve_time = self.solve_cftoc(
            U_guess=U_guess,
            X_guess=X_guess,
            A_guess=A_guess,
            x_star=x_star,
            u_star=u_star,
            a_star=a_star,
            f_eef_val = np.zeros((3, 1)),
            l_eef_val = np.zeros((3, 1)),
            ref_eef_pos=ref_eef_pos,
            ref_eef_att=ref_eef_att,
            lam_g_prev=self.lam_g_warm
        )

        if X_opt is None:
            return None, None, constraint_flags, None, solve_time
        
        q_pos_pred = X_opt[0:self.n_joints, :]
        veh_lin_vel_pred = X_opt[self.n_joints: self.n_joints + 3, :]
        veh_ang_vel_pred = X_opt[self.n_joints + 3: self.n_joints + self.n_dof, :]
        veh_pos_pred = X_opt[self.n_joints + self.n_dof: self.n_joints + self.n_dof + 3, :]
        veh_att_pred = X_opt[self.n_joints + self.n_dof + 3: self.n_joints + self.n_dof + 7, :]

        # store solution for warm start in next iteration
        self.x_warm = X_opt
        self.u_warm = U_opt
        self.a_warm = A_opt
        self.lam_g_warm = lam_g_last

        # reverse augmentation
        uv_apply = U_opt[self.n_joints:, 0].copy()
        uq_apply = X_opt[self.state_dim:self.state_dim + self.n_joints, 1].copy()

        # thrust adaption
        uv_adapted = self.thruster_model.mpc_thruster_command_adaption(uv_apply, self.v_bat) # returns adapted PWM commands [1100, 1900]
        

        if self.ctrl_pwm:
            # return 8 normalized commands for ActuatorControls
            uv_apply = self._to_np(uv_adapted)  # in [1100..1900]
            uv_norm = (uv_apply - 1500.0) / 400.0
            return self._to_np(uq_apply), uv_norm.astype(float), constraint_flags, float(J_opt), solve_time, q_pos_pred, veh_lin_vel_pred, veh_ang_vel_pred, veh_pos_pred, veh_att_pred

        else:
            uv_forces = self.thruster_model.pwm_to_force(uv_adapted, self.v_bat)
            uv_forces = self._to_np(uv_forces).reshape(-1)              # (8,)
            mixer_np = self._to_np(self.mixer)                           # (6,8)

            wrench6 = mixer_np @ uv_forces                          # (6,)
            thrust3 = wrench6[0:3]
            torque3 = wrench6[3:6]

            return self._to_np(uq_apply), (thrust3, torque3), constraint_flags, float(J_opt), solve_time, q_pos_pred, veh_lin_vel_pred, veh_ang_vel_pred, veh_pos_pred, veh_att_pred
    
    def _to_np(self, x):
        # CasADi DM/MX -> numpy, numpy -> numpy
        if hasattr(x, "full"):
            return np.array(x.full()).squeeze()
        return np.asarray(x).squeeze()



    def init_model(self, bluerov_params, manipulator_dh_params, alpha_params, path_thruster_model_params):
        bluerov_dyn    = sym_brv.BlueROVDynamicsSymbolic(bluerov_params)
        manip_kin      = sym_manip_kin.KinematicsSymbolic(manipulator_dh_params)
        manip_dyn      = sym_manip_dyn.DynamicsSymbolic(manip_kin, alpha_params)

        c_fun = bluerov_dyn.C
        d_fun = bluerov_dyn.D
        g_fun = bluerov_dyn.g_quat
        j_fun = bluerov_dyn.J_quat

        l_gain = bluerov_dyn.L
        self.mixer = bluerov_dyn.mixer
        m_inv = bluerov_dyn.M_inv
        # MASS = self.bluerov_dyn.M  

        self.n_joints  = manip_dyn.kinematics_.n_joints
        self.n_dof = bluerov_dyn.M_inv.size1() 

        self.state_dim = self.n_dof + 7 + self.n_joints
        self.ctrl_dim = self.n_joints + 8

        tau_coupling = self._build_tau_coupling_func(manipulator_dh_params, alpha_params)
        dyn_fossen = self._build_dynamics_fossen_func(self.mixer, l_gain, self.v_bat, m_inv, c_fun, d_fun, g_fun)
        self.eef_pose = self._build_eef_blocks(manipulator_dh_params, manip_dyn)

        self.algebraic_constraint = self._build_algebraic_constraint(tau_coupling, dyn_fossen)
        
        f_sys_augment = self._build_f_sys_augmented(j_fun, tau_coupling, dyn_fossen, self.eef_pose)

        step_augment = self._build_step_func_augmented(f_sys_augment)

        self.lin_disc = self._build_discrte_linearized_step_func(step_augment)

        self.eef_output_lin_disc = self._build_eef_output_func_lin_dicrete(self.eef_pose)

        h_non_linear = self._build_h_constraints_nonlinear_xu_fun(self.eef_pose)
        self.h_linear = self._build_h_constraints_linearized_fun(h_non_linear)

        self.thruster_model = ThrusterInversePoly.load(path_thruster_model_params)

    def build_ocp_template(self, solver: str, solver_opts: dict, weights: dict):
        opti = ca.Opti()

        # decision variables
        X = []
        U = []
        A = []
        for _ in range(self.n_horizon):
            X.append(opti.variable(self.state_dim + self.n_joints))
            U.append(opti.variable(self.ctrl_dim))
            A.append(opti.variable(self.n_dof))
        X.append(opti.variable(self.state_dim + self.n_joints))  # terminal state

        # eef pose reference trajectory
        ref_eef_pos   = opti.parameter(3, self.n_horizon)
        ref_eef_att   = opti.parameter(4, self.n_horizon)

        # # cost function weights
        # w_eef_pos_run = opti.parameter(3)
        # w_eef_att_run = opti.parameter(3)

        # w_u  = opti.parameter(self.ctrl_dim)

        # w_manip_joint0 = opti.parameter()

        w_eef_pos_run = weights["eef_pos_run"]
        w_eef_att_run = weights["eef_att_run"]
        # w_u_joint = weights["w_u_joint"]
        # w_u_thruster = weights["w_u_thruster"]
        w_u = np.concatenate((
            weights["w_u_joint"],
            np.ones(self.ctrl_dim - self.n_joints) * weights["w_u_thruster"],))
        w_manip_joint0 = weights["w_manip_joint0"]

        # operating point for linearization
        x_star = opti.parameter(self.state_dim + self.n_joints)
        u_star = opti.parameter(self.ctrl_dim)
        a_star = opti.parameter(self.n_dof)

        # linearized state equation matrices and affine term
        Ad   = opti.parameter(self.state_dim + self.n_joints, self.state_dim + self.n_joints)
        Bd_u = opti.parameter(self.state_dim + self.n_joints, self.ctrl_dim)
        Bd_a = opti.parameter(self.state_dim + self.n_joints, self.n_dof)
        b_param = opti.parameter(self.state_dim + self.n_joints)

        # linearized algebraic constraint matrices
        Gx   = opti.parameter(self.n_dof, self.state_dim + self.n_joints)
        Gu   = opti.parameter(self.n_dof, self.ctrl_dim)
        Ga   = opti.parameter(self.n_dof, self.n_dof)

        # linearized output equation matrix and affine term
        Cd = opti.parameter(7, self.state_dim + self.n_joints)
        y_eef_star = opti.parameter(7)

        # linearized nonlinear constraints matrices and nominal values
        # full constraint set
        h_nom = opti.parameter(46) 
        Hx = opti.parameter( h_nom.size1(), self.state_dim + self.n_joints)
        Hu = opti.parameter( h_nom.size1(), self.ctrl_dim)

        # constraint set without hard state constraints at first steps
        h_nom_flags = opti.parameter(h_nom.size1())  # number of nonlinear constraints
        Hx_flags = opti.parameter(h_nom.size1(), self.state_dim + self.n_joints)
        Hu_flags = opti.parameter(h_nom.size1(), self.ctrl_dim)
        
        # initial condition
        opti.subject_to(X[0] == x_star)

        # initialize cost function
        cost = 0

        for k in range(self.n_horizon):
            # decision variables at step k
            xk = X[k]
            xkp1 = X[k + 1]
            uk = U[k]
            ak = A[k]

            # deviations
            delta_xk = xk - x_star
            delta_uk = uk - u_star
            delta_ak = ak - a_star

            # dynamics
            delta_x_next = Ad @ delta_xk + Bd_u @ delta_uk + Bd_a @ delta_ak
            opti.subject_to(xkp1 == b_param + delta_x_next)

            # algebraic constraint
            opti.subject_to(Gx @ delta_xk + Gu @ delta_uk + Ga @ delta_ak == 0)

            # output equation for eef pose
            y_eef_k = y_eef_star + Cd @ delta_xk
            p_eef = y_eef_k[0:3]
            q_eef = y_eef_k[3:7]

            # nonlinear constraints
            if k <3: #k < 2: # disable hard constraints on state to avoid infeasibility when x0 violates constraints
                h_k = Hx_flags @ delta_xk + Hu_flags @ delta_uk + h_nom_flags
            else:
                h_k = Hx @ delta_xk + Hu @ delta_uk + h_nom

            opti.subject_to(h_k <= 0)

            # --------------------- #
            # cost function terms
            # --------------------- #

            # vehcile tilt
            v_nu = xk[self.n_joints:self.n_joints + self.n_dof]  # bluerov body velocity
            v_lin_vel = v_nu[0:3]
            v_ang_vel = v_nu[3:6]
            v_eta = xk[self.n_joints+self.n_dof: self.n_joints+self.n_dof +7]          # bluerov pose
            v_pos = v_eta[0:3]
            v_att = v_eta[3:7]

            qw = v_att[0]
            qx = v_att[1]
            qy = v_att[2]
            qz = v_att[3]

            tilt_ref_deg = 5.0
            tilt_ref = ca.sin(ca.pi * tilt_ref_deg / 360.0)**2  # sin^2(25°/2)

            tilt_norm = (qx**2 + qy**2) / tilt_ref
            w_tilt = 1.0 

            cost += w_tilt * tilt_norm

            # vehicle velocities
            v_lin_vel_norm = v_lin_vel / 0.3
            cost += v_lin_vel_norm.T @ v_lin_vel_norm

            v_ang_vel_norm = v_ang_vel / 0.5
            cost += v_ang_vel_norm.T @ v_ang_vel_norm

            # ak_q = uk[:(self.n_joints-1)] / 1000.0

            # aq0 = ak_q[0]

            # cost = ak_q.T @ ak_q



            # eef pose tracking
            pos_eef_err_run = ref_eef_pos[:, k] - p_eef
            pos_eef_err_run_norm = pos_eef_err_run / ca.DM(self.cost_scaling["eef_pos"])

            r0 = ref_eef_att[:, k][0]
            rvec = ref_eef_att[:, k][1:4]
            q0 = q_eef[0]
            qvec = q_eef[1:4]
            e = r0 * qvec - q0 * rvec - utils_sym.skew(rvec) @ qvec
            q_eef_err_run_norm = e / ca.DM(self.cost_scaling["eef_att"])

            cost += pos_eef_err_run_norm.T @ ca.diag(w_eef_pos_run) @ pos_eef_err_run_norm
            cost += q_eef_err_run_norm.T @ ca.diag(w_eef_att_run) @ q_eef_err_run_norm

            # control effort
            uk_q = xk[self.state_dim:self.state_dim + self.n_joints]               # joint velocity commands
            uk_q_norm = uk_q / ca.DM(self.cost_scaling["control_effort_joint"])

            uk_v = uk[self.n_joints:]                                    # thruster PWM commands
            uk_v_norm = uk_v / ca.DM(self.cost_scaling["control_effort_thruster"])

            uk_norm = ca.vertcat(uk_q_norm, uk_v_norm)
            
            cost += uk_norm.T @ ca.diag(w_u) @ uk_norm

            # manipulator infront of vehicle
            dev_joint0 = xk[0] - np.pi

            cost += w_manip_joint0 * (dev_joint0 / ca.DM(self.cost_scaling["manip_joint0"]))**2

            dev_joint2 = xk[2] - np.pi/2
            cost += w_manip_joint0 * (dev_joint2 / ca.DM(self.cost_scaling["manip_joint0"]))**2

 
        # finalize opti
        opti.minimize(cost)
        opti.solver(solver, solver_opts)

        # pack decision variables as matrices
        X = ca.hcat(X)
        U = ca.hcat(U)
        A = ca.hcat(A)

        return {
            "opti": opti,
            "X": X,
            "U": U,
            "A": A,
            "ref_eef_pos": ref_eef_pos,
            "ref_eef_att": ref_eef_att,
            # "w_eef_pos_run": w_eef_pos_run,
            # "w_eef_att_run": w_eef_att_run,
            # "w_u": w_u,
            # "w_manip_joint0": w_manip_joint0,
            "x_star": x_star,
            "u_star": u_star,
            "a_star": a_star,
            "Ad": Ad,
            "Bd_u": Bd_u,
            "Bd_a": Bd_a,
            "b_param": b_param,
            "Gx": Gx,
            "Gu": Gu,
            "Ga": Ga,
            "Cd": Cd,
            "y_eef_star": y_eef_star,
            "h_nom": h_nom,
            "Hx": Hx,
            "Hu": Hu,
            "h_nom_flags": h_nom_flags,
            "Hx_flags": Hx_flags,
            "Hu_flags": Hu_flags,
        }

    def solve_cftoc(self,
        U_guess: np.ndarray,
        X_guess: np.ndarray,
        A_guess: np.ndarray,
        x_star: np.ndarray,
        u_star: np.ndarray,
        a_star: np.ndarray,
        f_eef_val: np.ndarray,
        l_eef_val: np.ndarray,
        ref_eef_pos: np.ndarray,
        ref_eef_att: np.ndarray,
        lam_g_prev: np.ndarray,
        ):
        tA = time.perf_counter()

        # unpack optimization handles

        opti = self.handles["opti"]
        X    = self.handles["X"]
        U    = self.handles["U"]
        A    = self.handles["A"]

        # # ---------------------- #
        # # weights
        # # ---------------------- #

        # def safe_set(name, val):
        #     if name in self.handles:
        #         opti.set_value(self.handles[name], val)

        # # eef pose weights

        # safe_set("w_eef_pos_run", self.weights["eef_pos_run"])
        # safe_set("w_eef_att_run", self.weights["eef_att_run"])

        # # control effort weights

        # w_u_vec = np.concatenate((
        #     self.weights["w_u_joint"],
        #     np.ones(self.ctrl_dim - self.n_joints) * self.weights["w_u_thruster"],
        # ))
        # safe_set("w_u", w_u_vec)

        # # maniulator infront of vehicle weight

        # safe_set("w_manip_joint0", self.weights["w_manip_joint0"])


        # ---------------------- #
        # warm start 
        # hier ggf nochmal erst die algebraische constraint aufrufen, mit der ich A_GUESS berechne für konsistenz
        # ---------------------- #
        t0 = time.perf_counter()

        U_guess = U_guess.copy()
        X_guess = X_guess.copy()

        # Thrusters: clip to allowed PWM band
        U_guess[self.n_joints:, :] = np.clip(
            U_guess[self.n_joints:, :],
            -self.thrust_limits + self.mpc_limits["thruster_pwm_inflate"],
            self.thrust_limits - self.mpc_limits["thruster_pwm_inflate"]
        )

        # Joint velocities in augmented state:
        min_vel = -self.joint_vel_lim[:self.n_joints].reshape(-1, 1)
        max_vel =  self.joint_vel_lim[:self.n_joints].reshape(-1, 1)
        X_guess[self.state_dim:self.state_dim + self.n_joints, :] = np.clip(
            X_guess[self.state_dim:self.state_dim + self.n_joints, :],
            min_vel,
            max_vel
        )

        # warm starts for decision variables
        t1 = time.perf_counter()

        opti.set_initial(U, U_guess)
        opti.set_initial(X, X_guess)
        opti.set_initial(A, A_guess)

        # Warm start duals if available
        if lam_g_prev is not None:
            opti.set_initial(opti.lam_g, lam_g_prev) 

        t2 = time.perf_counter()


        # ---------------------- #
        # set parameters
        # ---------------------- #

        # eef references (running + terminal)
        
        opti.set_value(self.handles["ref_eef_pos"], ref_eef_pos)
        opti.set_value(self.handles["ref_eef_att"], ref_eef_att)

        # operating point for linearization

        opti.set_value(self.handles["x_star"], x_star)
        opti.set_value(self.handles["u_star"], u_star)
        opti.set_value(self.handles["a_star"], a_star)

        t3 = time.perf_counter()


        # ---------------------- #
        # linearization at (x_star, u_star, a_star)
        # ---------------------- #

        # state equation + algebraic constraint
        t4 = time.perf_counter()

        x_next, Fx, Fu, Fa, Gx, Gu, Ga = self.lin_disc(x_star, u_star, a_star, self.dt, f_eef_val, l_eef_val)

        t5 = time.perf_counter()


        opti.set_value(self.handles["Ad"], Fx)
        opti.set_value(self.handles["Bd_u"], Fu)
        opti.set_value(self.handles["Bd_a"], Fa)
        opti.set_value(self.handles["b_param"], x_next)

        opti.set_value(self.handles["Gx"], Gx)
        opti.set_value(self.handles["Gu"], Gu)
        opti.set_value(self.handles["Ga"], Ga)

        # output equation
        t6 = time.perf_counter()

        y_eef_star, Cd = self.eef_output_lin_disc(x_star)
        t7 = time.perf_counter()

        opti.set_value(self.handles["y_eef_star"], y_eef_star)
        opti.set_value(self.handles["Cd"], Cd)

        # constraints
        t8 = time.perf_counter()

        flags = self._compute_activation_flags(x_star)
        t9 = time.perf_counter()

        t10 = time.perf_counter()

        h_nom, Hx, Hu = self.h_linear(x_star, u_star, 1.0, 1.0, 1.0, 1.0, 1.0)
        t11 = time.perf_counter()
        t12 = time.perf_counter()

        h_nom_flags, Hx_flags, Hu_flags = self.h_linear(x_star, u_star, flags["active_joint"], flags["active_eef_selfcollision"],
                        flags["active_elbow_selfcollision"], flags["active_eef_col"], flags["active_vehicle_col"])
        t13 = time.perf_counter()
        opti.set_value(self.handles["h_nom"], h_nom)
        opti.set_value(self.handles["Hx"], Hx)
        opti.set_value(self.handles["Hu"], Hu)

        opti.set_value(self.handles["h_nom_flags"], h_nom_flags)
        opti.set_value(self.handles["Hx_flags"], Hx_flags)
        opti.set_value(self.handles["Hu_flags"], Hu_flags)
        

        # ---------------------- #
        # solve OCP
        # ---------------------- #

        try:
            t14 = time.perf_counter()

            sol = opti.solve()
            t15 = time.perf_counter()

            t16 = time.perf_counter()

            Xv = sol.value(X)
            Uv = sol.value(U)
            Av = sol.value(A)
            Jv = float(sol.value(opti.f))
            lamg = sol.value(opti.lam_g)
            t17 = time.perf_counter()
            print(
            f"prep={(t1-t0)*1e3:6.2f}ms | init={(t2-t1)*1e3:6.2f}ms | "
            f"params={(t3-t2)*1e3:6.2f}ms | lin={(t5-t4)*1e3:6.2f}ms | "
            f"eef={(t7-t6)*1e3:6.2f}ms | flags={(t9-t8)*1e3:6.2f}ms | "
            f"h_all={(t11-t10)*1e3:6.2f}ms | h_flags={(t13-t12)*1e3:6.2f}ms | "
            f"solve={(t15-t14)*1e3:6.2f}ms | extract={(t17-t16)*1e3:6.2f}ms"
            )
            solve_time = t17-t16
            return Xv, Uv, Av, Jv, lamg, flags, solve_time
        
        except RuntimeError as e:
            Xv = None
            Uv = None
            Av = None
            Jv = np.nan
            lamg = None
            solve_time = -1.0
            return Xv, Uv, Av, Jv, lamg, flags, solve_time

    def _build_discrte_linearized_step_func(self, step_fun: ca.Function) -> ca.Function:
        x = ca.MX.sym('x', self.state_dim + self.n_joints)
        u = ca.MX.sym('u', self.ctrl_dim)
        a = ca.MX.sym('a', self.n_dof)
        dt = ca.MX.sym('dt')
        f_eef = ca.MX.sym('f_eef', 3)
        l_eef = ca.MX.sym('l_eef', 3)

        x_next, res, _, _ = step_fun(dt, x, u, a, f_eef, l_eef)
        Fx = ca.jacobian(x_next, x)
        Fu = ca.jacobian(x_next, u)
        Fa = ca.jacobian(x_next, a)

        Gx = ca.jacobian(res, x)
        Gu = ca.jacobian(res, u)
        Ga = ca.jacobian(res, a)

        return ca.Function("lin_disc_map", [x, u, a, dt, f_eef, l_eef], [x_next, Fx, Fu, Fa, Gx, Gu, Ga]).expand()
    
    def _build_step_func_augmented(self, f_sys: ca.Function) -> ca.Function:
        dt     = ca.MX.sym('dt')
        x      = ca.MX.sym('x', self.state_dim + self.n_joints)
        u      = ca.MX.sym('u', self.ctrl_dim)
        a      = ca.MX.sym('a', self.n_dof)
        f_eef  = ca.MX.sym('f_eef', 3)
        l_eef  = ca.MX.sym('l_eef', 3)

        def normalize_quat(eta_vec):
            pos = eta_vec[0:3]; q = eta_vec[3:]
            q_u = unit_quat(q)
            return ca.vertcat(pos, q_u)

        k1, res, P1, A1 = f_sys(x, u, a, f_eef, l_eef)
        x_next = x + dt * k1

        x_next = ca.vertcat(x_next[0:self.n_joints+self.n_dof],
                            normalize_quat(x_next[self.n_joints+self.n_dof:self.n_joints+self.n_dof+7]),
                            x_next[self.state_dim:])

        return ca.Function('step', [dt, x, u, a, f_eef, l_eef],
                        [x_next, res, P1, A1])#.expand()
    
    def _build_f_sys_augmented(
            self,
            j_fun,
            tau_coupling: ca.Function,
            dyn_fossen: ca.Function,
            eef_pose: ca.Function,
        ) -> ca.Function:
        x      = ca.MX.sym('x', self.state_dim + self.n_joints)
        u      = ca.MX.sym('u', self.ctrl_dim)
        a      = ca.MX.sym('a', self.n_dof)
        f_eef  = ca.MX.sym('f_eef', 3)
        l_eef  = ca.MX.sym('l_eef', 3)

        eta = x[self.n_joints+self.n_dof:self.n_joints+self.n_dof+7]
        pos = eta[0:3]
        quat_eta   = unit_quat(eta[3:])
        eta_used = ca.vertcat(pos, quat_eta)

        q   = x[0:self.n_joints]
        nu  = x[self.n_joints:self.n_joints+self.n_dof]
        uq  = x[self.state_dim:self.state_dim+self.n_joints]
        aq  = u[0:self.n_joints]
        uv  = u[self.n_joints:]

        quat  = eta_used[3:]
        v_ref  = nu[0:3]
        w_ref  = nu[3:6]
        a_ref  = a[0:3]
        dw_ref = a[3:6]

        tau_c = tau_coupling(q, uq, aq, v_ref, a_ref, w_ref, dw_ref, quat, f_eef, l_eef)
        dnu_dyn = dyn_fossen(eta_used, nu, uv, tau_c)

        # explicit ODE xdot(x,u,a)
        dq   = uq
        deta = j_fun(eta_used) @ nu
        xdot = ca.vertcat(dq, a, deta, aq)

        # residual for implicit dynamics
        res = a - dnu_dyn

        # EEF pose + Jacobian
        p_eef, att_eef = eef_pose(eta_used, q)

        return ca.Function('f_sys', [x, u, a, f_eef, l_eef],
                        [xdot, res, p_eef, att_eef])# .expand()

    def _build_tau_coupling_func(self, manip_params, alpha_params) -> ca.Function:
        q = ca.MX.sym('q', self.n_joints)
        dq = ca.MX.sym('dq', self.n_joints)
        ddq = ca.MX.sym('ddq', self.n_joints)
        v_ref = ca.MX.sym('v_ref', 3)
        a_ref = ca.MX.sym('a_ref', 3)
        w_ref = ca.MX.sym('w_ref', 3)
        dw_ref = ca.MX.sym('dw_ref', 3)
        quaternion_ref = ca.MX.sym('quat_ref', 4)
        f_eef = ca.MX.sym('f_eef', 3)
        l_eef = ca.MX.sym('l_eef', 3)

        kin = sym_manip_kin.KinematicsSymbolic(manip_params)
        dyn = sym_manip_dyn.DynamicsSymbolic(kin, alpha_params)

        dyn.kinematics_.update(q)

        tau = dyn.rnem_symbolic(q, dq, ddq, v_ref, a_ref, w_ref, dw_ref, quaternion_ref, f_eef, l_eef)
        return ca.Function(
            'rnem_func',
            [q, dq, ddq, v_ref, a_ref, w_ref, dw_ref, quaternion_ref, f_eef, l_eef],
            [tau]
        )# .expand()
    
    def _build_dynamics_fossen_func(self, mixer, l_gain, v_bat, m_inv, c_fun, d_fun, g_fun) -> ca.Function:
        nu   = ca.MX.sym('nu', self.n_dof)
        eta  = ca.MX.sym('eta', 7)
        uv   = ca.MX.sym('uv', 8)
        tau_c = ca.MX.sym('tau_c', self.n_dof)

        pos = eta[0:3]
        q   = unit_quat(eta[3:])
        eta_used = ca.vertcat(pos, q)

        tau_v = (l_gain * v_bat * (mixer @ uv))
        dnu  = m_inv @ (tau_v + tau_c - c_fun(nu) @ nu - d_fun(nu) @ nu - g_fun(eta_used))
        return ca.Function('dyn_fossen', [eta,nu,uv,tau_c], [dnu])# .expand()

    def _build_eef_blocks(self,manip_params, manip_dyn) -> ca.Function:
        eta = ca.MX.sym('eta', 7)
        q   = ca.MX.sym('q', self.n_joints)

        pos = eta[0:3]
        quat   = unit_quat(eta[3:])
        eta_used = ca.vertcat(pos, quat)

        kin = sym_manip_kin.KinematicsSymbolic(manip_params)
        kin.update(q)

        # R_I_B = utils_sym.rotation_matrix_from_quat(eta_used[3:])
        Q_I_B = eta_used[3:]

        r_B_0, Q_B_0 = manip_dyn.tf_vec, manip_dyn.Q_reference # Quaternion, das zur fixen Matrix R_B_0 gehört
        r_0_eef      = kin.get_eef_position()
        att_0_eef    = kin.get_eef_attitude()

        # p_eef   = eta_used[0:3] + R_I_B @ r_B_0 + R_I_B @ R_B_0 @ r_0_eef
        p_eef = eta_used[0:3] + utils_sym.quat_rotate_fast(Q_I_B, r_B_0) + utils_sym.quat_rotate_fast(utils_sym.quat_mult(Q_I_B, Q_B_0), r_0_eef)
        att_eef = utils_sym.quat_mult(eta_used[3:], utils_sym.quat_mult(Q_B_0, att_0_eef))

        f_pose = ca.Function('eef_pose',  [eta, q], [p_eef, att_eef])# .expand()
        return f_pose

    def _build_eef_output_func_lin_dicrete(self, eef_fun: ca.Function) -> ca.Function:
        x      = ca.MX.sym('x', self.state_dim + self.n_joints)


        p_eef, att_eef = eef_fun(x[self.n_joints+self.n_dof:self.n_joints+self.n_dof+7], x[0:self.n_joints])

        y_eef = ca.vertcat(p_eef, att_eef)

        Cd = ca.jacobian(y_eef, x)

        return ca.Function('eef_output_lin_disc', 
                        [x], 
                        [y_eef, Cd]).expand()

    def _build_h_constraints_linearized_fun(
            self,
            h_nonlinear: ca.Function,
            ) -> ca.Function:
        """Build a CasADi Function for the linearized inequality constraints."""
        x = ca.MX.sym('x', self.state_dim + self.n_joints)
        u = ca.MX.sym('u', self.ctrl_dim)
        active_joint = ca.MX.sym('joint')
        active_eef_selfcollision = ca.MX.sym('eef_selfcollision')
        active_elbow_selfcollision = ca.MX.sym('elbow_selfcollision')
        active_eef_col = ca.MX.sym('eef_col')
        active_vehicle_col = ca.MX.sym('vehicle_col')

        h_nom = h_nonlinear(x, u, 
            active_joint,
            active_eef_selfcollision,
            active_elbow_selfcollision,
            active_eef_col,
            active_vehicle_col,
        )
        Hx = ca.jacobian( h_nom, x)
        Hu = ca.jacobian( h_nom, u)

        return ca.Function(
            'h_constraints_linearized',
            [x, u, active_joint, active_eef_selfcollision, active_elbow_selfcollision, active_eef_col, active_vehicle_col],
            [h_nom, Hx, Hu]
        ).expand()
    
    def _build_h_constraints_nonlinear_xu_fun(self, eef_pose: ca.Function) -> ca.Function:
        """Build a CasADi Function h(x,u) for the nonlinear inequality constraints."""
        x = ca.MX.sym('x', self.state_dim + self.n_joints)
        u = ca.MX.sym('u', self.ctrl_dim)
        active_joint = ca.MX.sym('joint')
        active_eef_selfcollision = ca.MX.sym('eef_selfcollision')
        active_elbow_selfcollision = ca.MX.sym('elbow_selfcollision')
        active_eef_col = ca.MX.sym('eef_col')
        active_vehicle_col = ca.MX.sym('vehicle_col')

        h = self._h_constraints_nonlinear_xu(x, u, eef_pose, 
            active_joint,
            active_eef_selfcollision,
            active_elbow_selfcollision,
            active_eef_col,
            active_vehicle_col,
        )

        return ca.Function('h_constraints_xu', [x, u, active_joint, active_eef_selfcollision, active_elbow_selfcollision, active_eef_col, active_vehicle_col], [h])

    def _h_constraints_nonlinear_xu(
            self,
            xk: ca.MX,
            uk: ca.MX,
            eef_pose: ca.Function,
            active_joint: ca.MX,
            active_eef_selfcollision: ca.MX,
            active_elbow_selfcollision: ca.MX,
            active_eef_col: ca.MX,
            active_vehicle_col: ca.MX,
        ) -> ca.MX:
        """Return stacked nonlinear inequality constraints h(x,u) <= 0.

        Differences to `h_constraints_nonlinear`:
        - Does NOT take `y_eef_k` as input.
        - Computes the EEF pose internally via the global CasADi function `EEF_POSE`.

        Notes
        -----
        - Two-sided bounds are written as two inequalities.
        - Activation flags are implemented as multiplication (active * h <= 0).
        - Expects the *augmented* state layout used in this file:
            x = [q(0..N_JOINTS-1), nu, eta(7), uq_cmd(0..N_JOINTS-1)]
        """

        h_list = []

        # --- unpack commonly used quantities ---
        qk = xk[:self.n_joints]
        eta_k = xk[self.n_joints + self.n_dof : self.n_joints + self.n_dof + 7]

        # EEF pose from nonlinear kinematics
        p_eef, q_eef = eef_pose(eta_k, qk)

        # Vehicle position from state (as used in your constraints)
        p_vehicle = xk[self.n_joints + self.n_dof : self.n_joints + self.n_dof + 3]

        # Controls
        uk_q = xk[self.state_dim : self.state_dim + self.n_joints]  # joint velocity commands (augmented state)
        uk_v = uk[self.n_joints:]                         # thruster PWM commands

        # ------------------------------------------------------------------
        # 1) Actuator bounds
        # ------------------------------------------------------------------
        thr_max = self.thrust_limits - self.mpc_limits["thruster_pwm_inflate"]
        # uk_v <=  thr_max  ->  uk_v - thr_max <= 0
        h_list.append(uk_v - thr_max)
        # uk_v >= -thr_max  -> -uk_v - thr_max <= 0
        h_list.append(-uk_v - thr_max)

        # Joint velocity bounds (physical)
        jv = ca.DM(self.joint_vel_lim[:self.n_joints]) * self.mpc_limits["joint_velocity_inflate"]
        h_list.append(uk_q - jv)     # uk_q <= jv
        h_list.append(-uk_q - jv)    # uk_q >= -jv

        # ------------------------------------------------------------------
        # 2) Joint position limits with margin (optional)
        # ------------------------------------------------------------------

        # margin = 10% of range
        lower = ca.DM(self.joint_pos_lim[0, :self.n_joints])
        upper = ca.DM(self.joint_pos_lim[1, :self.n_joints])
        margin = 0.1 * (upper - lower)
        lower_m = lower + margin
        upper_m = upper - margin

        # qk <= upper_m  -> qk - upper_m <= 0
        # qk >= lower_m  -> lower_m - qk <= 0
        h_list.append(active_joint * (qk - upper_m))
        h_list.append(active_joint * (lower_m - qk))

        # ------------------------------------------------------------------
        # 3) Self-collision and tank bounds (optional)
        # ------------------------------------------------------------------

        # --- vehicle - eef distance: dist_sq >= dmin^2 -> dmin^2 - dist_sq <= 0
        dmin_eef = self.mpc_limits["self_collision_vehicle_eef_min_dist"]
        dist_vehicle_eef_sq = (p_vehicle - p_eef).T @ (p_vehicle - p_eef)
        h_list.append(active_eef_selfcollision * (dmin_eef**2 - dist_vehicle_eef_sq))

        # --- vehicle - elbow distance
        p_unit = ca.DM([1.0, 0.0, 0.0])
        p_unit_rot = utils_sym.quaternion_rotation(q_eef, p_unit)
        p_elbow = p_eef - p_unit_rot * self.mpc_limits["elbow_offset"]

        dmin_elbow = self.mpc_limits["self_collision_vehicle_elbow_min_dist"]
        dist_vehicle_elbow_sq = (p_vehicle - p_elbow).T @ (p_vehicle - p_elbow)
        h_list.append(active_elbow_selfcollision * (dmin_elbow**2 - dist_vehicle_elbow_sq))

        # --- tank bounds for EEF and vehicle
        eef_min = ca.DM(self.tank_bounds["eef_min"])
        eef_max = ca.DM(self.tank_bounds["eef_max"])
        veh_min = ca.DM(self.tank_bounds["vehicle_min"])
        veh_max = ca.DM(self.tank_bounds["vehicle_max"])

        # p >= min -> min - p <= 0
        # p <= max -> p - max <= 0
        h_list.append(active_eef_col * (eef_min - p_eef))
        h_list.append(active_eef_col * (p_eef - eef_max))

        h_list.append(active_vehicle_col * (veh_min - p_vehicle))
        h_list.append(active_vehicle_col * (p_vehicle - veh_max))

        # Stack all constraints into one vector
        return ca.vertcat(*h_list)

    def _compute_activation_flags(self, x0):
        """
        Check if x0 satisfies:
        - joint limits with 10% margin
        - vehicle–EEF distance
        - vehicle–elbow distance
        - tank bounds for EEF and vehicle

        Returns a dict with {active_joint, active_eef_selfcollision,
                            active_elbow_selfcollision,
                            active_eef_col, active_vehicle_col},
        each being 0 or 1.
        """
        numerical_tol_percent = 0.95
        x0 = np.asarray(x0, dtype=float).flatten()

        p_eef0, q_eef0 = self.eef_pose(x0[self.n_joints + self.n_dof:self.n_joints + self.n_dof + 7], x0[:self.n_joints])
        p_eef0 = np.asarray(p_eef0, dtype=float).flatten()
        q_eef0 = np.asarray(q_eef0, dtype=float).flatten()

        # -------------------------------
        # 1) Joint margin constraints
        # -------------------------------
        q0 = x0[:self.n_joints]
        lower = self.joint_pos_lim[0, :self.n_joints]
        upper = self.joint_pos_lim[1, :self.n_joints]
        lower_margin = (lower + (0.1 * (upper - lower)*(2-numerical_tol_percent)))
        upper_margin = (upper - (0.1 * (upper - lower)*(2-numerical_tol_percent)))

        joint_ok = np.all(q0 >= lower_margin) and np.all(q0 <= upper_margin)

        # -------------------------------
        # vehicle position from state
        # -------------------------------
        p_vehicle0 = x0[self.n_joints + self.n_dof : self.n_joints + self.n_dof + 3]

        # -------------------------------
        # 2) vehicle–EEF distance
        # -------------------------------
        min_dist_eef = self.mpc_limits["self_collision_vehicle_eef_min_dist"] * (2-numerical_tol_percent)
        dist_vehicle_eef_sq = np.dot(p_vehicle0 - p_eef0, p_vehicle0 - p_eef0)

        eef_self_ok = dist_vehicle_eef_sq >= min_dist_eef**2

        # -------------------------------
        # 3) vehicle–elbow distance
        # -------------------------------
        p_unit = np.array([1.0, 0.0, 0.0])
        p_unit_rot = utils_math.quaternion_rotation(q_eef0, p_unit)
        p_elbow0 = p_eef0 - p_unit_rot * self.mpc_limits["elbow_offset"]
        min_dist_elbow = self.mpc_limits["self_collision_vehicle_elbow_min_dist"] * (2-numerical_tol_percent)
        dist_vehicle_elbow_sq = np.dot(p_vehicle0 - p_elbow0, p_vehicle0 - p_elbow0)

        elbow_self_ok = dist_vehicle_elbow_sq >= min_dist_elbow**2


        # -------------------------------
        # 4) tank bounds (EEF + vehicle)
        # -------------------------------
        eef_min = np.asarray(self.tank_bounds["eef_min"], dtype=float) * np.array([(2-numerical_tol_percent), (2-numerical_tol_percent), numerical_tol_percent], dtype=float)
        eef_max = np.asarray(self.tank_bounds["eef_max"], dtype=float) * np.array([numerical_tol_percent, numerical_tol_percent, (2-numerical_tol_percent)], dtype=float)
        veh_min = np.asarray(self.tank_bounds["vehicle_min"], dtype=float) * np.array([(2-numerical_tol_percent), (2-numerical_tol_percent), numerical_tol_percent], dtype=float)
        veh_max = np.asarray(self.tank_bounds["vehicle_max"], dtype=float) * np.array([numerical_tol_percent, numerical_tol_percent, (2-numerical_tol_percent)], dtype=float)

        eef_col_ok = np.all(p_eef0 >= eef_min) and np.all(p_eef0 <= eef_max)
        veh_col_ok = np.all(p_vehicle0 >= veh_min) and np.all(p_vehicle0 <= veh_max)


        return {
            "active_joint":               int(joint_ok),
            "active_eef_selfcollision":   int(eef_self_ok),
            "active_elbow_selfcollision": int(elbow_self_ok),
            "active_eef_col":             int(eef_col_ok),
            "active_vehicle_col":         int(veh_col_ok),
        }

    def _build_algebraic_constraint(
            self,
            tau_coupling: ca.Function,
            dyn_fossen: ca.Function,
        ) -> ca.Function:
        x      = ca.MX.sym('x', self.state_dim + self.n_joints)
        u      = ca.MX.sym('u', self.ctrl_dim)
        a      = ca.MX.sym('a', self.n_dof)
        f_eef  = ca.MX.sym('f_eef', 3)
        l_eef  = ca.MX.sym('l_eef', 3)

        eta = x[self.n_joints+self.n_dof:self.n_joints+self.n_dof+7]
        pos = eta[0:3]
        quat_eta   = unit_quat(eta[3:])
        eta_used = ca.vertcat(pos, quat_eta)

        q   = x[0:self.n_joints]
        nu  = x[self.n_joints:self.n_joints+self.n_dof]
        uq  = x[self.state_dim:self.state_dim+self.n_joints]
        aq  = u[0:self.n_joints]
        uv  = u[self.n_joints:]

        quat  = eta_used[3:]
        v_ref  = nu[0:3]
        w_ref  = nu[3:6]
        a_ref  = a[0:3]
        dw_ref = a[3:6]

        tau_c = tau_coupling(q, uq, aq, v_ref, a_ref, w_ref, dw_ref, quat, f_eef, l_eef)
        dnu_dyn = dyn_fossen(eta_used, nu, uv, tau_c)

        res = a - dnu_dyn

        Ga = ca.jacobian(res, a)    # ∂res/∂a

        # newton step:
        a_star = a - ca.solve(Ga, res)

        return ca.Function('constraint_algebraic', 
                        [x, u, a, f_eef, l_eef], 
                        [a_star])#.expand()

