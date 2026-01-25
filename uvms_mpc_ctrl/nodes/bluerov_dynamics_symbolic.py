import casadi as ca
import numpy as np

import utils_sym

class BlueROVDynamicsSymbolic:
    def __init__(self, params):
        self.mass = ca.DM(params['mass'])
        self.inertia = ca.DM(np.array(params['inertia'])) #
        self.cog = ca.DM(np.array(params['cog'])) # center of gravity
        self.added_mass = ca.DM(np.array(params['added_mass'])) # Added mass is a vector of 6 elements, one for each degree of freedom
        self.buoyancy = params['buoyancy']
        self.cob = ca.DM(np.array(params['cob'])) # center of buoyancy
        self.damping_linear = ca.DM(np.array(params['damping_linear']))
        self.damping_nonlinear = ca.DM(np.array(params['damping_nonlinear']))
        self.gravity = ca.DM(9.81)
        # self.L = ca.DM(2.5166) # scaling factor for PWM to thrust conversion from thruster_model.py
        self.L = ca.DM(2.51978) # scaling factor for PWM to thrust conversion (updated value from thruster_b2_inversepoly.py, considering V=12V, 14V, 16V)

        self.M = self.compute_mass_matrix()
        # self.M_inv = ca.pinv(self.M)  # Inverse of the mass matrix
        self.M_inv = self.compute_inverse_mass_matrix()

        self.mixer = self.get_mixer_matrix_wu2018()
        self.mixer_inv = ca.pinv(self.mixer)
        self.mixer_nullspace = self.nullspace_mixer_matrix(self.mixer)

    def get_mixer_matrix_niklas(self):
        # Thruster geometry parameters (from YAML or vehicle config)
        alpha_f = 0.733   # 42 / 180 * pi
        alpha_r = 0.8378  # 48 / 180 * pi
        l_hf = 0.163
        l_hr = 0.177
        l_vx = 0.12
        l_vy = 0.218

        calpha_f = np.cos(alpha_f)
        salpha_f = np.sin(alpha_f)
        calpha_r = np.cos(alpha_r)
        salpha_r = np.sin(alpha_r)

        # Mixer matrix for thrusters (NumPy first)
        mixer_np = np.array([
            [calpha_f, calpha_f  , calpha_r  , calpha_r , 0     , 0     , 0     , 0   ],
            [salpha_f, -salpha_f , -salpha_r , salpha_r , 0     , 0     , 0     , 0   ],
            [0       , 0         , 0         , 0        , 1     , -1    , -1    , 1   ],
            [0       , 0         , 0         , 0        , -l_vy , -l_vy , l_vy  , l_vy],
            [0       , 0         , 0         , 0        , -l_vx , l_vx  , -l_vx , l_vx],
            [l_hf    , -l_hf     , l_hr      , -l_hr    , 0     , 0     , 0     , 0   ]
        ])
        mixer = ca.DM(mixer_np)
        return mixer
    
    def get_mixer_matrix_wu2018(self):
        # Thruster geometry parameters (from Wu 2018)
        alpha_f = np.pi / 4
        alpha_r = np.pi / 4
        # unit direction of thrust
        n1 = np.array([np.cos(alpha_f), np.sin(alpha_f), 0])
        n2 = np.array([np.cos(alpha_f), -np.sin(alpha_f), 0])
        n3 = np.array([np.cos(alpha_r), -np.sin(alpha_r), 0])
        n4 = np.array([np.cos(alpha_r), np.sin(alpha_r), 0])
        n5 = np.array([0, 0, 1])
        n6 = np.array([0, 0, -1])
        n7 = np.array([0, 0, -1])
        n8 = np.array([0, 0, 1])
        # thruster lever arms (from Wu 2018)
        l_xt = 0.156
        l_yt = 0.111
        l_zt = 0.0
        l_xb = 0.12
        l_yb = 0.218
        l_zb = 0.085
        l1 = np.array([l_xt, -l_yt, l_zt])
        l2 = np.array([l_xt, l_yt, l_zt])
        l3 = np.array([-l_xt, -l_yt, l_zt])
        l4 = np.array([-l_xt, l_yt, l_zt])
        l5 = np.array([l_xb, -l_yb, -l_zb])
        l6 = np.array([l_xb, l_yb, -l_zb])
        l7 = np.array([-l_xb, -l_yb, -l_zb])
        l8 = np.array([-l_xb, l_yb, -l_zb])

        n_arr = np.stack([n1, n2, n3, n4, n5, n6, n7, n8], axis=1)  # shape (3, 8)
        l_arr = np.stack([l1, l2, l3, l4, l5, l6, l7, l8], axis=1)  # shape (3, 8)

        # Compute cross product for each column
        cross_arr = np.cross(l_arr.T, n_arr.T).T  # shape (3, 8)

        mixer_np = np.vstack((n_arr, cross_arr))  # shape (6, 8)
        mixer_np[np.abs(mixer_np) < 1e-6] = 0.0  # Enforce exact zeros for small values

        mixer = ca.DM(mixer_np)
        return mixer
    
    def nullspace_mixer_matrix(self,mixer: ca.DM):
        mixer_np = np.array(mixer)   # or mixer.full()
        U, S, Vh = np.linalg.svd(mixer_np, full_matrices=True)
        V = Vh.T
        tol = np.finfo(float).eps * max(mixer.shape) * S[0]
        r = np.sum(S > tol)
        N = V[:, r:]   # right nullspace basis
        N[np.abs(N) < 1e-6] = 0.0  # Enforce exact zeros for small values
        return ca.DM(N)

    def compute_mass_matrix(self):
        M_rb = ca.DM.zeros(6, 6)
        M_rb[0:3, 0:3] = self.mass * ca.DM.eye(3)
        M_rb[0:3, 3:6] = -self.mass * utils_sym.skew(self.cog) # self.cog is the vector from body-fixed frame to the center of gravity expressed in the body-fixed frame
        M_rb[3:6, 0:3] = self.mass * utils_sym.skew(self.cog)
        # inertia tensor with respect to the center of gravity - Steiner's theorem to account for the offset of the center of gravity, 
        # when reference system for the generalized coordinates 
        M_rb[3:6, 3:6] = ca.diag(self.inertia) - self.mass * utils_sym.skew(self.cog) @ utils_sym.skew(self.cog)
        M = M_rb + ca.diag(self.added_mass)
        return M
    
    def compute_inverse_mass_matrix(self):
        """
        Compute a numerically stable inverse of the mass matrix.
        - Try a direct NumPy inverse when the condition number is reasonable.
        - If ill-conditioned or singular, try Tikhonov regularization (small diag) or fallback to pseudo-inverse.
        Returns a casadi.DM inverse and stores it in self.M_inv.
        """
        # Convert to NumPy for numeric routines
        M_np = np.array(self.M)  # shape (6,6)

        # Safely get condition number
        try:
            cond = np.linalg.cond(M_np)
            print(f"Mass matrix condition number: {cond:.2e}")
        except Exception:
            cond = np.inf

        M_inv_np = None

        # Use direct inverse when well-conditioned
        if np.isfinite(cond) and cond < 1e12:
            try:
                M_inv_np = np.linalg.inv(M_np)
            except np.linalg.LinAlgError:
                M_inv_np = None

        # If direct inverse failed or matrix is ill-conditioned, try regularization then pinv
        if M_inv_np is None:
            # Regularization scale: small fraction of the average diagonal magnitude
            eps = 1e-8
            diag_scale = max(np.trace(M_np) / max(1, M_np.shape[0]), 1.0)
            reg = eps * diag_scale
            try:
                M_inv_np = np.linalg.inv(M_np + reg * np.eye(M_np.shape[0]))
            except np.linalg.LinAlgError:
                # Last resort: pseudo-inverse
                M_inv_np = np.linalg.pinv(M_np)

        # Convert back to CasADi DM, store and return
        M_inv = ca.DM(M_inv_np)
        self.M_inv = M_inv
        return M_inv
    
    def D(self, nu):  # Damping matrix (CasADi compatible)
        D = ca.MX.zeros(6, 6)
        D_diag = self.damping_linear + self.damping_nonlinear * ca.fabs(nu)
        for i in range(6):
            D[i, i] = D_diag[i]
        return D

    def C(self, nu):  # Coriolis-centripetal matrix: forces due to the motion of the vehicle (CasADi compatible)
        C = ca.MX.zeros(6, 6) # C cant be a ca.DM matrix (numeric values only) as symbolic parameters (MX or SX) - as nu is a vector being optimized - are inserted in C
        v = nu[0:3]  # Linear velocities [u, v, w]
        w = nu[3:6]  # Angular velocities [p, q, r]
        # Compute the Coriolis forces based on the velocity and angular velocity
        C[0:3, 3:6] = -utils_sym.skew(self.M[0:3, 0:3] @ v + self.M[0:3, 3:6] @ w)
        C[3:6, 0:3] = -utils_sym.skew(self.M[0:3, 0:3] @ v + self.M[0:3, 3:6] @ w)
        C[3:6, 3:6] = -utils_sym.skew(self.M[3:6, 0:3] @ v + self.M[3:6, 3:6] @ w)
        return C

    def g(self, eta):  # gravitational and buoyancy forces + moments (CasADi compatible)
        # eta is the pose vector [x, y, z, phi, theta, psi]
        phi = eta[3]
        theta = eta[4]
        psi = eta[5]
        R = utils_sym.rotation_matrix_from_euler(phi, theta, psi)
        fg = self.mass * R.T @ ca.DM([0, 0, -self.gravity])
        fb = self.buoyancy * R.T @ ca.DM([0, 0, self.gravity])
        g_vec = ca.MX.zeros(6, 1)
        g_vec[0:3] = -(fg + fb)
        # Moments due to the forces acting at the center of gravity and center of buoyancy
        g_vec[3:6] = -(utils_sym.skew(self.cog) @ fg + utils_sym.skew(self.cob) @ fb)
        return g_vec

    def g_quat(self, eta):  # gravitational and buoyancy forces + moments (CasADi, quaternion version)
        # quat: [w, x, y, z]
        quat = eta[3:]
        R = utils_sym.rotation_matrix_from_quat(quat)
        fg = self.mass * R.T @ ca.DM([0, 0, -self.gravity])
        fb = self.buoyancy * R.T @ ca.DM([0, 0, self.gravity])
        g_vec = ca.MX.zeros(6, 1)
        g_vec[0:3] = -(fg + fb)
        g_vec[3:6] = -(utils_sym.skew(self.cog) @ fg + utils_sym.skew(self.cob) @ fb)
        return g_vec

    def J(self, eta):
        # Transformation from body to inertial (Euler)
        phi, theta, psi = eta[3], eta[4], eta[5]
        R = utils_sym.rotation_matrix_from_euler(phi, theta, psi)
        cphi = ca.cos(phi)
        sphi = ca.sin(phi)
        ctheta = ca.cos(theta)
        stheta = ca.sin(theta)
        T = ca.vertcat(
            ca.horzcat(1, sphi*stheta/ctheta, cphi*stheta/ctheta),
            ca.horzcat(0, cphi, -sphi),
            ca.horzcat(0, sphi/ctheta, cphi/ctheta)
        )
        J = ca.vertcat(
            ca.horzcat(R, ca.DM.zeros(3,3)),
            ca.horzcat(ca.DM.zeros(3,3), T)
        )
        return J

    def J_quat(self, eta):
        # Transformation from body to inertial (quaternion)
        # quat: [w, x, y, z]
        quat = eta[3:]
        R = utils_sym.rotation_matrix_from_quat(quat)
        w, x, y, z = quat[0], quat[1], quat[2], quat[3]
        # Angular velocity mapping for quaternion
        Omega = ca.vertcat(
            ca.horzcat(-x, -y, -z),
            ca.horzcat(w, -z, y),
            ca.horzcat(z, w, -x),
            ca.horzcat(-y, x, w)
        )
        J = ca.vertcat(
            ca.horzcat(R, ca.DM.zeros(3,3)),
            ca.horzcat(ca.DM.zeros(4,3), 0.5*Omega)
        )
        return J