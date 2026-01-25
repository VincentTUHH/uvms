import casadi as ca
import numpy as np

import utils_math
import utils_sym

class KinematicsSymbolic:
    def __init__(self, this_DH_table):
        DH_table = np.array(this_DH_table)
        self.DH_table = ca.MX(DH_table)
        self.n_joints = DH_table.shape[0] - 1
        self.q0 = ca.DM(DH_table[:, 1])
        self.TF_i = [ca.MX.eye(4) for _ in range(self.n_joints + 1)]
        self.TF_iminus1_i = [ca.MX.eye(4) for _ in range(self.n_joints + 1)]
        self.update(ca.MX.zeros(self.n_joints, 1))

    def updateDHTable(self, q):
        for i in range(self.n_joints):
            self.DH_table[i, 1] = self.q0[i] + q[i]

    def update(self, q):
        self.updateDHTable(q)
        self.TF_i[0] = utils_sym.dh2matrix(*[self.DH_table[0, j] for j in range(4)])
        self.TF_iminus1_i[0] = self.TF_i[0]
        for i in range(1, self.n_joints + 1):
            self.TF_iminus1_i[i] = utils_sym.dh2matrix(*[self.DH_table[i, j] for j in range(4)])
            self.TF_i[i] = self.TF_i[i-1] @ self.TF_iminus1_i[i]

    def get_rotation_iminus1_i(self, idx):
        return self.TF_iminus1_i[idx-1][0:3, 0:3]

    def get_eef_position(self):
        return self.TF_i[-1][:3, 3]

    def get_eef_attitude(self):
        R = self.TF_i[-1][:3, :3]
        return utils_sym.rotation_matrix_to_quaternion(R)

    def get_link_position(self, idx):
        return self.TF_i[idx][:3, 3]
    
    def get_position_jacobian(self):
        J = ca.MX.zeros(3, self.n_joints)
        p_eef = self.get_eef_position()
        J[:, 0] = utils_sym.skew(utils_math.UNIT_Z) @ p_eef
        for i in range(1, self.n_joints):
            z = self.TF_i[i-1][:3, 2]
            p = self.TF_i[i-1][:3, 3]
            J[:, i] = utils_sym.skew(z) @ (p_eef - p)
        return J
    
    def get_rotation_jacobian(self):
        J = ca.MX.zeros(3, self.n_joints)
        J[:, 0] = utils_sym.UNIT_Z
        for i in range(1, self.n_joints):
            J[:, i] = self.TF_i[i-1][:3, 2]
        return J
    
    def get_full_jacobian(self):
        return [self.get_position_jacobian(), self.get_rotation_jacobian()]
    
    def eef_pose_function(self):
        """
        Returns a CasADi function that computes the end-effector pose given joint angles.
        """
        q_sym = ca.MX.sym('q', self.n_joints)
        self.update(q_sym)
        eef_pose = ca.vertcat(self.get_eef_position(), self.get_eef_attitude())
        return ca.Function('eef_pose', [q_sym], [eef_pose])

    # Helper function for pure symbolic evaluation without external update() dependencies
    def get_eef_position_symbolic(self, q: ca.MX) -> ca.MX:
        self.update(q)
        return self.get_eef_position()
    
    def get_eef_attitude_symbolic(self, q: ca.MX) -> ca.MX:
        self.update(q)
        return self.get_eef_attitude()

    def get_link_positions_symbolic(self, q: ca.MX) -> list[ca.MX]:
        self.update(q)
        return [self.get_link_position(i) for i in range(self.n_joints + 1)]

    def get_full_jacobian_symbolic(self, q: ca.MX) -> tuple[ca.MX, ca.MX]:
        self.update(q)
        return self.get_full_jacobian()