import casadi as ca

# Define commonly used constants
GRAVITY_VECTOR = ca.DM([0, 0, -9.81])

UNIT_Z = ca.DM([0, 0, 1])

def softplus(x):
    """Numerically stable calculation for log(1 + exp(x)), CasADi symbolic version."""
    return ca.log(1 + ca.exp(x))

def softminus(x):
    """CasADi symbolic version."""
    return -softplus(-x)

def softclip(x, a=None, b=None, beta=None):
    """
    Clipping with softplus and softminus, with parameterized corner sharpness.
    Set either (or both) endpoint to None to indicate no clipping at that end.
    CasADi symbolic version.
    """
    # when clipping at both ends, make beta dimensionless w.r.t. (b - a) / 2
    beta = beta / ((b - a) / 2)
    v = x
    v = v - softminus(beta * (x - a)) / beta
    v = v - softplus(beta * (x - b)) / beta
    return v

def smoothmin(a, b, beta):
    """Smooth minimum between a and b, with parameterized sharpness, CasADi symbolic version."""
    return -1/beta * ca.log(ca.exp(-beta*a) + ca.exp(-beta*b))

def quaternion_rotation(q, v):
    """
    Rotate vector v by unit quaternion q using CasADi.
    v_new = R(q) * v
    where v is treated as a pure quaternion [0, v_x, v_y, v_z]

    Args:
        q: CasADi DM or SX, shape (4,) quaternion [w, x, y, z]
        v: CasADi DM or SX, shape (3,) vector

    Returns:
        Rotated vector, shape (3,)
    """
    w = q[0]
    u = q[1:4]  # vector part of quaternion

    t = 2 * skew(u) @ v
    v_rot = v + w * t + skew(u) @ t

    return v_rot

def quat_rotate_fast(q, v):
    w = q[0]
    ux, uy, uz = q[1], q[2], q[3]
    vx, vy, vz = v[0], v[1], v[2]

    # t = 2 * (u × v)
    tx = 2 * (uy * vz - uz * vy)
    ty = 2 * (uz * vx - ux * vz)
    tz = 2 * (ux * vy - uy * vx)

    # v_rot = v + w * t + u × t
    cx = uy * tz - uz * ty
    cy = uz * tx - ux * tz
    cz = ux * ty - uy * tx

    vx_r = vx + w * tx + cx
    vy_r = vy + w * ty + cy
    vz_r = vz + w * tz + cz

    return ca.vertcat(vx_r, vy_r, vz_r)

def rotation_matrix_from_quat(quat):
    """
    Convert quaternion [w, x, y, z] to rotation matrix (3x3).
    """
    w, x, y, z = quat[0], quat[1], quat[2], quat[3]
    R = ca.vertcat(
        ca.horzcat(1 - 2*(y**2 + z**2),     2*(x*y - w*z),         2*(x*z + w*y)),
        ca.horzcat(2*(x*y + w*z),           1 - 2*(x**2 + z**2),   2*(y*z - w*x)),
        ca.horzcat(2*(x*z - w*y),           2*(y*z + w*x),         1 - 2*(x**2 + y**2))
    )
    return R

def rotation_matrix_from_euler(phi, theta, psi):
    """
    Create a rotation matrix from Euler angles (phi, theta, psi) = (roll, pitch, yaw).
    ZYX intrinsicconvention is used
    """
    cphi = ca.cos(phi)
    sphi = ca.sin(phi)
    ctheta = ca.cos(theta)
    stheta = ca.sin(theta)
    cpsi = ca.cos(psi)
    spsi = ca.sin(psi)
    # R = Rz(psi) @ Ry(theta) @ Rx(phi)
    # R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    R = ca.vertcat(
        ca.horzcat(cpsi * ctheta, cpsi * stheta * sphi - spsi * cphi, cpsi * stheta * cphi + spsi * sphi),
        ca.horzcat(spsi * ctheta, spsi * stheta * sphi + cpsi * cphi, spsi * stheta * cphi - cpsi * sphi),
        ca.horzcat(-stheta,        ctheta * sphi,                   ctheta * cphi)
    )
    return R

def rotation_matrix_to_quaternion(R):
    m00, m01, m02 = R[0,0], R[0,1], R[0,2]
    m10, m11, m12 = R[1,0], R[1,1], R[1,2]
    m20, m21, m22 = R[2,0], R[2,1], R[2,2]

    tr = m00 + m11 + m22

    # Symbolic branch selection (CasADi does not support regular if-statements for symbolic data)
    def quat_case1():
        S = ca.sqrt(tr + 1.0) * 2
        qw = 0.25 * S
        qx = (m21 - m12) / S
        qy = (m02 - m20) / S
        qz = (m10 - m01) / S
        return ca.vertcat(qw, qx, qy, qz)

    def quat_case2():
        S = ca.sqrt(1.0 + m00 - m11 - m22) * 2
        qw = (m21 - m12) / S
        qx = 0.25 * S
        qy = (m01 + m10) / S
        qz = (m02 + m20) / S
        return ca.vertcat(qw, qx, qy, qz)

    def quat_case3():
        S = ca.sqrt(1.0 + m11 - m00 - m22) * 2
        qw = (m02 - m20) / S
        qx = (m01 + m10) / S
        qy = 0.25 * S
        qz = (m12 + m21) / S
        return ca.vertcat(qw, qx, qy, qz)

    def quat_case4():
        S = ca.sqrt(1.0 + m22 - m00 - m11) * 2
        qw = (m10 - m01) / S
        qx = (m02 + m20) / S
        qy = (m12 + m21) / S
        qz = 0.25 * S
        return ca.vertcat(qw, qx, qy, qz)

    # Use CasADi symbolic `if_else` to select branch
    quat = ca.if_else(tr > 0, quat_case1(),
            ca.if_else(ca.logic_and(m00 > m11, m00 > m22), quat_case2(),
            ca.if_else(m11 > m22, quat_case3(), quat_case4())))
    
    return quat

def euler_to_quat(roll, pitch, yaw):
    cy, sy = ca.cos(yaw * 0.5), ca.sin(yaw * 0.5)
    cp, sp = ca.cos(pitch * 0.5), ca.sin(pitch * 0.5)
    cr, sr = ca.cos(roll * 0.5), ca.sin(roll * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return ca.vertcat(w, x, y, z)

def quat_mult(q1, q2):
    # Hamilton-Produkt zweier Quaternionen
    w1, x1, y1, z1 = q1[0], q1[1], q1[2], q1[3]
    w2, x2, y2, z2 = q2[0], q2[1], q2[2], q2[3]
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    return ca.vertcat(w, x, y, z)

def quaternion_error(q_goal, q_current):
    # Kürzeste Verbindung auf der Quaternionen-Sphäre wählen
    q_goal_adj = ca.if_else(ca.dot(q_goal, q_current) < 0, -q_goal, q_goal) ########!!!!!!!!!! try different approach than if_else for better differntiability
    # Konjugierte von q_current
    q_current_conj = ca.vertcat(q_current[0], -q_current[1], -q_current[2], -q_current[3])
    # Fehlerquaternion
    q_err = quat_mult(q_goal_adj, q_current_conj)
    # Vektoranteil als Fehler (für kleine Winkel proportional zur Rotationsachse)
    return q_err[1:4]

def quaternion_error_Niklas(q_goal, q_current):
    # Use CasADi's if_else for symbolic logic
    dot_prod = ca.dot(q_goal, q_current)
    q_goal_adj = ca.if_else(dot_prod < 0, -q_goal, q_goal)
    w_g = q_goal_adj[0]
    x_g = q_goal_adj[1]
    y_g = q_goal_adj[2]
    z_g = q_goal_adj[3]
    w_c = q_current[0]
    x_c = q_current[1]
    y_c = q_current[2]
    z_c = q_current[3]
    v_g = ca.vertcat(x_g, y_g, z_g)
    v_c = ca.vertcat(x_c, y_c, z_c)
    goal_att_tilde = ca.vertcat(
        ca.horzcat(0, -z_g, y_g),
        ca.horzcat(z_g, 0, -x_g),
        ca.horzcat(-y_g, x_g, 0)
    )
    att_error = ca.mtimes(w_g, v_c) - ca.mtimes(w_c, v_g) - ca.mtimes(goal_att_tilde, v_c)
    return att_error

def skew(v):
    return ca.vertcat(
        ca.horzcat(0, -v[2], v[1]),
        ca.horzcat(v[2], 0, -v[0]),
        ca.horzcat(-v[1], v[0], 0)
    )

def dh2matrix(d, theta, a, alpha):
    """
    Convert Denavit-Hartenberg parameters to a transformation matrix.
    """
    ct, st = ca.cos(theta), ca.sin(theta)
    ca_, sa = ca.cos(alpha), ca.sin(alpha)
    return ca.vertcat(
        ca.horzcat(ct, -st*ca_,  st*sa, a*ct),
        ca.horzcat(st,  ct*ca_, -ct*sa, a*st),
        ca.horzcat(0,      sa,     ca_,    d),
        ca.horzcat(0,       0,      0,    1)
    )