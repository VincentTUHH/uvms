import yaml
from copy import deepcopy
from types import SimpleNamespace

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as Rot

# Define commonly used constants
GRAVITY_VECTOR = np.array([0, 0, -9.81])

UNIT_Z = np.array([0, 0, 1])

def softplus(x):
    """numerically stable calcuation for log(1 + exp(x))"""
    return np.log(1 + np.exp(x))

def softplus_stable(x):
    """Numerisch stabilere Berechnung für log(1 + exp(x))"""
    return np.maximum(x, 0) + np.log(1 + np.exp(-np.abs(x)))

def softminus(x):
    return -softplus(-x)

def softclip(x, a=None, b=None, beta=None):
    """
    Clipping with softplus and softminus, with paramterized corner sharpness.
    Set either (or both) endpoint to None to indicate no clipping at that end.
    """
    # when clipping at both ends, make c dimensionless w.r.t. b - a / 2  
    if a is not None and b is not None:
        beta /= (b - a) / 2

    v = x
    if a is not None:
        v = v - softminus(beta*(x - a)) / beta
    if b is not None:
        v = v - softplus(beta*(x - b)) / beta
    return v

def quaternion_rotation(q, v):
    """
    Rotate vector v by unit quaternion q.
    v_new = q_conj * v * q = R(q) * v
    where v is treated as a pure quaternion [0, v_x, v_y, v_z]

    Args:
        q: array-like, shape (4,) quaternion [w, x, y, z]
        v: array-like, shape (3,) vector

    Returns:
        Rotated vector, shape (3,)
    """
    q = np.asarray(q, dtype=np.float64)
    v = np.asarray(v, dtype=np.float64)
    w = q[0]

    u = q[1:]  # vector part of quaternion

    t = 2 * skew(u) @ v

    v_rot = v + w*t + skew(u) @ t

    return v_rot

def rotation_matrix_from_quat(q):
    """
    Convert quaternion [w, x, y, z] to rotation matrix (3x3).
    """
    q = np.asarray(q, dtype=float).reshape(4)

    # normalize (optional, but good)
    n = np.linalg.norm(q)
    if n > 1e-12:
        q = q / n

    # SciPy expects [x, y, z, w]
    q_xyzw = np.array([q[1], q[2], q[3], q[0]], dtype=float)

    return Rot.from_quat(q_xyzw).as_matrix()

def rotation_matrix_from_euler(phi, theta, psi):
    """
    Create a rotation matrix from Euler angles (phi, theta, psi) = (roll, pitch, yaw).
    ZYX intrinsic convention is used
    """
    # cphi = np.cos(phi)
    # sphi = np.sin(phi)
    # ctheta = np.cos(theta)
    # stheta = np.sin(theta)
    # cpsi = np.cos(psi)
    # spsi = np.sin(psi)
    # R = np.array([
    #     [cpsi * ctheta, cpsi * stheta * sphi - spsi * cphi, cpsi * stheta * cphi + spsi * sphi],
    #     [spsi * ctheta, spsi * stheta * sphi + cpsi * cphi, spsi * stheta * cphi - cpsi * sphi],
    #     [-stheta,        ctheta * sphi,                     ctheta * cphi]
    # ])
    # Rx = np.array([
    #     [1, 0, 0],
    #     [0, np.cos(phi), -np.sin(phi)],
    #     [0, np.sin(phi), np.cos(phi)]
    # ])
    # Ry = np.array([
    #     [np.cos(theta), 0, np.sin(theta)],
    #     [0, 1, 0],
    #     [-np.sin(theta), 0, np.cos(theta)]
    # ])
    # Rz = np.array([
    #     [np.cos(psi), -np.sin(psi), 0],
    #     [np.sin(psi), np.cos(psi), 0],
    #     [0, 0, 1]
    # ])

    # R1 = Rz @ Ry @ Rx

    R = Rot.from_euler('ZYX', [psi, theta, phi]).as_matrix() # case sensitive: upper case is extrinsic -> rotating in world frame
    return R

def euler_to_quat(roll, pitch, yaw):
    cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)
    cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
    cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return np.array([w, x, y, z])

def quat_to_euler(q):
    # q is [w, x, y, z]
    q = np.asarray(q, dtype=float).reshape(4)

    # optional: flip for consistency
    if q[0] < 0:
        q = -q

    # normalize (optional)
    n = np.linalg.norm(q)
    if n > 1e-12:
        q = q / n

    # SciPy expects [x, y, z, w]
    q_xyzw = np.array([q[1], q[2], q[3], q[0]], dtype=float)

    r = Rot.from_quat(q_xyzw)

    # match your rotation_matrix_from_euler (you used 'ZYX')
    euler_zyx = r.as_euler('ZYX', degrees=False)  # [z, y, x] = [yaw, pitch, roll]
    return euler_zyx[::-1]  # -> [roll, pitch, yaw]

def rotation_matrix_to_quaternion(R):
    m00, m01, m02 = R[0, 0], R[0, 1], R[0, 2]
    m10, m11, m12 = R[1, 0], R[1, 1], R[1, 2]
    m20, m21, m22 = R[2, 0], R[2, 1], R[2, 2]
    tr = m00 + m11 + m22

    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2  # S=4*qw
        qw = 0.25 * S
        qx = (m21 - m12) / S
        qy = (m02 - m20) / S
        qz = (m10 - m01) / S
    elif (m00 > m11) and (m00 > m22):
        S = np.sqrt(1.0 + m00 - m11 - m22) * 2  # S=4*qx
        qw = (m21 - m12) / S
        qx = 0.25 * S
        qy = (m01 + m10) / S
        qz = (m02 + m20) / S
    elif m11 > m22:
        S = np.sqrt(1.0 + m11 - m00 - m22) * 2  # S=4*qy
        qw = (m02 - m20) / S
        qx = (m01 + m10) / S
        qy = 0.25 * S
        qz = (m12 + m21) / S
    else:
        S = np.sqrt(1.0 + m22 - m00 - m11) * 2  # S=4*qz
        qw = (m10 - m01) / S
        qx = (m02 + m20) / S
        qy = (m12 + m21) / S
        qz = 0.25 * S
    quat = np.array([qw, qx, qy, qz])
    return quat

def skew(v):
    """
    Create a skew-symmetric matrix from a vector.
    """
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])

def load_yaml(file_path):
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)

def load_model_params(yaml_path):
    """
    Load model parameters from a ROS2 YAML config file.
    Args:
        yaml_path: str, path to the YAML file
    Returns:
        params: dict, containing model parameters
    """
    data = load_yaml(yaml_path)
    # Find the first key (wildcard node name)
    node_key = list(data.keys())[0]
    model_params = data[node_key]['ros__parameters']['model']
    return model_params

def load_dh_params(yaml_filename):
    dh_data = load_yaml(yaml_filename)
    links = []
    if isinstance(dh_data, dict) and '/**' in dh_data and 'ros__parameters' in dh_data['/**']:
        params = dh_data['/**']['ros__parameters']
        for key in sorted(params.keys()):
            link = params[key]
            if isinstance(link, dict):
                d = link.get('d', 0)
                theta = link.get('theta0', 0)
                a = link.get('a', 0)
                alpha = link.get('alp', 0)
                links.append([d, theta, a, alpha])
        DH_table = np.array(links)
    else:
        if isinstance(dh_data, dict) and 'dh_table' in dh_data:
            DH_table = np.array(dh_data['dh_table'])
        else:
            DH_table = np.array(dh_data)
    return DH_table

def load_joint_limits(yaml_filename):
    joint_data = load_yaml(yaml_filename)
    joint_limits, joint_efforts, joint_velocities, all_joints = [], [], [], {}
    for joint_name in sorted(joint_data.keys()):
        entry = joint_data[joint_name]
        joint_limits.append((entry.get('lower', None), entry.get('upper', None)))
        joint_efforts.append(entry.get('effort', None))
        joint_velocities.append(entry.get('velocity', None))
        all_joints[joint_name] = entry
    return joint_limits, joint_efforts, joint_velocities, all_joints

def load_all_thruster_data(file_path):
    voltages = [10, 12, 14, 16, 18, 20]
    data = {}
    for v in voltages:
        fname = f'multiplied_cleaned_T200_{v}V.csv'
        data_path = file_path + fname
        d = read_thruster_csv(data_path)
        data[v] = {
            'pwm': d['PWM (mus)'],
            'force': d['Force (N)']
        }
    return data

def read_thruster_csv(csv_path):
    df = pd.read_csv(csv_path, sep=';', decimal=',', engine='python')
    df.dropna(axis=0, how='all', inplace=True)
    df.dropna(axis=1, how='all', inplace=True)
    data = {col.strip(): df[col].to_numpy() for col in df.columns}
    return data

def load_dynamic_params(file_paths):
    merged_dict = {}
    for file in file_paths:
        data = load_yaml(file)
        params = data.get('/**', {}).get('ros__parameters', {})
        merged_dict = recursive_merge(merged_dict, params)
    return dict_to_namespace(merged_dict)

def recursive_merge(dict1, dict2):
    result = deepcopy(dict1)
    for key, value in dict2.items():
        if key in result and isinstance(result[key], dict) and isinstance(value, dict):
            result[key] = recursive_merge(result[key], value)
        else:
            result[key] = deepcopy(value)
    return result

def dict_to_namespace(d):
    if not isinstance(d, dict):
        return d
    return SimpleNamespace(**{k: dict_to_namespace(v) for k, v in d.items()})

def read_wrench_txt(filename):
    data = []
    with open(filename, 'r') as f:
        for line in f:
            parts = [float(x.strip()) for x in line.split(',')]
            data.append(parts)
    data = np.array(data)
    t = data[:, 0]
    tau = data[:, 1:7]
    return t, tau

def dh2matrix(d, theta, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca_, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st*ca_,  st*sa, a*ct],
        [st,  ct*ca_, -ct*sa, a*st],
        [0,      sa,     ca_,    d],
        [0,       0,      0,    1]
    ])

def quaternion_error_Niklas(q_goal, q_current):
    w_g, x_g, y_g, z_g = q_goal
    w_c, x_c, y_c, z_c = q_current
    v_g = np.array([x_g, y_g, z_g])
    v_c = np.array([x_c, y_c, z_c])
    goal_att_tilde = np.array([
        [0, -z_g, y_g],
        [z_g, 0, -x_g],
        [-y_g, x_g, 0]
    ])
    att_error = w_g * v_c - w_c * v_g - goal_att_tilde @ v_c
    return np.linalg.norm(att_error)

def quat_mult(q1, q2):
    # Hamilton product of two quaternions
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    return np.array([w, x, y, z])

def quaternion_error(q_goal, q_current):
    # Choose shortest path on quaternion sphere
    if np.dot(q_goal, q_current) < 0:
        q_goal_adj = -q_goal
    else:
        q_goal_adj = q_goal
    # Conjugate of q_current
    q_current_conj = np.array([q_current[0], -q_current[1], -q_current[2], -q_current[3]])
    # Error quaternion
    q_err = quat_mult(q_goal_adj, q_current_conj)
    # Vector part as error (for small angles proportional to rotation axis)
    return 2 * q_err[1:4]

def quat_conjugate(q):
    """Conjugate of quaternion [w, x, y, z]."""
    return np.array([q[0], -q[1], -q[2], -q[3]])