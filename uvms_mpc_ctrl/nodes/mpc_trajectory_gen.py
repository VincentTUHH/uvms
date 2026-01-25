# mpc_utils.py
import numpy as np
from args import TRAJ_ARGS
import utils_math
from scipy.integrate import quad

def make_eef_connecting_traj_with_wait(
    p_eef_real_0: np.ndarray,
    att_eef_real_0: np.ndarray,
    ref_eef_pos_run: np.ndarray,
    ref_eef_att_run: np.ndarray,
    dt: float,
    v_max: float,
    N_wait: int,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Build a connecting trajectory that
      1) moves the EEF in a straight line with moderate speed
         from the *current* pose (p_eef_real_0, att_eef_real_0)
         to the *initial* pose of (ref_eef_pos_run, ref_eef_att_run),
      2) then waits at that final pose for N_wait steps.

    Parameters
    ----------
    p_eef_real_0 : (3,)
        Current EEF position.
    att_eef_real_0 : (4,)
        Current EEF attitude quaternion.
    ref_eef_pos_run : (3, T)
        Existing reference EEF position trajectory.
    ref_eef_att_run : (4, T)
        Existing reference EEF attitude trajectory.
    dt : float
        Sampling time.
    v_max : float
        Desired maximum translational speed [m/s].
    N_wait : int
        Number of samples to stay at the final pose
        (typically N_HORIZON).

    Returns
    -------
    conn_pos_full : (3, N_conn + N_wait)
        Connecting + waiting position trajectory.
    conn_att_full : (4, N_conn + N_wait)
        Connecting + waiting attitude trajectory.
    """
    p_start = np.asarray(p_eef_real_0, dtype=float).reshape(3)
    q_start = np.asarray(att_eef_real_0, dtype=float).reshape(4)

    p_target = np.asarray(ref_eef_pos_run[:, 0], dtype=float).reshape(3)
    q_target = np.asarray(ref_eef_att_run[:, 0], dtype=float).reshape(4)

    # distance and duration
    d = np.linalg.norm(p_target - p_start)
    if d < 1e-6:
        # almost same point → at least one step
        T = dt
    else:
        T = max(d / v_max, dt)

    N_conn = int(np.ceil(T / dt)) + 1  # +1 to include final point
    t = np.linspace(0.0, 1.0, N_conn)

    # straight-line interpolation in position
    conn_pos = (p_start[:, None] +
                (p_target - p_start)[:, None] * t[None, :])

    # quaternion SLERP for attitude
    conn_att = slerp(q_start, q_target, t)

    # ----------------------------------------------------------------------
    # Wait at final pose for N_wait steps
    # ----------------------------------------------------------------------
    final_pos = conn_pos[:, -1].reshape(3, 1)
    final_att = conn_att[:, -1].reshape(4, 1)

    wait_pos = np.tile(final_pos, (1, N_wait))
    wait_att = np.tile(final_att, (1, N_wait))

    conn_pos_full = np.hstack([conn_pos, wait_pos])
    conn_att_full = np.hstack([conn_att, wait_att])

    return conn_pos_full, conn_att_full

def slerp(q0: np.ndarray, q1: np.ndarray, t: np.ndarray) -> np.ndarray:
    """
    Spherical linear interpolation between quaternions q0 and q1.
    q0, q1: shape (4,)
    t: array of shape (N,) in [0, 1]
    Returns: array of shape (4, N)
    """
    q0 = np.asarray(q0, dtype=float)
    q1 = np.asarray(q1, dtype=float)

    # normalize
    q0 = q0 / np.linalg.norm(q0)
    q1 = q1 / np.linalg.norm(q1)

    # ensure shortest path
    if np.dot(q0, q1) < 0.0:
        q1 = -q1

    dot = np.clip(np.dot(q0, q1), -1.0, 1.0)
    omega = np.arccos(dot)

    if np.isclose(omega, 0.0):
        # quaternions are almost identical
        return np.tile(q0.reshape(4, 1), (1, t.size))

    sin_omega = np.sin(omega)
    s0 = np.sin((1.0 - t) * omega) / sin_omega
    s1 = np.sin(t * omega) / sin_omega

    return (q0[:, None] * s0[None, :] +
            q1[:, None] * s1[None, :])

def back_to_start_with_wait(
    ref_eef_pos_run: np.ndarray,
    ref_eef_att_run: np.ndarray,
    dt: float,
    v_max: float,
    N_wait_begin: int,
    N_wait_end: int,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Build a "back-to-start" trajectory that:
      1) waits at the *last* pose of the reference for N_wait_begin steps,
      2) moves the EEF in a straight line (pos) and via SLERP (att)
         from the *last* pose of the reference to the *first* pose,
      3) waits at the *first* pose for N_wait_end steps.

    Parameters
    ----------
    ref_eef_pos_run : (3, T)
        Reference EEF position trajectory.
    ref_eef_att_run : (4, T)
        Reference EEF attitude quaternion trajectory (w, x, y, z).
    dt : float
        Sampling time.
    v_max : float
        Desired maximum translational speed [m/s] for the transition segment.
    N_wait_begin : int
        Number of samples to hold the last pose before transitioning.
    N_wait_end : int
        Number of samples to hold the first pose after transitioning.

    Returns
    -------
    back_pos_full : (3, N_wait_begin + N_back + N_wait_end)
        Wait + transition + wait position trajectory.
    back_att_full : (4, N_wait_begin + N_back + N_wait_end)
        Wait + transition + wait attitude trajectory.
    """
    ref_pos = np.asarray(ref_eef_pos_run, dtype=float)
    ref_att = np.asarray(ref_eef_att_run, dtype=float)

    if ref_pos.shape[0] != 3:
        raise ValueError(f"ref_eef_pos_run must have shape (3,T), got {ref_pos.shape}")
    if ref_att.shape[0] != 4:
        raise ValueError(f"ref_eef_att_run must have shape (4,T), got {ref_att.shape}")
    if ref_pos.shape[1] < 1 or ref_att.shape[1] < 1:
        raise ValueError("Reference trajectories must have at least one sample.")
    if ref_pos.shape[1] != ref_att.shape[1]:
        raise ValueError(f"Position and attitude must share T. Got {ref_pos.shape[1]} vs {ref_att.shape[1]}")
    if dt <= 0.0:
        raise ValueError(f"dt must be > 0, got {dt}")
    if v_max <= 0.0:
        raise ValueError(f"v_max must be > 0, got {v_max}")
    if N_wait_begin < 0 or N_wait_end < 0:
        raise ValueError("N_wait_begin and N_wait_end must be >= 0.")

    # last -> first
    p_start = ref_pos[:, -1].reshape(3)
    q_start = ref_att[:, -1].reshape(4)
    p_target = ref_pos[:, 0].reshape(3)
    q_target = ref_att[:, 0].reshape(4)

    # duration based on distance and v_max
    d = np.linalg.norm(p_target - p_start)
    if d < 1e-6:
        T_move = dt
    else:
        T_move = max(d / v_max, dt)

    N_back = int(np.ceil(T_move / dt)) + 1  # include final point
    t = np.linspace(0.0, 1.0, N_back)

    # transition segment
    back_pos = (p_start[:, None] + (p_target - p_start)[:, None] * t[None, :])
    back_att = slerp(q_start, q_target, t)  # (4, N_back)

    # wait at last pose (begin)
    wait_begin_pos = np.tile(p_start.reshape(3, 1), (1, N_wait_begin))
    wait_begin_att = np.tile(q_start.reshape(4, 1), (1, N_wait_begin))

    # wait at first pose (end)
    wait_end_pos = np.tile(p_target.reshape(3, 1), (1, N_wait_end))
    wait_end_att = np.tile(q_target.reshape(4, 1), (1, N_wait_end))

    back_pos_full = np.hstack([wait_begin_pos, back_pos, wait_end_pos])
    back_att_full = np.hstack([wait_begin_att, back_att, wait_end_att])

    return back_pos_full, back_att_full


def build_eef_reference_trajectory(dt: float, traj_type: str):
    """
    Build the EEF reference position and attitude trajectory based on TRAJ_ARGS
    configured in args.py. Returns (ref_eef_pos_run, ref_eef_att_run).
    """

    if traj_type == "line":
        cfg = TRAJ_ARGS["line"]
        p_start = cfg["p_start"]
        q_start = cfg["q_start"]
        p_goal = cfg["p_goal"]
        q_goal = cfg["q_goal"]
        fwd_speed = cfg["fwd_speed"]
        dt_traj = dt  # use same sampling as MPC

        ref_eef_pos_run, ref_eef_att_run, *_ = generate_minimum_jerk_eef_trajectory(
            p_start, q_start, p_goal, q_goal, fwd_speed, dt_traj
        )
        return ref_eef_pos_run, ref_eef_att_run

    
    elif traj_type == "circ_oscillation_3d_radial":
        cfg = TRAJ_ARGS["circ_oscillation_3d_radial"]
        start_pos        = cfg["start_pos"]
        radius           = cfg["radius"]
        n_revs           = cfg["n_revs"]
        A_z              = cfg["A_z"]
        n_osc_per_circle = cfg["n_osc_per_circle"]
        fwd_speed        = cfg["fwd_speed"]

        ref_eef_pos_run, ref_eef_att_run, *_ = generate_eef_circle_with_z_osc_3d_radial(
            start_pos=start_pos,
            radius=radius,
            n_revs=n_revs,
            A_z=A_z,
            n_osc_per_circle=n_osc_per_circle,
            fwd_speed=fwd_speed,
            dt=dt,
        )
        return ref_eef_pos_run, ref_eef_att_run

    
    elif traj_type == "sine_xz":
        cfg = TRAJ_ARGS["sine_xz"]
        p_start    = cfg["p_start"]
        p_goal     = cfg["p_goal"]
        n_osc      = cfg["n_osc"]
        A          = cfg["A"]
        fwd_speed  = cfg["fwd_speed"]

        ref_eef_pos_run, ref_eef_att_run, *_ = generate_eef_sine_xz_const_speed(
            p_start=p_start,
            p_goal=p_goal,
            n_osc=n_osc,
            A=A,
            fwd_speed=fwd_speed,
            dt=dt,
        )
        return ref_eef_pos_run, ref_eef_att_run
    
    # (Removed duplicate branch for circ_oscillation_3d_radial_with_eights)

    else:
        raise ValueError(f"Unknown trajectory type in TRAJ_ARGS: {traj_type!r}")
    

def generate_minimum_jerk_eef_trajectory(
    p_start: np.ndarray,
    q_start: np.ndarray,
    p_goal: np.ndarray,
    q_goal: np.ndarray,
    v: float,
    dt: float,
):
    """
    Generate a minimum-jerk straight-line trajectory between two EEF poses.

    Parameters
    ----------
    p_start, p_goal : np.ndarray, shape (3,)
        Start and goal positions in the inertial frame [m].
    q_start, q_goal : np.ndarray, shape (4,)
        Start and goal unit quaternions (w, x, y, z).
    T : float
        Total motion duration [s].
    dt : float
        Sampling time [s].

    Returns
    -------
    pos_traj : np.ndarray, shape (3, N)
        Positions along the minimum-jerk trajectory.
    att_traj : np.ndarray, shape (4, N)
        Unit quaternions along the trajectory.
    vel_traj : np.ndarray, shape (3, N)
        Linear velocities along the trajectory [m/s].
    omega_traj : np.ndarray, shape (3, N)
        Angular velocities along the trajectory [rad/s].
    t_grid : np.ndarray, shape (N,)
        Time stamps from 0 to T.
    """
    assert v > 0.0, "Forward speed v must be positive."
    assert dt > 0.0, "Sampling time dt must be positive."

    T = np.linalg.norm(p_goal - p_start) / v

    # Number of samples including both endpoints 0 and T
    N = int(np.floor(T / dt)) + 1
    t_grid = np.linspace(0.0, T, N)

    p_start = np.asarray(p_start, dtype=float).reshape(3)
    p_goal = np.asarray(p_goal, dtype=float).reshape(3)
    q_start = np.asarray(q_start, dtype=float).reshape(4)
    q_goal = np.asarray(q_goal, dtype=float).reshape(4)

    pos_traj = np.zeros((3, N))
    att_traj = np.zeros((4, N))
    vel_traj = np.zeros((3, N))

    dp = p_goal - p_start

    for i, t in enumerate(t_grid):
        # Normalized time in [0, 1]
        tau = t / T
        tau = min(max(tau, 0.0), 1.0)

        # Minimum-jerk time scaling
        s = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5
        # Time derivative s_dot = ds/dt
        s_dot = (30.0 * tau**2 - 60.0 * tau**3 + 30.0 * tau**4) / T

        # Position and linear velocity
        pos_traj[:, i] = p_start + s * dp
        vel_traj[:, i] = s_dot * dp

        # Quaternion interpolation (lerp + renormalization)
        q_interp = (1.0 - s) * q_start + s * q_goal
        norm_q = np.linalg.norm(q_interp)
        if norm_q < 1e-12:
            q_interp = q_start
        else:
            q_interp = q_interp / norm_q
        att_traj[:, i] = q_interp

    # Angular velocity from quaternion trajectory
    omega_traj = quat_angular_velocity_traj(att_traj, t_grid)

    return pos_traj, att_traj, vel_traj, omega_traj, t_grid

def quat_angular_velocity_traj(q_traj, t_grid):
    """
    Approximate angular velocity from a quaternion trajectory.

    Parameters
    ----------
    q_traj : np.ndarray, shape (4, N)
        Quaternion at each time step (w, x, y, z).
    t_grid : np.ndarray, shape (N,)

    Returns
    -------
    omega_traj : np.ndarray, shape (3, N)
        Angular velocity for each time step (rad/s), expressed in the
        local EEF frame associated with q_k.
    """
    N = q_traj.shape[1]
    omega_traj = np.zeros((3, N))

    for k in range(N - 1):
        dt = t_grid[k+1] - t_grid[k]
        qk = q_traj[:, k]
        qk1 = q_traj[:, k+1]

        # Relative rotation q_rel such that qk1 = qk ⊗ q_rel
        q_rel = utils_math.quat_mult(utils_math.quat_conjugate(qk), qk1)
        q_rel = q_rel / np.linalg.norm(q_rel)

        w, x, y, z = q_rel
        v = np.array([x, y, z])
        v_norm = np.linalg.norm(v)

        if v_norm < 1e-12:
            omega_traj[:, k] = 0.0
        else:
            angle = 2.0 * np.arctan2(v_norm, w)   # rotation angle
            axis = v / v_norm                     # rotation axis
            omega_traj[:, k] = (angle / dt) * axis

    # For the last sample, just copy the previous one
    omega_traj[:, -1] = omega_traj[:, -2] if N > 1 else 0.0
    return omega_traj
    
def generate_eef_circle_with_z_osc_3d_radial(
    start_pos: np.ndarray,
    radius: float,
    n_revs: int,
    A_z: float,
    n_osc_per_circle: int,
    fwd_speed: float,
    dt: float,
    speed_mode: str = "arc",
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Generate an EEF reference trajectory:
      - circle in xy of given radius
      - sinusoidal oscillation in z
      - total n_revs circles
      - z-axis of EEF frame tangent to trajectory (aligned with velocity)
      - x-axis of EEF frame pointing from circle center to current point
        in 3D (projected onto the plane orthogonal to the tangent)

    Parameters
    ----------
    start_pos : (3,)
        Starting position [x0, y0, z0]. Also final point of the trajectory.
    radius : float
        Circle radius in the xy-plane.
    n_revs : int
        Number of full circles to perform.
    A_z : float
        Amplitude of z-oscillation.
    n_osc_per_circle : int
        Number of full sine oscillations in z per full circle.
    fwd_speed : float
        Desired speed [m/s]. Interpretation depends on `speed_mode`:
          - "xy":  speed along the xy circle (arc length 2*pi*radius per revolution).
          - "arc": average speed along the full 3D curve (includes z-oscillation),
                   i.e., total 3D arc length per revolution divided by T_circle.
    speed_mode : str
        "xy" (default) or "arc".
    dt : float
        Sampling time [s].

    Returns
    -------
    pos : (3, N)
        Position trajectory.
    quat : (4, N)
        Orientation quaternion trajectory [w,x,y,z], with EEF z-axis tangent.
    t : (N,)
        Time stamps.
    """
    start_pos = np.asarray(start_pos, dtype=float).reshape(3)
    x0, y0, z0 = start_pos

    if fwd_speed <= 0.0:
        raise ValueError(f"fwd_speed must be > 0, got {fwd_speed}")

    r = float(radius)
    A = float(A_z)
    k = float(n_osc_per_circle)

    # Time for one full revolution.
    # speed_mode="xy":  T = (2*pi*r) / v
    # speed_mode="arc": T = (L_3d_per_rev) / v, where
    #   L_3d_per_rev = ∫_0^1 2*pi * sqrt(r^2 + (A*k*cos(2*pi*k*u))^2) du
    mode = str(speed_mode).lower()
    if mode == "xy":
        T_circle = (2.0 * np.pi * r) / float(fwd_speed)
    elif mode == "arc":
        # Numerically approximate the dimensionless integral over u in [0,1].
        # This does NOT depend on T_circle, only on r, A, k.
        def integrand(s: float) -> float:
            dz_ds = (A * k / r) * np.cos((k / r) * s)
            return np.sqrt(1.0 + dz_ds * dz_ds)

        L_3d_per_rev, _ = quad(integrand, 0.0, 2.0 * np.pi * r, limit=200)
        print(L_3d_per_rev)
        T_circle = L_3d_per_rev / float(fwd_speed)
    else:
        raise ValueError(f"Unknown speed_mode={speed_mode!r}. Use 'xy' or 'arc'.")

    # total time and samples
    T_total = n_revs * T_circle
    N = int(np.round(T_total / dt)) + 1
    t = np.linspace(0.0, T_total, N)

    # ----------------------------------------------------------------------
    # Parametrization of circle in xy
    # ----------------------------------------------------------------------
    # Choose circle center such that at t=0 we are at start_pos:
    # center = (x0 - radius, y0, z0) → at theta=0: (cx+R, cy, cz) = (x0, y0, z0)
    cx = x0 - radius
    cy = y0
    cz = z0

    # angle over time: each T_circle → 2π; over n_revs → 2π * n_revs
    theta = 2.0 * np.pi * (t / T_circle)  # shape (N,)
    theta_dot = 2.0 * np.pi / T_circle

    # positions
    x = cx + radius * np.cos(theta)
    y = cy + radius * np.sin(theta)

    # z oscillation: n_osc_per_circle per T_circle
    omega_z = 2.0 * np.pi * n_osc_per_circle / T_circle
    z = z0 + A_z * np.sin(omega_z * t)

    pos = np.vstack([x, y, z])

    # ----------------------------------------------------------------------
    # Velocity for tangent direction
    # ----------------------------------------------------------------------
    vx = -radius * np.sin(theta) * theta_dot
    vy =  radius * np.cos(theta) * theta_dot
    vz =  A_z * omega_z * np.cos(omega_z * t)

    v = np.vstack([vx, vy, vz])           # (3, N)
    v_norm = np.linalg.norm(v, axis=0)    # (N,)

    # avoid division by 0
    v_norm_safe = np.where(v_norm < 1e-9, 1e-9, v_norm)
    v_hat = v / v_norm_safe               # unit tangent, (3, N)

    # ----------------------------------------------------------------------
    # Orientation:
    #   ez = tangent (v_hat),
    #   ex from center -> current point in 3D, projected onto plane ⟂ ez,
    #   ey completes right-handed frame.
    # ----------------------------------------------------------------------
    quat_list = []

    center = np.array([cx, cy, cz], dtype=float)

    for k in range(N):
        # tangent direction = ez
        ez = v_hat[:, k]  # already unit

        # full 3D radial vector from center to current point p = [x,y,z]
        p_k = np.array([x[k], y[k], z[k]], dtype=float)
        radial_3d = p_k - center
        r_norm = np.linalg.norm(radial_3d)
        if r_norm < 1e-9:
            # degenerate: just pick something orthogonal to ez
            if abs(ez[2]) < 0.9:
                radial_3d = np.array([0.0, 0.0, 1.0])
            else:
                radial_3d = np.array([1.0, 0.0, 0.0])
            r_norm = np.linalg.norm(radial_3d)

        r_hat = radial_3d / r_norm

        # project radial direction onto plane orthogonal to ez
        ex_raw = r_hat - np.dot(r_hat, ez) * ez
        ex_norm = np.linalg.norm(ex_raw)
        if ex_norm < 1e-9:
            # if projection degenerates, pick any orthogonal direction
            if abs(ez[2]) < 0.9:
                ex_raw = np.array([0.0, 0.0, 1.0]) - ez[2] * ez
            else:
                ex_raw = np.array([1.0, 0.0, 0.0]) - ez[0] * ez
            ex_norm = np.linalg.norm(ex_raw)

        ex = ex_raw / ex_norm

        # ey completes right-handed frame: ey = ez × ex
        ey = np.cross(ez, ex)
        ey /= np.linalg.norm(ey) + 1e-15

        # build rotation matrix R = [ex ey ez]
        R = np.column_stack([ex, ey, ez])

        qk = utils_math.rotation_matrix_to_quaternion(R)
        quat_list.append(qk)

    quat = np.array(quat_list)  # shape (N,4)

    # enforce continuity (avoid sudden sign flips)
    for k in range(1, N):
        if np.dot(quat[k - 1], quat[k]) < 0.0:
            quat[k] = -quat[k]

    # put in shape (4, N)
    quat = quat.T

    return pos, quat, t

def generate_eef_sine_xz_const_speed(
    p_start: np.ndarray,
    p_goal: np.ndarray,
    n_osc: int,
    A: float,
    fwd_speed: float,
    dt: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Forward motion in y with sine oscillations in z (historical name: sine_xz).

    - y goes from p_start[1] to p_goal[1] with constant average forward speed fwd_speed
    - z oscillates with n_osc full oscillations over the total duration (starts/ends at 0 offset)
    - x is linearly interpolated (typically constant)

    Orientation:
      - EEF z-axis tangent to trajectory (velocity direction)
      - EEF x-axis: as close as possible to world +x if p_start[0] >= 1.0 else world -x,
        while staying orthogonal to z-axis
      - EEF y-axis completes right-handed frame
    """
    p_start = np.asarray(p_start, dtype=float).reshape(3)
    p_goal  = np.asarray(p_goal,  dtype=float).reshape(3)

    if fwd_speed <= 0.0:
        raise ValueError(f"fwd_speed must be > 0, got {fwd_speed}")
    if dt <= 0.0:
        raise ValueError(f"dt must be > 0, got {dt}")

    A = float(A)
    n_osc = int(n_osc)
    if n_osc < 0:
        raise ValueError(f"n_osc must be >= 0, got {n_osc}")

    # Total time from forward y distance
    dy = float(p_goal[1] - p_start[1])
    y_dist = abs(dy)
    if y_dist < 1e-9:
        pos = p_start.reshape(3, 1)
        quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=float).reshape(4, 1)
        t = np.array([0.0], dtype=float)
        return pos, quat, t

    T_total = y_dist / float(fwd_speed)
    N = int(np.round(T_total / dt)) + 1
    t = np.linspace(0.0, T_total, N)
    tau = t / T_total  # in [0,1]

    # Linear baseline from start to goal
    x = p_start[0] + (p_goal[0] - p_start[0]) * tau
    y = p_start[1] + (p_goal[1] - p_start[1]) * tau
    z_lin = p_start[2] + (p_goal[2] - p_start[2]) * tau

    # Sine in z: starts/ends at zero offset so endpoints match p_start/p_goal
    if n_osc > 0 and abs(A) > 0.0:
        z = z_lin + A * np.sin(2.0 * np.pi * float(n_osc) * tau)
    else:
        z = z_lin

    pos = np.vstack([x, y, z])  # (3, N)

    # Velocity for tangent direction
    v = np.zeros((3, N), dtype=float)
    if N >= 2:
        dt_vec = t[1:] - t[:-1]
        v[:, :-1] = (pos[:, 1:] - pos[:, :-1]) / dt_vec
        v[:, -1] = v[:, -2]

    v_norm = np.linalg.norm(v, axis=0)
    v_norm_safe = np.where(v_norm < 1e-9, 1e-9, v_norm)
    ez = v / v_norm_safe  # tangent as z-axis

    # Preferred world-x direction based on start x
    x_pref_sign = 1.0 if p_start[0] >= 1.0 else -1.0
    a_world = np.array([x_pref_sign, 0.0, 0.0], dtype=float)

    quat_list = []
    for k in range(N):
        ez_k = ez[:, k]

        # Project preferred world-x into plane orthogonal to ez
        ex_raw = a_world - np.dot(a_world, ez_k) * ez_k
        ex_n = np.linalg.norm(ex_raw)
        if ex_n < 1e-9:
            # fallback if ez parallel to a_world
            b_world = np.array([0.0, 1.0, 0.0], dtype=float)
            ex_raw = b_world - np.dot(b_world, ez_k) * ez_k
            ex_n = np.linalg.norm(ex_raw)
            if ex_n < 1e-9:
                b_world = np.array([0.0, 0.0, 1.0], dtype=float)
                ex_raw = b_world - np.dot(b_world, ez_k) * ez_k
                ex_n = np.linalg.norm(ex_raw)

        ex_k = ex_raw / (ex_n + 1e-15)

        ey_k = np.cross(ez_k, ex_k)
        ey_k /= np.linalg.norm(ey_k) + 1e-15

        # re-orthogonalize
        ex_k = np.cross(ey_k, ez_k)
        ex_k /= np.linalg.norm(ex_k) + 1e-15

        R = np.column_stack([ex_k, ey_k, ez_k])
        qk = utils_math.rotation_matrix_to_quaternion(R)
        quat_list.append(qk)

    quat = np.array(quat_list)  # (N,4)

    # continuity
    for k in range(1, N):
        if np.dot(quat[k - 1], quat[k]) < 0.0:
            quat[k] = -quat[k]

    quat = quat.T  # (4,N)
    return pos, quat, t