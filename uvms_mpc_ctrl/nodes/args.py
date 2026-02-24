import numpy as np

SOLVER_ARGS = {
    "fatrop": {
        "opts": {
            "expand": True,
            "print_time": False,
            "structure_detection": "auto",
            "fatrop.print_level": 0,

            "fatrop.max_iter": 20,
            "fatrop.tol": 1e-2,
            "fatrop.mu_init": 1e-3,
            "fatrop.bound_push": 1e-4,
            "fatrop.warm_start_init_point": True,
            "fatrop.warm_start_mult_bound_push": 1e-6,
        }
    },
    "ipopt": {
        "opts": {
            "expand": True,
            "print_time": False,
            "ipopt.print_level": 0,
            "ipopt.sb": "yes",
            "ipopt.file_print_level": 0,

            # linear solver & scaling
            "ipopt.linear_solver": "mumps",
            "ipopt.nlp_scaling_method": "gradient-based",

            # accuracy / speed trade-offs
            "ipopt.tol": 1e-6,
            "ipopt.constr_viol_tol": 1e-6,

            "ipopt.acceptable_tol": 1e-3,
            "ipopt.acceptable_constr_viol_tol": 1e-6,
            "ipopt.acceptable_obj_change_tol": 1e-4,

            "ipopt.mu_strategy": "adaptive",
            "ipopt.mu_init": 1e-4,
            "ipopt.max_iter": 100,

            "ipopt.fast_step_computation": "yes",

            # warm start
            "ipopt.warm_start_init_point": "yes",
            "ipopt.warm_start_bound_push": 1e-8,
            "ipopt.warm_start_mult_bound_push": 1e-6,
            "ipopt.warm_start_slack_bound_push": 1e-8,

            # larger bound_push, bound_frac → stay further from bounds during iterations
            "ipopt.bound_push": 1e-3,   # default is around 1e-2, you can tune
            "ipopt.bound_frac": 1e-3,
        }
    },
}

MPC_SIM_ARGS = {
    # "T_duration": 10.0,    # total simulated time [s]
    # "dt": 0.02,            # sample time [s]
    # "max_steps": np.inf,      # max MPC iterations in run_mpc_ref
    "solver": "fatrop",    # key into SOLVER_ARGS: "fatrop" or "ipopt"
}

# ---------------------------------------------------------------------
# Cost function weights
# Everything that used to be hard-coded in solve_cftoc now lives here.
# ---------------------------------------------------------------------
COST_WEIGHTS = {
    # EEF pose tracking (running cost – currently not used, but configurable)
    "eef_pos_run": np. array([1.0, 1.0, 1.0]),
    "eef_att_run": np. array([1.0, 1.0, 1.0]),

    "w_u_joint": np. array([1.0, 1.0, 1.0, 1.0]),
    "w_u_thruster": 1.0,

    "w_manip_joint0": 1.0,
}

# ---------------------------------------------------------------------
# Normalization factors used inside cost terms
# ---------------------------------------------------------------------
COST_SCALING = {
    # normalize EEF position error by tank dimensions so axes contribute equally
    "eef_pos": np.array([0.05, 0.05, 0.05]),                                        # max Fehler bei 10 cm
    "eef_att": np.array([0.0872 , 0.0872, 0.0872]),                                 # max Fehler 10 deg in rad, der fehler ist |err| = sin(theta/2) ~ theta/2 für kleine winkel, dann sin(10°/2) ~ 0.0872
    "control_effort_thruster": np.array([0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2]),  # max 40% von max PWM
    "control_effort_joint": np.array([0.2, 0.3, 0.5, 0.9]),                      # max 60% von den max joint velociteis max: [0.7, 0.7, 0.7, 1.0] rad/s
    "manip_joint0": np.array([0.485]),                                              # max 45 deg from pi
}

# ---------------------------------------------------------------------
# Limits and fixed geometry data used in NLP constraints
# ---------------------------------------------------------------------
MPC_LIMITS = {
    # Thruster actuation bounds (PWM inflation to avoid saturation)
    "thruster_pwm_inflate": 0.05,  # 5 % margin von thrust in je richtung, also 5 % vor dem max egal in welche richtung hört er auf
    "joint_velocity_inflate": 0.95,  # 5% margin

    # Simple self-collision distances (squared distance constraints)
    "self_collision_vehicle_eef_min_dist": 0.25,   # meters #0.3m dann sind es etwa 5cm margin mehr wie die längste Kante des Bluerovs, wenn auf deser kante der joint liegen würde
    "self_collision_vehicle_elbow_min_dist": 0.23, # meters # 0.3m
    "elbow_offset": 0.18,                          # meters

    # Tank boundaries (before deflation)
    "tank_bounds_min": np.array([0.0, 0.0, -1.5]),
    "tank_bounds_max": np.array([2.0, 4.0, 1.0]), # make upper tank bound disappear
    "tank_vehicle_deflation": np.array([0.228, 0.228, 0.228]) + 0.1,  # BlueROV longest side/2 + margin 10cm
    "tank_eef_deflation": 0.1,               # manipulator safety margin 10cm
}

# Precomputed tank bounds (already deflated)
TANK_BOUNDS = {
    "vehicle_min": MPC_LIMITS["tank_bounds_min"] + MPC_LIMITS["tank_vehicle_deflation"],
    "vehicle_max": MPC_LIMITS["tank_bounds_max"] - MPC_LIMITS["tank_vehicle_deflation"],
    "eef_min": MPC_LIMITS["tank_bounds_min"] + MPC_LIMITS["tank_eef_deflation"],
    "eef_max": MPC_LIMITS["tank_bounds_max"] - MPC_LIMITS["tank_eef_deflation"],
}

# Thruster limits (PWM normalized)
THRUST_LIMITS = 1.0

# ---------------------------------------------------------------------
# Reference trajectory configuration
# ---------------------------------------------------------------------
TRAJ_ARGS = {
    # Select which trajectory generator to use in main_nlp.py.
    # Supported: "line", "circ_oscillation"
    "type": "sine_xz", #"circ_oscillation_3d_radial",

    # Parameters for a simple minimum-jerk line trajectory
    "line": {
        "p_start": np.array([1.0, 1.0, -0.75]),
        "q_start": np.array([1.0, 0.0, 0.0, 0.0]),
        "p_goal":  np.array([1.0, 3.0, -0.75]),
        "q_goal":  np.array([1.0, 0.0, 0.0, 0.0]),
        "fwd_speed": 0.2,  # [m/s] desired speed
    },

    "sine_xz": {
        "p_start": np.array([1.0, 1.0, -0.75]),
        "p_goal":  np.array([1.0, 3.0, -0.75]),
        "n_osc":            2,
        "A":                0.2,
        "fwd_speed":        0.1,  # [m/s] desired speed (see speed_mode in generator)
    },

    # z-axis tangential to trajectory
    # x-axis radial outwards from circle center, in direction from center to trajectory point
    "circ_oscillation_3d_radial": {
        "start_pos":        np.array([1.8, 2.0, -0.75]),
        "radius":           0.80,
        "n_revs":           3.0,
        "A_z":              0.2,
        "n_osc_per_circle": 5,
        "fwd_speed":        0.15,  # [m/s] desired speed (see speed_mode in generator)
    },

    "hold_pose": {
        "p_start": np.array([1.5, 2.0, -0.75]),
        "q_start": np.array([1.0, 0.0, 0.0, 0.0]),
        "duration": 5.0,  # seconds
    },
}
