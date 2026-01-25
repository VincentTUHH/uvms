#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import socket
import numpy as np
from ament_index_python.packages import get_package_share_directory
import os
# from softrobot_msgs.msg import RobotPWM, RobotPose6D
# from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import qos_profile_system_default
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R

from std_msgs.msg import Int64
from std_msgs.msg import Float32, Float64
from alpha_msgs.msg import JointData
from sensor_msgs.msg import JointState
from hippo_control_msgs.msg import VelocityControlTarget
from hippo_control_msgs.msg import ControlTarget
from geometry_msgs.msg import PoseStamped
from hippo_control_msgs.msg import ActuatorSetpoint
from hippo_control_msgs.msg import ActuatorControls
from ltv_mpc_ctrl import CFTOCSolver




import args
import mpc_trajectory_gen
import utils_math
import pprint

import threading
import time
from dataclasses import dataclass


SEND_FREQ = 50.0  # Hz
TIME_STEP = 1.0 / SEND_FREQ  # s
NUM_JOINTS = 4
NUM_MOTORS = 8
N_HORIZON = 20  # prediction horizon steps
N_RUNS = 5
CTRL_PWM = True


# before starting the mpc controller make a hard coded start up controller, like the once in uvms kinematic control,
# that just drives the vehicle and then the manipulator to ausgangspose as in Niklas and then starts the mpc controller
# like in python, where then from the current state a trajectory is generated to the start pose of the trajectory
# and just vorweg getan and die refernz trajektorie. dann wird wie 


@dataclass
class MPCCommands:
    joint_vel: np.ndarray          # (NUM_JOINTS,)
    pwm: np.ndarray | None         # (NUM_MOTORS,) if CTRL_PWM else None
    thrust: np.ndarray | None      # (3,) if not CTRL_PWM else None
    torque: np.ndarray | None      # (3,) if not CTRL_PWM else None
    feasible: bool = True
    constraint_flags: dict | None = None
    solve_time_ms: float | None = None
    cost: float | None = None


class LTVMPCNode(Node):
    def __init__(self):
        super().__init__('ltv_mpc_node')
        self.get_logger().info('Starting ltv mpc node...')

        self._initialized = False
        self._start_controller = False
        self.got_first_feasible = False

        self.last_eef_pos: np.ndarray | None = None
        self.last_eef_att: np.ndarray | None = None
        self.last_eef_pos = np.array([1.0, 1.0, 1.0])
        self.last_eef_att = np.array([1.0, 0.0, 0.0, 0.0])

        self.eef_pos_ref_traj: np.ndarray | None = None
        self.eef_att_ref_traj: np.ndarray | None = None

        self.sample = 0 # for taking current sample step from the reference trajectory

        solver = args.MPC_SIM_ARGS["solver"] 
        solver_cfg = args.SOLVER_ARGS[solver]
        solver_opts = solver_cfg["opts"]

        joint_pos_lim, _, joint_vel_lim, bluerov_params, manipulator_dh_params, alpha_params, path_thruster_model_params = self.load_model_and_joint_params()

        self.mpc_controller = CFTOCSolver(
            dt=TIME_STEP,
            solver=solver,
            solver_opts=solver_opts,
            weights=args.COST_WEIGHTS,
            cost_scaling=args.COST_SCALING,
            n_horizon=N_HORIZON,
            v_bat=15.0,
            bluerov_params=bluerov_params,
            manipulator_dh_params=manipulator_dh_params,
            alpha_params=alpha_params,
            path_thruster_model_params=path_thruster_model_params,
            joint_pos_lim=joint_pos_lim,
            joint_vel_lim=joint_vel_lim,
            tank_bounds=args.TANK_BOUNDS,
            thrust_limits=args.THRUST_LIMITS,
            mpc_limits=args.MPC_LIMITS,
            ctrl_pwm=CTRL_PWM,
        )

        # --- State (updated by subscriptions) ---
        self.state_lock = threading.Lock()
        self.state_joint = np.zeros(4, dtype=float)  # [q0..q3]
        self.state_vehicle = np.zeros(6 + 7, dtype=float)  # [v_lin(3), w(3) | p(3), quat(wxyz)(4)]
        self.have_odom = False
        self.have_joints = False

        # --- MPC command buffer (read by timer, written by MPC thread) ---
        self.cmd_lock = threading.Lock()
        self.last_cmd = MPCCommands(
            joint_vel=np.zeros(NUM_JOINTS, dtype=float),
            pwm=np.zeros(NUM_MOTORS, dtype=float) if CTRL_PWM else None,
            thrust=np.zeros(3, dtype=float) if not CTRL_PWM else None,
            torque=np.zeros(3, dtype=float) if not CTRL_PWM else None,
            feasible=True, constraint_flags=None, solve_time_ms=None,
            cost=None
        )

        # --- MPC execution control ---
        self.mpc_busy = threading.Event()
        self.last_solve_stamp = self.get_clock().now()
        self.mpc_min_period = 0.0  # seconds; set >0 if you want to throttle solves (e.g. 0.04 for 25 Hz)

        self.init_publishers()
        self.init_mpc()
        self.init_subscribers()
        self.init_timers()
        self.get_logger().info('ltv mpc startup finished.')

    def load_model_and_joint_params(self):
        self.declare_parameter("file_joint_limits", 'alpha_joint_lim_real.yaml')
        file_joint_limits = self.get_parameter("file_joint_limits").get_parameter_value().string_value
        path_joint_limits = os.path.join(
            get_package_share_directory('uvms_mpc_ctrl'),
            'src',
            file_joint_limits
        )

        joint_pos_lim, joint_eefort_lim, joint_vel_lim, _ = utils_math.load_joint_limits(path_joint_limits)
        joint_pos_lim = np.array(joint_pos_lim).T
        joint_eefort_lim = np.array(joint_eefort_lim).T
        joint_vel_lim = np.array(joint_vel_lim).T

        self.get_logger().info("Joint position limits received.")
        # self.get_logger().info(pprint.pformat(joint_pos_lim.tolist()))
        self.get_logger().info("Joint effort limits received.")
        # self.get_logger().info(pprint.pformat(joint_eefort_lim.tolist()))
        self.get_logger().info("Joint velocity limits received.")
        # self.get_logger().info(pprint.pformat(joint_vel_lim.tolist()))


        self.declare_parameter("file_bluerov_params", 'model_params.yaml')
        file_bluerov_params = self.get_parameter("file_bluerov_params").get_parameter_value().string_value
        path_bluerov_params = os.path.join(
            get_package_share_directory('uvms_mpc_ctrl'),
            'src',
            file_bluerov_params
        )

        bluerov_params = utils_math.load_model_params(path_bluerov_params)

        self.get_logger().info("BlueROV parameters received.")
        # self.get_logger().info(pprint.pformat(bluerov_params))


        self.declare_parameter("file_dh_params", 'alpha_kin_params.yaml')
        file_dh_params = self.get_parameter("file_dh_params").get_parameter_value().string_value
        path_dh_params = os.path.join(
            get_package_share_directory('uvms_mpc_ctrl'),
            'src',
            file_dh_params
        )

        manipulator_dh_params = utils_math.load_dh_params(path_dh_params)

        self.get_logger().info("Manipulator DH parameters received.")
        # self.get_logger().info(pprint.pformat(manipulator_dh_params))


        self.declare_parameter("file_alpha_base_tf_params_bluerov", 'alpha_base_tf_params_bluerov.yaml')
        file_alpha_base_tf_params_bluerov = self.get_parameter("file_alpha_base_tf_params_bluerov").get_parameter_value().string_value
        path_alpha_base_tf_params_bluerov = os.path.join(
            get_package_share_directory('uvms_mpc_ctrl'),
            'src',
            file_alpha_base_tf_params_bluerov
        )

        self.declare_parameter("file_alpha_inertial_params_dh", 'alpha_inertial_params_dh.yaml')
        file_alpha_inertial_params_dh = self.get_parameter("file_alpha_inertial_params_dh").get_parameter_value().string_value
        path_alpha_inertial_params_dh = os.path.join(
            get_package_share_directory('uvms_mpc_ctrl'),
            'src',
            file_alpha_inertial_params_dh
        )

        manipulator_dyn_params_paths = [path_dh_params, 
                                        path_alpha_base_tf_params_bluerov, 
                                        path_alpha_inertial_params_dh,
                                        ]

        alpha_params = utils_math.load_dynamic_params(manipulator_dyn_params_paths)

        self.get_logger().info("Manipulator dynamic parameters received.")
        # self.get_logger().info(pprint.pformat(alpha_params))

        self.declare_parameter("file_thruster_model_params", 'thruster_inversepoly_deg2.npz')
        file_thruster_model_params = self.get_parameter("file_thruster_model_params").get_parameter_value().string_value
        path_thruster_model_params = os.path.join(
            get_package_share_directory('uvms_mpc_ctrl'),
            'src',
            file_thruster_model_params
        )

        return joint_pos_lim, joint_eefort_lim, joint_vel_lim, bluerov_params, manipulator_dh_params, alpha_params, path_thruster_model_params

    def init_mpc(self):
        self.get_logger().info("Initializing MPC trajectory...")
        traj_type = args.TRAJ_ARGS.get("type", "line")
        eef_pos_ref, eef_att_ref = mpc_trajectory_gen.build_eef_reference_trajectory(TIME_STEP, traj_type)
        
        ref_traj_length = eef_pos_ref.shape[1]
        msg = Int64()
        msg.data = ref_traj_length
        self.start_sequence_length_pub.publish(msg)

        pos_back, att_back = mpc_trajectory_gen.back_to_start_with_wait(
            ref_eef_pos_run=eef_pos_ref,
            ref_eef_att_run=eef_att_ref,
            dt=TIME_STEP,
            v_max=0.1,
            N_wait_begin=10 * N_HORIZON,
            N_wait_end=10 * N_HORIZON,
        )
        self.get_logger().info(f"Back traj length: {pos_back.shape}")

        back_traj_length = pos_back.shape[1]
        msg = Int64()
        msg.data = back_traj_length
        self.start_sequence_length_pub.publish(msg)

        self.eef_pos_ref_traj = eef_pos_ref
        self.eef_att_ref_traj = eef_att_ref

        for _ in range(N_RUNS - 1):
            self.eef_pos_ref_traj = np.hstack([self.eef_pos_ref_traj, pos_back, eef_pos_ref])
            self.eef_att_ref_traj = np.hstack([self.eef_att_ref_traj, att_back, eef_att_ref])
        
        self.get_logger().info("MPC trajectory initialized.")
        self.get_logger().info(f"Size of eef_pos_ref_traj: {self.eef_pos_ref_traj.shape}")
        self.get_logger().info(f"Size of eef_att_ref_traj: {self.eef_att_ref_traj.shape}")

    def init_publishers(self):
        qos_default = qos_profile_system_default
        qos_sensor = qos_profile_sensor_data

        self.manipulator_cmd_pub = self.create_publisher( # desired joint velocities for the manipulator
            JointData,
            "joint_vel_cmds_mpc",
            qos_default
        )

        self.actuator_cmd_pub = self.create_publisher( # 8 thruster commands in [-1,1]
            ActuatorControls,
            "thruster_command_mpc",
            qos_sensor # the mixer requires this qos
        ) # "thruster_values" ist in 1100-1900 pwm werte, das macht der esc commander, der die thruster_command umwandelt

        self.thrust_pub = self.create_publisher( # force part of the vehicle wrenhc tau_v
            ActuatorSetpoint,
            "thrust_setpoint_mpc", # the mixer requires this qos
            qos_sensor
        )

        self.torque_pub = self.create_publisher( # torque part of the vehicle wrench tau_v
            ActuatorSetpoint,
            "torque_setpoint_mpc", # the mixer requires this qos
            qos_sensor
        )

        self.eef_pose_ref_pub = self.create_publisher( # desired end-effector pose for visualization
            PoseStamped,
            "pose_eef_ref",
            qos_default
        )

        self.start_sequence_length_pub = self.create_publisher(
            Int64,
            'start_sequence_length',
            qos_default
        )

        self.constraint_joint_pos_pub = self.create_publisher(
            Int64,
            'mpc_constraint/joint_pos_violations',
            qos_default
        )

        self.constraint_self_collision_eef_pub = self.create_publisher(
            Int64,
            'mpc_constraint/self_collision_eef_violations',
            qos_default
        )

        self.constraint_self_collision_elbow_pub = self.create_publisher(
            Int64,
            'mpc_constraint/self_collision_elbow_violations',
            qos_default
        )

        self.constraint_collision_vehicle_pub = self.create_publisher(
            Int64,
            'mpc_constraint/collision_vehicle_violations',
            qos_default
        )

        self.constraint_collision_eef_pub = self.create_publisher(
            Int64,
            'mpc_constraint/collision_eef_violations',
            qos_default
        )

        self.optimization_cost_pub = self.create_publisher(
            Float64,
            'mpc_optimization/cost_value',
            qos_default
        )

        self.optimization_time_pub = self.create_publisher(
            Float32,
            'mpc_optimization/solve_time_ms',
            qos_default
        )

    def init_subscribers(self):
        qos_default = qos_profile_system_default

        self.odometry_sub = self.create_subscription(
            Odometry,
            "odometry",
            self.on_odometry,
            qos_default
        )

        self.state_sub_manipulator = self.create_subscription(
            JointState,
            'joint_states',
            self.on_joint_state,
            qos_default
        )

        self.eef_pose_sub = self.create_subscription(
            PoseStamped,
            'pose_endeffector',
            self.on_eef_pose,
            qos_default
        )

        self.eef_traj_sub = self.create_subscription(
            ControlTarget,
            'traj_setpoint',
            self.on_start_controller,
            qos_default
        )

    def init_timers(self):
        # will call the function control_loop at SEND_FREQ Hz
        self.get_logger().info("Creating control timer...")
        self.control_timer = self.create_timer(1.0 / SEND_FREQ, self.control_loop)

    def on_odometry(self, msg: Odometry):
        # Twist (velocity)
        v = msg.twist.twist.linear
        w = msg.twist.twist.angular

        # Pose
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation  # ROS: x,y,z,w

        quat_wxyz = np.array([q.w, q.x, q.y, q.z], dtype=float)
        n = np.linalg.norm(quat_wxyz)
        if n > 1e-12:
            quat_wxyz /= n

        with self.state_lock:
            # state layout:
            # [0:6] vehicle vel (v_lin xyz, w xyz)
            # [6:9] position xyz
            # [9:13] quaternion wxyz
            self.state_vehicle[0:3] = [v.x, v.y, v.z]
            self.state_vehicle[3:6] = [w.x, w.y, w.z]
            self.state_vehicle[6:9] = [p.x, p.y, p.z]
            self.state_vehicle[9:13] = quat_wxyz

        self.have_odom = True

    def on_joint_state(self, msg: JointState):
        with self.state_lock:
            if len(msg.position) >= 4:  # Ensure there are at least 4 joint positions
                self.state_joint[0] = msg.position[0]
                self.state_joint[1] = msg.position[1]
                self.state_joint[2] = msg.position[2]
                self.state_joint[3] = msg.position[3]
            else:
                self.get_logger().warn("Received JointState message with insufficient positions.")

        self.have_joints = True

    def publish_zero_commands(self):
        stamp = self.get_clock().now().to_msg()

        # Manipulator: JointData expects NUM_JOINTS+1 in your code (why +1? gripper maybe)
        zero_joint_data = JointData()
        zero_joint_data.header.stamp = stamp
        zero_joint_data.data = [0.0] * (NUM_JOINTS + 1)
        self.manipulator_cmd_pub.publish(zero_joint_data)

        if CTRL_PWM:
            zero_actuator_controls = ActuatorControls()
            zero_actuator_controls.header.stamp = stamp
            zero_actuator_controls.control = [0.0] * NUM_MOTORS
            self.actuator_cmd_pub.publish(zero_actuator_controls)
        else:
            # publish wrench split
            tmsg = ActuatorSetpoint()
            tmsg.header.stamp = stamp
            tmsg.x, tmsg.y, tmsg.z = 0.0, 0.0, 0.0
            self.thrust_pub.publish(tmsg)

            rmsg = ActuatorSetpoint()
            rmsg.header.stamp = stamp
            rmsg.x, rmsg.y, rmsg.z = 0.0, 0.0, 0.0
            self.torque_pub.publish(rmsg)

    def publish_last_commands(self):
        # Read buffered cmd atomically
        with self.cmd_lock:
            cmd = self.last_cmd

        stamp = self.get_clock().now().to_msg()

        # Manipulator joint vel
        joint_msg = JointData()
        joint_msg.header.stamp = stamp
        joint_msg.data = list(cmd.joint_vel.astype(float)) + [0.0]  # keep your +1 slot as 0.0
        self.manipulator_cmd_pub.publish(joint_msg)

        if CTRL_PWM:
            act = ActuatorControls()
            act.header.stamp = stamp
            act.control = list(cmd.pwm.astype(float))
            self.actuator_cmd_pub.publish(act)
        else:
            tmsg = ActuatorSetpoint()
            tmsg.header.stamp = stamp
            tmsg.x, tmsg.y, tmsg.z = float(cmd.thrust[0]), float(cmd.thrust[1]), float(cmd.thrust[2])
            self.thrust_pub.publish(tmsg)

            rmsg = ActuatorSetpoint()
            rmsg.header.stamp = stamp
            rmsg.x, rmsg.y, rmsg.z = float(cmd.torque[0]), float(cmd.torque[1]), float(cmd.torque[2])
            self.torque_pub.publish(rmsg)

        if cmd.constraint_flags is not None:
            self.publish_constraint_violations(cmd.constraint_flags)
        if cmd.cost is not None:
            self.publish_cost_value(cmd.cost)
        if cmd.solve_time_ms is not None:
            self.publish_solve_time(cmd.solve_time_ms)

    def try_start_mpc_solve(self):
        if self.mpc_busy.is_set():
            return

        # Optional throttling (e.g. if you want solve <= 25 Hz)
        # now = self.get_clock().now()
        # dt = (now - self.last_solve_stamp).nanoseconds * 1e-9
        # if dt < self.mpc_min_period:
        #     return

        # Need valid state
        if not (self.have_odom and self.have_joints):
            return

        # Need reference available
        if self.eef_pos_ref_traj is None or self.eef_att_ref_traj is None:
            return

        # If reference is done, don't solve anymore
        if self.sample >= self.eef_pos_ref_traj.shape[1]:
            return
        
        self.mpc_busy.set()

        # Snapshot (state + current reference index) atomically
        with self.state_lock:
            current_joint = self.state_joint.copy()
            current_odometry = self.state_vehicle.copy()
        x0 = np.hstack([current_joint, current_odometry])

        sample_idx = int(self.sample)

        self.last_solve_stamp = self.get_clock().now()
        threading.Thread(
            target=self._mpc_worker,
            args=(x0, sample_idx),
            daemon=True
        ).start()

    def _mpc_worker(self, x0: np.ndarray, sample_idx: int):
        try:
            # Build reference slice for the horizon (pad with last sample if near end)
            pos_ref = self.eef_pos_ref_traj
            att_ref = self.eef_att_ref_traj

            T = pos_ref.shape[1]
            end = min(sample_idx + N_HORIZON, T)

            pos_slice = pos_ref[:, sample_idx:end]
            att_slice = att_ref[:, sample_idx:end]

            if pos_slice.shape[1] < N_HORIZON:
                # pad with last column to length N
                pad_n = N_HORIZON - pos_slice.shape[1]
                pos_last = pos_slice[:, -1:].repeat(pad_n, axis=1) if pos_slice.shape[1] > 0 else pos_ref[:, -1:].repeat(N_HORIZON, axis=1)
                att_last = att_slice[:, -1:].repeat(pad_n, axis=1) if att_slice.shape[1] > 0 else att_ref[:, -1:].repeat(N_HORIZON, axis=1)
                pos_slice = np.hstack([pos_slice, pos_last])
                att_slice = np.hstack([att_slice, att_last])

            # ---- CALL YOUR SOLVER HERE ----
            t0 = time.perf_counter()
            uq, uv, constraint_flags, J_opt = self.mpc_controller.get_ctrl_cmd(x0, 
                                                                                pos_slice, 
                                                                                att_slice)
            # self.get_logger().error(f"type(uq)={type(uq)}, type(uv)={type(uv)}, uv={uv}")
            t1 = time.perf_counter()
            solve_time_ms = (t1 - t0) * 1000.0

            self.get_logger().info(f"MPC solve time: {solve_time_ms:.1f} ms")

            feasible = (uq is not None)

            if not feasible:
                uq = np.zeros(NUM_JOINTS)
                if CTRL_PWM:
                    uv = np.zeros(NUM_MOTORS)
                else:
                    uv = (np.zeros(3), np.zeros(3))
                    
            
            if CTRL_PWM:
                u_pwm = uv
                new_cmd = MPCCommands(joint_vel=uq, pwm=u_pwm, thrust=None, torque=None, feasible=feasible,
                                      constraint_flags=constraint_flags,
                                      solve_time_ms=solve_time_ms,
                                      cost=J_opt)
            else:
                u_thrust, u_torque = uv
                new_cmd = MPCCommands(joint_vel=uq, pwm=None, thrust=u_thrust, torque=u_torque, feasible=feasible,
                                      constraint_flags=constraint_flags,
                                      solve_time_ms=solve_time_ms,
                                      cost=J_opt)


            # Update shared cmd buffer
            with self.cmd_lock:
                # If infeasible, you can choose:
                # - keep last_cmd (do nothing) OR
                # - overwrite with zeros OR
                # - overwrite with safe fallback
                if new_cmd.feasible:
                    self.last_cmd = new_cmd
                    if not self.got_first_feasible:
                        self.got_first_feasible = True
                        self.get_logger().info(f"Got first feasible MPC solution. It took {solve_time_ms:.1f} ms.")

        except Exception as e:
            self.get_logger().error(f"MPC worker failed: {e}")
        finally:
            self.mpc_busy.clear()

    def control_loop(self):
        if not self._start_controller:
            return

        # 1) Publish reference pose for visualization and advance sample
        if self.eef_pos_ref_traj is not None and self.eef_att_ref_traj is not None:
            if self.sample < self.eef_pos_ref_traj.shape[1]:
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = "map"

                pose_msg.pose.position.x = float(self.eef_pos_ref_traj[0, self.sample])
                pose_msg.pose.position.y = float(self.eef_pos_ref_traj[1, self.sample])
                pose_msg.pose.position.z = float(self.eef_pos_ref_traj[2, self.sample])

                quat = self.eef_att_ref_traj[:, self.sample]  # wxyz
                pose_msg.pose.orientation.x = float(quat[1])
                pose_msg.pose.orientation.y = float(quat[2])
                pose_msg.pose.orientation.z = float(quat[3])
                pose_msg.pose.orientation.w = float(quat[0])

                self.eef_pose_ref_pub.publish(pose_msg)

                # advance reference sample at SEND_FREQ
                self.sample += 1
            else:
                # reference finished -> publish zeros and stop solving
                self.publish_zero_commands()
                return

        # 2) Start MPC solve if possible (non-blocking)
        self.try_start_mpc_solve()

        # 3) Publish latest available commands (last_cmd)
        # If MPC isn't finished in time, this republishes the previous command.
        if not self.got_first_feasible:
            self.publish_zero_commands()
        else:
            self.publish_last_commands()

    def on_eef_pose(self, msg: PoseStamped):
        p = msg.pose.position
        q = msg.pose.orientation  # ROS quaternion order is (x,y,z,w)

        self.last_eef_pos = np.array([p.x, p.y, p.z], dtype=float)
        self.last_eef_att = np.array([q.w, q.x, q.y, q.z], dtype=float)

        # Optional but recommended: normalize quaternion to be safe
        n = np.linalg.norm(self.last_eef_att)
        if n > 1e-12:
            self.last_eef_att /= n

        # self.get_logger().info(f"Received EEF pose: pos={self.last_eef_pos}, att={self.last_eef_att}")

    def on_start_controller(self, msg: ControlTarget):
        if self._start_controller:
            return

        # Append the connecting trajectory to the reference trajectory
        eef_pos_ref_traj_start, eef_att_ref_traj_start = mpc_trajectory_gen.make_eef_connecting_traj_with_wait(
            self.last_eef_pos,
            self.last_eef_att,
            self.eef_pos_ref_traj,
            self.eef_att_ref_traj,
            TIME_STEP,
            0.1,  # max approach speed [m/s]
            10 * N_HORIZON  # wait time at the start pose
        )

        self.start_sequence_length = eef_pos_ref_traj_start.shape[1]

        msg = Int64()
        msg.data = self.start_sequence_length
        self.start_sequence_length_pub.publish(msg)


        self.eef_pos_ref_traj = np.hstack([eef_pos_ref_traj_start, self.eef_pos_ref_traj])
        self.eef_att_ref_traj = np.hstack([eef_att_ref_traj_start, self.eef_att_ref_traj]) 

        self._start_controller = True
        self.get_logger().info("Starting LTV MPC Controller...")  
        
    def publish_constraint_violations(self, violations: dict):
        msg_joint_pos = Int64()
        msg_joint_pos.data = violations.get("active_joint", 0)
        self.constraint_joint_pos_pub.publish(msg_joint_pos)

        msg_self_collision_eef = Int64()
        msg_self_collision_eef.data = violations.get("self_collision_eef", 0)
        self.constraint_self_collision_eef_pub.publish(msg_self_collision_eef)

        msg_self_collision_elbow = Int64()
        msg_self_collision_elbow.data = violations.get("self_collision_elbow", 0)
        self.constraint_self_collision_elbow_pub.publish(msg_self_collision_elbow)

        msg_collision_vehicle = Int64()
        msg_collision_vehicle.data = violations.get("collision_vehicle", 0)
        self.constraint_collision_vehicle_pub.publish(msg_collision_vehicle)

        msg_collision_eef = Int64()
        msg_collision_eef.data = violations.get("collision_eef", 0)
        self.constraint_collision_eef_pub.publish(msg_collision_eef)   

    def publish_cost_value(self, cost: float):
        msg = Float64()
        msg.data = float(cost)
        self.optimization_cost_pub.publish(msg)

    def publish_solve_time(self, time_ms: float):
        msg = Float32()
        msg.data = float(time_ms)
        self.optimization_time_pub.publish(msg)

    def destroy_node(self):
        self.get_logger().info("Shutting down, closing socket.")
        if hasattr(self, 'sock'):
            self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LTVMPCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (Ctrl+C)")
    finally:
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()