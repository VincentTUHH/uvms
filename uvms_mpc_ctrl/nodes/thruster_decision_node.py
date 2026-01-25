#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from hippo_control_msgs.msg import ActuatorControls
from std_msgs.msg import Int64, Float64

from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default



class ThrusterDecisionDirector(Node):
    """
    Subscribes:
      - thruster_command_mixer
      - thruster_command_mpc

    Publishes:
      - thruster_command

    Logic:
      - If MPC is fresh (age < mpc_timeout_s): forward MPC
      - Else: forward mixer
      - Publish at publish_rate_hz continuously

    Debug:
      - thruster_decision/mode        (Int64) 0=mixer, 1=mpc
      - thruster_decision/mpc_age_ms  (Float64)
      - thruster_decision/mixer_age_ms(Float64)
    """

    MODE_MIXER = 0
    MODE_MPC = 1
    N_THRUSTERS = 8

    def __init__(self):
        super().__init__('thruster_decision_director')

        # Parameters
        self.declare_parameter('publish_rate_hz', 100.0)     # set to 100.0 if needed
        self.declare_parameter('mpc_timeout_s', 0.5)
        self.declare_parameter('publish_zeros_if_none', False)

        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.mpc_timeout_s = float(self.get_parameter('mpc_timeout_s').value)
        self.publish_zeros_if_none = bool(self.get_parameter('publish_zeros_if_none').value)

        qos_default = qos_profile_system_default
        qos_sensor = qos_profile_sensor_data

        # Publisher (final output)
        self.pub_thruster = self.create_publisher(ActuatorControls, 'thruster_command', qos_sensor)

        # Debug publishers
        self.pub_mode = self.create_publisher(Int64, 'thruster_decision/mode', qos_default)
        self.pub_mpc_age = self.create_publisher(Float64, 'thruster_decision/mpc_age_ms', qos_default)
        self.pub_mixer_age = self.create_publisher(Float64, 'thruster_decision/mixer_age_ms', qos_default)

        # Subscribers (sources)
        self.sub_mixer = self.create_subscription(
            ActuatorControls, 'thruster_command_mixer', self.cb_mixer, qos_sensor
        )
        self.sub_mpc = self.create_subscription(
            ActuatorControls, 'thruster_command_mpc', self.cb_mpc, qos_sensor
        )

        # State
        self.last_mixer_control = None  # list[float] length 8
        self.last_mpc_control = None
        self.last_mixer_time = None     # rclpy.time.Time
        self.last_mpc_time = None

        self._mode = None  # last published mode (0/1)

        # Publish timer
        self.pub_timer = self.create_timer(1.0 / self.publish_rate_hz, self.on_publish)

        self.get_logger().info(
            f"ThrusterDecisionDirector started. publish_rate={self.publish_rate_hz}Hz, "
            f"mpc_timeout={self.mpc_timeout_s}s"
        )

    # -------------------------
    # Callbacks
    # -------------------------
    def cb_mixer(self, msg: ActuatorControls):
        if len(msg.control) != self.N_THRUSTERS:
            self.get_logger().error(f"Mixer ActuatorControls.control has length {len(msg.control)} != {self.N_THRUSTERS}")
            return
        self.last_mixer_control = list(msg.control)
        self.last_mixer_time = self.get_clock().now()

    def cb_mpc(self, msg: ActuatorControls):
        if len(msg.control) != self.N_THRUSTERS:
            self.get_logger().error(f"MPC ActuatorControls.control has length {len(msg.control)} != {self.N_THRUSTERS}")
            return
        self.last_mpc_control = list(msg.control)
        self.last_mpc_time = self.get_clock().now()

    # -------------------------
    # Helpers
    # -------------------------
    def _age_s(self, t) -> float:
        if t is None:
            return float('inf')
        now = self.get_clock().now()
        return (now - t).nanoseconds * 1e-9

    def _set_mode(self, mode_int: int):
        if mode_int != self._mode:
            self._mode = mode_int
            name = "MPC" if mode_int == self.MODE_MPC else "MIXER"
            self.get_logger().warn(f"Switching mode -> {name}")

    # -------------------------
    # Decision + publish
    # -------------------------
    def on_publish(self):
        mpc_age = self._age_s(self.last_mpc_time)
        mixer_age = self._age_s(self.last_mixer_time)

        use_mpc = (self.last_mpc_control is not None) and (mpc_age < self.mpc_timeout_s)
        mode = self.MODE_MPC if use_mpc else self.MODE_MIXER
        self._set_mode(mode)

        src_control = self.last_mpc_control if use_mpc else self.last_mixer_control

        if src_control is None:
            if not self.publish_zeros_if_none:
                return
            src_control = [0.0] * self.N_THRUSTERS

        out = ActuatorControls()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = ""
        out.control = list(src_control)

        self.pub_thruster.publish(out)

        # Debug
        msg_mode = Int64()
        msg_mode.data = int(mode)
        self.pub_mode.publish(msg_mode)

        msg_mpc_age = Float64()
        msg_mpc_age.data = float(mpc_age * 1000.0)
        self.pub_mpc_age.publish(msg_mpc_age)

        msg_mixer_age = Float64()
        msg_mixer_age.data = float(mixer_age * 1000.0)
        self.pub_mixer_age.publish(msg_mixer_age)


def main(args=None):
    rclpy.init(args=args)
    node = ThrusterDecisionDirector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()