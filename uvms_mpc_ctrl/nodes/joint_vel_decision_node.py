#!/usr/bin/env python3

import copy
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from alpha_msgs.msg import JointData


class JointVelDecisionDirector(Node):
    """
    Decision director for joint velocity commands:

    Subscribes:
      - joint_vel_cmds_kin   (non-MPC source)
      - joint_vel_cmds_mpc   (MPC source)

    Publishes:
      - joint_vel_cmds       (final output)

    Logic:
      - If MPC is fresh (age < timeout_s): forward MPC
      - Else: forward KIN
      - Publish continuously at 50 Hz

    QoS:
      - Uses default QoS for all pubs/subs (qos_profile_system_default)
    """

    def __init__(self):
        super().__init__('joint_vel_decision_director')

        # Parameters
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('timeout_s', 0.5)

        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.timeout_s = float(self.get_parameter('timeout_s').value)

        qos_default = qos_profile_system_default

        # Publisher (final output)
        self.pub_joint_vel = self.create_publisher(JointData, 'joint_vel_cmds', qos_default)

        # Subscribers (sources)
        self.sub_kin = self.create_subscription(
            JointData, 'joint_vel_cmds_kin', self.cb_kin, qos_default
        )
        self.sub_mpc = self.create_subscription(
            JointData, 'joint_vel_cmds_mpc', self.cb_mpc, qos_default
        )

        # State
        self.last_kin = None
        self.last_mpc = None
        self.last_kin_time = None
        self.last_mpc_time = None

        self._mode = None  # "mpc" / "kin" (for transition logs)

        # Publish timer
        self.pub_timer = self.create_timer(1.0 / self.publish_rate_hz, self.on_publish)

        self.get_logger().info(
            f"JointVelDecisionDirector started. publish_rate={self.publish_rate_hz}Hz, timeout={self.timeout_s}s"
        )

    # -------------------------
    # Helpers
    # -------------------------
    def _age_s(self, t) -> float:
        if t is None:
            return float('inf')
        now = self.get_clock().now()
        return (now - t).nanoseconds * 1e-9

    def _use_mpc(self) -> bool:
        if self.last_mpc is None:
            return False
        return self._age_s(self.last_mpc_time) < self.timeout_s

    def _set_mode(self, mode: str):
        if mode != self._mode:
            self._mode = mode
            self.get_logger().warn(f"Switching mode -> {mode.upper()}")

    # -------------------------
    # Subscriber callbacks
    # -------------------------
    def cb_kin(self, msg: JointData):
        self.last_kin = msg
        self.last_kin_time = self.get_clock().now()

    def cb_mpc(self, msg: JointData):
        self.last_mpc = msg
        self.last_mpc_time = self.get_clock().now()

    # -------------------------
    # Publish timer
    # -------------------------
    def on_publish(self):
        use_mpc = self._use_mpc()
        mode = "mpc" if use_mpc else "kin"
        self._set_mode(mode)

        src = self.last_mpc if use_mpc else self.last_kin

        # If selected source hasn't provided anything yet -> do nothing
        if src is None:
            return

        # Publish a copy so we don't mutate stored incoming messages
        out = copy.deepcopy(src)
        out.header.stamp = self.get_clock().now().to_msg()

        self.pub_joint_vel.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = JointVelDecisionDirector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()