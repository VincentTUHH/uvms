#!/usr/bin/env python3

import copy
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data
from hippo_control_msgs.msg import ActuatorSetpoint


class WrenchDecisionDirector(Node):
    """
    Decision director:
      - subscribes: thrust_setpoint_auv, torque_setpoint_auv
                    thrust_setpoint_mpc, torque_setpoint_mpc
      - publishes:  thrust_setpoint, torque_setpoint
      - publishes continuously at publish_rate_hz
      - MPC is used ONLY if BOTH MPC thrust and MPC torque are fresh (age < timeout_s).
      - If EITHER MPC thrust OR MPC torque is stale -> fall back to AUV.
    """

    def __init__(self):
        super().__init__('wrench_decision_director')

        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('timeout_s', 0.5)

        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.timeout_s = float(self.get_parameter('timeout_s').value)

        qos_default = qos_profile_system_default
        qos_sensor = qos_profile_sensor_data

        # Output pubs (keep QoS)
        self.pub_thrust = self.create_publisher(ActuatorSetpoint, 'thrust_setpoint', qos_sensor)
        self.pub_torque = self.create_publisher(ActuatorSetpoint, 'torque_setpoint', qos_sensor)

        # Input subs (keep QoS)
        self.sub_thrust_auv = self.create_subscription(
            ActuatorSetpoint, 'thrust_setpoint_auv', self.cb_thrust_auv, qos_sensor
        )
        self.sub_torque_auv = self.create_subscription(
            ActuatorSetpoint, 'torque_setpoint_auv', self.cb_torque_auv, qos_sensor
        )
        self.sub_thrust_mpc = self.create_subscription(
            ActuatorSetpoint, 'thrust_setpoint_mpc', self.cb_thrust_mpc, qos_sensor
        )
        self.sub_torque_mpc = self.create_subscription(
            ActuatorSetpoint, 'torque_setpoint_mpc', self.cb_torque_mpc, qos_sensor
        )

        # Last messages + receipt times
        self.last_thrust_auv = None
        self.last_torque_auv = None
        self.last_thrust_mpc = None
        self.last_torque_mpc = None

        self.last_thrust_auv_time = None
        self.last_torque_auv_time = None
        self.last_thrust_mpc_time = None
        self.last_torque_mpc_time = None

        self._mode = None  # "mpc" / "auv" for transition logs

        self.pub_timer = self.create_timer(1.0 / self.publish_rate_hz, self.on_publish)

        self.get_logger().info(
            f"WrenchDecisionDirector started. publish_rate={self.publish_rate_hz}Hz, timeout={self.timeout_s}s"
        )

    # -------------------------
    # Helpers
    # -------------------------
    def _age_s(self, t) -> float:
        if t is None:
            return float('inf')
        now = self.get_clock().now()
        return (now - t).nanoseconds * 1e-9

    def _set_mode(self, mode: str):
        if mode != self._mode:
            self._mode = mode
            self.get_logger().warn(f"Switching mode -> {mode.upper()}")

    def _use_mpc(self) -> bool:
        # Need both MPC messages AND both must be fresh
        if self.last_thrust_mpc is None or self.last_torque_mpc is None:
            return False

        thrust_age = self._age_s(self.last_thrust_mpc_time)
        torque_age = self._age_s(self.last_torque_mpc_time)

        return (thrust_age < self.timeout_s) and (torque_age < self.timeout_s)

    # -------------------------
    # Subscriber callbacks
    # -------------------------
    def cb_thrust_auv(self, msg: ActuatorSetpoint):
        self.last_thrust_auv = msg
        self.last_thrust_auv_time = self.get_clock().now()

    def cb_torque_auv(self, msg: ActuatorSetpoint):
        self.last_torque_auv = msg
        self.last_torque_auv_time = self.get_clock().now()

    def cb_thrust_mpc(self, msg: ActuatorSetpoint):
        self.last_thrust_mpc = msg
        self.last_thrust_mpc_time = self.get_clock().now()

    def cb_torque_mpc(self, msg: ActuatorSetpoint):
        self.last_torque_mpc = msg
        self.last_torque_mpc_time = self.get_clock().now()

    # -------------------------
    # Publish timer
    # -------------------------
    def on_publish(self):
        use_mpc = self._use_mpc()
        mode = "mpc" if use_mpc else "auv"
        self._set_mode(mode)

        if use_mpc:
            thrust_in = self.last_thrust_mpc
            torque_in = self.last_torque_mpc
        else:
            thrust_in = self.last_thrust_auv
            torque_in = self.last_torque_auv

        # If selected source hasn't provided anything yet -> do nothing
        if thrust_in is None or torque_in is None:
            return

        # Publish copies so we don't mutate stored incoming messages
        thrust = copy.deepcopy(thrust_in)
        torque = copy.deepcopy(torque_in)

        stamp = self.get_clock().now().to_msg()
        thrust.header.stamp = stamp
        torque.header.stamp = stamp

        self.pub_thrust.publish(thrust)
        self.pub_torque.publish(torque)


def main(args=None):
    rclpy.init(args=args)
    node = WrenchDecisionDirector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()