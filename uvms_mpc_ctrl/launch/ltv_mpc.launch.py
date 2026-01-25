import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description() -> LaunchDescription:

    # --- Launch args (same style as your example) ---
    vehicle_name = LaunchConfiguration('vehicle_name')
    use_sim_time = LaunchConfiguration('use_sim_time')

    use_sim_time_launch_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation(Gazebo) clock if true',
    )

    vehicle_name_launch_arg = DeclareLaunchArgument(
        name='vehicle_name',
        default_value='klopsi00',
        description='Vehicle name used as namespace',
    )

    # --- Nodes ---
    motor_node = Node(
        package='uvms_mpc_ctrl',
        executable='ltv_mpc_node.py',
        namespace=vehicle_name,
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
    )

    wrench_decision_node = Node(
        package='uvms_mpc_ctrl',
        executable='wrench_decision_node.py',
        namespace=vehicle_name,
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
    )

    thruster_decision_node = Node(
        package='uvms_mpc_ctrl',
        executable='thruster_decision_node.py',
        namespace=vehicle_name,
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
    )

    joint_vel_decision_node = Node(
        package='uvms_mpc_ctrl',
        executable='joint_vel_decision_node.py',
        namespace=vehicle_name,
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
    )

    return LaunchDescription([
        use_sim_time_launch_arg,
        vehicle_name_launch_arg,
        motor_node,
        wrench_decision_node,
        thruster_decision_node,
        joint_vel_decision_node,
    ])