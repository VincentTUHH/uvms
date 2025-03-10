import launch
import launch_ros
from ament_index_python.packages import get_package_share_path


def generate_launch_description():
    package_path = get_package_share_path('uvms_kinematic_ctrl')
    config_file_path = str(package_path / 'config/ctrl_params.yaml')
    alpha_model_path = get_package_share_path('alpha_model')
    bluerov_manipulator_base_tf_path = str(
        alpha_model_path / 'config/alpha_base_tf_params_bluerov.yaml'
    )
    manipulator_tf_path = str(alpha_model_path / 'config/alpha_kin_params.yaml')
    manipulator_joint_space_control_config_path = str(
        get_package_share_path('alpha_ctrl') / 'config/ctrl_param.yaml'
    )
    bluerov_pose_control_config_path = str(
        get_package_share_path('bluerov_ctrl')
        / 'config/ctrl_params_real_uvms.yaml'
    )

    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')
    vehicle_name = launch.substitutions.LaunchConfiguration('vehicle_name')

    use_sim_time_launch_arg = launch.actions.DeclareLaunchArgument(
        name='use_sim_time',
        default_value=str(False),
        description='decide if sim time should be used for nodes',
    )
    vehicle_name_launch_arg = launch.actions.DeclareLaunchArgument(
        name='vehicle_name',
        default_value='bluerov',
        description='used for node namespace',
    )

<<<<<<< HEAD
    control_node = launch_ros.actions.Node(
        package='uvms_kinematic_ctrl',
        executable='uvms_kin_ctrl_node',
        namespace=vehicle_name,
        parameters=[
            {'use_sim_time': use_sim_time},
            bluerov_manipulator_base_tf_path,
            manipulator_tf_path,
            config_file_path,
            manipulator_joint_space_control_config_path,
            bluerov_pose_control_config_path,
        ],
        output='screen',
    )
=======

    # the topic name from joint_vel_cmds had to be changed for the Setup from Niklas
    # since the hardware interface on the pi was changed to accept arm_vel_commands instead of joint_vel_commands
    # for the pick and place setup, the remap must not be done, since another node ist started 
    # that merges joint_vel_cmds and gripper_vel_cmds and publishes those to ar_vel_cmds then
    control_node = launch_ros.actions.Node(package='uvms_kinematic_ctrl',
                                           executable='uvms_kin_ctrl_node',
                                           namespace=vehicle_name,
                                           parameters=[{'use_sim_time': use_sim_time}
                                               , bluerov_manipulator_base_tf_path,
                                                       manipulator_tf_path,
                                                       config_file_path,
                                                       manipulator_joint_space_control_config_path,
                                                       bluerov_pose_control_config_path
                                                       ],
                                           output='screen',
                                        #    remappings=[('joint_vel_cmds', 'arm_vel_cmds'),],
                                           )
    
>>>>>>> 3174e6f (Initial Commit)

    return launch.LaunchDescription(
        [use_sim_time_launch_arg, vehicle_name_launch_arg, control_node]
    )
