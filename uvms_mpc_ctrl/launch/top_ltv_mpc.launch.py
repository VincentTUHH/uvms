import launch
import launch_ros

from launch.actions import GroupAction
from launch_ros.actions import SetRemap
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_path


def generate_launch_description():
    # --- Package paths ---
    sim_package_path = get_package_share_path('uvms_sim')
    tf_tree_model_path = str(sim_package_path / 'models/urdf/uvms_rviz.urdf.xacro')

    alpha_model_path = get_package_share_path('alpha_model')
    alpha_estimation_path = get_package_share_path('alpha_estimation')
    alpha_ctrl_path = get_package_share_path('alpha_ctrl')

    bluerov_ctrl_path = get_package_share_path('bluerov_ctrl')
    bluerov_low_level_ctrl_path = get_package_share_path('hippo_control')
    bluerov_acceleration_estimation_path = get_package_share_path('bluerov_estimation')

    uvms_kin_ctrl_path = get_package_share_path('uvms_kinematic_ctrl')
    uvms_trajectory_gen_path = get_package_share_path('uvms_trajectory_gen')
    uvms_visualization_path = get_package_share_path('uvms_visualization')

    # --- NEW: MPC package path ---
    uvms_mpc_ctrl_path = get_package_share_path('uvms_mpc_ctrl')
    ltv_mpc_launch_path = str(uvms_mpc_ctrl_path / 'launch/ltv_mpc.launch.py')

    # --- Mixer files ---
    mixer_path = str(bluerov_low_level_ctrl_path / 'launch/node_actuator_mixer_bluerov.launch.py')
    mixer_config_file_path = str(bluerov_low_level_ctrl_path / 'config/actuator_mixer_bluerov_advanced.yaml')

    # --- Estimation drift watchdog ---
    estimation_watchdog_path = str(uvms_kin_ctrl_path / 'launch/node_estimation_drift_watchdog.launch.py')

    # --- Launch configs ---
    vehicle_name = 'klopsi00'
    use_sim_time = False
    use_hydro = True

    # --- Includes ---
    alpha_estimation = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(alpha_estimation_path / 'launch/estimation.launch.py')),
        launch_arguments={
            'vehicle_name': vehicle_name,
            'use_hydro': str(use_hydro),
            'use_sim_time': str(use_sim_time),
        }.items(),
    )

    alpha_force_torque_calc = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(alpha_model_path / 'launch/dyn_calc.launch.py')),
        launch_arguments={
            'vehicle_name': vehicle_name,
            'use_hydrodynamics': str(use_hydro),
            'base_tf_file': str(alpha_model_path / 'config/alpha_base_tf_params_bluerov.yaml'),
            'moving_base': 'true',
            'use_sim_time': str(use_sim_time),
        }.items(),
    )

    bluerov_acceleration_estimation = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bluerov_acceleration_estimation_path / 'launch/estimation.launch.py')),
        launch_arguments={
            'use_sim_time': str(use_sim_time),
            'vehicle_name': vehicle_name,
        }.items(),
    )

    # --- Scoped remaps for BlueROV controller include ---
    bluerov_ctrl = GroupAction([
        SetRemap(src='thrust_setpoint', dst='thrust_setpoint_auv'),
        SetRemap(src='torque_setpoint', dst='torque_setpoint_auv'),

        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bluerov_ctrl_path / 'launch/node_velocity_control.launch.py')),
            launch_arguments={
                'use_sim_time': str(use_sim_time),
                'vehicle_name': vehicle_name,
                'controller_type': '1',  # REAL
                'config_file': str(bluerov_ctrl_path / 'config/ctrl_params_real_uvms.yaml'),
            }.items(),
        ),
    ])

    estimation_drift_watchdog = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(estimation_watchdog_path),
        launch_arguments={
            'use_sim_time': str(use_sim_time),
            'vehicle_name': vehicle_name,
        }.items(),
    )

    # --- Scoped remaps for mixer include ---
    bluerov_mixer = GroupAction([
        SetRemap(src='thruster_command', dst='thruster_command_mixer'),

        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mixer_path),
            launch_arguments={
                'use_sim_time': str(use_sim_time),
                'vehicle_name': vehicle_name,
                'mixer_path': mixer_config_file_path,
            }.items(),
        ),
    ])

    # --- Scoped remaps for kinematic controller include ---
    uvms_kinematic_ctrl = GroupAction([
        SetRemap(src='joint_vel_cmds', dst='joint_vel_cmds_kin'),

        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(uvms_kin_ctrl_path / 'launch/uvms_kin_ctrl.launch.py')),
            launch_arguments={
                'vehicle_name': vehicle_name,
                'use_sim_time': str(use_sim_time),
            }.items(),
        ),
    ])

    uvms_trajectory_gen = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(uvms_trajectory_gen_path / 'launch/traj_gen.launch.py')),
        launch_arguments={
            'vehicle_name': vehicle_name,
            'use_sim_time': str(use_sim_time),
        }.items(),
    )

    # --- robot_state_publisher (kept as in your real file) ---
    state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=vehicle_name,
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'robot_description': launch_ros.descriptions.ParameterValue(
                    launch.substitutions.Command(
                        [
                            'xacro ',
                            tf_tree_model_path,
                            ' ',
                            'vehicle_name:=',
                            vehicle_name,
                        ]
                    ),
                    value_type=str,
                ),
            }
        ],
    )

    # --- TF publisher vehicle (kept as in your real file) ---
    tf_publisher_vehicle = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(get_package_share_path('hippo_common') / 'launch/tf_publisher_hippo.launch.py')
        ),
        launch_arguments={
            'vehicle_name': vehicle_name,
            'use_sim_time': str(use_sim_time),
        }.items(),
    )

    uvms_visualization = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(uvms_visualization_path / 'launch/visualization.launch.py')),
        launch_arguments={
            'visualization_modules': '[1, 2, 4, 5, 7, 8, 9]',
            'vehicle_name': vehicle_name,
            'use_sim_time': str(use_sim_time),
        }.items(),
    )

    rviz = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(uvms_kin_ctrl_path / 'launch/rviz.launch.py')),
        launch_arguments={'use_sim_time': str(use_sim_time)}.items(),
    )

    velocity_command = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(alpha_ctrl_path / 'launch/velocity_command.launch.py')),
        launch_arguments={
            'vehicle_name': vehicle_name,
            'use_sim_time': str(use_sim_time),
        }.items(),
    )

    # --- NEW: MPC include (same pattern as your sim refactor) ---
    ltv_mpc = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ltv_mpc_launch_path),
        launch_arguments={
            'vehicle_name': vehicle_name,
            'use_sim_time': str(use_sim_time),
        }.items(),
    )

    return launch.LaunchDescription(
        [
            alpha_estimation,
            alpha_force_torque_calc,
            bluerov_acceleration_estimation,

            bluerov_ctrl,
            bluerov_mixer,

            uvms_kinematic_ctrl,
            # estimation_drift_watchdog,  # (now actually included)

            uvms_trajectory_gen,

            # NEW
            ltv_mpc,

            state_publisher,
            tf_publisher_vehicle,

            uvms_visualization,
            rviz,
            velocity_command,
        ]
    )