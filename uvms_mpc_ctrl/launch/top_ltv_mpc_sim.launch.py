import launch
from launch.actions import GroupAction
from launch_ros.actions import SetRemap
from ament_index_python.packages import get_package_share_path


def generate_launch_description():
    # --- Package paths ---
    alpha_model_path = get_package_share_path('alpha_model')
    alpha_estimation_path = get_package_share_path('alpha_estimation')
    bluerov_ctrl_path = get_package_share_path('bluerov_ctrl')
    bluerov_low_level_ctrl_path = get_package_share_path('hippo_control')
    bluerov_acceleration_estimation_path = get_package_share_path('bluerov_estimation')
    alpha_ctrl_path = get_package_share_path('alpha_ctrl')

    uvms_mpc_ctrl_path = get_package_share_path('uvms_mpc_ctrl')
    uvms_kin_ctrl_path = get_package_share_path('uvms_kinematic_ctrl')
    uvms_trajectory_gen_path = get_package_share_path('uvms_trajectory_gen')
    uvms_visualization_path = get_package_share_path('uvms_visualization')

    # --- Mixer files ---
    mixer_path = str(bluerov_low_level_ctrl_path / 'launch/node_actuator_mixer_bluerov.launch.py')
    mixer_config_file_path = str(bluerov_low_level_ctrl_path / 'config/actuator_mixer_bluerov_advanced.yaml')

    # --- Estimation drift watchdog (comes from uvms_kinematic_ctrl as before) ---
    estimation_watchdog_path = str(uvms_kin_ctrl_path / 'launch/node_estimation_drift_watchdog.launch.py')

    # --- NEW: include MPC launch from uvms_mpc_ctrl ---
    ltv_mpc_launch_path = str(uvms_mpc_ctrl_path / 'launch/ltv_mpc.launch.py')

    # --- Launch configurations (args) ---
    # vehicle_name = launch.substitutions.LaunchConfiguration('vehicle_name')
    # use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')
    # use_hydro = launch.substitutions.LaunchConfiguration('use_hydro')
    vehicle_name = 'klopsi00'
    use_sim_time = True
    use_hydro = True

    # vehicle_name_launch_arg = launch.actions.DeclareLaunchArgument(
    #     name='vehicle_name',
    #     default_value='klopsi00',
    #     description='Vehicle name used as namespace',
    # )

    # use_sim_time_launch_arg = launch.actions.DeclareLaunchArgument(
    #     name='use_sim_time',
    #     default_value='true',
    #     description='Use simulation(Gazebo) clock if true',
    # )

    # use_hydro_launch_arg = launch.actions.DeclareLaunchArgument(
    #     name='use_hydro',
    #     default_value='true',
    #     description='Use hydrodynamics if true',
    # )

    # --- Includes (same as your original file, but parameterized) ---
    alpha_estimation = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            str(alpha_estimation_path / 'launch/estimation.launch.py')
        ),
        launch_arguments={
            'vehicle_name': vehicle_name,
            'use_hydro': str(use_hydro),
            'use_sim_time': str(use_sim_time),
        }.items(),
    )

    alpha_force_torque_calc = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            str(alpha_model_path / 'launch/dyn_calc.launch.py')
        ),
        launch_arguments={
            'vehicle_name': vehicle_name,
            'use_hydrodynamics': str(use_hydro),
            'base_tf_file': str(alpha_model_path / 'config/alpha_base_tf_params_bluerov.yaml'),
            'moving_base': 'true',
            'use_sim_time': str(use_sim_time),
        }.items(),
    )

    bluerov_acceleration_estimation = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            str(bluerov_acceleration_estimation_path / 'launch/estimation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': str(use_sim_time),
            'vehicle_name': vehicle_name,
        }.items(),
    )

    bluerov_ctrl = GroupAction([
        # Remap only for nodes inside this include
        SetRemap(src='thrust_setpoint', dst='thrust_setpoint_auv'),
        SetRemap(src='torque_setpoint', dst='torque_setpoint_auv'),

        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                str(bluerov_ctrl_path / 'launch/node_velocity_control.launch.py')
            ),
            launch_arguments={
                'use_sim_time': str(use_sim_time),
                'vehicle_name': vehicle_name,
                'controller_type': '4',
                'config_file': str(bluerov_ctrl_path / 'config/ctrl_params_sim_uvms.yaml'),
            }.items(),
        ),
    ])

    estimation_drift_watchdog = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(estimation_watchdog_path),
        launch_arguments={
            'use_sim_time': str(use_sim_time),
            'vehicle_name': vehicle_name,
        }.items(),
    )

    bluerov_mixer = GroupAction([
        # apply ONLY to nodes started inside this group/include
        SetRemap(src='thruster_command', dst='thruster_command_mixer'),

        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(mixer_path),
            launch_arguments={
                'use_sim_time': str(use_sim_time),
                'vehicle_name': vehicle_name,
                'mixer_path': mixer_config_file_path,
            }.items(),
        ),
    ])

    uvms_kinematic_ctrl = GroupAction([
        # apply ONLY to nodes started inside this group/include
        SetRemap(src='joint_vel_cmds', dst='joint_vel_cmds_kin'),

        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                str(uvms_kin_ctrl_path / 'launch/uvms_kin_ctrl.launch.py')
            ),
            launch_arguments={
                'vehicle_name': vehicle_name,
                'use_sim_time': str(use_sim_time),
            }.items(),
        ),
    ])

    uvms_trajectory_gen = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            str(uvms_trajectory_gen_path / 'launch/traj_gen.launch.py')
        ),
        launch_arguments={
            'vehicle_name': vehicle_name,
            'use_sim_time': str(use_sim_time),
        }.items(),
    )

    uvms_visualization = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            str(uvms_visualization_path / 'launch/visualization.launch.py')
        ),
        launch_arguments={
            'visualization_modules': '[1, 2, 4, 5, 7, 8, 9]',
            'vehicle_name': vehicle_name,
            'use_sim_time': str(use_sim_time),
        }.items(),
    )

    rviz = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            str(uvms_kin_ctrl_path / 'launch/rviz.launch.py')
        ),
        launch_arguments={'use_sim_time': str(use_sim_time)}.items(),
    )

    velocity_command = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            str(alpha_ctrl_path / 'launch/velocity_command.launch.py')
        ),
        launch_arguments={
            'vehicle_name': vehicle_name,
            'use_sim_time': str(use_sim_time),
        }.items(),
    )

    # --- NEW: include your MPC launch file (in uvms_mpc_ctrl) ---
    ltv_mpc = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(ltv_mpc_launch_path),
        launch_arguments={
            'vehicle_name': vehicle_name,
            'use_sim_time': str(use_sim_time),
        }.items(),
    )

    return launch.LaunchDescription(
        [
            # vehicle_name_launch_arg,
            # use_sim_time_launch_arg,
            # use_hydro_launch_arg,

            alpha_estimation,
            alpha_force_torque_calc,
            bluerov_acceleration_estimation,
            bluerov_ctrl,
            bluerov_mixer,

            uvms_kinematic_ctrl,
            # estimation_drift_watchdog,
            uvms_trajectory_gen,

            # NEW: start MPC nodes (ltv_mpc.launch.py) in addition
            ltv_mpc,

            uvms_visualization,
            rviz,
            velocity_command,
        ]
    )