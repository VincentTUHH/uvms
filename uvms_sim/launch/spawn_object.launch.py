from ament_index_python.packages import get_package_share_path
import launch
import launch_ros


def generate_launch_description():
    object_name = launch.substitutions.LaunchConfiguration('object_name')

    object_name_launch_arg = launch.actions.DeclareLaunchArgument(
        name='object_name',
        default_value='cylinder',
        description='object name used as namespace ',
    )

    alpha_sim_path = get_package_share_path('alpha_sim')
    model_path = str(alpha_sim_path / 'models/object/urdf/cylinder.urdf.xacro')

    object_description = launch.substitutions.LaunchConfiguration(
        'object_description',
        default=launch.substitutions.Command([
            'ros2 run hippo_sim create_robot_description.py ', '--input ',
            model_path
        ]))

    description = {'object_description': object_description}

    spawner = launch_ros.actions.Node(package='hippo_sim',
                                      executable='spawn',
                                      parameters=[description],
                                      arguments=[
                                          '--param',
                                          'object_description',
                                          '--remove_on_exit',
                                          'true',
                                          '--x',
                                          '1.0',
                                          '--y',
                                          '2.5',
                                          '--z',
                                          '-0.5',
                                      ])

    spawn_group = launch.actions.GroupAction([
        launch_ros.actions.PushRosNamespace(
            object_name),
        spawner,
        ])

    return launch.LaunchDescription([
        object_name_launch_arg,
        spawn_group,
    ])
