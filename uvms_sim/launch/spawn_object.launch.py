"""Spawn the physical object used in the UVMS manipulation simulation.

This object was tested as a physical body to be manipulated in Gazebo. Actual
gripping was not possible because the gripper could not be modeled correctly
without closed kinematic loops. TODO: Revisit the gripper and object interaction
to improve the simulation as a physical digital twin.
"""

from ament_index_python.packages import get_package_share_path
import launch
import launch_ros


def generate_launch_description():
    alpha_sim_path = get_package_share_path('alpha_sim')
    object_name = launch.substitutions.LaunchConfiguration('object_name')

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
        spawn_group,
    ])
