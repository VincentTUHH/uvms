# Start px4 bridge to merge visual data with sensor data + EKF2
# visual_localization/launch/top_localization.launch.py starts px4 bridge
# among april tag computation nodes

import launch
import launch_ros


def generate_launch_description():
    vehicle_name = launch.substitutions.LaunchConfiguration('vehicle_name')

    vehicle_name_launch_arg = launch.actions.DeclareLaunchArgument(
        name='vehicle_name',
        default_value='klopsi00',
        description='used for node namespace'
    )
    
    px4_bridge = launch_ros.actions.Node(package='visual_localization',
                                         executable='px4_bridge',
                                         namespace=vehicle_name,
                                         name='px4_bridge',
                                         output='screen',
                                         emulate_tty=True,
                                         )

    return launch.LaunchDescription([
        vehicle_name_launch_arg,
        px4_bridge
    ])
