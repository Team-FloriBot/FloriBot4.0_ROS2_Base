from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Parameter
    front_length_arg = DeclareLaunchArgument('frontLength', default_value='0.38')
    rear_length_arg = DeclareLaunchArgument('rearLength', default_value='0.38')
    wheel_diameter_arg = DeclareLaunchArgument('wheelDiameter', default_value='0.26') # Durchmesser in Meter
    axes_length_arg = DeclareLaunchArgument('axesLength', default_value='0.335')     # Spurweite/Achsabstand

    base_node = Node(
        package='base',
        executable='base_node',
        name='base',
        output='screen',
        parameters=[{
            'wheelDiameter': LaunchConfiguration('wheelDiameter'),
            'axesLength': LaunchConfiguration('axesLength'),
            'frontLength': LaunchConfiguration('frontLength'),
            'rearLength': LaunchConfiguration('rearLength'),
            'use_sim_time': False
        }]
    )

    return LaunchDescription([
        front_length_arg,
        rear_length_arg,
        wheel_diameter_arg,
        axes_length_arg,
        base_node
    ])