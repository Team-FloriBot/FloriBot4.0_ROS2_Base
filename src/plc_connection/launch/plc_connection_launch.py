from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # PLC / Xavier network configuration
        DeclareLaunchArgument('plc_ip', default_value='192.168.0.43'),
        DeclareLaunchArgument('plc_port', default_value='50000'),
        DeclareLaunchArgument('xavier_ip', default_value=''),
        DeclareLaunchArgument('xavier_port', default_value='50000'),

        # Testserver configuration
        # DeclareLaunchArgument('plc_ip', default_value='127.0.0.1'),
        # DeclareLaunchArgument('plc_port', default_value='50001'),
        # DeclareLaunchArgument('xavier_ip', default_value='127.0.0.1'),
        # DeclareLaunchArgument('xavier_port', default_value='50002'),

        # PLC communication timing
        DeclareLaunchArgument('plc_timeout', default_value='1.5'),

        # 0 s + 50000 us = 50 ms receive timeout
        DeclareLaunchArgument('receive_timeout_sec', default_value='0'),
        DeclareLaunchArgument('receive_timeout_usec', default_value='50000'),

        # Send/read cycle time
        DeclareLaunchArgument('period_send_read', default_value='0.05'),

        # Encoder / engine parameters
        DeclareLaunchArgument('zero_count_encoder', default_value='41229'),
        DeclareLaunchArgument('count_per_rotation_encoder', default_value='4096.0'),
        DeclareLaunchArgument('engine_acceleration', default_value='1000.0'),
        DeclareLaunchArgument('engine_jerk', default_value='10000.0'),

        Node(
            package='plc_connection',
            executable='plc_connection_node',
            name='PLC_Connection',
            output='screen',
            parameters=[{
                'PLC_IP': LaunchConfiguration('plc_ip'),
                'PLC_Port': LaunchConfiguration('plc_port'),
                'PLC_Timeout': LaunchConfiguration('plc_timeout'),

                'Xavier_IP': LaunchConfiguration('xavier_ip'),
                'Xavier_Port': LaunchConfiguration('xavier_port'),

                'Receive_Timeout_sec': LaunchConfiguration('receive_timeout_sec'),
                'Receive_Timeout_usec': LaunchConfiguration('receive_timeout_usec'),

                'Period_Send_Read': LaunchConfiguration('period_send_read'),

                'ZeroCount_Encoder': LaunchConfiguration('zero_count_encoder'),
                'CountPerRotation_Encoder': LaunchConfiguration('count_per_rotation_encoder'),

                'Engine_Acceleration': LaunchConfiguration('engine_acceleration'),
                'Engine_Jerk': LaunchConfiguration('engine_jerk'),

                'use_sim_time': False,
            }],
            remappings=[
                # ('/engine/actualSpeed', '/engine/actualSpeed'),
                # ('/engine/targetSpeed', '/engine/targetSpeed'),
                # ('/engine/targetAcceleration', '/engine/targetAcceleration'),
                # ('/engine/targetTorque', '/engine/targetTorque'),
                # ('/sensors/bodyAngle', '/sensors/bodyAngle'),
            ]
        ),
    ])
