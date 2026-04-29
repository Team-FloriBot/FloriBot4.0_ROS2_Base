from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():

    package_name = "robot_description"

    urdf_path = os.path.join(
        get_package_share_directory(package_name),
        "urdf",
        "Floribot.urdf.xacro"
    )
    
    # Process xacro file
    doc = xacro.process_file(urdf_path)
    robot_description_content = doc.toxml()

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{
                "robot_description": robot_description_content
            }]
        )
    ])
