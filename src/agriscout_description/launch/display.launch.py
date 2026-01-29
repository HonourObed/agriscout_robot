import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Find the package
    pkg_share = get_package_share_directory('agriscout_description')
    
    # Find the URDF file
    urdf_file = os.path.join(pkg_share, 'urdf', 'agriscout.urdf.xacro')

    # Read the URDF
    robot_description = Command(['xacro ', urdf_file])
    
    # Robot State Publisher Node (The most important node for URDFs)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

   # Find the RViz config file (Make sure the name 'agriscout.rviz' matches what you saved!)
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'agriscout.rviz')

    # Rviz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]  # <--- THIS IS THE MAGIC LINE
    )

    # Joint State Publisher GUI (Slider bar to move wheels)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node
    ])