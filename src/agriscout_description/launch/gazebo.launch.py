import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

def generate_launch_description():
    # Paths
    agriscout_description = get_package_share_directory("agriscout_description")
    
    # Resource path so Gazebo finds your meshes
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(agriscout_description).parent.resolve())]
    )

    # Robot State Publisher
    robot_description = ParameterValue(Command([
        "xacro ", os.path.join(agriscout_description, "urdf", "agriscout.urdf.xacro")
        ]), value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}]
    )

    # Launch Modern Gazebo (ros_gz_sim)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")]),
            launch_arguments=[("gz_args", [" -v 4", " -r", " empty.sdf"])]
    )

    # Spawn Robot
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "agriscout", "-z", "0.1"],
    )

    # ROS-GZ Bridge (Needed for the clock in modern Gazebo)
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]"]
    )

    # Launch your Controllers
    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('agriscout_controller'), 'launch', 'controller.launch.py')]),
    )

    return LaunchDescription([
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
        controllers
    ])