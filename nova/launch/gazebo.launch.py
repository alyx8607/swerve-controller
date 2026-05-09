import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    package_name = 'nova'
    
    urdf_file = os.path.join(get_package_share_directory(package_name), 'model', 'nova.urdf')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
        
    # Launch Gazebo with a specified world
    gazebo = IncludeLaunchDescription(
                 PythonLaunchDescriptionSource([
                     os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )
    
    # Publish robot description
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # Spawn the robot in Gazebo using the robot description topic
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_robot'],
        output='screen'
    )
    
    launchDescriptionObject = LaunchDescription()
    launchDescriptionObject.add_action(nodeRobotStatePublisher)  # Ensure the robot description is published first
    launchDescriptionObject.add_action(gazebo)
    launchDescriptionObject.add_action(spawn_robot)

    return launchDescriptionObject  # Fixed return statement

