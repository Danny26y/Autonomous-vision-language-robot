import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    gazebo_ros = get_package_share_directory('gazebo_ros')
    rover_sim = get_package_share_directory('rover_sim')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = os.path.join(tb3_gazebo, 'worlds', 'turtlebot3_world.world')
    sdf = os.path.join(rover_sim, 'models', 'turtlebot3_waffle', 'model.sdf')

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world}.items())

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')))

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'waffle', '-file', sdf,
                   '-x', '-2.0', '-y', '-0.5', '-z', '0.01',
                   '-timeout', '120'],
        output='screen')

    return LaunchDescription([gzserver, gzclient, robot_state_publisher, spawn])
