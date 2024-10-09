import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_race_gazebo_ros = get_package_share_directory('race_gazebo_ros')

    # Bridge ROS topics and Gazebo messages for establishing communication using config file
    bridges_config = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_race_gazebo_ros, 'config', 'bridges_config.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    # Bridge ROS topics and Gazebo messages for establishing communication using python code
    bridges = []
    for i in range(3):
        # cmd_vel
        bridges.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[f'/model/vehicle_{i}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist'],
            output='screen'
        ))

        # contact sensor
        bridges.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[f'/lane{i}/touched@std_msgs/msg/Bool[ignition.msgs.Boolean'],
            output='screen'
        ))

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {pkg_race_gazebo_ros}/worlds/running_track.sdf --gui-config {pkg_race_gazebo_ros}/config/gui.config'}.items(),
    )

    # RQt
    rqt = Node(
        package='rqt_topic',
        executable='rqt_topic',
        arguments=['-t'],
        condition=IfCondition(LaunchConfiguration('rqt'))
    )

    start_node = Node(
        package='race_gazebo_ros',
        executable='start_node',
        output='screen',
        emulate_tty = True,
    )

    race_director = Node(
        package='race_gazebo_ros',
        executable='race_director',
        output='screen',
        emulate_tty = True,
    )

    return LaunchDescription([
        gz_sim,
        DeclareLaunchArgument('rqt', default_value='false',
                              description='Open RQt.'),
        bridges_config,
        *bridges,
        rqt,
        start_node,
        race_director
    ])