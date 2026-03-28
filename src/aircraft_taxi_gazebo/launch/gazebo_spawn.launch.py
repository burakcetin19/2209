import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_desc    = get_package_share_directory('aircraft_taxi_description')
    pkg_gazebo  = get_package_share_directory('aircraft_taxi_gazebo')

    urdf_path   = os.path.join(pkg_desc, 'urdf', 'aircraft_taxi.urdf.xacro')
    world_file  = os.path.join(pkg_gazebo, 'worlds', 'runway.world')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'world',
            default_value=world_file,
            description='Gazebo dünya dosyası'
        ),

        # 1. Ignition/Gazebo Harmonic sunucusu + GUI
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', LaunchConfiguration('world'), '-v', '4'],
            output='screen',
        ),

        # 2. robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }],
        ),

        # 3. Aracı Gazebo sahnesine spawn et
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_aircraft_taxi',
            arguments=[
                '-name', 'aircraft_taxi',
                '-topic', 'robot_description',
                '-x', '0',
                '-y', '0',
                '-z', '0.25',
                '-world', 'runway_world',
            ],
            output='screen',
        ),

        # 4. ROS2 <-> Ignition topic köprüsü
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ign_bridge',
            arguments=[
                # cmd_vel: ROS2 -> GZ
                '/aircraft_taxi/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                # odom: GZ -> ROS2
                '/aircraft_taxi/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                # tf: GZ -> ROS2
                '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                # kamera info: GZ -> ROS2
                '/aircraft_taxi/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                # lidar: GZ -> ROS2
                '/aircraft_taxi/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                # joint states: GZ -> ROS2
                '/aircraft_taxi/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            ],
            output='screen',
        ),

        # 5. Kamera görüntüsü için ayrı bridge (ros_gz_image daha güvenilir)
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            name='camera_image_bridge',
            arguments=['/aircraft_taxi/image_raw'],
            output='screen',
        ),

    ])
