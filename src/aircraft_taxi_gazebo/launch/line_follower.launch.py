import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo  = get_package_share_directory('aircraft_taxi_gazebo')
    pkg_control = get_package_share_directory('aircraft_taxi_control')

    default_model_path = os.path.join(pkg_control, 'models', 'best.pt')

    return LaunchDescription([

        # ── Launch argümanları ────────────────────────────────────────
        DeclareLaunchArgument(
            'model_path',
            default_value=default_model_path,
            description='YOLOv8 segmentasyon modeli (.pt dosyası)'
        ),
        DeclareLaunchArgument(
            'linear_speed', default_value='0.5',
            description='Otonom mod ileri hızı (m/s)'
        ),
        DeclareLaunchArgument(
            'speed_slow', default_value='0.3',
            description='Dönüş / yüksek sapma hızı (m/s)'
        ),
        DeclareLaunchArgument(
            'manual_speed', default_value='0.5',
            description='Manuel mod hızı (m/s)'
        ),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Başlangıçta otonom modu otomatik aç (true/false)'
        ),
        DeclareLaunchArgument(
            'kp_lateral', default_value='0.25',
            description='Lateral PD oransal kazanç'
        ),
        DeclareLaunchArgument(
            'kp_heading', default_value='0.15',
            description='Heading PD oransal kazanç'
        ),
        DeclareLaunchArgument(
            'kd_lateral', default_value='0.35',
            description='Lateral PD türevsel kazanç'
        ),
        DeclareLaunchArgument(
            'kd_heading', default_value='0.10',
            description='Heading PD türevsel kazanç'
        ),
        DeclareLaunchArgument(
            'max_steering', default_value='1.0',
            description='Maksimum angular.z (rad/s)'
        ),
        DeclareLaunchArgument(
            'blind_turn_steering', default_value='0.30',
            description='Blind turn sabit direksiyon açısı (rad/s)'
        ),
        DeclareLaunchArgument(
            'obstacle_distance', default_value='2.0',
            description='Engel tepki mesafesi (m)'
        ),
        DeclareLaunchArgument(
            'front_fov_deg', default_value='30.0',
            description='Lidar ön tarama açısı (±derece, toplam 2x)'
        ),
        DeclareLaunchArgument(
            'reverse_speed', default_value='0.2',
            description='Geri gidiş hızı (m/s)'
        ),
        DeclareLaunchArgument(
            'reverse_duration', default_value='2.0',
            description='Geri gidiş süresi (s)'
        ),
        DeclareLaunchArgument(
            'debug_image', default_value='true',
            description='Debug görüntüsü ROS topic\'e yayımla'
        ),

        # ── Gazebo + araç spawn ───────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo, 'launch', 'gazebo_spawn.launch.py')
            )
        ),

        # ── Çizgi takip node'u ────────────────────────────────────────
        Node(
            package='aircraft_taxi_control',
            executable='line_follower',
            name='line_follower',
            output='screen',
            parameters=[{
                'model_path':           LaunchConfiguration('model_path'),
                'linear_speed':         LaunchConfiguration('linear_speed'),
                'speed_slow':           LaunchConfiguration('speed_slow'),
                'manual_speed':         LaunchConfiguration('manual_speed'),
                'kp_lateral':           LaunchConfiguration('kp_lateral'),
                'kp_heading':           LaunchConfiguration('kp_heading'),
                'kd_lateral':           LaunchConfiguration('kd_lateral'),
                'kd_heading':           LaunchConfiguration('kd_heading'),
                'max_steering':         LaunchConfiguration('max_steering'),
                'blind_turn_steering':  LaunchConfiguration('blind_turn_steering'),
                'obstacle_distance':    LaunchConfiguration('obstacle_distance'),
                'front_fov_deg':        LaunchConfiguration('front_fov_deg'),
                'reverse_speed':        LaunchConfiguration('reverse_speed'),
                'reverse_duration':     LaunchConfiguration('reverse_duration'),
                'autostart':            LaunchConfiguration('autostart'),
                'debug_image':          LaunchConfiguration('debug_image'),
                'use_sim_time':         True,
            }],
        ),

    ])
