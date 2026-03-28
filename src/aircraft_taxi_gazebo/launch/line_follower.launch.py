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
 
    # Varsayılan model yolu: paket içindeki models/ dizini
    default_model_path = os.path.join(pkg_control, 'models', 'best.pt')
 
    return LaunchDescription([
 
        # --- Launch argümanları ---
        DeclareLaunchArgument(
            'model_path',
            default_value=default_model_path,
            description='YOLOv8 segmentasyon modeli (.pt dosyası tam yolu)'
        ),
        DeclareLaunchArgument(
            'linear_speed',
            default_value='0.5',
            description='Araç ileri hızı (m/s)'
        ),
        DeclareLaunchArgument(
            'kp',
            default_value='0.005',
            description='PID oransal kazanç'
        ),
        DeclareLaunchArgument(
            'ki',
            default_value='0.0001',
            description='PID integral kazanç'
        ),
        DeclareLaunchArgument(
            'kd',
            default_value='0.001',
            description='PID türevsel kazanç'
        ),
        DeclareLaunchArgument(
            'max_angular',
            default_value='1.0',
            description='Maksimum angular.z hızı (rad/s)'
        ),
        DeclareLaunchArgument(
            'debug_image',
            default_value='true',
            description='Debug görüntüsü yayımla (/aircraft_taxi/line_debug)'
        ),
 
        # --- Gazebo + spawn + bridge ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo, 'launch', 'gazebo_spawn.launch.py')
            )
        ),
 
        # --- Çizgi takip node'u ---
        Node(
            package='aircraft_taxi_control',
            executable='line_follower',
            name='line_follower',
            output='screen',
            parameters=[{
                'model_path':    LaunchConfiguration('model_path'),
                'linear_speed':  LaunchConfiguration('linear_speed'),
                'kp':            LaunchConfiguration('kp'),
                'ki':            LaunchConfiguration('ki'),
                'kd':            LaunchConfiguration('kd'),
                'max_angular':   LaunchConfiguration('max_angular'),
                'debug_image':   LaunchConfiguration('debug_image'),
                'use_sim_time':  True,
            }],
        ),
 
    ])