import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

def generate_launch_description():

    # --- Package paths ---
    sar_rover_dir = get_package_share_directory('sar_rover')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    # --- Launch arguments ---
    use_gps_arg = DeclareLaunchArgument(
        'use_gps',
        default_value='false',
        description='true = outdoor GPS+SLAM mode, false = indoor SLAM only'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Isaac Sim clock'
    )

    # --- Grab argument values ---
    use_gps = LaunchConfiguration('use_gps')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # --- Param file paths ---
    nav2_params = os.path.join(sar_rover_dir, 'config', 'nav2_params.yaml')
    slam_params = os.path.join(sar_rover_dir, 'config', 'slam_params.yaml')

    # --- Global sim time setting — applies to all nodes below ---
    sim_time_param = SetParameter(name='use_sim_time', value=use_sim_time)

    # --- SLAM Toolbox (both modes use it) ---
    slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time': use_sim_time,
        }.items()
    )

    # --- Nav2 stack ---
    nav2_node = IncludeLaunchDescription(
       PythonLaunchDescriptionSource(
          os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
       ),
       launch_arguments={
           'params_file': nav2_params,
           'use_sim_time': use_sim_time,
           'autostart': 'true',
       }.items()
    )

    # --- robot_localization EKF — outdoor GPS+SLAM mode only ---
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frequency': 30.0,
            'sensor_timeout': 0.1,
            'two_d_mode': True,         # ground rover, ignore Z
            'publish_tf': True,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_link_frame': 'base_footprint',
            'world_frame': 'odom',
            # Fuse odometry and GPS
            'odom0': '/odom',
            'odom0_config': [
                True, True, False,      # x, y, z
                False, False, False,    # roll, pitch, yaw
                True, True, False,      # vx, vy, vz
                False, False, False,    # vroll, vpitch, vyaw
                False, False, False     # ax, ay, az
            ],
            'imu0': '/imu/data',
            'imu0_config': [
                False, False, False,
                False, False, True,     # yaw only
                False, False, False,
                False, False, True,     # vyaw only
                False, False, False
            ],
        }],
        condition=IfCondition(use_gps)  # only launch in GPS mode
    )

    # --- Rover node ---
    rover_node = Node(
        package='sar_rover',
        executable='rover_node',
        name='rover_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    print(f"Nav2 params path: {nav2_params}")

    return LaunchDescription([
        use_gps_arg,
        use_sim_time_arg,
        sim_time_param,
        slam_node,
        nav2_node,
        ekf_node,
        rover_node,
    ])
