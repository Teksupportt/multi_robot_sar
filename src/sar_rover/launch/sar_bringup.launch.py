import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    sar_rover_dir = get_package_share_directory('sar_rover')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')
        )
    )

    sar_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sar_rover_dir, 'launch', 'sar_phase2.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')
        )
    )

    return LaunchDescription([
        gazebo,
        sar_stack,
        rviz,
    ])
