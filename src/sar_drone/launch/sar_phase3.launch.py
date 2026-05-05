import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Launch Arguments ──────────────────────────────────────────────────────
    px4_dir = DeclareLaunchArgument(
        'px4_dir',
        default_value=os.path.expanduser('~/PX4-Autopilot'),
        description='Path to PX4-Autopilot directory'
    )

    # ── PX4 SITL + Gazebo ─────────────────────────────────────────────────────
    # Runs: make px4_sitl gazebo-classic from the PX4 directory
    px4_sitl = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'cd {} && make px4_sitl gazebo-classic'.format(
                os.path.expanduser('~/PX4-Autopilot'))
        ],
        output='screen',
        name='px4_sitl'
    )

    # ── Micro XRCE-DDS Agent ──────────────────────────────────────────────────
    # Delay 5s to give PX4 SITL time to start before the agent connects
    xrce_agent = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
                output='screen',
                name='xrce_dds_agent'
            )
        ]
    )

    # ── Drone Node ────────────────────────────────────────────────────────────
    # Delay 10s to give PX4 + agent time to fully connect before node starts
    drone_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='sar_drone',
                executable='drone_node',
                name='drone_node',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                }]
            )
        ]
    )

    return LaunchDescription([
        px4_dir,
        px4_sitl,
        xrce_agent,
        drone_node,
    ])
