import os
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    TimerAction,
    RegisterEventHandler,
    LogInfo,
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch_ros.actions import Node


# ─── Paths ────────────────────────────────────────────────────────────────────
SAR_WS   = os.path.expanduser('~/ros/sar_ws')
PX4_DIR  = os.path.expanduser('~/PX4-Autopilot')
PX4_BUILD = os.path.join(PX4_DIR, 'build', 'px4_sitl_default')
SWARM_SH = os.path.join(SAR_WS, 'scripts', 'start_swarm.sh')

# ─── Drone configs ────────────────────────────────────────────────────────────
DRONES = [
    {'drone_id': 0, 'x':  0.0, 'y':  0.0},
    {'drone_id': 1, 'x': 10.0, 'y':  0.0},
    {'drone_id': 2, 'x':  0.0, 'y': 10.0},
    {'drone_id': 3, 'x': 10.0, 'y': 10.0},
]


def generate_launch_description():

    # ── 1. Start swarm (PX4 x4, gzserver, XRCE agents, param setter) ─────────
    start_swarm = ExecuteProcess(
        cmd=['bash', SWARM_SH],
        output='screen',
        name='start_swarm',
    )

    # ── 2. Gazebo client — delayed 10s to let gzserver come up ───────────────
    gzclient = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'bash', '-c',
                    f'source {PX4_DIR}/Tools/simulation/gazebo-classic/setup_gazebo.bash '
                    f'{PX4_DIR} {PX4_BUILD} 2>/dev/null && gzclient'
                ],
                output='screen',
                name='gzclient',
                additional_env={
                    'DISPLAY':    os.environ.get('DISPLAY', ':0'),
                    'XAUTHORITY': os.environ.get('XAUTHORITY', ''),
                },
            )
        ]
    )

    # ── 3. Drone nodes — delayed 25s to let swarm fully boot ─────────────────
    drone_nodes = []
    for d in DRONES:
        n = d['drone_id']
        node = Node(
            package='sar_drone',
            executable='drone_node',
            name=f'drone_node_{n}',
            namespace=f'drone_{n}',
            output='screen',
            parameters=[{
                'drone_id':            n,
                'test_mode':           True,
                'sector_size':         10.0,
                'detection_threshold': 1.0,
                'use_sim_time':        True,
            }],
        )
        drone_nodes.append(node)

    delayed_nodes = TimerAction(
        period=25.0,
        actions=drone_nodes,
    )

    # ── 4. Cleanup on shutdown or swarm exit ──────────────────────────────────
    cleanup = ExecuteProcess(
        cmd=[
            'bash', '-c',
            '''
            echo "[phase3] Shutting down swarm..."
            pkill -x px4         || true
            pkill gzserver       || true
            pkill gzclient       || true
            pkill MicroXRCEAgent || true
            screen -ls | grep -E "px4_|xrce_|mav_" \
                | awk '{print $1}' \
                | xargs -I{} screen -S {} -X quit 2>/dev/null || true
            echo "[phase3] Cleanup complete."
            '''
        ],
        output='screen',
        name='cleanup',
    )

    on_swarm_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=start_swarm,
            on_exit=[
                LogInfo(msg='[phase3] start_swarm exited — running cleanup'),
                cleanup,
            ]
        )
    )

    on_shutdown = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                LogInfo(msg='[phase3] Shutdown signal received — running cleanup'),
                cleanup,
            ]
        )
    )

    return LaunchDescription([
        start_swarm,
        gzclient,
        delayed_nodes,
        on_swarm_exit,
        on_shutdown,
    ])
