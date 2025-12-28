from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os, shutil, sys

def generate_launch_description():
    robot_ids = [1, 2, 3]
    leader_id = 1  # <-- leader utama

    # Resolve Webots path secara dinamis
    webots_exec = shutil.which("webots")
    if webots_exec is None:
        raise RuntimeError("Webots tidak ditemukan. Pastikan tersedia di PATH.")

    world_path = os.path.expanduser('~/swarm/webots/worlds/swarm.wbt')
    rviz_path = os.path.expanduser('~/swarm/ros2_ws/src/rviz2/swarm.rviz')
    assert os.path.exists(world_path), f"World tidak ditemukan: {world_path}"
    assert os.path.exists(rviz_path),  f"RViz config tidak ditemukan: {rviz_path}"

    launch_nodes = []

    # Jalankan Webots
    launch_nodes.append(
        ExecuteProcess(
            cmd=[webots_exec, world_path],
            output='screen',
            additional_env={'DISPLAY': os.environ.get('DISPLAY', ':0')}
        )
    )

    # Jalankan setiap robot node sesuai ID-nya
    for rid in robot_ids:

        if rid == leader_id:
            # ===== LEADER: global planner + path executor + safety_zone =====
            launch_nodes += [
                Node(
                    package='path_planner',
                    executable='path_planner_node',
                    name=f'path_planner_{rid}',
                    namespace=f'robot{rid}',
                    parameters=[{
                        'robot_id': rid,
                        'leader_id': leader_id,
                        'debug_enable': True,
                        'debug_pub_hz': 3.0,
                    }],
                    output='screen'
                ),
                Node(
                    package='path_executor',
                    executable='path_executor_node',
                    name=f'path_executor_{rid}',
                    namespace=f'robot{rid}',
                    parameters=[{'robot_id': rid}],
                    output='screen'
                ),
            ]
        else:
            # ===== FOLLOWER: local planner + safety_zone =====
            launch_nodes += [
                Node(
                    package='path_planner',
                    executable='follower_local_planner',
                    name=f'follower_local_planner_{rid}',
                    namespace=f'robot{rid}',
                    parameters=[{
                        'robot_id': rid,
                        'anchor_id': leader_id,      # ikut robot depan
                        'slot_index': (rid - leader_id),   # robot2=1, robot3=2, robot4=3
                        'follow_distance': 0.15,
                        'slot_spacing': 0.25,
                        'k_lin': 1.6,
                        'k_ang': 3.0,
                        'max_lin_vel': 0.55,
                        'max_ang_vel': 1.5,
                        'cmd_topic': 'cmd_vel_raw' # kirim ke safety_zone
                    }],
                    output='screen'
                ),
                Node(
                    package='safety_zone',
                    executable='safety_zone_node',
                    name=f'safety_zone_{rid}',
                    namespace=f'robot{rid}',
                    parameters=[{
                        'robot_id': rid,
                        'robot_ids': robot_ids,
                        'front_radius': 0.16,
                        'front_width':  0.09,
                        'side_radius':  0.12,
                        'side_depth':   0.10,
                        'body_radius': 0.09,
                        'prox_buffer': 0.03,
                        'head_on_turn_bias': 1.0,
                        'head_on_y_eps': 0.02,
                        'reverse_hold_time': 0.25,
                        'frame_id': 'map',
                        'cmd_in_topic': 'cmd_vel_raw',
                        'cmd_out_topic': 'cmd_vel',
                    }],
                    output='screen'
                ),
            ]

    # Vision Node (satu untuk semua robot)
    launch_nodes.append(
        Node(
            package='vision_node',
            executable='vision_node',
            name='vision_node',
            parameters=[{'robot_ids': robot_ids}],
            output='screen'
        )
    )

    # Role Manager Node (satu untuk semua robot)
    launch_nodes.append(
        Node(
            package='role_manager',
            executable='role_manager_node',
            name='role_manager_node',
            parameters=[{'robot_ids': robot_ids}],
            output='screen'
        )
    )

    # GUI:
    gui_script = os.path.expanduser('~/swarm/ros2_ws/src/gui_pkg/gui_pkg/gui.py')
    assert os.path.exists(gui_script), f"GUI script tidak ditemukan: {gui_script}"
    launch_nodes.append(
        ExecuteProcess(
            cmd=[sys.executable, gui_script],
            cwd=os.path.dirname(gui_script),
            output='screen',
            additional_env={'DISPLAY': os.environ.get('DISPLAY', ':0')}
        )
    )

    # Bridge Node
    bridge_node = Node(
        package='gui_pkg',  # Ganti dengan nama paket kamu jika diperlukan
        executable='bridge',  # Pastikan executable sudah benar
        name='bridge',
        output='screen',
    )
    launch_nodes.append(bridge_node)

    launch_nodes.append(
        Node(
            package='path_planner',
            executable='pf_visualizer_node',
            name='pf_visualizer_node',
            namespace=f'robot{leader_id}',
            output='screen',
            parameters=[{
                'robot_id': leader_id,
                'rep_range_cells': 7,
                'publish_hz': 5.0,
                'show_status_text': False,
            }]
        )
    )

    # RViz2
    launch_nodes.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_path]
        )
    )

    return LaunchDescription(launch_nodes)