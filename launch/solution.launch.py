import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_dir = get_package_share_directory('mpc_rbt_student')
    solution_dir = get_package_share_directory('mpc_rbt_student')

    rviz_config_path = os.path.join(package_dir, 'rviz', 'config.rviz')
    bt_config = os.path.join(solution_dir, 'config', 'bt_server.yaml')

    return LaunchDescription([

        # === Localization ===
        Node(
            package='mpc_rbt_student',
            executable='localization_node',
            name='localization_node',
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        # === Planning ===
        Node(
            package='mpc_rbt_student',
            executable='planning_node',
            name='planning_node',
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        # === Motion Control ===
        Node(
            package='mpc_rbt_student',
            executable='motion_control_node',
            name='motion_control_node',
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        # === Warehouse Manager ===
        Node(
            package='mpc_rbt_student',
            executable='warehouse_manager',
            name='warehouse_manager',
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        # === BT Server ===
        Node(
            package='mpc_rbt_student',
            executable='bt_server',
            name='bt_server',
            parameters=[
                {'use_sim_time': True},
                bt_config
            ],
            output='screen'
        ),

        # === RViz ===
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
    ])
