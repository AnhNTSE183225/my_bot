# launch/sim.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'my_bot'  # <-- set your package name

    # 1) robot_state_publisher (publishes /robot_description), with sim time on
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 2) Gazebo Harmonic (ros_gz_sim), empty world (change to your world if needed)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items()
    )

    # 3) Spawn robot FROM /robot_description (replacement for gazebo_ros/spawn_entity.py)
    spawn = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-name', 'my_bot',
            '-x', '0', '-y', '0', '-z', '0.5',
            '-topic', '/robot_description'
        ],
        output='screen'
    )

    # Delay spawn a bit so /robot_description is ready
    delayed_spawn = TimerAction(period=2.0, actions=[spawn])

    # 5) Bridge for manual control (temporary)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        # Same names on both sides, no remap needed since your plugin uses /cmd_vel and /odom
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        ],
        output='screen'
    )

    delayed_spawner_js = TimerAction(
        period=5.0,  # Wait 5 seconds after launch
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
                output="screen",
            )
        ]
    )

    delayed_spawner_diff = TimerAction(
        period=6.0,  # Wait 6 seconds after launch  
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["diff_drive_controller"],
                output="screen",
            )
        ]
    )

    # In the return LaunchDescription, replace spawner_js and spawner_diff with:
    return LaunchDescription([rsp, gazebo, delayed_spawn, bridge, delayed_spawner_js, delayed_spawner_diff])