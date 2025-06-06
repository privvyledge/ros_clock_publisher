from datetime import datetime
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    autostart = LaunchConfiguration('autostart', default=True)
    start_time = LaunchConfiguration('start_time', default='')
    end_time = LaunchConfiguration('end_time', default='')
    duration = LaunchConfiguration('duration', default=0.0)
    time_format = LaunchConfiguration('time_format', default='%Y-%m-%d %H:%M:%S.%f')
    tick_rate = LaunchConfiguration('tick_rate', default=100.0)
    real_time_factor = LaunchConfiguration('real_time_factor', default=1.0)
    tick_interval = LaunchConfiguration('tick_interval', default=0.0)
    delay = LaunchConfiguration('delay', default=0.0)
    clock_queue_size = LaunchConfiguration('clock_queue_size', default=1)
    loop = LaunchConfiguration('loop', default=False)
    loop_delay = LaunchConfiguration('loop_delay', default=0.0)
    loop_count = LaunchConfiguration('loop_count', default=-1)

    # Launch arguments
    autostart_launch_arg = DeclareLaunchArgument(
        'autostart',
        default_value=autostart,
        description='Automatically startup the clock publisher node'
    )
    start_time_launch_arg = DeclareLaunchArgument(
        'start_time',
        default_value=start_time,
        description='Start time for the clock publisher node'
    )
    end_time_launch_arg = DeclareLaunchArgument(
        'end_time',
        default_value=end_time,
        description='End time for the clock publisher node'
    )
    duration_launch_arg = DeclareLaunchArgument(
        'duration',
        default_value=duration,
        description='Duration for the clock publisher node'
    )
    time_format_launch_arg = DeclareLaunchArgument(
        'time_format',
        default_value=time_format,
        description='Time format for the clock publisher node'
    )
    tick_rate_launch_arg = DeclareLaunchArgument(
        'tick_rate',
        default_value=tick_rate,
        description='Tick rate for the clock publisher node'
    )
    real_time_factor_launch_arg = DeclareLaunchArgument(
        'real_time_factor',
        default_value=real_time_factor,
        description='Real time factor for the clock publisher node'
    )
    tick_interval_launch_arg = DeclareLaunchArgument(
        'tick_interval',
        default_value=tick_interval,
        description='Tick interval for the clock publisher node'
    )
    delay_launch_arg = DeclareLaunchArgument(
        'delay',
        default_value=delay,
        description='Delay for the clock publisher node'
    )
    clock_queue_size_launch_arg = DeclareLaunchArgument(
        'clock_queue_size',
        default_value=clock_queue_size,
        description='Clock queue size for the clock publisher node'
    )
    loop_launch_arg = DeclareLaunchArgument(
        'loop',
        default_value=loop,
        description='Loop for the clock publisher node'
    )
    loop_delay_launch_arg = DeclareLaunchArgument(
        'loop_delay',
        default_value=loop_delay,
        description='Loop delay for the clock publisher node'
    )
    loop_count_launch_arg = DeclareLaunchArgument(
        'loop_count',
        default_value=loop_count,
        description='Loop count for the clock publisher node'
    )

    # Create Launch Description
    ld = LaunchDescription(
            [
                autostart_launch_arg,
                start_time_launch_arg,
                end_time_launch_arg,
                duration_launch_arg,
                time_format_launch_arg,
                tick_rate_launch_arg,
                real_time_factor_launch_arg,
                tick_interval_launch_arg,
                delay_launch_arg,
                clock_queue_size_launch_arg,
                loop_launch_arg,
                loop_delay_launch_arg,
                loop_count_launch_arg
            ]
    )

    # Setup nodes
    clock_publisher_node = Node(
        package='ros_clock_publisher',
        executable='clock_publisher_node',
        name='looping_clock',
        output='screen',
        parameters=[
            {
                # use_sim_time must be False, else the timer never fires since the node will be waiting
                # for the simulated time in "/clock" which this node is intended to publish.
                # Other nodes should set use_sim_time=True to subscribe to the "clock" published by this node.
                'use_sim_time': False,
                'autostart': True,
                'start_time': start_time,
                'end_time': end_time,
                'duration': duration,
                'time_format': time_format,
                'tick_rate': tick_rate,
                'real_time_factor': real_time_factor,
                'tick_interval': tick_interval,
                'delay': delay,
                'clock_queue_size': clock_queue_size,
                'loop': loop,
                'loop_delay': loop_delay,
                'loop_count': loop_count
            },
        ],
    )

    ld.add_action(clock_publisher_node)

    return ld
