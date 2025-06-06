from datetime import datetime
import launch
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
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

    return LaunchDescription([clock_publisher_node])
