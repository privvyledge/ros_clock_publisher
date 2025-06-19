"""
Author: Boluwatife Olabiran
Date: 06/04/2025
clock_publisher.py

This node acts as a ROS time source that publishes ROS Clock messages at a specified tick rate to the '/clock' topic.

See http://design.ros2.org/articles/clock_and_time.html#ros-time for more context.

Usage (choose one):
    * ros2 run ros_clock_publisher clock_publisher_node
    * ros2 run ros_clock_publisher clock_publisher_node --ros-args -p start_time:="2022-10-10 10:10:10.123456" -p end_time:="2022-10-10 10:10:10.123456"

Todo:
    * Add support for parameter callback (deny use_sim_time change and enable playing/pausing) [done]
    * Add ability to start paused like ROS bag, i.e do not autostart the timer [done]
    * Add support for starting delay [done
    * Add support for using python time, e.g time.monotonic_ns or time.time_ns instead of using ROS timer. This could be a separate node.
    * Allow jumping to (seeking) a specific time
    * Cleanup, i.e DRY in init and parameter_change_callback
"""
import sys
import time
from datetime import datetime, timedelta
from typing import Union, Tuple, Optional
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult, ParameterType
from rosgraph_msgs.msg import Clock

from ros_clock_publisher.utils import normalize_to_ros_time


class ClockState(Enum):
    PAUSED = 0
    PLAYING = 1
    STOPPED = 2


class ClockPublisher(Node):
    def __init__(self):
        super(ClockPublisher, self).__init__('clock_publisher')
        # Describe parameters
        start_time_descriptor = ParameterDescriptor(
                name='start_time',
                description='Starting time in nanoseconds. Accepts datetime, UNIX timestamp, float/int, or string.',
                type=Parameter.Type.STRING.value
        )
        tick_rate_descriptor = ParameterDescriptor(
                name='tick_rate',
                description='Frequency (Hz) at which the clock is published. '
                            'Also the frequency at which the timer ticks and updates the state of this node as well '
                            'as performs calculations. Higher values result in higher CPU usage but result in smooth '
                            'and continuous clock updates.',
                type=Parameter.Type.DOUBLE.value
        )

        # Declare Parameters
        self.declare_parameters(
                namespace='',
                parameters=[
                    (
                        'autostart',
                        True,
                        ParameterDescriptor(
                                description=f"If True, automatically start the clock timer and publisher, "
                                            f"else starts in a paused state. "
                                            f"Default: True. "
                                            f"Run 'ros2 param set {self.get_fully_qualified_name()} autostart true' "
                                            f"to enable.",
                                type=rclpy.Parameter.Type.BOOL.value)),
                    ('state', 'play', ParameterDescriptor(type=rclpy.Parameter.Type.STRING.value)),
                    ('start_time', '', start_time_descriptor),
                    (
                        'end_time',
                        '',
                        ParameterDescriptor(
                            description='Optional end time represented as a string of UNIX timestamp in float/int '
                                        'or human readable format.',
                            type=Parameter.Type.STRING.value)
                    ),
                    ('duration', 0.0, ParameterDescriptor(type=rclpy.Parameter.Type.DOUBLE.value)),
                    (
                        'time_format',
                        '%Y-%m-%d %H:%M:%S.%f',
                        ParameterDescriptor(type=rclpy.Parameter.Type.STRING.value)
                    ),
                    ('tick_rate', 100.0, tick_rate_descriptor),
                    ('real_time_factor', 1.0, ParameterDescriptor(description='Real-time factor. 1.0 means real-time. '
                                                                              'If set to > 1.0, the clock will run faster than real-time, '
                                                                              'if 0.0 < RTF < 1.0, the clock will run slower than real-time, '
                                                                              'if RTF == 0.0, the clock will pause/freeze, '
                                                                              'if RTF < 0.0, the clock will go backwards. '
                                                                              'RTF = tick_rate * tick_interval = simulated_time / real_time',
                                                                  type=rclpy.Parameter.Type.DOUBLE.value)),
                    ('tick_interval', 0.0, ParameterDescriptor(description='Real-time seconds between clock ticks. If set, overrides adding 1 / tick_rate to current time. Values <= 0.0 are ignored.', type=rclpy.Parameter.Type.DOUBLE.value)),
                    ('delay', 0.0, ParameterDescriptor(description='Real-time seconds to wait before starting the clock. Used to give the user time to launch other nodes when not using ROS 2 launch.', type=rclpy.Parameter.Type.DOUBLE.value)),
                    ('clock_queue_size', 1, ParameterDescriptor(type=rclpy.Parameter.Type.INTEGER.value)),
                    ('loop', False, ParameterDescriptor(description='If True, reset to start time after waiting loop_delay seconds and repeat loop_count times.', type=rclpy.Parameter.Type.BOOL.value)),
                    ('loop_delay', 0.0,  ParameterDescriptor(description='Real-time seconds to wait between loop iterations if loop=True. Defaults to delay if set.', type=rclpy.Parameter.Type.DOUBLE.value)),
                    ('loop_count', -1, ParameterDescriptor(description='Number of total runs if loop=True. Negative value indicates infinite loop or until manual shutdown.', type=rclpy.Parameter.Type.INTEGER.value)),
                ]
        )

        # Initialize variables
        self.autostart = bool(self.get_parameter('autostart').get_parameter_value().bool_value)
        state = self.get_parameter('state').get_parameter_value().string_value
        self.state = ClockState.PLAYING if state == 'play' else ClockState.PAUSED
        self.start_time = self.get_parameter('start_time').get_parameter_value().string_value
        self.end_time = self.get_parameter('end_time').get_parameter_value().string_value
        self.duration = float(self.get_parameter('duration').get_parameter_value().double_value)
        self.time_format = self.get_parameter('time_format').get_parameter_value().string_value
        self.tick_rate = float(self.get_parameter('tick_rate').get_parameter_value().double_value)
        self.real_time_factor = float(self.get_parameter('real_time_factor').get_parameter_value().double_value)
        tick_interval = self.real_time_factor / self.tick_rate
        self.tick_interval = float(self.get_parameter('tick_interval').get_parameter_value().double_value)
        if self.tick_interval <= 0.0:
            self.tick_interval = tick_interval
        self.delay = float(self.get_parameter('delay').get_parameter_value().double_value)
        self.clock_queue_size = self.get_parameter('clock_queue_size').get_parameter_value().integer_value
        self.loop = bool(self.get_parameter('loop').get_parameter_value().bool_value)
        self.loop_delay = float(self.get_parameter('loop_delay').get_parameter_value().double_value) or self.delay
        self.loop_count = int(self.get_parameter('loop_count').get_parameter_value().integer_value)
        self.current_loop = 0
        self.last_wall_time = time.time()  # last wall-clock (real) time

        # Setup dynamic parameter reconfiguring.
        # Register a callback function that will be called whenever there is an attempt to
        # change one or more parameters of the node.
        self.add_on_set_parameters_callback(self.parameter_change_callback)

        self.clock_msg = Clock()

        # Handle empty time parameters
        if not self.start_time:
            self.start_time = time.time_ns()  # datetime.utcnow() or 0.0
            self.get_logger().info(f"No start_time provided. Using current time: {self.start_time}")

        if not self.end_time:
            self.end_time = None

        # Normalize start time to ROS time and combine into single integer nanoseconds for easy comparison/manipulation
        self.start_sec, self.start_nsec = normalize_to_ros_time(self.start_time, time_format=self.time_format)
        assert self.start_sec is not None
        self.start_ns = (self.start_sec * 1_000_000_000) + self.start_nsec
        self.end_sec, self.end_nsec, self.end_ns = None, None, None

        if self.end_time:
            self.end_sec, self.end_nsec = normalize_to_ros_time(self.end_time, time_format=self.time_format)
            self.end_ns = (self.end_sec * 1_000_000_000) + self.end_nsec
            duration_ns = (self.end_ns - self.start_ns)
            self.duration = duration_ns * 1e-9
            self.get_logger().info(f"Using provided end time: {self.end_time} ({self.end_sec}, {self.end_nsec}). "
                                   f"Ignoring 'duration' parameter.")

        # No end time provided, calculate end time from duration
        elif self.duration:
            # create a datetime (or time) object from start_sec/start_nsec
            start_time = self.start_sec + self.start_nsec * 1e-9  # self.start_time
            self.end_time = start_time + self.duration
            # start_datetime = datetime.fromtimestamp(start_time)  # could remove datetime dependency
            # self.end_time = start_datetime + timedelta(seconds=self.duration)  # could remove datetime dependency
            self.end_sec, self.end_nsec = normalize_to_ros_time(self.end_time, time_format=self.time_format)

            self.end_ns = (self.end_sec * 1_000_000_000) + self.end_nsec  # self.start_ns + int(self.duration * 1e9)
            self.get_logger().info(f"No end time provided. Using provided duration: {self.duration} seconds. End time: {self.end_ns}")

        # No duration or end time provided
        else:
            self.end_sec, self.end_nsec, self.end_ns = None, None, None  # or float('inf')
            self.duration = float('inf')
            self.get_logger().info("No duration or end time provided. Running indefinitely.")

        self.curr_time_ns = int(self.start_ns)

        # Set up publisher
        self.clock_publisher = self.create_publisher(Clock, 'clock', self.clock_queue_size)
        if self.tick_rate <= 0:
            self.get_logger().error("Tick rate must be greater than 0.")
            exit(1)
        self.dt = 1.0 / self.tick_rate  # in seconds (float)
        self.dt_ns = int(1e9 / self.tick_rate)  # in nanoseconds (int)

        # Force use_sim_time to False since this node will not spin otherwise
        self.set_parameters(
            [
                Parameter('use_sim_time', Parameter.Type.BOOL, False)
            ])

        # Create timer using wall/system clock
        node_name = self.get_fully_qualified_name()  # self.get_name()
        enabled_msg = "enabled" if self.autostart else f"disabled (run 'ros2 param set {node_name} autostart true' to enable)"

        if self.delay > 0.0:
            time.sleep(self.delay)

        try:
            # ROS2 Humble does not have an autostart argument
            self.clock_timer = self.create_timer(self.dt, self.tick, autostart=self.autostart)
        except TypeError:
            self.clock_timer = self.create_timer(self.dt, self.tick)
            if not self.autostart:
                self.clock_timer.cancel()

        self.get_logger().info(f"simple_clock_publisher node started in {enabled_msg} state "
                               f"with the following settings: "
                               f"Start time: {self.start_sec}.{self.start_nsec}, "
                               f"End time: {self.end_time}, "
                               f"Duration: {self.duration}, "
                               f"Tick Rate: {self.tick_rate}, "
                               f"Real Time Factor: {self.real_time_factor}, "
                               f"Tick Interval: {self.tick_interval}, "
                               f"Delay: {self.delay}, "
                               f"Loop: {self.loop}, "
                               f"Loop Delay: {self.loop_delay}, "
                               f"Loop Count: {self.loop_count}")

    def tick(self):
        """
        Clock update function called at 1 / tick_rate Hz. Advances the simulated clock/time by dt_ns.
        Handles looping and shutdown as well.
        :return:
        """
        # check if the state is ClockState.PLAYING
        if self.state != ClockState.PLAYING:
            return

        # we use the wall clock time to determine how much time has passed in case the timer drifts, is paused/busy, etc.
        current_wall_time = time.time()
        real_dt = current_wall_time - self.last_wall_time

        # Decide how much to advance simulated time:
        if self.tick_interval > 0.0:
            # In Fixed‐tick/interval mode: always add exactly self.tick_interval seconds
            dt_sim = self.tick_interval
        else:
            # In Real-time factor mode: dt_sim = real_dt * rtf
            dt_sim = real_dt * self.real_time_factor

        self.curr_time_ns += int(dt_sim * 1e9)

        # Check if end time has been reached, then either loop or shutdown
        if self.end_ns and (self.curr_time_ns >= self.end_ns):
            # if looping is disabled, or loop count has been reached, then shutdown
            if not self.loop or (
                    (self.loop_count > 0) and (self.current_loop >= self.loop_count - 1)
            ):
                self.get_logger().info(f"Reached end time ({self.end_sec}.{self.end_nsec}) and/or maximum loop count. "
                                       f"Shutting down. ")

                # Stop publishing "/clock", shut down cleanly
                raise SystemExit()

            # Otherwise, increment loop count, wait for loop_delay seconds, and reset curr_time_ns
            self.current_loop += 1
            self.get_logger().info(f"Reached end time ({self.end_sec}.{self.end_nsec}). "
                                   f"Sleeping for loop_delay={self.loop_delay}s before loop #{self.current_loop + 1}.")

            # Pause publishing/updating this ROS node by temporarily destroying the timer, sleeping, then resetting.
            self.clock_timer.cancel()

            # Sleep (blocking) or the real/wall/system clock.
            time.sleep(self.loop_delay)

            # Reset the simulation time to the start time and restart the timer
            self.curr_time_ns = int(self.start_ns)
            self.get_logger().info(f"Starting loop #{self.current_loop + 1} at time ({self.start_sec}.{self.start_nsec}).")
            self.clock_timer.reset()  # could check if self.clock_timer.is_canceled()

        # Update clock within the start and end times
        self.clock_msg.clock.sec = self.curr_time_ns // 1_000_000_000  # valid over all int32 values
        self.clock_msg.clock.nanosec = self.curr_time_ns % 1_000_000_000  # valid in the range [0, 10e9), i.e uint32

        self.clock_publisher.publish(self.clock_msg)
        # advance time
        # self.curr_time_ns += self.dt_ns
        self.last_wall_time = current_wall_time

        # to avoid exceeding end_ns due to floating point errors
        if self.end_ns and (self.curr_time_ns >= self.end_ns):
            self.curr_time_ns = self.end_ns

    def parameter_change_callback(self, params):
        """
        Triggered whenever there is a change request for one or more parameters.

        Args:
            params (List[Parameter]): A list of Parameter objects representing the parameters that are
                being attempted to change.

        Returns:
            SetParametersResult: Object indicating whether the change was successful.
        """
        result = SetParametersResult()
        result.successful = True

        # Iterate over each parameter in this node
        for param in params:
            # Check the parameter's name and type
            if param.name == 'use_sim_time' and param.type_ == Parameter.Type.BOOL:
                if param.value:
                    # Force use_sim_time to False since this node will not spin otherwise
                    self.set_parameters(
                            [
                                Parameter('use_sim_time', Parameter.Type.BOOL, False)
                            ])
                    result.successful = False
                    self.get_logger().info(f"Use Sim Time set to param must be False for this node. Resetting to False.")
            elif param.name == 'autostart' and param.type_ == Parameter.Type.BOOL:
                self.autostart = param.value
                if self.autostart:
                    self.clock_timer.reset()
                else:
                    self.state = ClockState.STOPPED
                    self.clock_timer.cancel()
            elif param.name == 'state' and param.type_ == Parameter.Type.STRING:
                state = param.value
                self.state = ClockState.PLAYING if state == 'play' else ClockState.PAUSED
            elif param.name == 'start_time' and param.type_ == Parameter.Type.STRING:
                self.start_time = param.value
                if not self.start_time:
                    self.start_time = time.time_ns()  # datetime.utcnow()
                    self.get_logger().info(f"No start_time provided. Using current time: {self.start_time}")
                self.start_sec, self.start_nsec = normalize_to_ros_time(self.start_time, time_format=self.time_format)
                self.curr_time_ns = int(self.start_ns)
            elif param.name == 'end_time' and param.type_ == Parameter.Type.STRING:
                self.end_time = param.value
                if not self.end_time:
                    self.end_time = None
                self.end_sec, self.end_nsec, self.end_ns = None, None, None

                if self.end_time:
                    self.end_sec, self.end_nsec = normalize_to_ros_time(self.end_time, time_format=self.time_format)
                    self.end_ns = (self.end_sec * 1_000_000_000) + self.end_nsec
            elif param.name == 'duration' and param.type_ == Parameter.Type.DOUBLE:
                self.duration = param.value
                # create a datetime (or time) object from start_sec/start_nsec
                start_datetime = datetime.fromtimestamp(
                    self.start_sec + self.start_nsec * 1e-9)  # + timedelta(nanoseconds=self.start_nsec)
                self.end_time = start_datetime + timedelta(seconds=self.duration)
                self.end_sec, self.end_nsec = normalize_to_ros_time(self.end_time, time_format=self.time_format)

                self.end_ns = (self.end_sec * 1_000_000_000) + self.end_nsec  # self.start_ns + int(self.duration * 1e9)
            elif param.name == 'time_format' and param.type_ == Parameter.Type.STRING:
                self.time_format = param.value
            elif param.name == 'tick_rate' and param.type_ == Parameter.Type.DOUBLE:
                tick_rate = param.value
                if tick_rate <= 0:
                    self.get_logger().error("Tick rate must be greater than 0.")
                    continue
                self.tick_rate = tick_rate
                self.dt = 1.0 / self.tick_rate  # in seconds (float)
                self.dt_ns = int(1e9 / self.tick_rate)  # in nanoseconds (int)

                if self.delay > 0.0:
                    time.sleep(self.delay)

                # Destroy previous timer if it exists
                if self.clock_timer:
                    self.clock_timer.destroy()
                    del self.clock_timer

                try:
                    # ROS2 Humble does not have an autostart argument
                    self.clock_timer = self.create_timer(self.dt, self.tick, autostart=self.autostart)
                except TypeError:
                    self.clock_timer = self.create_timer(self.dt, self.tick)
                    if not self.autostart:
                        self.clock_timer.cancel()
            elif param.name == 'real_time_factor' and param.type_ == Parameter.Type.DOUBLE:
                self.real_time_factor = param.value
            elif param.name == 'tick_interval' and param.type_ == Parameter.Type.DOUBLE:
                self.tick_interval = param.value
            elif param.name == 'delay' and param.type_ == Parameter.Type.DOUBLE:
                self.delay = param.value
                if self.delay > 0.0:
                    self.clock_timer.cancel()
                    time.sleep(self.delay)
                    self.clock_timer.reset()
            elif param.name == 'loop' and param.type_ == Parameter.Type.BOOL:
                self.loop = param.value
            elif param.name == 'loop_delay' and param.type_ == Parameter.Type.DOUBLE:
                self.loop_delay = param.value
            elif param.name == 'loop_count' and param.type_ == Parameter.Type.INT:
                self.loop_count = param.value
            else:
                result.successful = False
            self.get_logger().info(f"Success = {result.successful} for param {param.name} to value {param.value}")

        return result


def main(args=None):
    if args is None:
        args = sys.argv
    rclpy.init(args=args)
    clock_publisher_node = ClockPublisher()
    try:
        rclpy.spin(clock_publisher_node)
    except (KeyboardInterrupt, SystemExit):
        clock_publisher_node.get_logger().info("Shutting down node...")
    finally:
        status = clock_publisher_node.destroy_timer(clock_publisher_node.clock_timer)
        if not status:
            clock_publisher_node.clock_timer.destroy()
        clock_publisher_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
