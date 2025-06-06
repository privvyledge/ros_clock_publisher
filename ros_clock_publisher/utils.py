import time
from datetime import datetime, timedelta
from typing import Union, Tuple, Optional


def normalize_to_ros_time(time_object: Union[datetime, float, int, str],
                          time_format: Optional[str] = "%Y-%m-%d %H:%M:%S.%f") -> Tuple[int, int]:
    """
    Takes a time argument containing either a datetime.datetime object, a UNIX timestamp (float/int) or a string,
    and return (sec, nanosec) tuple suitable for ROS Time (builtin_interfaces/Time).
    Todo: move to utils script

    :param time_object: Either:
        * datetime.datetime object
        * UNIX timestamp (float/int)
        * string of form "time_format"
    :param time_format: String of the format to parse time_object "%Y-%m-%d %H:%M:%S.%f"
    :return:
        * sec = integer of seconds since epoch
        * nanosec = remainder in nanoseconds

    Example:
        >>> import time
        >>> from datetime import datetime
        >>> current_time_int = time.time_ns(); print(current_time_int)
        >>> current_time_float = float(current_time_int) / 1e9; print(current_time_float)  # time.time()
        >>> current_datetime = datetime.fromtimestamp(current_time_float)
        >>> current_time_str = datetime.fromtimestamp(current_time_float).strftime("%Y-%m-%d %H:%M:%S.%f")
        >>> current_time_float2 = time.time(); print(current_time_float2)
        >>> current_datetime2 = datetime.now()  # .strftime("%Y-%m-%d %H:%M:%S.%f")
        >>> normalize_to_ros_time(current_time_int)
        (1749057695, 426467180)
        >>> normalize_to_ros_time(current_time_float)
        (1749057695, 426467180)
        >>> normalize_to_ros_time(current_datetime)
        (1749057906, 287734985)
        >>> normalize_to_ros_time(current_time_str, time_format="%Y-%m-%d %H:%M:%S.%f")
        (1749057906, 287734985)
        >>> normalize_to_ros_time(current_time_float2)
        (1749058054, 681048870)
        >>> normalize_to_ros_time(current_datetime2)
        (1749058054, 681048870)
        >>> normalize_to_ros_time("2022-10-10 10:10:10.123456")
        (1662506867, 123456000)
        >>> normalize_to_ros_time("2022-10-10 10:10:10.123456", "%Y-%m-%d %H:%M:%S.%f")
        (1662506867, 123456000)
        >>> normalize_to_ros_time("2022/10/10 10:10:10_123456", "%Y/%m/%d %H:%M:%S_%f")
        (1665411010, 123456001)
    """
    # 1. Convert time_object to UNIX time
    # If time_object is a string, convert to a datetime object then to UNIX time
    if isinstance(time_object, str):
        try:
            # first try converting to an integer, e.g the result of time.time_ns()
            timestamp = int(time_object)  # raises an error if the string is not an integer even if it is a float
            timestamp = float(time_object) / 1e9  # convert to float
        except ValueError:
            # try converting to a float if the string is not an integer
            try:
                timestamp = float(time_object)  # raises an error if the string is not a float
            except ValueError:
                timestamp = datetime.strptime(time_object, time_format).timestamp()

    # If time_object is a datetime object, convert to UNIX time
    elif isinstance(time_object, datetime):
        timestamp = time_object.timestamp()

    # If time_object is an int, convert to UNIX time
    elif isinstance(time_object, int):
        timestamp = float(time_object) / 1e9

    # If time_object is already a UNIX timestamp, leave it alone
    elif isinstance(time_object, float):
        timestamp = time_object

    else:
        raise TypeError(f"time_object must be a datetime.datetime object, a UNIX timestamp (float/int), or a string."
                        f" Got {type(time_object)} instead.")

    # (optional) If timestamp <= 0, assign current time
    if timestamp <= 0:
        timestamp = time.time()

    # 2. Convert UNIX time to ROS time
    sec = int(timestamp)  # valid over all int32 values
    nanosec = int((timestamp - sec) * 1e9)  # valid in the range [0, 10e9), i.e uint32

    # 3. Correct for negative nanoseconds (if t < 0)
    # In rare cases, Python's float subtraction can yield nanosec == 1e9.
    if nanosec >= 1e9:
        sec += 1
        nanosec -= 1e9
    if nanosec < 0:
        sec -= 1
        nanosec += 1e9
    return sec, nanosec
