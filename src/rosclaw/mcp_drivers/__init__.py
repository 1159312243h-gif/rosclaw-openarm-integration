"""ROSClaw MCP Drivers - Hardware Abstraction Layer

Provides uniform robot control across real hardware, simulation, and serial devices.
All drivers implement BaseDriver for consistent interface.
"""

from rosclaw.mcp_drivers.base import BaseDriver, DriverState, TrajectoryCommand

try:
    from rosclaw.mcp_drivers.ros2_driver import ROS2Driver
except ImportError:
    ROS2Driver = None  # type: ignore

try:
    from rosclaw.mcp_drivers.mujoco_sim_driver import MuJoCoSimDriver
except ImportError:
    MuJoCoSimDriver = None  # type: ignore

try:
    from rosclaw.mcp_drivers.serial_driver import SerialDriver
except ImportError:
    SerialDriver = None  # type: ignore

__all__ = [
    "BaseDriver",
    "DriverState",
    "TrajectoryCommand",
    "ROS2Driver",
    "MuJoCoSimDriver",
    "SerialDriver",
]
