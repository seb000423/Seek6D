"""Executable entry point for the MoveIt 2 Any6D robot controller."""

from __future__ import annotations

from typing import Optional

import rclpy

from .config import DEFAULT_CONFIG
from .control_node import RobotControlNode


def main(args=None) -> None:
    config = DEFAULT_CONFIG
    rclpy.init(args=args)
    controller: Optional[RobotControlNode] = None
    try:
        controller = RobotControlNode(config)
        controller.initialize_hardware()
        controller.run()

    except KeyboardInterrupt:
        pass
    finally:
        if controller is not None:
            controller.shutdown_hardware()
            controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
