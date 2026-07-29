#!/usr/bin/env python3

import time
import uuid

import rclpy
from rclpy import wait_for_message
from rclpy.node import Node

from fauna_msgs.msg import MotorControlInputMode, MotorControlSystemState, VelocityCommand

# Priority value used for priority based command arbitration
# See DOCS-SITE for the full priority table
FAUNA_SDK_COMMAND_PRIORITY = 0


class WalkBackForth(Node):
    """
    A simple example node that commands a fauna robot to walk forward and backward.
    """

    def __init__(self) -> None:
        super().__init__("walk_back_forth")
        self.publisher = self.create_publisher(VelocityCommand, "/motor_control/velocity/command", 10)
        self._sender_id = f"sdk-walk-example-{uuid.uuid4()}"
        self.get_logger().info("Walk Back Forth node initialized")

    def walk(self) -> None:
        _, system_state = wait_for_message.wait_for_message(
            MotorControlSystemState, self, "/motor_control/system_state"
        )
        if system_state.mode != "walk":
            self.get_logger().error(f"Robot is not in walk mode. Aborting. Current mode: '{system_state.mode}'")
            return
        if system_state.input_mode != MotorControlInputMode.API:
            current_mode = "JOYSTICK" if system_state.input_mode == MotorControlInputMode.JOYSTICK else "UNKNOWN"
            self.get_logger().error(
                f"Robot is not in API input mode (currently in {current_mode} mode). "
                "To enable API control, use the Fauna app or joystick to switch to API mode."
            )
            return
        # Walk forward for 2 seconds
        self.walk_forward(duration=2.0)

        # Stop for 1 second
        self.stop(duration=1.0)

        # Walk backward for 2 seconds
        self.walk_backward(duration=2.0)

        # Stop the robot
        self.stop()

    def walk_forward(self, duration: float, velocity: float = 0.4) -> None:
        """Command the robot to walk forward."""
        self.get_logger().info("Walking forward...")
        msg = VelocityCommand()
        msg.command_priority.sender_id = self._sender_id
        msg.command_priority.priority = FAUNA_SDK_COMMAND_PRIORITY
        msg.vx = velocity  # Forward velocity in m/s
        msg.vy = 0.0
        msg.vyaw = 0.0

        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher.publish(msg)
            time.sleep(0.1)  # Publish at 10 Hz

    def walk_backward(self, duration: float, velocity: float = 0.4) -> None:
        """Command the robot to walk backward."""
        self.get_logger().info("Walking backward...")
        msg = VelocityCommand()
        msg.command_priority.sender_id = self._sender_id
        msg.command_priority.priority = FAUNA_SDK_COMMAND_PRIORITY
        msg.vx = -velocity  # Backward velocity in m/s
        msg.vy = 0.0
        msg.vyaw = 0.0

        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher.publish(msg)
            time.sleep(0.1)  # Publish at 10 Hz

    def stop(self, duration: float | None = None) -> None:
        """Stop the robot."""
        self.get_logger().info("Stopping...")
        msg = VelocityCommand()
        msg.command_priority.sender_id = self._sender_id
        msg.command_priority.priority = FAUNA_SDK_COMMAND_PRIORITY
        msg.vx = 0.0
        msg.vy = 0.0
        msg.vyaw = 0.0
        if duration:
            start_time = time.time()
            while time.time() - start_time < duration:
                self.publisher.publish(msg)
                time.sleep(0.1)  # Publish at 10 Hz
        else:
            self.publisher.publish(msg)


def main(args: list | None = None) -> None:
    rclpy.init(args=args)
    node = WalkBackForth()

    try:
        node.walk()

    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
