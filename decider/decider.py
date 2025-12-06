# decider.py
#
#   @description : The entry py file for decision-making on clients
#

import sys, os, math, signal, time
import numpy as np
from pathlib    import Path
from datetime   import datetime
from typing     import Optional, Dict, Any, Callable, List

# ROS 2
import rclpy
from rclpy.node import Node

# Interfaces with other components
import interfaces.action
import interfaces.vision
import configuration

# Import user_entry
import user_entry


class Agent(Node):
    """
    Main agent class responsible for decision-making and robot control.
    Manages state machines and handles communication with other components.
    """

    def __init__(self, args=None):
        """Initialize the Agent instance and set up ROS 2 node and components."""
        super().__init__('decider')
        self.logger = self.get_logger()
        self.logger.info("[Core] Initializing the core")
        self._config = configuration.load_config()
        self._action = interfaces.action.Action(self)
        self._vision = interfaces.vision.Vision(self)
        self.logger.info("[Core] Core initialized. Calling user's init()")

        user_entry.init(self)

        # 创建定时器，替代ROS 1中的while循环
        timer_period = 1.0 / 10  # 默认10Hz
        self.timer = self.create_timer(timer_period, self._timer_callback)
        self.logger.info("[Core] ALl initializing finished. ")

    def _timer_callback(self):
        """定时器回调函数，替代ROS 1中的主循环"""
        try:
            user_entry.loop(self)
        except Exception as e:
            self.logger.error(f"Error in timer callback: {e}")

    def cmd_vel(self, vel_x: float, vel_y: float, vel_theta: float) -> None:
        """Set robot velocity with scaling from configuration."""
        vel_x *= self._config.get("max_walk_vel_x", 0.25)
        vel_y *= self._config.get("max_walk_vel_y", 0.1)
        vel_theta *= self._config.get("max_walk_vel_theta", 0.5)
        if vel_x > 0.001:
            vel_x += self._config.get("min_walk_vel_x", 0.2)
        elif vel_x < -0.001:
            vel_x -= self._config.get("min_walk_vel_x", 0.2)
        if vel_y > 0.001:
            vel_y += self._config.get("min_walk_vel_y", 0.2)
        elif vel_y < -0.001:
            vel_y -= self._config.get("min_walk_vel_y", 0.2)
        if vel_theta > 0.001:
            vel_theta += self._config.get("min_walk_vel_theta", 0.3)
        elif vel_theta < -0.001:
            vel_theta -= self._config.get("min_walk_vel_thetea", 0.3)
        self._action.cmd_vel(vel_x, vel_y, vel_theta)

    def look_at(self, args) -> None:
        """Set robot's head and neck to look at specified position."""
        self._vision.look_at(args)

    def move_head(self, pitch: float, yaw: float) -> None:
        """Set robot's head and neck angles."""
        self._action._move_head(pitch, yaw)

    def stop(self, sleep_time: float = 0) -> None:
        """Stop the robot's movement and optionally sleep."""
        self._action.cmd_vel(0.0, 0.0, 0.0)
        time.sleep(sleep_time)

    def kick(self, foot=0, death=0) -> None:
        """Execute the kicking action."""
        self._action.do_kick(foot, death)

    def save_ball(self, direction):
        if direction not in [1, 2]:
            self.logger.error("[Core] Save ball direction must be 1 or 2, actually: {}".format(direction))
            return False

        for _ in range(1):
            self._action.save_ball(direction)

    # normalize(-pi,pi)
    def angle_normalize(self, angle: float) -> float:
        """Normalize angle to the range (-pi, pi)."""
        if angle is None:
            return None
        angle = angle % (2 * math.pi)
        if angle > math.pi:
            angle -= 2 * math.pi
        elif angle < -math.pi:
            angle += 2 * math.pi
        self.get_logger().debug(f"Normalized angle: {angle}")
        return angle

    # Data retrieval methods
    def get_robots_data(self) -> Dict:
        """Return data about all robots."""
        return self._robots_data

    def get_command(self) -> Dict:
        """Return current command."""
        return self._command

    def get_self_pos(self) -> np.ndarray:
        """Return self position in map coordinates."""
        return self._vision.self_pos

    def get_self_yaw(self) -> float:
        """Return self orientation angle."""
        return self._vision.self_yaw

    def get_ball_pos(self) -> List[Optional[float]]:
        """Return ball position relative to robot or [None, None] if not found."""
        if self.get_if_ball():
            return self._vision.get_ball_pos()
        else:
            return [None, None]

    def get_ball_angle(self) -> Optional[float]:
        """Calculate and return ball angle relative to robot."""
        ball_pos = self.get_ball_pos()
        ball_x = ball_pos[0]
        ball_y = ball_pos[1]

        if ball_pos is not None and ball_x is not None and ball_y is not None and self.get_if_ball():
            if ball_x == 0 and ball_y == 0:
                return None
            else:
                angle_rad = - math.atan2(ball_x, ball_y)
                return angle_rad
        else:
            return None

    def get_ball_pos_in_vis(self) -> np.ndarray:
        """Return ball position in vision coordinates."""
        return self._vision.get_ball_pos_in_vis()

    def get_ball_pos_in_map(self) -> np.ndarray:
        """Return ball position in global map coordinates."""
        if not self.get_if_ball():
            self.get_logger().warning("Ball not detected, returning None")
            return None
        return self._vision.get_ball_pos_in_map()

    def get_if_ball(self) -> bool:
        """Return whether ball is detected."""
        return self._vision.get_if_ball()

    def get_if_close_to_ball(self) -> bool:
        """Return whether robot is close to the ball."""
        if self.get_if_ball():
            return self.get_ball_distance() < self._config.get("close_to_ball_threshold", 0.5)
        else:
            return False

    def get_ball_distance(self) -> float:
        """Return distance to ball or a large value if not detected."""
        if self.get_if_ball():
            return self._vision.ball_distance
        else:
            return 1e6

    def get_neck(self) -> float:
        """Return neck angle."""
        return self._vision.neck

    def get_head(self) -> float:
        """Return head angle."""
        return self._vision.head


    def get_ball_history(self):
        """Return the history of ball positions."""
        return self._vision.get_ball_history()

    def get_config(self) -> Dict:
        """Return agent configuration."""
        return self._config


if __name__ == "__main__":
    """Main entry point for the decider program."""
    rclpy.init(args=sys.argv)
    agent = Agent(sys.argv)

    def signal_handler(sig, frame):
        if agent:
            print("\n")
            agent.get_logger().warning(f"[Core] Interrupted")
            try:
                agent.stop()
                agent.destroy_node()
                rclpy.shutdown()
                sys.exit(0)
            except Exception as e:
                agent.get_logger().error(f"[Core] Error during stop command in signal handler: {e}")


    signal.signal(signal.SIGINT, signal_handler) 
    rclpy.spin(agent)
