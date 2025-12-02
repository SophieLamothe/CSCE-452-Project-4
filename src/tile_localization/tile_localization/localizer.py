import math
import os

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from ament_index_python.packages import get_package_share_directory

from .world_loader import load_world


class HistogramLocalizer(Node):
    def __init__(self):
        super().__init__("localizer")

        # --- Parameters ---
        # world file must match the one used by map_publisher
        self.declare_parameter("world_file", "cave.world")
        world_file = self.get_parameter("world_file").value

        share_dir = get_package_share_directory("tile_localization")
        world_path = os.path.join(share_dir, "worlds", world_file)
        self.get_logger().info(f"Localizer loading world file: {world_path}")

        self.resolution, self.width, self.height, self.map_grid = load_world(world_path)
        # map_grid: 0 = light, 100 = dark
        self.is_dark = (self.map_grid == 100)

        # Belief over tiles, uniform to start
        self.belief = np.ones((self.height, self.width), dtype=np.float64)
        self.belief /= self.belief.sum()

        # Current heading (from compass) and last cmd time
        self.theta = 0.0
        self.last_cmd_time = None

        # --- Sensor model parameters (you will tune these) ---
        # Assume color sensor gives float ~[0,1]
        # light tiles ~ 0.2, dark tiles ~ 0.8
        self.light_mean = 0.2
        self.light_sigma = 0.15
        self.dark_mean = 0.8
        self.dark_sigma = 0.15

        # Publishers and subscribers
        self.pose_pub = self.create_publisher(Pose2D, "/estimated_pose", 10)

        self.cmd_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_callback, 10
        )
        self.color_sub = self.create_subscription(
            Float32, "/color_sensor", self.color_callback, 10
        )
        self.compass_sub = self.create_subscription(
            Float32, "/compass", self.compass_callback, 10
        )

        # Publish best estimate every 2 seconds
        self.timer = self.create_timer(2.0, self.publish_estimate)

        self.get_logger().info("HistogramLocalizer initialized")

    # ---------------- MOTION UPDATE ----------------
    def cmd_callback(self, msg: Twist):
        now = self.get_clock().now()

        if self.last_cmd_time is None:
            self.last_cmd_time = now
            return

        dt = (now - self.last_cmd_time).nanoseconds / 1e9
        self.last_cmd_time = now

        # distance = linear velocity * dt
        dist = msg.linear.x * dt
        if abs(dist) < 1e-4:
            return

        # Project motion in world frame
        dx = dist * math.cos(self.theta)
        dy = dist * math.sin(self.theta)

        # Convert to cell shifts
        sx = int(round(dx / self.resolution))
        sy = int(round(dy / self.resolution))

        if sx == 0 and sy == 0:
            return

        # Shift belief grid
        shifted = np.roll(self.belief, shift=(sy, sx), axis=(0, 1))

        # Simple diffusion (blur) to model noise
        new_belief = 0.4 * shifted
        for dx_n, dy_n, w in [
            (1, 0, 0.075),
            (-1, 0, 0.075),
            (0, 1, 0.075),
            (0, -1, 0.075),
            (1, 1, 0.05),
            (1, -1, 0.05),
            (-1, 1, 0.05),
            (-1, -1, 0.05),
        ]:
            new_belief += w * np.roll(shifted, shift=(dy_n, dx_n), axis=(0, 1))

        self.belief = new_belief
        self._normalize_belief()

    # ---------------- SENSOR UPDATE ----------------
    def color_callback(self, msg: Float32):
        z = msg.data  # color sensor reading

        # Precompute likelihoods for dark vs light tiles
        like_dark = self._gaussian(z, self.dark_mean, self.dark_sigma)
        like_light = self._gaussian(z, self.light_mean, self.light_sigma)

        likelihood_grid = np.where(self.is_dark, like_dark, like_light)

        self.belief *= likelihood_grid
        self._normalize_belief()

    # ---------------- COMPASS ----------------
    def compass_callback(self, msg: Float32):
        # Assume msg.data is heading in radians
        self.theta = msg.data

    # ---------------- PUBLISH ESTIMATE ----------------
    def publish_estimate(self):
        if self.belief is None:
            return

        idx = np.argmax(self.belief)
        y_idx, x_idx = np.unravel_index(idx, self.belief.shape)

        pose = Pose2D()
        # Cell center coordinates
        pose.x = (x_idx + 0.5) * self.resolution
        pose.y = (y_idx + 0.5) * self.resolution
        pose.theta = self.theta

        self.pose_pub.publish(pose)

    # ---------------- HELPERS ----------------
    def _normalize_belief(self):
        total = self.belief.sum()
        if total <= 0.0:
            # Reset to uniform if something went wrong
            self.belief[:] = 1.0 / (self.width * self.height)
        else:
            self.belief /= total

    @staticmethod
    def _gaussian(z, mean, sigma):
        if sigma <= 0:
            return 1.0
        return math.exp(-0.5 * ((z - mean) / sigma) ** 2)


def main(args=None):
    rclpy.init(args=args)
    node = HistogramLocalizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
