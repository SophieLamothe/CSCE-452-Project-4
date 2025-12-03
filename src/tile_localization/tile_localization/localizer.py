import math
import os
import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D, Twist
from example_interfaces.msg import Float32, UInt8
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from ament_index_python.packages import get_package_share_directory

from .world_loader import load_world


class HistogramLocalizer(Node):
    def __init__(self):
        super().__init__("localizer")

        # Loading cmd parameters
        self.declare_parameter("world_file", "cave.world")
        self.declare_parameter("output_poses_file", "initial_poses.txt")
        
        world_file = self.get_parameter("world_file").value
        self.output_file = self.get_parameter("output_poses_file").value

        # Loading the world map
        share_dir = get_package_share_directory("tile_localization")
        world_path = os.path.join(share_dir, "worlds", world_file)
        
        self.resolution, self.width, self.height, self.map_grid = load_world(world_path)
        self.is_dark = (self.map_grid == 100)
        
        # Logging important statistics
        num_dark = np.sum(self.is_dark)
        num_light = np.sum(~self.is_dark)
        self.get_logger().info("="*60)
        self.get_logger().info(f"Loaded: {world_file}")
        self.get_logger().info(f"Map size: {self.width}x{self.height} cells, resolution: {self.resolution}m")
        self.get_logger().info(f"Tiles - Dark: {num_dark} ({num_dark/(num_dark+num_light)*100:.1f}%), "
                              f"Light: {num_light} ({num_light/(num_dark+num_light)*100:.1f}%)")
        self.get_logger().info("="*60)

        # Histogram Filter
        # Start with uniform belief over all cells
        self.belief = np.ones((self.height, self.width), dtype=np.float64)
        self.belief /= self.belief.sum()

        # Robot's State
        self.theta = 0.0  # From compass
        self.last_cmd_time = None
        self.accumulated_dx = 0.0
        self.accumulated_dy = 0.0
        
        # Tracking robot's state
        self.initial_pose_recorded = False
        self.initial_pose = None
        self.sensor_update_count = 0
        self.pose_history = []  # For visualization trail
        
        # Calibration
        self.sensor_readings = []
        self.calibration_samples = 100
        self.is_calibrated = False
        
        self.light_mean = None
        self.light_sigma = None
        self.dark_mean = None
        self.dark_sigma = None
        self.min_likelihood = 1e-12

        # Publisheres
        self.pose_pub = self.create_publisher(Pose2D, "/estimated_pose", 10)
        self.marker_pub = self.create_publisher(Marker, "/position_marker", 10)
        
        # Subscribers
        self.cmd_sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_callback, 10)
        self.color_sub = self.create_subscription(UInt8, "/floor_sensor", self.color_callback, 10)
        self.compass_sub = self.create_subscription(Float32, "/compass", self.compass_callback, 10)

        self.estimate_timer = self.create_timer(2.0, self.publish_estimate)
        self.marker_timer = self.create_timer(5.0, self.publish_marker)

        self.get_logger().info("Localizer initialized - collecting calibration data...")
    
    
    def calibrate_sensor_model(self):
        # Calibrating sensor from given data
        if len(self.sensor_readings) < self.calibration_samples:
            return False
        
        readings = np.array(self.sensor_readings)
        
        # Logs that we might remove
        self.get_logger().info("="*60)
        self.get_logger().info("AUTO-CALIBRATING SENSOR MODEL")
        self.get_logger().info(f"Collected {len(readings)} samples")
        self.get_logger().info(f"Statistics - Mean: {readings.mean():.1f}, Std: {readings.std():.1f}")
        self.get_logger().info(f"Range: [{readings.min()}, {readings.max()}]")
        
        sorted_readings = np.sort(readings)
        best_threshold = readings.mean()
        best_variance = float('inf')
        
        # Trying different split points
        for i in range(len(sorted_readings) // 4, 3 * len(sorted_readings) // 4):
            lower = sorted_readings[:i]
            upper = sorted_readings[i:]
            
            if len(lower) > 10 and len(upper) > 10:
                # Within-class variance
                var = (len(lower) * lower.var() + len(upper) * upper.var()) / len(sorted_readings)
                if var < best_variance:
                    best_variance = var
                    best_threshold = sorted_readings[i]
        
        # Splitting into two clusters
        lower_cluster = readings[readings <= best_threshold]
        upper_cluster = readings[readings > best_threshold]
        
        # Setting parameters; normalized to [0,1]
        self.light_mean = lower_cluster.mean() / 255.0
        self.light_sigma = max(lower_cluster.std() / 255.0, 0.015)
        self.dark_mean = upper_cluster.mean() / 255.0
        self.dark_sigma = max(upper_cluster.std() / 255.0, 0.015)
        
        # Calculate separation metric
        separation = abs(self.dark_mean - self.light_mean) / (self.dark_sigma + self.light_sigma)
        
        self.get_logger().info("-"*60)
        self.get_logger().info(f"Light cluster: n={len(lower_cluster)}, "
                              f"mean={lower_cluster.mean():.1f}, std={lower_cluster.std():.1f}")
        self.get_logger().info(f"Dark cluster:  n={len(upper_cluster)}, "
                              f"mean={upper_cluster.mean():.1f}, std={upper_cluster.std():.1f}")
        self.get_logger().info(f"Threshold: {best_threshold:.1f}")
        self.get_logger().info("-"*60)
        self.get_logger().info("Normalized parameters:")
        self.get_logger().info(f"  Light: μ={self.light_mean:.4f}, σ={self.light_sigma:.4f}")
        self.get_logger().info(f"  Dark:  μ={self.dark_mean:.4f}, σ={self.dark_sigma:.4f}")
        self.get_logger().info(f"  Separation: {separation:.2f} (higher is better)")
        self.get_logger().info("="*60)
        
        self.is_calibrated = True

        return True

    
    def cmd_callback(self, msg: Twist):
        # Wait for calibration and some sensor updates
        if not self.is_calibrated or self.sensor_update_count < 20:
            return
        
        now = self.get_clock().now()
        if self.last_cmd_time is None:
            self.last_cmd_time = now
            return

        dt = (now - self.last_cmd_time).nanoseconds / 1e9
        self.last_cmd_time = now
        
        # Calculate distance traveled
        dist = msg.linear.x * dt
        if abs(dist) < 1e-4:
            return

        # Project motion into world frame using compass heading
        dx = dist * math.cos(self.theta)
        dy = dist * math.sin(self.theta)
        
        # Accumulate sub cell motion
        self.accumulated_dx += dx
        self.accumulated_dy += dy

        # Convert to cell shifts
        sx = int(round(self.accumulated_dx / self.resolution))
        sy = int(round(self.accumulated_dy / self.resolution))
        
        if sx == 0 and sy == 0:
            return  # Not enough motion yet
        
        # Update accumulator
        self.accumulated_dx -= sx * self.resolution
        self.accumulated_dy -= sy * self.resolution
        
        # Apply motion with diffusion
        # Grid: y=0 at bottom, +y goes up, so positive sy shifts to higher indices (up)
        shifted = np.roll(self.belief, shift=(sy, sx), axis=(0, 1))  # FIXED: no negative
        
        # Diffusion weights
        center = 0.7
        cardinal = 0.05
        diagonal = 0.025
        
        new_belief = center * shifted
        for dx_n, dy_n, weight in [
            (1, 0, cardinal), (-1, 0, cardinal),
            (0, 1, cardinal), (0, -1, cardinal),
            (1, 1, diagonal), (1, -1, diagonal),
            (-1, 1, diagonal), (-1, -1, diagonal),
        ]:
            new_belief += weight * np.roll(shifted, shift=(dy_n, dx_n), axis=(0, 1))

        self.belief = new_belief
        self._normalize_belief()

    
    def color_callback(self, msg: UInt8):
        self.sensor_update_count += 1
        
        # Collect calibration data
        if not self.is_calibrated:
            self.sensor_readings.append(msg.data)
            
            if len(self.sensor_readings) == self.calibration_samples:
                self.calibrate_sensor_model()
            elif len(self.sensor_readings) % 30 == 0:
                self.get_logger().info(
                    f"Calibrating: {len(self.sensor_readings)}/{self.calibration_samples}..."
                )
            return
        
        # Normalize sensor reading to [0, 1]
        z = msg.data / 255.0

        # Calculate likelihoods for each tile type
        like_light = self._gaussian(z, self.light_mean, self.light_sigma)
        like_dark = self._gaussian(z, self.dark_mean, self.dark_sigma)
        
        # Enforce minimum likelihood to prevent numerical underflow
        like_light = max(like_light, self.min_likelihood)
        like_dark = max(like_dark, self.min_likelihood)

        # Create likelihood map based on world map
        likelihood_grid = np.where(self.is_dark, like_dark, like_light)

        # Bayesian update: belief = prior * likelihood
        self.belief *= likelihood_grid
        self._normalize_belief()

    def compass_callback(self, msg: Float32):
        self.theta = msg.data
    
    def publish_estimate(self):
        if not self.is_calibrated:
            return
        idx = np.argmax(self.belief)
        y_idx, x_idx = np.unravel_index(idx, self.belief.shape)

        pose = Pose2D()
        pose.x = (x_idx + 0.5) * self.resolution
        pose.y = (y_idx + 0.5) * self.resolution
        pose.theta = self.theta
        
        confidence = self.belief[y_idx, x_idx]
        
        # Record initial pose when confidence reaches threshold
        if not self.initial_pose_recorded and confidence > 0.005:  # 0.5% confidence
            self.initial_pose = (pose.x, pose.y, pose.theta)
            self.initial_pose_recorded = True
            self._write_initial_pose()
            self.get_logger().info(
                f"★★★ INITIAL POSE: ({pose.x:.2f}, {pose.y:.2f}, {pose.theta:.2f}) "
                f"at confidence {confidence:.5f} ★★★"
            )
        
        # Track history for trail visualization
        self.pose_history.append((pose.x, pose.y))
        if len(self.pose_history) > 50:
            self.pose_history.pop(0)
        
        # Publish required Pose2D message
        self.pose_pub.publish(pose)
        
        # Log estimate
        confidence = self.belief[y_idx, x_idx]
        self.get_logger().info(
            f"Estimate: ({pose.x:.2f}, {pose.y:.2f}, θ={pose.theta:.2f}), "
            f"confidence={confidence:.5f}"
        )
    
    def publish_marker(self):
        if not self.is_calibrated or self.sensor_update_count < 20:
            return
        
        # Get best estimate
        idx = np.argmax(self.belief)
        y_idx, x_idx = np.unravel_index(idx, self.belief.shape)
        
        x_world = (x_idx + 0.5) * self.resolution
        y_world = (y_idx + 0.5) * self.resolution
        confidence = self.belief[y_idx, x_idx]
        
        # Create sphere marker at estimated position
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "map"
        marker.ns = "robot_estimate"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Position
        marker.pose.position.x = x_world
        marker.pose.position.y = y_world
        marker.pose.position.z = 0.3  # Raised to be visible
        marker.pose.orientation.w = 1.0
        
        # Size (larger = more visible)
        marker.scale.x = 0.8
        marker.scale.y = 0.8
        marker.scale.z = 0.4
        
        # Color based on confidence
        if confidence > 0.01:
            # High confidence - green
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.9
        elif confidence > 0.001:
            # Medium confidence - yellow
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8
        else:
            # Low confidence - red
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.7
        
        # Lifetime - persist until next update
        marker.lifetime.sec = 6
        
        self.marker_pub.publish(marker)
        
        self.get_logger().info(
            f"Marker published at ({x_world:.2f}, {y_world:.2f}) "
            f"with confidence {confidence:.5f}"
        )

    def _write_initial_pose(self):
        """Write initial pose estimate to file."""
        if self.initial_pose is None:
            return
        
        try:
            with open(self.output_file, 'a') as f:
                x, y, theta = self.initial_pose
                f.write(f"{x:.2f}, {y:.2f}, {theta:.2f}\n")
            self.get_logger().info(f"Initial pose written to {self.output_file}")
        except Exception as e:
            self.get_logger().error(f"Failed to write initial pose: {e}")
    
    def _normalize_belief(self):
        """Normalize belief to sum to 1.0."""
        total = self.belief.sum()
        if total <= 1e-12:
            self.get_logger().warn("Belief collapsed - resetting to uniform")
            self.belief[:] = 1.0 / (self.width * self.height)
        else:
            self.belief /= total
    
    @staticmethod
    def _gaussian(x, mean, sigma):
        """Calculate Gaussian probability density."""
        if sigma <= 0:
            return 1.0
        return math.exp(-0.5 * ((x - mean) / sigma) ** 2)


def main(args=None):
    rclpy.init(args=args)
    node = HistogramLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()