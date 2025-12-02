import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from ament_index_python.packages import get_package_share_directory
from .world_loader import load_world
import os


class MapPublisher(Node):
    def __init__(self):
        super().__init__("map_publisher")

        # parameter: which world file to load
        self.declare_parameter("world_file", "cave.world")
        world_file = self.get_parameter("world_file").value

        # Get the installed share directory of this package
        share_dir = get_package_share_directory("tile_localization")
        world_path = os.path.join(share_dir, "worlds", world_file)

        self.get_logger().info(f"Loading world file: {world_path}")

        self.resolution, self.width, self.height, self.grid = load_world(world_path)

        self.pub = self.create_publisher(OccupancyGrid, "/floor", 10)
        self.timer = self.create_timer(5.0, self.publish_map)

    def publish_map(self):
        msg = OccupancyGrid()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height

        # origin at (0,0) facing along +x
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = list(self.grid.flatten())

        self.pub.publish(msg)
        self.get_logger().info("Published /floor map")


def main(args=None):
    rclpy.init(args=args)
    node = MapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
