import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from ament_index_python.packages import get_package_share_directory
from .world_loader import load_world
import os

class MapPublisher(Node):
    def __init__(self):
        super().__init__("map_publisher")
        
        # Get world file parameter
        self.declare_parameter("world_file", "cave.world")
        world_file = self.get_parameter("world_file").value
        
        # Load world file from package share directory
        share_dir = get_package_share_directory("tile_localization")
        world_path = os.path.join(share_dir, "worlds", world_file)
        
        self.get_logger().info(f"Loading world file: {world_path}")
        self.resolution, self.width, self.height, self.grid = load_world(world_path)
        
        # Create publisher for /floor topic (required by assignment)
        self.pub = self.create_publisher(OccupancyGrid, "/floor", 10)
        
        # Timer to publish every 5 seconds (required by assignment)
        self.timer = self.create_timer(5.0, self.publish_map)
        
        # Publish immediately on startup
        self.publish_map()
        
        self.get_logger().info(
            f"Map publisher initialized - {self.width}x{self.height} @ {self.resolution}m"
        )
    
    def publish_map(self):
        """Publish the floor map as an OccupancyGrid."""
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        # Map metadata
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        
        # Origin at (0, 0) with no rotation
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        
        # Grid data (flattened row-major)
        msg.data = list(self.grid.flatten())
        
        self.pub.publish(msg)
        self.get_logger().info("Published /floor map")

def main(args=None):
    rclpy.init(args=args)
    node = MapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()