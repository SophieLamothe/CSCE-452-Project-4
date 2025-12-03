#!/usr/bin/env python3
"""
Analyze sensor data from a bag file to tune sensor model parameters.
Usage: 
  1. ros2 bag play bag12
  2. python3 analyze_sensor.py
"""

import rclpy
from rclpy.node import Node
from example_interfaces.msg import UInt8
import numpy as np
import sys

class SensorAnalyzer(Node):
    def __init__(self):
        super().__init__("sensor_analyzer")
        
        self.readings = []
        
        self.sub = self.create_subscription(
            UInt8,
            "/floor_sensor",
            self.sensor_callback,
            10
        )
        
        self.timer = self.create_timer(5.0, self.print_stats)
        
        self.get_logger().info("Sensor analyzer started. Play a bag file and let it run...")
    
    def sensor_callback(self, msg):
        self.readings.append(msg.data)
    
    def print_stats(self):
        if len(self.readings) < 10:
            self.get_logger().info(f"Collected {len(self.readings)} readings so far...")
            return
        
        arr = np.array(self.readings)
        
        self.get_logger().info("="*60)
        self.get_logger().info(f"Total readings: {len(arr)}")
        self.get_logger().info(f"Mean: {arr.mean():.2f}")
        self.get_logger().info(f"Std Dev: {arr.std():.2f}")
        self.get_logger().info(f"Min: {arr.min()}")
        self.get_logger().info(f"Max: {arr.max()}")
        self.get_logger().info(f"Median: {np.median(arr):.2f}")
        
        # Try to identify two clusters (light and dark)
        sorted_readings = np.sort(arr)
        
        # Simple approach: split at midpoint and analyze each half
        mid_idx = len(sorted_readings) // 2
        lower_half = sorted_readings[:mid_idx]
        upper_half = sorted_readings[mid_idx:]
        
        self.get_logger().info("-"*60)
        self.get_logger().info("Assuming two tile types:")
        self.get_logger().info(f"  Lower cluster (likely light): mean={lower_half.mean():.2f}, std={lower_half.std():.2f}")
        self.get_logger().info(f"  Upper cluster (likely dark):  mean={upper_half.mean():.2f}, std={upper_half.std():.2f}")
        self.get_logger().info("-"*60)
        self.get_logger().info("Normalized (divide by 255):")
        self.get_logger().info(f"  Light: mean={lower_half.mean()/255:.4f}, std={lower_half.std()/255:.4f}")
        self.get_logger().info(f"  Dark:  mean={upper_half.mean()/255:.4f}, std={upper_half.std()/255:.4f}")
        self.get_logger().info("="*60)

def main():
    rclpy.init()
    node = SensorAnalyzer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()