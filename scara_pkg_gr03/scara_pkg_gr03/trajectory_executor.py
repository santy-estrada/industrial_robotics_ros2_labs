#!/usr/bin/env python3
"""
Trajectory Executor Node
========================
This node receives the interpolated trajectory and publishes configurations
to the Pico at a controlled rate.

Topics:
-------
Subscribers:
    - interpolated_trajectory (String): JSON array of configurations

Publishers:
    - inv_kin (Twist): Configuration sent to Pico

Parameters:
-----------
    - publish_rate_hz (float): Publishing rate in Hz
    - start_execution (bool): Whether to start execution immediately
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import json


class TrajectoryExecutor(Node):
    def __init__(self):
        super().__init__('trajectory_executor')
        
        # Parameters
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('start_execution', False)
        
        self.publish_rate = self.get_parameter('publish_rate_hz').value
        self.start_execution = self.get_parameter('start_execution').value
        
        self.get_logger().info(f"=== Trajectory Executor Configuration ===")
        self.get_logger().info(f"Publishing rate: {self.publish_rate} Hz")
        self.get_logger().info(f"Auto-start execution: {self.start_execution}")
        
        # QoS for latched messages
        qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        # Subscriber
        self.create_subscription(
            String,
            'interpolated_trajectory',
            self.trajectory_callback,
            qos
        )
        
        # Publisher - publish to inv_kin to go through the mux
        self.publisher = self.create_publisher(Twist, 'inv_kin', 10)
        
        # State
        self.trajectory = []
        self.current_index = 0
        self.is_executing = False
        self.timer = None
        
        self.get_logger().info("✅ Waiting for interpolated trajectory...")
    
    def trajectory_callback(self, msg: String):
        """Load the trajectory."""
        try:
            self.trajectory = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"❌ Failed to parse trajectory: {e}")
            return
        
        self.get_logger().info(f"📥 Loaded trajectory with {len(self.trajectory)} configurations")
        
        # Reset execution state
        self.current_index = 0
        
        if self.start_execution:
            self.start_trajectory_execution()
        else:
            self.get_logger().info("⏸️  Trajectory loaded. Call start_execution service to begin.")
    
    def start_trajectory_execution(self):
        """Start executing the trajectory."""
        if not self.trajectory:
            self.get_logger().warn("⚠️  No trajectory loaded!")
            return
        
        if self.is_executing:
            self.get_logger().warn("⚠️  Execution already in progress!")
            return
        
        self.is_executing = True
        self.current_index = 0
        
        # Create timer for publishing
        period = 1.0 / self.publish_rate
        self.timer = self.create_timer(period, self.publish_next_configuration)
        
        self.get_logger().info(f"▶️  Started trajectory execution ({len(self.trajectory)} configs)")
    
    def stop_trajectory_execution(self):
        """Stop executing the trajectory."""
        if self.timer:
            self.timer.cancel()
            self.timer = None
        
        self.is_executing = False
        self.get_logger().info("⏹️  Stopped trajectory execution")
    
    def publish_next_configuration(self):
        """Publish the next configuration in the trajectory."""
        if self.current_index >= len(self.trajectory):
            self.get_logger().info("✅ Trajectory execution complete!")
            self.stop_trajectory_execution()
            return
        
        config = self.trajectory[self.current_index]
        
        # Create Twist message
        msg = Twist()
        msg.linear.x = config['j1']  # J1 in degrees
        msg.linear.y = config['j2']  # J2 in degrees
        msg.linear.z = config['j3']   # J3 in mm
        
        # Publish
        self.publisher.publish(msg)
        
        # Log progress periodically
        if self.current_index % 100 == 0:
            progress = (self.current_index / len(self.trajectory)) * 100
            self.get_logger().info(
                f"📍 Progress: {self.current_index}/{len(self.trajectory)} ({progress:.1f}%) | "
                f"J1={config['j1']:.2f}° J2={config['j2']:.2f}° J3={config['j3']:.2f}mm"
            )
        
        self.current_index += 1


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryExecutor()
    
    # If start_execution is True, the node will auto-start
    # Otherwise, it waits for manual trigger (future service implementation)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
