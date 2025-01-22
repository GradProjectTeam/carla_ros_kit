#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
import socket
import pickle
import time

class CarlaLocationPublisher(Node):
    def __init__(self):
        super().__init__('carla_location_publisher')
        
        # Create publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/carla/vehicle/pose', 10)
        self.twist_pub = self.create_publisher(TwistStamped, '/carla/vehicle/twist', 10)
        
        # Initialize socket connection
        self.socket = None
        self.connect_to_carla()
        
        # Create timer for receiving and publishing data
        self.create_timer(0.1, self.timer_callback)  # 10Hz
        
        self.get_logger().info("Node initialized")

    def connect_to_carla(self):
        if self.socket:
            self.socket.close()
        
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        connected = False
        retry_count = 0
        
        while not connected and retry_count < 5:
            try:
                self.get_logger().info("Attempting to connect to CARLA...")
                self.socket.connect(('localhost', 12345))
                connected = True
                self.get_logger().info("Connected to CARLA successfully")
            except ConnectionRefusedError:
                retry_count += 1
                self.get_logger().warn(f"Connection failed, retrying... ({retry_count}/5)")
                time.sleep(2)
        
        if not connected:
            self.get_logger().error("Failed to connect to CARLA after 5 attempts")
            return False
        return True

    def timer_callback(self):
        if not self.socket:
            if not self.connect_to_carla():
                return
        
        try:
            # Receive data from CARLA
            data = pickle.loads(self.socket.recv(4096))
            
            # Create and publish pose message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "map"
            pose_msg.pose.position.x = float(data['location']['x'])
            pose_msg.pose.position.y = float(data['location']['y'])
            pose_msg.pose.position.z = float(data['location']['z'])
            self.pose_pub.publish(pose_msg)
            
            # Create and publish twist message
            twist_msg = TwistStamped()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.header.frame_id = "map"
            twist_msg.twist.linear.x = float(data['velocity']['x'])
            twist_msg.twist.linear.y = float(data['velocity']['y'])
            twist_msg.twist.linear.z = float(data['velocity']['z'])
            self.twist_pub.publish(twist_msg)
            
            self.get_logger().debug(f"Published location: ({data['location']['x']:.2f}, {data['location']['y']:.2f}, {data['location']['z']:.2f})")
            
        except Exception as e:
            self.get_logger().error(f"Error in timer callback: {str(e)}")
            self.socket = None

def main():
    rclpy.init()
    node = CarlaLocationPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.socket:
            node.socket.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
