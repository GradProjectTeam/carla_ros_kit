#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
import socket
import struct
import time

class CarlaSubscriber(Node):
    def __init__(self):
        super().__init__('carla_subscriber')
        
        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/carla/vehicle/pose', 10)
        self.twist_pub = self.create_publisher(TwistStamped, '/carla/vehicle/twist', 10)
        
        # Socket setup
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        
        # Connect to CARLA
        connected = False
        while not connected:
            try:
                self.socket.connect(('localhost', 12345))
                connected = True
                self.get_logger().info("Connected to CARLA")
            except ConnectionRefusedError:
                self.get_logger().warn("Connection failed, retrying in 1 second...")
                time.sleep(1)
        
        self.create_timer(0.01, self.timer_callback)  # 100Hz

    def timer_callback(self):
        try:
            # Read message size (4 bytes)
            size_data = self.socket.recv(4)
            if not size_data:
                return
            size = struct.unpack('!I', size_data)[0]
            
            # Read data (24 bytes: 6 floats)
            data = self.socket.recv(size)
            if len(data) != size:
                return
            
            # Unpack the data
            x, y, z, vx, vy, vz = struct.unpack('!ffffff', data)
            
            # Create and publish pose message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "map"
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.position.z = z
            self.pose_pub.publish(pose_msg)
            
            # Create and publish twist message
            twist_msg = TwistStamped()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.header.frame_id = "map"
            twist_msg.twist.linear.x = vx
            twist_msg.twist.linear.y = vy
            twist_msg.twist.linear.z = vz
            self.twist_pub.publish(twist_msg)
            
        except Exception as e:
            self.get_logger().error("Error in callback: {}".format(str(e)))
            rclpy.shutdown()

    def __del__(self):
        if hasattr(self, 'socket'):
            self.socket.close()

def main():
    rclpy.init()
    node = CarlaSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
