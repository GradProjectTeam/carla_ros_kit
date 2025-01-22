#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import socket
import struct
import time

class CarlaIMUSubscriber(Node):
    def __init__(self):
        super().__init__('carla_imu_subscriber')
        
        # Publisher
        self.imu_pub = self.create_publisher(Imu, '/carla/vehicle/imu', 10)
        
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
            
            # Unpack IMU data
            ax, ay, az, gx, gy, gz = struct.unpack('!ffffff', data)
            
            # Create IMU message
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "imu_link"
            
            # Set linear acceleration
            imu_msg.linear_acceleration.x = ax
            imu_msg.linear_acceleration.y = ay
            imu_msg.linear_acceleration.z = az
            
            # Set angular velocity
            imu_msg.angular_velocity.x = gx
            imu_msg.angular_velocity.y = gy
            imu_msg.angular_velocity.z = gz
            
            # Set orientation covariance to -1 to indicate no orientation data
            imu_msg.orientation_covariance[0] = -1
            
            # Publish IMU data
            self.imu_pub.publish(imu_msg)
            
        except Exception as e:
            self.get_logger().error("Error in callback: {}".format(str(e)))
            rclpy.shutdown()

    def __del__(self):
        if hasattr(self, 'socket'):
            self.socket.close()

def main():
    rclpy.init()
    node = CarlaIMUSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
