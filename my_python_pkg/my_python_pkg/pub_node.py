#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import socket
import struct
import numpy as np
import time

class CarlaLidarPublisher(Node):
    def __init__(self):
        super().__init__('carla_lidar_publisher')
        
        # Create publisher
        self.publisher = self.create_publisher(PointCloud2, '/carla/lidar', 10)
        
        # Socket setup
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_address = ('localhost', 12345)
        
        # Connect to CARLA script
        try:
            self.socket.connect(server_address)
            self.get_logger().info(f'Connected to {server_address}')
            
            # Create timer for receiving data
            self.timer = self.create_timer(0.1, self.timer_callback)
            
        except Exception as e:
            self.get_logger().error(f'Failed to connect: {str(e)}')
            raise e

    def timer_callback(self):
        try:
            # Read header (number of points)
            header_data = self.socket.recv(4)
            if not header_data:
                self.get_logger().error('No data received')
                return
                
            num_points = struct.unpack('!I', header_data)[0]
            
            # Read point cloud data
            data_size = num_points * 16  # 4 float32 values per point
            point_data = self.socket.recv(data_size)
            
            if len(point_data) != data_size:
                self.get_logger().error(f'Incomplete data received: {len(point_data)} != {data_size}')
                return
            
            # Convert to numpy array
            points = np.frombuffer(point_data, dtype=np.float32).reshape((num_points, 4))
            
            # Create PointCloud2 message
            msg = PointCloud2()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'lidar_link'
            
            msg.height = 1
            msg.width = num_points
            
            msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
            ]
            
            msg.is_bigendian = False
            msg.point_step = 16
            msg.row_step = msg.point_step * num_points
            msg.is_dense = True
            msg.data = point_data
            
            # Publish message
            self.publisher.publish(msg)
            self.get_logger().info(f'Published {num_points} points')
            
        except Exception as e:
            self.get_logger().error(f'Error in callback: {str(e)}')
            if not rclpy.ok():
                raise e

    def __del__(self):
        if hasattr(self, 'socket'):
            self.socket.close()

def main(args=None):
    try:
        rclpy.init(args=args)
        node = CarlaLidarPublisher()
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
