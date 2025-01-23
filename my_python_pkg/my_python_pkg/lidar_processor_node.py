#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import struct
import socket

class LidarProcessorNode(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        
        # Socket setup
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.socket.connect(('localhost', 12345))
        self.get_logger().info('Connected to LIDAR socket server')
        
        # Create publisher for processed LIDAR data
        self.publisher = self.create_publisher(
            PointCloud2,
            '/carla/lidar',  # Topic for RViz2
            10)
        
        # Create timer for processing LIDAR data
        self.create_timer(0.01, self.timer_callback)  # 100Hz
        self.get_logger().info('LIDAR Processor Node initialized')

    def timer_callback(self):
        try:
            # Receive data size
            size_data = self.socket.recv(4)
            if not size_data:
                return
            data_size = struct.unpack('!I', size_data)[0]
            
            # Receive point cloud data
            data = bytearray()
            remaining = data_size
            while remaining > 0:
                chunk = self.socket.recv(min(remaining, 4096))
                if not chunk:
                    return
                data.extend(chunk)
                remaining -= len(chunk)
            
            # Convert to numpy array
            points = np.frombuffer(data, dtype=np.float32)
            num_points = len(points) // 4  # Each point has 4 values (x,y,z,intensity)
            points = points.reshape(num_points, 4)
            
            # Process points
            # 1. Filter ground points
            height_threshold = -0.3
            non_ground_mask = points[:, 2] > height_threshold
            filtered_points = points[non_ground_mask]
            
            # 2. Remove points too far away
            distance = np.sqrt(np.sum(filtered_points[:, :3]**2, axis=1))
            distance_mask = distance < 50.0  # 50 meters max range
            filtered_points = filtered_points[distance_mask]
            
            # Create PointCloud2 message
            msg = PointCloud2()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "lidar_link"
            
            # Set up fields
            msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            
            # Set message properties
            msg.point_step = 16  # 4 fields * 4 bytes
            msg.row_step = msg.point_step * len(filtered_points)
            msg.height = 1
            msg.width = len(filtered_points)
            msg.is_dense = True
            msg.data = filtered_points.tobytes()
            
            # Publish point cloud
            self.publisher.publish(msg)
            
            # Log statistics (less frequently to avoid spam)
            if self.count_subscribers('/carla/lidar') > 0:
                self.get_logger().info(
                    f'Published {len(filtered_points)} points '
                    f'(filtered from {num_points} points)'
                )
            
        except Exception as e:
            self.get_logger().error(f'Error processing LIDAR data: {str(e)}')
            # Try to reconnect if connection is lost
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.socket.connect(('localhost', 12345))
                self.get_logger().info('Reconnected to LIDAR socket server')
            except Exception as conn_err:
                self.get_logger().error(f'Failed to reconnect: {str(conn_err)}')

    def __del__(self):
        if hasattr(self, 'socket'):
            self.socket.close()

def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
