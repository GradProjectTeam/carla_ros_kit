#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import socket
import struct
import numpy as np
import time
from std_msgs.msg import Header

class SensorReaderNode(Node):
    def __init__(self):
        super().__init__('sensor_reader')
        
        # Create publishers
        self.lidar_pub = self.create_publisher(PointCloud2, '/carla/lidar', 10)
        self.radar_pub = self.create_publisher(PointCloud2, '/carla/radar', 10)
        self.imu_pub = self.create_publisher(Imu, '/carla/imu', 10)
        
        # Connect to sensors
        self.lidar_socket = self.connect_to_sensor(12345, "LIDAR")
        self.radar_socket = self.connect_to_sensor(12346, "RADAR")
        self.imu_socket = self.connect_to_sensor(12347, "IMU")
        
        # Create timer for reading data
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        
        self.get_logger().info("Sensor reader node initialized")

    def connect_to_sensor(self, port, sensor_name):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_ip = 'localhost'
        
        self.get_logger().info(f"Connecting to {sensor_name} on port {port}...")
        while True:
            try:
                sock.connect((server_ip, port))
                self.get_logger().info(f"{sensor_name} connected successfully")
                return sock
            except ConnectionRefusedError:
                self.get_logger().warn(f"Waiting for {sensor_name} server...")
                time.sleep(1)
            except Exception as e:
                self.get_logger().error(f"{sensor_name} connection error: {e}")
                time.sleep(1)

    def create_point_cloud_msg(self, points, timestamp, frame_id):
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        
        # Set point cloud fields
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        msg.point_step = 16  # 4 fields * 4 bytes
        msg.row_step = msg.point_step * points.shape[0]
        msg.height = 1
        msg.width = points.shape[0]
        msg.is_dense = True
        msg.data = points.tobytes()
        
        return msg

    def create_imu_msg(self, imu_data, timestamp):
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"
        
        # Set accelerometer data
        msg.linear_acceleration = Vector3()
        msg.linear_acceleration.x = float(imu_data[0])
        msg.linear_acceleration.y = float(imu_data[1])
        msg.linear_acceleration.z = float(imu_data[2])
        
        # Set gyroscope data
        msg.angular_velocity = Vector3()
        msg.angular_velocity.x = float(imu_data[3])
        msg.angular_velocity.y = float(imu_data[4])
        msg.angular_velocity.z = float(imu_data[5])
        
        return msg

    def timer_callback(self):
        # Read and publish LIDAR data
        try:
            header = self.lidar_socket.recv(8)
            if len(header) == 8:
                size, timestamp = struct.unpack('!II', header)
                data = self.lidar_socket.recv(size)
                points = np.frombuffer(data, dtype=np.float32).reshape(-1, 4)
                msg = self.create_point_cloud_msg(points, timestamp, "lidar_link")
                self.lidar_pub.publish(msg)
                self.get_logger().debug(f'Published LIDAR points: {len(points)}')
        except Exception as e:
            self.get_logger().error(f"LIDAR error: {e}")

        # Read and publish RADAR data
        try:
            header = self.radar_socket.recv(8)
            if len(header) == 8:
                size, timestamp = struct.unpack('!II', header)
                data = self.radar_socket.recv(size)
                points = np.frombuffer(data, dtype=np.float32).reshape(-1, 4)
                msg = self.create_point_cloud_msg(points, timestamp, "radar_link")
                self.radar_pub.publish(msg)
                self.get_logger().debug(f'Published RADAR points: {len(points)}')
        except Exception as e:
            self.get_logger().error(f"RADAR error: {e}")

        # Read and publish IMU data
        try:
            header = self.imu_socket.recv(8)
            if len(header) == 8:
                size, timestamp = struct.unpack('!II', header)
                data = self.imu_socket.recv(size)
                imu_data = np.frombuffer(data, dtype=np.float32)
                if len(imu_data) == 7:  # 3 accel + 3 gyro + 1 compass
                    msg = self.create_imu_msg(imu_data, timestamp)
                    self.imu_pub.publish(msg)
                    self.get_logger().debug('Published IMU data')
        except Exception as e:
            self.get_logger().error(f"IMU error: {e}")

    def __del__(self):
        # Close sockets
        for sock in [self.lidar_socket, self.radar_socket, self.imu_socket]:
            if hasattr(self, sock):
                sock.close()

def main(args=None):
    rclpy.init(args=args)
    node = SensorReaderNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()