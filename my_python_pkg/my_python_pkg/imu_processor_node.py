#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3, Quaternion, Point
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import socket
import struct
import numpy as np
import math
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class ImuProcessorNode(Node):
    def __init__(self):
        super().__init__('imu_processor')
        
        # Socket setup
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.socket.connect(('localhost', 12346))
        self.get_logger().info('Connected to IMU socket server')
        
        # Publishers with QoS settings
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.imu_publisher = self.create_publisher(
            Imu, 
            '/carla/imu/data', 
            qos_profile)
            
        self.marker_publisher = self.create_publisher(
            Marker, 
            '/carla/imu/marker', 
            qos_profile)
            
        self.compass_publisher = self.create_publisher(
            Float32, 
            '/carla/imu/compass', 
            qos_profile)
        
        # TF2 broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create timer for processing IMU data
        self.create_timer(0.1, self.timer_callback)  # 10Hz
        
        # Initialize vehicle position and orientation
        self.position = Point(x=0.0, y=0.0, z=1.0)
        self.velocity = Vector3(x=0.0, y=0.0, z=0.0)
        self.last_time = self.get_clock().now()
        
        # Initialize markers
        self.setup_markers()
        
        self.get_logger().info('IMU Processor Node initialized')

    def setup_markers(self):
        # Vehicle marker (arrow)
        self.vehicle_marker = Marker()
        self.vehicle_marker.header.frame_id = "map"
        self.vehicle_marker.ns = "vehicle"
        self.vehicle_marker.id = 0
        self.vehicle_marker.type = Marker.ARROW
        self.vehicle_marker.action = Marker.ADD
        
        # Set the scale of the marker
        self.vehicle_marker.scale.x = 2.0  # Length of the arrow
        self.vehicle_marker.scale.y = 0.2  # Width of the arrow
        self.vehicle_marker.scale.z = 0.2  # Height of the arrow
        
        # Set the color
        self.vehicle_marker.color.r = 1.0
        self.vehicle_marker.color.g = 0.0
        self.vehicle_marker.color.b = 0.0
        self.vehicle_marker.color.a = 1.0
        
        # Set the pose
        self.vehicle_marker.pose.position = self.position
        
        # Trail marker (line strip)
        self.trail_marker = Marker()
        self.trail_marker.header.frame_id = "map"
        self.trail_marker.ns = "trail"
        self.trail_marker.id = 1
        self.trail_marker.type = Marker.LINE_STRIP
        self.trail_marker.action = Marker.ADD
        
        # Set the scale of the trail
        self.trail_marker.scale.x = 0.1  # Width of line
        
        # Set the color
        self.trail_marker.color.r = 0.0
        self.trail_marker.color.g = 1.0
        self.trail_marker.color.b = 0.0
        self.trail_marker.color.a = 0.5
        
        # Initialize trail points
        self.trail_marker.points = [self.position]

    def euler_to_quaternion(self, yaw, pitch, roll):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cy * cp * cr + sy * sp * sr
        q.x = cy * cp * sr - sy * sp * cr
        q.y = sy * cp * sr + cy * sp * cr
        q.z = sy * cp * cr - cy * sp * sr
        return q

    def update_position(self, accel, dt):
        # Update velocity using acceleration
        self.velocity.x += accel[0] * dt
        self.velocity.y += accel[1] * dt
        self.velocity.z += accel[2] * dt
        
        # Update position using velocity
        self.position.x += self.velocity.x * dt
        self.position.y += self.velocity.y * dt
        self.position.z = 1.0  # Keep constant height
        
        # Simple damping to prevent unbounded motion
        damping = 0.98
        self.velocity.x *= damping
        self.velocity.y *= damping
        self.velocity.z *= damping

    def update_markers(self, compass):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # Update vehicle marker
        self.vehicle_marker.header.stamp = current_time.to_msg()
        self.vehicle_marker.pose.position = self.position
        
        # Convert compass to radians and create quaternion
        yaw = math.radians(compass)
        self.vehicle_marker.pose.orientation = self.euler_to_quaternion(yaw, 0.0, 0.0)
        
        # Update trail marker
        self.trail_marker.header.stamp = current_time.to_msg()
        self.trail_marker.points.append(self.position)
        
        # Limit trail points more aggressively
        if len(self.trail_marker.points) > 50:  # Reduced from 100
            self.trail_marker.points.pop(0)
            
        # Add small delay between marker publishes
        self.marker_publisher.publish(self.vehicle_marker)
        time.sleep(0.01)  # 10ms delay
        self.marker_publisher.publish(self.trail_marker)

    def timer_callback(self):
        try:
            # Receive data size
            size_data = self.socket.recv(4)
            if not size_data:
                return
            data_size = struct.unpack('!I', size_data)[0]
            
            # Receive IMU data
            data = self.socket.recv(data_size)
            if len(data) != data_size:
                return
            
            # Unpack IMU data
            imu_data = struct.unpack('!7f', data)
            accel = imu_data[0:3]
            gyro = imu_data[3:6]
            compass = imu_data[6]
            
            # Update position based on acceleration
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1e9
            self.update_position(accel, dt)
            
            # Update visualization
            self.update_markers(compass)
            
            # Create and publish IMU message
            imu_msg = Imu()
            imu_msg.header.stamp = current_time.to_msg()
            imu_msg.header.frame_id = "imu_link"
            
            # Set acceleration
            imu_msg.linear_acceleration.x = accel[0]
            imu_msg.linear_acceleration.y = accel[1]
            imu_msg.linear_acceleration.z = accel[2]
            
            # Set angular velocity
            imu_msg.angular_velocity.x = gyro[0]
            imu_msg.angular_velocity.y = gyro[1]
            imu_msg.angular_velocity.z = gyro[2]
            
            # Publish IMU data
            self.imu_publisher.publish(imu_msg)
            
            # Publish compass data
            compass_msg = Float32()
            compass_msg.data = compass
            self.compass_publisher.publish(compass_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing IMU data: {str(e)}')
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.socket.connect(('localhost', 12346))
                self.get_logger().info('Reconnected to IMU socket server')
            except Exception as conn_err:
                self.get_logger().error(f'Failed to reconnect: {str(conn_err)}')

    def __del__(self):
        if hasattr(self, 'socket'):
            self.socket.close()

def main(args=None):
    rclpy.init(args=args)
    node = ImuProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 