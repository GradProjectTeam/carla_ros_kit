#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import socket
import struct
import numpy as np
import cv2

class CameraProcessorNode(Node):
    def __init__(self):
        super().__init__('camera_processor')
        
        # Create publisher for camera images
        self.image_publisher = self.create_publisher(
            Image,
            '/carla/camera/rgb/image_raw',  # Changed topic name to match ROS conventions
            10)
            
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Socket setup
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect(('localhost', 12347))
        self.get_logger().info('Connected to camera socket server')
        
        # Create timer for processing camera data
        self.create_timer(0.1, self.timer_callback)  # 10Hz

    def publish_camera_tf(self, timestamp):
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'camera_link'
        
        # Camera position relative to vehicle (matches CARLA setup)
        t.transform.translation.x = 2.0  # 2m forward
        t.transform.translation.y = 0.0  # centered
        t.transform.translation.z = 1.5  # 1.5m up
        
        # Camera rotation (15 degrees down)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = -0.13053  # sin(-15°/2)
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 0.99144   # cos(-15°/2)
        
        self.tf_broadcaster.sendTransform(t)

    def timer_callback(self):
        try:
            # Receive image size
            size_data = self.socket.recv(4)
            if not size_data:
                return
            image_size = struct.unpack('!I', size_data)[0]
            
            # Receive image data
            image_data = b''
            while len(image_data) < image_size:
                chunk = self.socket.recv(image_size - len(image_data))
                if not chunk:
                    break
                image_data += chunk
            
            # Convert to numpy array
            image_array = np.frombuffer(image_data, dtype=np.uint8)
            image = image_array.reshape((600, 800, 4))  # RGBA format
            
            # Create timestamp
            timestamp = self.get_clock().now().to_msg()
            
            # Convert to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(image, encoding='rgba8')
            ros_image.header.stamp = timestamp
            ros_image.header.frame_id = 'camera_link'
            
            # Publish image and transform
            self.image_publisher.publish(ros_image)
            self.publish_camera_tf(timestamp)
            
        except Exception as e:
            self.get_logger().error(f'Error processing camera data: {str(e)}')

    def __del__(self):
        if hasattr(self, 'socket'):
            self.socket.close()

def main(args=None):
    rclpy.init(args=args)
    node = CameraProcessorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 