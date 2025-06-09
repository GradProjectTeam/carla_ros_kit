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
import math
from collections import deque
class CameraProcessorNode(Node):
    def __init__(self):
        super().__init__('camera_processor')
        
        # Fixed camera resolution
        self.IMAGE_WIDTH = 640
        self.IMAGE_HEIGHT = 480
        self.CHANNELS = 4  # RGBA format
        
        # Create publisher for camera images
        self.image_publisher = self.create_publisher(
            Image,
            '/carla/camera/rgb/image_raw',
            10)
            
        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Socket setup
        self.server_host = 'localhost'
        self.server_port = 12342  # Port for four_sensors camera
        
        # Create a timer for connection attempts
        self.socket = None
        self.connected = False
        self.reconnect_attempts = 0
        self.max_reconnect_attempts = 20  # Try 20 times before giving up
        
        # Create connection timer
        self.create_timer(1.0, self.connect_to_camera_server)
        
        # Memory for lane tracking
        self.last_valid_right_lane = None
        self.last_valid_left_lane = None
        self.last_lane_center = 320  # Default center
        self.lane_memory_frames = 10  # Remember lanes for this many frames
        
        # Queue to store recent lane positions for smoothing
        self.right_lane_history = deque(maxlen=5)
        self.left_lane_history = deque(maxlen=5)
        self.lane_center_history = deque(maxlen=5)
        self.lane_center_history.append(320)  # Initialize with center position
        
        # Define expected lane positions
        self.expected_left_x = 220   # Expected x-position of left lane at bottom of image
        self.expected_right_x = 420  # Expected x-position of right lane at bottom of image
        self.lane_width_tolerance = 100  # Tolerance for lane width variation
        
        # Lane detection parameters
        self.min_lane_points = 5     # Minimum points needed to fit a lane
        self.min_line_length = 20    # Minimum line length for Hough transform
        self.max_line_gap = 30       # Maximum line gap for Hough transform
        self.lane_detection_threshold = 20  # Hough transform threshold

    def connect_to_camera_server(self):
        """Attempt to connect to the camera server"""
        if self.connected:
            # Already connected, no need to reconnect
            return
            
        if self.reconnect_attempts >= self.max_reconnect_attempts:
            self.get_logger().error('Maximum reconnection attempts reached. Giving up.')
            return
            
        try:
            # Close any existing socket
            if self.socket:
                try:
                    self.socket.close()
                except Exception:
                    pass
                    
            # Create a new socket and attempt to connect
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5.0)  # 5 second timeout for connection
            
            self.get_logger().info(f'Connecting to camera server at {self.server_host}:{self.server_port} (attempt {self.reconnect_attempts+1}/{self.max_reconnect_attempts})')
            self.socket.connect((self.server_host, self.server_port))
            
            # Connection successful
            self.connected = True
            self.reconnect_attempts = 0
            self.get_logger().info(f'Successfully connected to camera server, expecting {self.IMAGE_WIDTH}x{self.IMAGE_HEIGHT} images')
            
            # Create timer for processing camera data once connected
            self.create_timer(0.1, self.timer_callback)  # 10Hz
            
        except ConnectionRefusedError:
            self.reconnect_attempts += 1
            self.get_logger().warn(f'Connection refused. Is the camera server running? Retrying in 1 second...')
            
        except Exception as e:
            self.reconnect_attempts += 1
            self.get_logger().error(f'Failed to connect to camera server: {e}. Retrying in 1 second...')

    def publish_camera_tf(self, timestamp):
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'camera_link'
        
        # Adjusted camera position for 640x480 view
        t.transform.translation.x = 1.5  # 1.5m forward
        t.transform.translation.y = 0.0  # centered
        t.transform.translation.z = 1.2  # 1.2m up
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = -0.13053
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 0.99144
        
        self.tf_broadcaster.sendTransform(t)

    def sumMatrix(self, pt1, pt2):
        A = np.array(pt1)
        B = np.array(pt2)
        ans = A + B
        return ans.tolist()
    

    def timer_callback(self):
        try:
            # Check if we're connected
            if not self.connected or not self.socket:
                self.get_logger().warn('Not connected to camera server. Skipping frame processing.')
                return
                
            # Receive image size
            try:
                size_data = self.socket.recv(4)
                if not size_data:
                    self.get_logger().warn('Connection lost (empty data). Will try to reconnect.')
                    self.connected = False
                    return
            except (socket.timeout, ConnectionResetError, BrokenPipeError) as e:
                self.get_logger().warn(f'Connection error: {e}. Will try to reconnect.')
                self.connected = False
                return
                
            image_size = struct.unpack('!I', size_data)[0]
            
            # Expected size check
            expected_size = self.IMAGE_WIDTH * self.IMAGE_HEIGHT * self.CHANNELS
            if image_size != expected_size:
                self.get_logger().warn(f'Received image size {image_size} differs from expected {expected_size}')
            
            # Receive image data
            image_data = b''
            try:
                while len(image_data) < image_size:
                    chunk = self.socket.recv(min(4096, image_size - len(image_data)))
                    if not chunk:
                        self.get_logger().warn('Connection lost while receiving image data. Will try to reconnect.')
                        self.connected = False
                        return
                    image_data += chunk
            except (socket.timeout, ConnectionResetError, BrokenPipeError) as e:
                self.get_logger().warn(f'Connection error while receiving image data: {e}. Will try to reconnect.')
                self.connected = False
                return
            
            # Convert to numpy array and reshape
            image_array = np.frombuffer(image_data, dtype=np.uint8)
            try:
                image = image_array.reshape((self.IMAGE_HEIGHT, self.IMAGE_WIDTH, self.CHANNELS))
            except ValueError as e:
                self.get_logger().error(f'Failed to reshape image: {str(e)}')
                return
            
            # Create timestamp
            timestamp = self.get_clock().now().to_msg()
            
            # Line detection code
            pt1_sum_ri = (0, 0)
            pt2_sum_ri = (0, 0)
            pt1_avg_ri = (0, 0)
            pt2_avg_ri = (0, 0)
            count_posi_num_ri = 0

            pt1_sum_le = (0, 0)
            pt2_sum_le = (0, 0)
            pt1_avg_le = (0, 0)
            pt2_avg_le = (0, 0)
            count_posi_num_le = 0

            # Convert the camera image to RGB format
            RGB_Camera_im = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

            # Resize the image to VGA resolution
            size_im = cv2.resize(RGB_Camera_im, dsize=(640, 480))

            # Define the region of interest (ROI)
            roi = size_im[240:480, 108:532]
            roi_im = cv2.resize(roi, (424, 240))
            # Apply Gaussian Blur to the ROI
            Blur_im = cv2.bilateralFilter(roi_im, d=-1, sigmaColor=5, sigmaSpace=5)

            # Detect edges using Canny edge detector
            edges = cv2.Canny(Blur_im, 50, 100)

            # Apply Hough Transformation to detect lines
            lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi / 180.0, threshold=25, minLineLength=10, maxLineGap=20)

            # Process detected lines
            valid_right_lane = False
            valid_left_lane = False
            
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    if x2 == x1:
                        continue  # Skip vertical lines to avoid division by zero
                    else:
                        a = x2 - x1
                    b = y2 - y1
                    radi = b / a
                    theta_atan = math.atan(radi) * 180.0 / math.pi

                    pt1_ri = (x1 + 108, y1 + 240)
                    pt2_ri = (x2 + 108, y2 + 240)
                    pt1_le = (x1 + 108, y1 + 240)
                    pt2_le = (x2 + 108, y2 + 240)

                    if 20.0 < theta_atan < 90.0:
                        count_posi_num_ri += 1
                        pt1_sum_ri = self.sumMatrix(pt1_ri, pt1_sum_ri)
                        pt2_sum_ri = self.sumMatrix(pt2_ri, pt2_sum_ri)

                    if -90.0 < theta_atan < -20.0:
                        count_posi_num_le += 1
                        pt1_sum_le = self.sumMatrix(pt1_le, pt1_sum_le)
                        pt2_sum_le = self.sumMatrix(pt2_le, pt2_sum_le)

            # Process right lane if valid points were found
            if count_posi_num_ri > 0:
                valid_right_lane = True
                pt1_avg_ri = (np.array(pt1_sum_ri) / count_posi_num_ri).astype(int)
                pt2_avg_ri = (np.array(pt2_sum_ri) / count_posi_num_ri).astype(int)
                x1_avg_ri, y1_avg_ri = pt1_avg_ri
                x2_avg_ri, y2_avg_ri = pt2_avg_ri
                
                # Avoid division by zero
                if x2_avg_ri != x1_avg_ri:
                    a_avg_ri = (y2_avg_ri - y1_avg_ri) / (x2_avg_ri - x1_avg_ri)
                    b_avg_ri = y2_avg_ri - (a_avg_ri * x2_avg_ri)
                    pt2_y2_fi_ri = 480
                    
                    # Check if slope is valid before doing division
                    if a_avg_ri != 0:
                        pt2_x2_fi_ri = int((pt2_y2_fi_ri - b_avg_ri) / a_avg_ri)
                        # Ensure point is within image bounds
                        pt2_x2_fi_ri = max(0, min(640, pt2_x2_fi_ri))
                        pt2_fi_ri = (pt2_x2_fi_ri, pt2_y2_fi_ri)
                        # Draw the right lane line
                        cv2.line(size_im, tuple(pt1_avg_ri), pt2_fi_ri, (0, 255, 0), 2)
                else:
                    valid_right_lane = False

            # Process left lane if valid points were found
            if count_posi_num_le > 0:
                valid_left_lane = True
                pt1_avg_le = (np.array(pt1_sum_le) / count_posi_num_le).astype(int)
                pt2_avg_le = (np.array(pt2_sum_le) / count_posi_num_le).astype(int)
                x1_avg_le, y1_avg_le = pt1_avg_le
                x2_avg_le, y2_avg_le = pt2_avg_le
                
                # Avoid division by zero
                if x2_avg_le != x1_avg_le:
                    a_avg_le = (y2_avg_le - y1_avg_le) / (x2_avg_le - x1_avg_le)
                    b_avg_le = y2_avg_le - (a_avg_le * x2_avg_le)
                    pt1_y1_fi_le = 480
                    
                    # Check if slope is valid before doing division
                    if a_avg_le != 0:
                        pt1_x1_fi_le = int((pt1_y1_fi_le - b_avg_le) / a_avg_le)
                        # Ensure point is within image bounds
                        pt1_x1_fi_le = max(0, min(640, pt1_x1_fi_le))
                        pt1_fi_le = (pt1_x1_fi_le, pt1_y1_fi_le)
                        # Draw the left lane line
                        cv2.line(size_im, tuple(pt2_avg_le), pt1_fi_le, (0, 255, 0), 2)
                else:
                    valid_left_lane = False

            # Draw center line
            cv2.line(size_im, (320, 480), (320, 360), (0, 228, 255), 1)

            # Only proceed with lane highlighting and steering if both lanes are valid
            if valid_right_lane and valid_left_lane:
                try:
                    # Highlight the possible lane area
                    FCP_img = np.zeros(shape=(480, 640, 3), dtype=np.uint8) + 0
                    FCP = np.array([pt2_avg_le, pt1_fi_le, pt2_fi_ri, pt1_avg_ri])
                    cv2.fillConvexPoly(FCP_img, FCP, color=(255, 242, 213))
                    alpha = 0.9
                    size_im = cv2.addWeighted(size_im, alpha, FCP_img, 1 - alpha, 0)

                    # Calculate the lane center and steering direction
                    lane_center_y_ri = 360
                    lane_center_y_le = 360
                    
                    # Avoid NaN values in calculations
                    if a_avg_ri != 0 and a_avg_le != 0:
                        lane_center_x_ri = int((lane_center_y_ri - b_avg_ri) / a_avg_ri)
                        lane_center_x_le = int((lane_center_y_le - b_avg_le) / a_avg_le)
                        
                        # Ensure values are within image bounds
                        lane_center_x_ri = max(0, min(640, lane_center_x_ri))
                        lane_center_x_le = max(0, min(640, lane_center_x_le))
                        
                        lane_center_x = ((lane_center_x_ri - lane_center_x_le) // 2) + lane_center_x_le

                        # Draw the lane center lines
                        cv2.line(size_im, (lane_center_x_le, lane_center_y_le - 10), (lane_center_x_le, lane_center_y_le + 10), (0, 228, 255), 1)
                        cv2.line(size_im, (lane_center_x_ri, lane_center_y_ri - 10), (lane_center_x_ri, lane_center_y_ri + 10), (0, 228, 255), 1)
                        cv2.line(size_im, (lane_center_x, lane_center_y_ri - 10), (lane_center_x, lane_center_y_le + 10), (0, 228, 255), 1)

                        # Display the steering direction
                        text_left = 'Turn Left'
                        text_right = 'Turn Right'
                        text_center = 'Center'
                        org = (320, 440)
                        font = cv2.FONT_HERSHEY_SIMPLEX

                        if 0 < lane_center_x <= 318:
                            cv2.putText(size_im, text_left, org, font, 0.7, (0, 0, 255), 2)
                        elif 318 < lane_center_x < 322:
                            cv2.putText(size_im, text_center, org, font, 0.7, (0, 0, 255), 2)
                        elif lane_center_x >= 322:
                            cv2.putText(size_im, text_right, org, font, 0.7, (0, 0, 255), 2)
                except Exception as e:
                    self.get_logger().warn(f'Error in lane calculation: {str(e)}')

            # Convert to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(size_im, encoding='rgb8')
            ros_image.header.stamp = timestamp
            ros_image.header.frame_id = 'camera_link'
            
            # Publish image and transform
            self.image_publisher.publish(ros_image)
            self.publish_camera_tf(timestamp)
            
        except Exception as e:
            self.get_logger().error(f'Error processing camera data: {str(e)}')

    def __del__(self):
        if hasattr(self, 'socket') and self.socket:
            try:
                self.socket.close()
            except Exception:
                pass

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