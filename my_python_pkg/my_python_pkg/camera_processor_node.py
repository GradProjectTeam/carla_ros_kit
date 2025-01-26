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
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect(('localhost', 12347))
        self.get_logger().info(f'Connected to camera socket server, expecting {self.IMAGE_WIDTH}x{self.IMAGE_HEIGHT} images')
        
        # Create timer for processing camera data
        self.create_timer(0.1, self.timer_callback)  # 10Hz

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
            # Receive image size
            size_data = self.socket.recv(4)
            if not size_data:
                return
            image_size = struct.unpack('!I', size_data)[0]
            
            # Expected size check
            expected_size = self.IMAGE_WIDTH * self.IMAGE_HEIGHT * self.CHANNELS
            if image_size != expected_size:
                self.get_logger().warn(f'Received image size {image_size} differs from expected {expected_size}')
            
            # Receive image data
            image_data = b''
            while len(image_data) < image_size:
                chunk = self.socket.recv(image_size - len(image_data))
                if not chunk:
                    break
                image_data += chunk
            
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
            count_posi_num_ri = 0

            pt1_sum_le = (0, 0)
            pt2_sum_le = (0, 0)
            pt1_avg_le = (0, 0)
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
            if lines is None:
                lines = [[0, 0, 0, 0]]
            else:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    if x2 == x1:
                        a = 1
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

                pt1_avg_ri = pt1_sum_ri // np.array(count_posi_num_ri)
                pt2_avg_ri = pt2_sum_ri // np.array(count_posi_num_ri)
                pt1_avg_le = pt1_sum_le // np.array(count_posi_num_le)
                pt2_avg_le = pt2_sum_le // np.array(count_posi_num_le)

                # Calculate the right lane line
                x1_avg_ri, y1_avg_ri = pt1_avg_ri
                x2_avg_ri, y2_avg_ri = pt2_avg_ri
                a_avg_ri = (y2_avg_ri - y1_avg_ri) / (x2_avg_ri - x1_avg_ri)
                b_avg_ri = y2_avg_ri - (a_avg_ri * x2_avg_ri)
                pt2_y2_fi_ri = 480
                pt2_x2_fi_ri = int((pt2_y2_fi_ri - b_avg_ri) // a_avg_ri) if a_avg_ri > 0 else 0
                pt2_fi_ri = (pt2_x2_fi_ri, pt2_y2_fi_ri)

                # Calculate the left lane line
                x1_avg_le, y1_avg_le = pt1_avg_le
                x2_avg_le, y2_avg_le = pt2_avg_le
                a_avg_le = (y2_avg_le - y1_avg_le) / (x2_avg_le - x1_avg_le)
                b_avg_le = y2_avg_le - (a_avg_le * x2_avg_le)
                pt1_y1_fi_le = 480
                pt1_x1_fi_le = int((pt1_y1_fi_le - b_avg_le) // a_avg_le) if a_avg_le < 0 else 0
                pt1_fi_le = (pt1_x1_fi_le, pt1_y1_fi_le)

                # Draw the lane lines on the image
                cv2.line(size_im, tuple(pt1_avg_ri), tuple(pt2_fi_ri), (0, 255, 0), 2)
                cv2.line(size_im, tuple(pt1_fi_le), tuple(pt2_avg_le), (0, 255, 0), 2)
                cv2.line(size_im, (320, 480), (320, 360), (0, 228, 255), 1)

                # Highlight the possible lane area
                FCP_img = np.zeros(shape=(480, 640, 3), dtype=np.uint8) + 0
                FCP = np.array([pt2_avg_le, pt1_fi_le, pt2_fi_ri, pt1_avg_ri])
                cv2.fillConvexPoly(FCP_img, FCP, color=(255, 242, 213))
                alpha = 0.9
                size_im = cv2.addWeighted(size_im, alpha, FCP_img, 1 - alpha, 0)

                # Calculate the lane center and steering direction
                lane_center_y_ri = 360
                lane_center_x_ri = int((lane_center_y_ri - b_avg_ri) // a_avg_ri) if a_avg_ri > 0 else 0
                lane_center_y_le = 360
                lane_center_x_le = int((lane_center_y_le - b_avg_le) // a_avg_le) if a_avg_le < 0 else 0
                lane_center_x = ((lane_center_x_ri - lane_center_x_le) // 2) + lane_center_x_le

                # Draw the lane center lines
                cv2.line(size_im, (lane_center_x_le, lane_center_y_le - 10), (lane_center_x_le, lane_center_y_le + 10), (0, 228, 255), 1)
                cv2.line(size_im, (lane_center_x_ri, lane_center_y_ri - 10), (lane_center_x_ri, lane_center_y_ri + 10), (0, 228, 255), 1)
                cv2.line(size_im, (lane_center_x, lane_center_y_ri - 10), (lane_center_x, lane_center_y_le + 10), (0, 228, 255), 1)

                # Display the steering direction
                text_left = 'Turn Left'
                text_right = 'Turn Right'
                text_center = 'Center'
                text_non = ''
                org = (320, 440)
                font = cv2.FONT_HERSHEY_SIMPLEX

                if 0 < lane_center_x <= 318:
                    cv2.putText(size_im, text_left, org, font, 0.7, (0, 0, 255), 2)
                elif 318 < lane_center_x < 322:
                    cv2.putText(size_im, text_center, org, font, 0.7, (0, 0, 255), 2)
                elif lane_center_x >= 322:
                    cv2.putText(size_im, text_right, org, font, 0.7, (0, 0, 255), 2)
                elif lane_center_x == 0:
                    cv2.putText(size_im, text_non, org, font, 0.7, (0, 0, 255), 2)

                global test_con
                test_con = 1

                # Reset variables
                count_posi_num_ri = 0
                pt1_sum_ri = (0, 0)
                pt2_sum_ri = (0, 0)
                pt1_avg_ri = (0, 0)
                pt2_avg_ri = (0, 0)
                count_posi_num_le = 0
                pt1_sum_le = (0, 0)
                pt2_sum_le = (0, 0)
                pt1_avg_le = (0, 0)
                pt2_avg_le = (0, 0)

            # Convert to ROS Image message

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