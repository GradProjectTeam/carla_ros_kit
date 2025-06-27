#! /usr/bin/python3.7
"""
Gui_Control.py - CARLA Simulator Vehicle Control and Sensor Management System

This script provides a comprehensive interface for controlling vehicles in the CARLA simulator
and managing various sensors (LiDAR, RADAR, IMU, Camera) attached to them. It handles:
1. Vehicle spawning, control, and traffic management
2. Sensor data collection, processing, and transmission via TCP
3. GUI interface for real-time control and visualization
4. Waypoint generation for navigation assistance

Authors: Shishtawy & Hendy
Project by: TechZ
Version: 6.0 - Added traffic vehicles with autopilot in front of the main vehicle
"""

import sys
import glob
import os
import socket  # For TCP communication
import pickle
import numpy as np
import time
import json
import struct  # For binary data packing
import math
import random
import pygame  # For GUI interface
from pygame.locals import *
import threading  # For parallel processing
from queue import Queue
import traceback

# Version 6: Added traffic vehicles in front of the main vehicle
# The traffic vehicles have no sensors and use CARLA's autopilot system
# The main vehicle remains user-controlled with sensors

class CARLASetup:
    def __init__(self):
        print("Starting CARLA setup...")
        self.carla_path = '/home/shishtawy/Carla/CARLA_0.9.12/PythonAPI/carla/dist'  # Path to CARLA Python API
        # self.carla_path = '/home/mostafa/ROS2andCarla/CARLA/CARLA_0.9.8/PythonAPI/carla/dist'
        self.setup_carla()
        
    def setup_carla(self):
        print("Looking for CARLA at:", self.carla_path)
        carla_eggs = glob.glob('{0}/carla-*{1}.{2}-{3}.egg'.format(  # Find CARLA egg file for current Python version
            self.carla_path,
            sys.version_info.major,
            sys.version_info.minor,
            "win-amd64" if os.name == "nt" else "linux-x86_64"
        ))
        sys.path.append(carla_eggs[0])  # Add CARLA egg to Python path
        print("Found CARLA egg:", carla_eggs[0])
        
        global carla
        import carla # type: ignore  # Import CARLA module
        print("CARLA imported successfully")

class SensorManager:
    def __init__(self, vehicle, world):
        print("\n=== Initializing Sensor Manager ===")
        self.vehicle = vehicle  # Vehicle to attach sensors to
        self.world = world  # CARLA world
        self.actor_list = []  # List to keep track of all actors (sensors)
        self.map = world.get_map()  # Get the map for waypoint generation
        
        # Waypoint configuration
        self.waypoint_distance = 2.0  # Distance between waypoints in meters
        self.waypoints = []  # Store generated waypoints
        
        # Data queues with thread-safe implementation
        self.lidar_queue = Queue(maxsize=1)  # Queue for LiDAR data
        self.radar_queue = Queue(maxsize=1)  # Queue for RADAR data
        self.imu_queue = Queue(maxsize=1)  # Queue for IMU data
        self.camera_queue = Queue(maxsize=1)  # Queue for camera data
        self.waypoint_queue = Queue(maxsize=1)  # Queue for waypoint data
        
        # TCP setup with different ports
        # self.host_ip = '192.168.1.2'
        self.host_ip = '127.0.0.1'  # Localhost for TCP communication
        self.lidar_port = 12349  # Port for LiDAR data
        self.radar_port = 12347  # Port for RADAR data
        self.imu_port = 12341  # Port for IMU data
        self.camera_port = 12342  # Port for camera data
        self.waypoint_port = 12343  # Port for waypoint data

        # Sensor flags - set these to control which sensors are active
        self.lidar_flag = True  # Enable/disable LiDAR
        self.radar_flag = True  # Enable/disable RADAR
        self.imu_flag = True  # Enable/disable IMU
        self.camera_flag = False  # Enable/disable camera
        self.waypoint_flag = True  # Enable/disable waypoint publishing
        
        # Waypoint configuration
        self.waypoint_distance = 2.0  # Distance between waypoints in meters
        self.waypoint_lifetime = 0.5  # Lifetime of visualization in seconds
        self.waypoints = []  # Store generated waypoints
        
        print("Initial sensor flags - LIDAR: {}, RADAR: {}, IMU: {}, CAMERA: {}, WAYPOINT: {}".format(
            self.lidar_flag, self.radar_flag, self.imu_flag, self.camera_flag, self.waypoint_flag))
        
        # Thread control
        self.running = True  # Flag to control thread execution
        self.lidar_thread = None  # Thread for LiDAR processing
        self.radar_thread = None  # Thread for RADAR processing
        self.imu_thread = None  # Thread for IMU processing
        self.camera_thread = None  # Thread for camera processing
        self.waypoint_thread = None  # Thread for waypoint processing
        self.waypoint_generation_thread = None  # Thread for generating waypoints
        
        # Setup separate sockets for each sensor
        self.setup_tcp_sockets()  # Initialize TCP connections
        self.setup_sensors()  # Setup all sensors
        
        # Start processing threads
        self.start_processing_threads()  # Start threads for data processing
        print("=== Sensor Manager Initialization Complete ===\n")
        
    def setup_tcp_sockets(self):
        # LiDAR socket
        if self.lidar_flag:
            try:
                self.lidar_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Create TCP socket
                self.lidar_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # Disable Nagle's algorithm
                print("LiDAR TCP configured for {0}:{1}".format(self.host_ip, self.lidar_port))
                
                # Try to connect but don't fail if connection fails
                try:
                    self.lidar_socket.connect((self.host_ip, self.lidar_port))  # Connect to LiDAR server
                    print("LiDAR TCP connected")
                except (ConnectionRefusedError, socket.error) as e:
                    print("LiDAR TCP connection failed: {}. Will continue with local data only.".format(e))
                    # Don't disable the sensor just because connection failed
            except Exception as e:
                print("Error setting up LiDAR socket: {}".format(e))
                # Keep the flag enabled - we'll just use local data
        
        # Radar socket
        if self.radar_flag:
            try:
                self.radar_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Create TCP socket
                self.radar_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # Disable Nagle's algorithm
                print("Radar TCP configured for {0}:{1}".format(self.host_ip, self.radar_port))
                
                # Try to connect but don't fail if connection fails
                try:
                    self.radar_socket.connect((self.host_ip, self.radar_port))  # Connect to RADAR server
                    print("Radar TCP connected")
                except (ConnectionRefusedError, socket.error) as e:
                    print("Radar TCP connection failed: {}. Will continue with local data only.".format(e))
                    # Don't disable the sensor just because connection failed
            except Exception as e:
                print("Error setting up Radar socket: {}".format(e))
                # Keep the flag enabled - we'll just use local data
        
        # IMU socket
        if self.imu_flag:
            try:
                self.imu_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Create TCP socket
                self.imu_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # Disable Nagle's algorithm
                print("IMU TCP configured for {0}:{1}".format(self.host_ip, self.imu_port))
                
                # Try to connect but don't fail if connection fails
                try:
                    self.imu_socket.connect((self.host_ip, self.imu_port))  # Connect to IMU server
                    print("IMU TCP connected")
                except (ConnectionRefusedError, socket.error) as e:
                    print("IMU TCP connection failed: {}. Will continue with local data only.".format(e))
                    # Don't disable the sensor just because connection failed
            except Exception as e:
                print("Error setting up IMU socket: {}".format(e))
                # Keep the flag enabled - we'll just use local data
        
        # Camera socket
        if self.camera_flag:
            try:
                self.camera_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Create TCP socket
                self.camera_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Allow address reuse
                self.camera_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # Disable Nagle's algorithm
                print("Camera TCP configured for {0}:{1}".format(self.host_ip, self.camera_port))
                
                # Setup camera server to listen for connections
                try:
                    self.camera_socket.bind(('0.0.0.0', self.camera_port))  # Bind to all interfaces
                    self.camera_socket.listen(1)  # Listen for connections
                    self.camera_socket.settimeout(0.5)  # Non-blocking accept
                    print("Camera TCP server listening on port {0}".format(self.camera_port))
                    self.camera_client = None  # No client connected yet
                except Exception as e:
                    print("Error setting up camera server: {0}".format(e))
                    # Don't disable the sensor just because server setup failed
            except Exception as e:
                print("Error setting up Camera socket: {}".format(e))
                # Keep the flag enabled if possible
        
        # Waypoint socket
        if self.waypoint_flag:
            try:
                self.waypoint_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Create TCP socket
                self.waypoint_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # Disable Nagle's algorithm
                print("Waypoint TCP configured for {0}:{1}".format(self.host_ip, self.waypoint_port))
                
                # Try to connect but don't fail if connection fails
                try:
                    self.waypoint_socket.connect((self.host_ip, self.waypoint_port))  # Connect to waypoint server
                    print("Waypoint TCP connected")
                except (ConnectionRefusedError, socket.error) as e:
                    print("Waypoint TCP connection failed: {}. Will continue with local data only.".format(e))
                    # Don't disable the sensor just because connection failed
            except Exception as e:
                print("Error setting up Waypoint socket: {}".format(e))
                # Keep the flag enabled - we'll just use local data
        
    def start_processing_threads(self):
        if self.lidar_flag:
            self.lidar_thread = threading.Thread(target=self.process_lidar_queue)  # Create LiDAR processing thread
        if self.radar_flag:
            self.radar_thread = threading.Thread(target=self.process_radar_queue)  # Create RADAR processing thread
        if self.imu_flag:
            self.imu_thread = threading.Thread(target=self.process_imu_queue)  # Create IMU processing thread
        if self.camera_flag:
            self.camera_thread = threading.Thread(target=self.process_camera_queue)  # Create camera processing thread
        if self.waypoint_flag:
            self.waypoint_thread = threading.Thread(target=self.process_waypoint_queue)  # Create waypoint processing thread
        
        if self.lidar_flag:
            self.lidar_thread.daemon = True  # Set as daemon thread to exit when main thread exits
        if self.radar_flag:
            self.radar_thread.daemon = True  # Set as daemon thread
        if self.imu_flag:
            self.imu_thread.daemon = True  # Set as daemon thread
        if self.camera_flag:
            self.camera_thread.daemon = True  # Set as daemon thread
        if self.waypoint_flag:
            self.waypoint_thread.daemon = True  # Set as daemon thread
        
        if self.lidar_flag:
            self.lidar_thread.start()  # Start LiDAR processing thread
        if self.radar_flag:
            self.radar_thread.start()  # Start RADAR processing thread
        if self.imu_flag:
            self.imu_thread.start()  # Start IMU processing thread
        if self.camera_flag:
            self.camera_thread.start()  # Start camera processing thread
        if self.waypoint_flag:
            self.waypoint_thread.start()  # Start waypoint processing thread
        
    def process_lidar_queue(self):
        point_counter = 0  # Counter for debug output
        while self.running:
            try:
                if not self.lidar_queue.empty():
                    point_cloud = self.lidar_queue.get()  # Get LiDAR data from queue
                    for point in point_cloud:
                        if not self.running:
                            break
                        
                        # Print debug info every 1000 points
                        point_counter += 1
                        if point_counter % 1000 == 0:
                            # Handle different LiDAR point formats
                            if hasattr(point, 'x'):
                                # Direct access for older CARLA versions
                                print("Sending LIDAR point #{0}: ({1:.2f}, {2:.2f}, {3:.2f})".format(
                                    point_counter, point.x, point.y, point.z))
                            elif hasattr(point, 'point'):
                                # Access through point attribute for newer CARLA versions
                                print("Sending LIDAR point #{0}: ({1:.2f}, {2:.2f}, {3:.2f})".format(
                                    point_counter, point.point.x, point.point.y, point.point.z))
                            else:
                                # Try to access as a tuple/list (some versions may use this format)
                                print("Sending LIDAR point #{0}: ({1:.2f}, {2:.2f}, {3:.2f})".format(
                                    point_counter, point[0], point[1], point[2]))
                        
                        # Convert to network byte order (big-endian)
                        # Pack as float32 values in network byte order
                        try:
                            # Handle different LiDAR point formats
                            if hasattr(point, 'x'):
                                # Direct access for older CARLA versions
                                point_data = struct.pack('!fff', point.x, point.y, point.z)  # Pack as 3 floats
                            elif hasattr(point, 'point'):
                                # Access through point attribute for newer CARLA versions
                                point_data = struct.pack('!fff', point.point.x, point.point.y, point.point.z)  # Pack as 3 floats
                            else:
                                # Try to access as a tuple/list (some versions may use this format)
                                point_data = struct.pack('!fff', point[0], point[1], point[2])  # Pack as 3 floats
                                
                            if self.lidar_flag:
                                self.lidar_socket.send(point_data)  # Send point data over TCP
                        except (AttributeError, IndexError, TypeError) as e:
                            print(f"Error processing LiDAR point: {e}")
                            print(f"Point type: {type(point)}, Point data: {point}")
                            # Skip this point and continue with the next one
                            continue
                        except socket.error as e:
                            print("LiDAR socket error: {0}".format(e))
                            break
                else:
                    time.sleep(0.001)  # Small sleep to prevent CPU hogging
            except Exception as e:
                print("Error in LiDAR processing thread: {0}".format(e))
                traceback.print_exc()
    
    def process_radar_queue(self):
        point_counter = 0  # Counter for radar points
        while self.running:
            try:
                if not self.radar_queue.empty():
                    radar_data = self.radar_queue.get()  # Get RADAR data from queue
                    points = np.array([[det.altitude, det.azimuth, det.depth, det.velocity] 
                                    for det in radar_data], dtype=np.float32)  # Convert to numpy array
                    
                    # Only log every 50 points batch
                    if point_counter % 50 == 0:
                        print("[RADAR DEBUG] Processing batch of {} radar points".format(len(points)))
                    
                    # Batch processing - send all points at once
                    if len(points) > 0 and self.radar_flag:
                        try:
                            # First send the number of points in the batch
                            num_points = struct.pack('!I', len(points))  # Pack as unsigned int
                            self.radar_socket.sendall(num_points)  # Send number of points
                            
                            # Then send all points data in one go
                            batch_data = bytearray()
                            for point in points:
                                # Pack each point
                                point_data = struct.pack('!ffff', point[0], point[1], point[2], point[3])  # Pack as 4 floats
                                batch_data.extend(point_data)  # Add to batch
                            
                            # Send the entire batch at once
                            self.radar_socket.sendall(batch_data)  # Send all points in one batch
                            point_counter += len(points)
                            
                            if point_counter % 50 == 0:
                                print("[RADAR DEBUG] Sent batch of {} points, total: {}".format(len(points), point_counter))
                        except socket.error as e:
                            print("[RADAR ERROR] Socket error in batch send: {}".format(e))
                else:
                    time.sleep(0.001)  # Small sleep to prevent CPU hogging
            except Exception as e:
                print("[RADAR ERROR] Processing error: {}".format(e))
                import traceback
                traceback.print_exc()
    
    def process_imu_queue(self):
        while self.running:
            try:
                if not self.imu_queue.empty():
                    imu_data = self.imu_queue.get()  # Get IMU data from queue
                    # Pack IMU data: acceleration (3 floats) + gyroscope (3 floats) + compass (1 float)
                    data = struct.pack('fffffff', 
                                     imu_data.accelerometer.x, imu_data.accelerometer.y, imu_data.accelerometer.z,
                                     imu_data.gyroscope.x, imu_data.gyroscope.y, imu_data.gyroscope.z,
                                     imu_data.compass)  # Pack as 7 floats
                    
                    try:
                        if self.imu_flag:
                            
                            self.imu_socket.sendall(data)  # Send IMU data over TCP
                    except socket.error as e:
                        print("IMU socket error: {0}".format(e))
                else:
                    time.sleep(0.001)  # Small sleep to prevent CPU hogging
            except Exception as e:
                print("Error in IMU processing thread: {0}".format(e))
    
    def process_camera_queue(self):
        frame_counter = 0  # Counter for camera frames
        while self.running:
            try:
                # Accept connections if no client is connected
                if not hasattr(self, 'camera_client') or self.camera_client is None:
                    try:
                        self.camera_client, addr = self.camera_socket.accept()  # Accept new connection
                        print("[CAMERA] Connected to client at {0}".format(addr))
                    except socket.timeout:
                        # No connection yet, that's fine
                        time.sleep(0.1)
                        continue
                    except Exception as e:
                        print("[CAMERA] Connection error: {0}".format(e))
                        time.sleep(0.5)
                        continue

                if not self.camera_queue.empty():
                    camera_data = self.camera_queue.get()  # Get camera data from queue
                    
                    # Get raw image data
                    raw_data = camera_data.raw_data  # Get raw image bytes
                    
                    # Log frame info periodically
                    frame_counter += 1
                    if frame_counter % 10 == 0:  # Log every 10 frames
                        print("[CAMERA DEBUG] Sending camera frame #{0}, size: {1} bytes".format(frame_counter, len(raw_data)))
                    
                    try:
                        if self.camera_flag and self.camera_client:
                            # Send image size first
                            size_header = struct.pack('!I', len(raw_data))  # Pack as unsigned int
                            self.camera_client.sendall(size_header)  # Send size header
                            # Send image data
                            self.camera_client.sendall(raw_data)  # Send raw image data
                    except (BrokenPipeError, ConnectionResetError, socket.error) as e:
                        print("[CAMERA] Client disconnected: {0}".format(e))
                        self.camera_client = None
                else:
                    time.sleep(0.001)  # Small sleep to prevent CPU hogging
            except Exception as e:
                print("Error in Camera processing thread: {0}".format(e))
                import traceback
                traceback.print_exc()
                time.sleep(0.5)  # Sleep longer after an error
    def lidar_callback(self, point_cloud):
        try:
            if self.lidar_queue.full():
                try:
                    self.lidar_queue.get(block=False)  # Remove old data if queue is full
                except Queue.Empty:
                    pass
            self.lidar_queue.put(point_cloud, block=False)  # Add new data to queue
        except Exception as e:
            print("Error in LiDAR callback: {0}".format(e))
        
    def radar_callback(self, radar_data):
        try:
            if self.radar_queue.full():
                try:
                    self.radar_queue.get(block=False)  # Remove old data if queue is full
                except Queue.Empty:
                    pass
            self.radar_queue.put(radar_data, block=False)  # Add new data to queue
        except Exception as e:
            print("Error in Radar callback: {0}".format(e))
        
    def imu_callback(self, imu_data):
        try:
            if self.imu_queue.full():
                try:
                    self.imu_queue.get(block=False)  # Remove old data if queue is full
                except Queue.Empty:
                    pass
            self.imu_queue.put(imu_data, block=False)  # Add new data to queue
        except Exception as e:
            print("Error in IMU callback: {0}".format(e))
        
    def camera_callback(self, image):
        try:
            if self.camera_queue.full():
                try:
                    self.camera_queue.get(block=False)  # Remove old data if queue is full
                except Queue.Empty:
                    pass
            self.camera_queue.put(image, block=False)  # Add new data to queue
        except Exception as e:
            print("Error in Camera callback: {0}".format(e))
        
    def generate_waypoints_ahead(self, distance=100.0, lane_change=False):
        """Generate waypoints ahead of the vehicle in coordinates relative to the vehicle"""
        if not self.vehicle or not self.waypoint_flag:
            return []
            
        # Get current vehicle location and transform
        vehicle_transform = self.vehicle.get_transform()  # Get vehicle transform
        vehicle_location = vehicle_transform.location  # Get vehicle location
        
        # Find the closest waypoint to the vehicle
        waypoint = self.map.get_waypoint(vehicle_location)  # Get closest waypoint
        
        # Generate waypoints ahead
        absolute_waypoints = [waypoint]  # List of waypoints in world coordinates
        relative_waypoints = []  # List of waypoints in vehicle coordinates
        distance_covered = 0.0
        
        while distance_covered < distance:
            # Get next waypoints
            next_waypoints = waypoint.next(self.waypoint_distance)  # Get next waypoints
            
            if not next_waypoints:
                break
                
            waypoint = next_waypoints[0]  # Take first waypoint
            distance_covered += self.waypoint_distance
            
            # Optional lane change logic
            if lane_change and distance_covered > distance / 2 and random.random() > 0.8:
                if waypoint.get_right_lane():
                    waypoint = waypoint.get_right_lane()  # Change to right lane
                elif waypoint.get_left_lane():
                    waypoint = waypoint.get_left_lane()  # Change to left lane
            
            absolute_waypoints.append(waypoint)
        
        # Convert absolute waypoints to relative waypoints
        for wp in absolute_waypoints:
            # Get the waypoint's world location
            wp_location = wp.transform.location
            
            # Calculate relative coordinates
            # First, get the vector from vehicle to waypoint in world coordinates
            relative_vector = carla.Location(
                x=wp_location.x - vehicle_location.x,
                y=wp_location.y - vehicle_location.y,
                z=wp_location.z - vehicle_location.z
            )
            
            # Convert to vehicle's local coordinate system
            # We need to rotate the vector based on vehicle's rotation
            # Forward is x, right is y in vehicle's local coordinates
            yaw_rad = math.radians(vehicle_transform.rotation.yaw)  # Convert yaw to radians
            cos_yaw = math.cos(yaw_rad)  # Cosine of yaw
            sin_yaw = math.sin(yaw_rad)  # Sine of yaw
            
            # Apply rotation transformation
            local_x = cos_yaw * relative_vector.x + sin_yaw * relative_vector.y  # Rotate x coordinate
            local_y = -sin_yaw * relative_vector.x + cos_yaw * relative_vector.y  # Rotate y coordinate
            local_z = relative_vector.z  # Z coordinate remains unchanged
            
            # Create a waypoint object with relative coordinates
            relative_wp = {
                'x': local_x,
                'y': local_y,
                'z': local_z,
                'road_id': wp.road_id,  # Store road ID for navigation
                'lane_id': wp.lane_id,  # Store lane ID for lane tracking
                'lane_type': int(wp.lane_type)  # Store lane type for lane classification
            }
            
            relative_waypoints.append(relative_wp)
        
        print("Generated {} waypoints over {:.1f} meters (relative to vehicle)".format(len(relative_waypoints), distance_covered))
        return relative_waypoints
        
    def process_waypoint_queue(self):
        """Process waypoints from queue and update map"""
        waypoint_counter = 0  # Counter for waypoints
        while self.running:
            try:
                # Generate new waypoints if needed
                if self.waypoint_flag and (not self.waypoints or waypoint_counter % 10 == 0):
                    self.waypoints = self.generate_waypoints_ahead(distance=100.0)  # Generate 100m of waypoints ahead
                    
                    # Put waypoints in queue, replacing old data if queue is full
                    if self.waypoints:
                        if self.waypoint_queue.full():
                            try:
                                self.waypoint_queue.get_nowait()  # Remove old waypoints
                            except Queue.Empty:
                                pass
                        self.waypoint_queue.put(self.waypoints)  # Add new waypoints to queue
                
                if not self.waypoint_queue.empty():
                    waypoints = self.waypoint_queue.get()  # Get waypoints from queue
                    
                    # Only log every 10 waypoint batches
                    waypoint_counter += 1
                    if waypoint_counter % 10 == 0:
                        print("[WAYPOINT DEBUG] Processing batch of {} waypoints".format(len(waypoints)))
                    
                    if len(waypoints) > 0 and self.waypoint_flag:
                        try:
                            # First send the number of waypoints in the batch
                            num_waypoints = struct.pack('!I', len(waypoints))  # Pack as unsigned int
                            self.waypoint_socket.sendall(num_waypoints)  # Send number of waypoints
                            
                            # Then send all waypoint data in one go
                            batch_data = bytearray()
                            for waypoint in waypoints:
                                # Pack each waypoint with the new relative format
                                # Format: x, y, z, road_id, lane_id, lane_type (as int)
                                # Now using the dictionary format from generate_waypoints_ahead
                                point_data = struct.pack('!fffiii', 
                                                       waypoint['x'],  # Relative x
                                                       waypoint['y'],  # Relative y
                                                       waypoint['z'],  # Relative z
                                                       waypoint['road_id'],  # Road ID
                                                       waypoint['lane_id'],  # Lane ID
                                                       waypoint['lane_type'])  # Lane type
                                batch_data.extend(point_data)  # Add to batch
                            
                            # Send the entire batch at once
                            self.waypoint_socket.sendall(batch_data)  # Send all waypoints in one batch
                            
                            if waypoint_counter % 10 == 0:
                                print("[WAYPOINT DEBUG] Sent batch of {} relative waypoints".format(len(waypoints)))
                        except socket.error as e:
                            print("[WAYPOINT ERROR] Socket error in batch send: {}".format(e))
                else:
                    time.sleep(0.1)  # Longer sleep for waypoints as they update less frequently
            except Exception as e:
                print("[WAYPOINT ERROR] Processing error: {}".format(e))
                import traceback
                traceback.print_exc()

    def generate_waypoints_periodically(self):
        """Generate waypoints periodically in a separate thread"""
        while self.running and self.waypoint_flag:
            try:
                # Generate waypoints with the new relative format
                waypoints = self.generate_waypoints_ahead(distance=100.0)  # Generate 100m of waypoints
                
                # Put waypoints in queue
                if waypoints:
                    if self.waypoint_queue.full():
                        try:
                            self.waypoint_queue.get_nowait()  # Remove old waypoints
                        except Queue.Empty:
                            pass
                    self.waypoint_queue.put(waypoints)  # Add new waypoints to queue
                    
                    # Log the first few waypoints to verify they're in relative format
                    if len(waypoints) > 0:
                        first_wp = waypoints[0]
                        print("First relative waypoint: x={:.2f}, y={:.2f}, z={:.2f}".format(
                            first_wp['x'], first_wp['y'], first_wp['z']))
                    
                # Wait before generating new waypoints
                time.sleep(1.0)  # Update every second
                
            except Exception as e:
                print("Error generating waypoints: {}".format(e))
                time.sleep(2.0)  # Wait longer after an error

    def setup_sensors(self):
        try:
            # Only setup sensors that are enabled by their flags
            if self.lidar_flag:
                self.setup_lidar()  # Setup LiDAR sensor
            if self.radar_flag:
                self.setup_radar()  # Setup RADAR sensor
            if self.imu_flag:
                self.setup_imu()  # Setup IMU sensor
            if self.camera_flag:
                self.setup_camera()  # Setup camera sensor
            if self.waypoint_flag:
                self.setup_waypoint()  # Setup waypoint generation
            print("Sensors setup complete")
        except Exception as e:
            print("Error in setup_sensors: {0}".format(e))
            raise

    def setup_lidar(self):
        try:
            lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast')  # Get LiDAR blueprint
            lidar_bp.set_attribute('channels', '32')  # 32 vertical channels
            lidar_bp.set_attribute('points_per_second', '100000')  # 100k points per second
            lidar_bp.set_attribute('rotation_frequency', '20')  # 20 Hz rotation
            lidar_bp.set_attribute('range', '70.0')  # 70 meter range
            lidar_bp.set_attribute('upper_fov', '10.0')  # 10 degrees up
            lidar_bp.set_attribute('lower_fov', '-10.0')  # 10 degrees down
            
            # Mount on top of the car, slightly forward
            lidar_transform = carla.Transform(
                carla.Location(x=1.5, z=2.0),  # x: forward, z: up
                carla.Rotation(yaw=270)  # Default rotation (0,0,0) will inherit car's rotation
            )
            
            self.lidar = self.world.spawn_actor(lidar_bp, lidar_transform, attach_to=self.vehicle)  # Spawn LiDAR
            self.actor_list.append(self.lidar)  # Add to actor list for cleanup
            self.lidar.listen(self.lidar_callback)  # Register callback function
            print("LiDAR sensor added at position: x=1.5m, z=2.0m")
            
        except Exception as e:
            print("Error in LiDAR setup: {0}".format(str(e)))
            raise
        
    def setup_radar(self):
        try:
            radar_bp = self.world.get_blueprint_library().find('sensor.other.radar')  # Get RADAR blueprint
            radar_bp.set_attribute('horizontal_fov', '60.0')  # 60 degree horizontal FOV
            radar_bp.set_attribute('vertical_fov', '-60.0')   # 60 degree vertical FOV
            radar_bp.set_attribute('points_per_second', '2000')  # 2000 points per second
            radar_bp.set_attribute('range', '100.0')  # 100 meter range
            
            # Mount next to the LiDAR with a slight horizontal offset
            radar_transform = carla.Transform(
                # Position radar at the same x (forward) position as LiDAR but offset to the right (y=0.5)
                carla.Location(x=1.5, y=0.5, z=2.0),  # x: forward, y: right, z: up
                carla.Rotation()  # Default rotation (0,0,0) will inherit car's rotation
            )
            
            self.radar = self.world.spawn_actor(radar_bp, radar_transform, attach_to=self.vehicle)  # Spawn RADAR
            self.actor_list.append(self.radar)  # Add to actor list for cleanup
            self.radar.listen(self.radar_callback)  # Register callback function
            print("Radar sensor added at position: x=1.5m, y=0.5m, z=2.0m")
            
        except Exception as e:
            print("Error in Radar setup: {0}".format(str(e)))
            raise

    def setup_imu(self):
        try:
            imu_bp = self.world.get_blueprint_library().find('sensor.other.imu')  # Get IMU blueprint
            
            # Set IMU parameters
            imu_bp.set_attribute('sensor_tick', '0.05')  # 20Hz update rate
            
            # Mount at the center of the car
            imu_transform = carla.Transform(
                carla.Location(x=0.0, z=0.0),  # Center of the vehicle
                carla.Rotation()  # Default rotation
            )
            
            self.imu = self.world.spawn_actor(imu_bp, imu_transform, attach_to=self.vehicle)  # Spawn IMU
            self.actor_list.append(self.imu)  # Add to actor list for cleanup
            self.imu.listen(self.imu_callback)  # Register callback function
            print("IMU sensor added at position: x=0.0m, z=0.0m")
            
        except Exception as e:
            print("Error in IMU setup: {0}".format(str(e)))
            raise

    def setup_camera(self):
        try:
            # Create camera blueprint
            camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')  # Get RGB camera blueprint
            
            # Set camera attributes for better image quality
            camera_bp.set_attribute('image_size_x', '640')  # 640 pixels width
            camera_bp.set_attribute('image_size_y', '480')  # 480 pixels height
            camera_bp.set_attribute('fov', '90')  # 90 degree field of view
            camera_bp.set_attribute('sensor_tick', '0.1')  # 10 FPS
            
            # Mount on front of the car, slightly elevated and tilted down for lane detection
            camera_transform = carla.Transform(
                carla.Location(x=2.0, z=1.5),  # Front of car, slightly elevated
                carla.Rotation(pitch=-15.0)    # Tilted down slightly
            )
            
            self.camera = self.world.spawn_actor(camera_bp, camera_transform, attach_to=self.vehicle)  # Spawn camera
            self.actor_list.append(self.camera)  # Add to actor list for cleanup
            self.camera.listen(self.camera_callback)  # Register callback function
            print("Camera sensor added at position: x=2.0m, z=1.5m, pitch=-15°")
            
        except Exception as e:
            print("Error in Camera setup: {0}".format(e))
            raise

    def setup_waypoint(self):
        try:
            print("Setting up waypoint generation...")
            
            # No need for a physical sensor, we'll generate waypoints using the map API
            
            # Start a thread for periodic waypoint generation if waypoint flag is enabled
            if self.waypoint_flag:
                # Generate initial waypoints
                try:
                    initial_waypoints = self.generate_waypoints_ahead(distance=100.0)  # Generate initial waypoints
                    if initial_waypoints:
                        self.waypoints = initial_waypoints  # Store waypoints
                        if self.waypoint_queue.full():
                            try:
                                self.waypoint_queue.get_nowait()  # Remove old waypoints if queue is full
                            except Queue.Empty:
                                pass
                        self.waypoint_queue.put(initial_waypoints)  # Add waypoints to queue
                        print("Initial waypoints generated: {} (relative to vehicle)".format(len(initial_waypoints)))
                        
                        # Log the first waypoint to verify it's in relative format
                        if len(initial_waypoints) > 0:
                            first_wp = initial_waypoints[0]
                            print("First relative waypoint: x={:.2f}, y={:.2f}, z={:.2f}".format(
                                first_wp['x'], first_wp['y'], first_wp['z']))
                        
                        # Start a thread for periodic waypoint generation
                        self.waypoint_generation_thread = threading.Thread(target=self.generate_waypoints_periodically)  # Create thread
                        self.waypoint_generation_thread.daemon = True  # Set as daemon thread
                        self.waypoint_generation_thread.start()  # Start thread
                        print("Waypoint generation thread started")
                except Exception as e:
                    print("Error generating initial waypoints: {}".format(e))
            
            print("Waypoint generation setup complete")
            
        except Exception as e:
            print("Error in Waypoint setup: {0}".format(e))
            raise

    def toggle_waypoint(self):
        """Toggle the waypoint sensor on/off"""
        self.waypoint_flag = not self.waypoint_flag  # Toggle flag
        print("Waypoint sensor toggled: {}".format(self.waypoint_flag))
        
        # If turned on, try to set up the socket if it doesn't exist
        if self.waypoint_flag and not hasattr(self, 'waypoint_socket'):
            try:
                self.waypoint_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Create TCP socket
                self.waypoint_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # Disable Nagle's algorithm
                try:
                    self.waypoint_socket.connect((self.host_ip, self.waypoint_port))  # Connect to waypoint server
                    print("Waypoint TCP connected")
                except (ConnectionRefusedError, socket.error) as e:
                    print("Waypoint TCP connection failed: {}. Will continue with local data only.".format(e))
            except Exception as e:
                print("Error setting up waypoint socket: {}".format(e))
        
        # If turned on, generate new waypoints
        if self.waypoint_flag:
            try:
                # Generate initial waypoints with the new relative format
                initial_waypoints = self.generate_waypoints_ahead(distance=100.0)  # Generate waypoints
                if initial_waypoints:
                    self.waypoints = initial_waypoints  # Store waypoints
                    if self.waypoint_queue.full():
                        try:
                            self.waypoint_queue.get_nowait()  # Remove old waypoints if queue is full
                        except Queue.Empty:
                            pass
                    self.waypoint_queue.put(initial_waypoints)  # Add waypoints to queue
                    print("Generated {} relative waypoints when toggling on".format(len(initial_waypoints)))
                    
                    # Log the first waypoint to verify it's in relative format
                    if len(initial_waypoints) > 0:
                        first_wp = initial_waypoints[0]
                        print("First relative waypoint: x={:.2f}, y={:.2f}, z={:.2f}".format(
                            first_wp['x'], first_wp['y'], first_wp['z']))
            except Exception as e:
                print("Error generating waypoints when toggling: {}".format(e))
        
        # If turned off and socket exists, close it
        elif not self.waypoint_flag and hasattr(self, 'waypoint_socket'):
            try:
                self.waypoint_socket.shutdown(socket.SHUT_RDWR)  # Shutdown socket
                self.waypoint_socket.close()  # Close socket
                delattr(self, 'waypoint_socket')  # Remove socket attribute
                print("Waypoint socket closed")
            except Exception as e:
                print("Error closing waypoint socket: {}".format(e))

    def toggle_lidar(self):
        """Toggle the LiDAR sensor on/off"""
        self.lidar_flag = not self.lidar_flag  # Toggle flag
        print("LiDAR sensor toggled: {}".format(self.lidar_flag))
        
        # If turned on, try to set up the socket and sensor if they don't exist
        if self.lidar_flag:
            # Set up socket if it doesn't exist
            if not hasattr(self, 'lidar_socket'):
                try:
                    self.lidar_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Create TCP socket
                    self.lidar_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # Disable Nagle's algorithm
                    try:
                        self.lidar_socket.connect((self.host_ip, self.lidar_port))  # Connect to LiDAR server
                        print("LiDAR TCP connected")
                    except (ConnectionRefusedError, socket.error) as e:
                        print("LiDAR TCP connection failed: {}. Will continue with local data only.".format(e))
                except Exception as e:
                    print("Error setting up LiDAR socket: {}".format(e))
            
            # Set up LiDAR sensor if it doesn't exist
            if not hasattr(self, 'lidar') or not self.lidar.is_alive:
                try:
                    self.setup_lidar()  # Setup LiDAR sensor
                    print("LiDAR sensor created")
                except Exception as e:
                    print("Error creating LiDAR sensor: {}".format(e))
            
            # Start processing thread if it doesn't exist or isn't alive
            if not self.lidar_thread or not self.lidar_thread.is_alive():
                self.lidar_thread = threading.Thread(target=self.process_lidar_queue)  # Create thread
                self.lidar_thread.daemon = True  # Set as daemon thread
                self.lidar_thread.start()  # Start thread
                print("LiDAR processing thread started")
        
        # If turned off, stop the sensor and close the socket
        else:
            # Destroy LiDAR sensor if it exists
            if hasattr(self, 'lidar') and self.lidar.is_alive:
                try:
                    self.lidar.stop()  # Stop listening
                    self.actor_list.remove(self.lidar)  # Remove from actor list
                    self.lidar.destroy()  # Destroy sensor
                    delattr(self, 'lidar')  # Remove attribute
                    print("LiDAR sensor destroyed")
                except Exception as e:
                    print("Error destroying LiDAR sensor: {}".format(e))
            
            # Close socket if it exists
            if hasattr(self, 'lidar_socket'):
                try:
                    self.lidar_socket.shutdown(socket.SHUT_RDWR)  # Shutdown socket
                    self.lidar_socket.close()  # Close socket
                    delattr(self, 'lidar_socket')  # Remove attribute
                    print("LiDAR socket closed")
                except Exception as e:
                    print("Error closing LiDAR socket: {}".format(e))

    def toggle_radar(self):
        """Toggle the RADAR sensor on/off"""
        self.radar_flag = not self.radar_flag  # Toggle flag
        print("RADAR sensor toggled: {}".format(self.radar_flag))
        
        # If turned on, try to set up the socket and sensor if they don't exist
        if self.radar_flag:
            # Set up socket if it doesn't exist
            if not hasattr(self, 'radar_socket'):
                try:
                    self.radar_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Create TCP socket
                    self.radar_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # Disable Nagle's algorithm
                    try:
                        self.radar_socket.connect((self.host_ip, self.radar_port))  # Connect to RADAR server
                        print("RADAR TCP connected")
                    except (ConnectionRefusedError, socket.error) as e:
                        print("RADAR TCP connection failed: {}. Will continue with local data only.".format(e))
                except Exception as e:
                    print("Error setting up RADAR socket: {}".format(e))
            
            # Set up RADAR sensor if it doesn't exist
            if not hasattr(self, 'radar') or not self.radar.is_alive:
                try:
                    self.setup_radar()  # Setup RADAR sensor
                    print("RADAR sensor created")
                except Exception as e:
                    print("Error creating RADAR sensor: {}".format(e))
            
            # Start processing thread if it doesn't exist or isn't alive
            if not self.radar_thread or not self.radar_thread.is_alive():
                self.radar_thread = threading.Thread(target=self.process_radar_queue)  # Create thread
                self.radar_thread.daemon = True  # Set as daemon thread
                self.radar_thread.start()  # Start thread
                print("RADAR processing thread started")
        
        # If turned off, stop the sensor and close the socket
        else:
            # Destroy RADAR sensor if it exists
            if hasattr(self, 'radar') and self.radar.is_alive:
                try:
                    self.radar.stop()  # Stop listening
                    self.actor_list.remove(self.radar)  # Remove from actor list
                    self.radar.destroy()  # Destroy sensor
                    delattr(self, 'radar')  # Remove attribute
                    print("RADAR sensor destroyed")
                except Exception as e:
                    print("Error destroying RADAR sensor: {}".format(e))
            
            # Close socket if it exists
            if hasattr(self, 'radar_socket'):
                try:
                    self.radar_socket.shutdown(socket.SHUT_RDWR)  # Shutdown socket
                    self.radar_socket.close()  # Close socket
                    delattr(self, 'radar_socket')  # Remove attribute
                    print("RADAR socket closed")
                except Exception as e:
                    print("Error closing RADAR socket: {}".format(e))

    def toggle_imu(self):
        """Toggle the IMU sensor on/off"""
        self.imu_flag = not self.imu_flag  # Toggle flag
        print("IMU sensor toggled: {}".format(self.imu_flag))
        
        # If turned on, try to set up the socket and sensor if they don't exist
        if self.imu_flag:
            # Set up socket if it doesn't exist
            if not hasattr(self, 'imu_socket'):
                try:
                    self.imu_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Create TCP socket
                    self.imu_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # Disable Nagle's algorithm
                    try:
                        self.imu_socket.connect((self.host_ip, self.imu_port))  # Connect to IMU server
                        print("IMU TCP connected")
                    except (ConnectionRefusedError, socket.error) as e:
                        print("IMU TCP connection failed: {}. Will continue with local data only.".format(e))
                except Exception as e:
                    print("Error setting up IMU socket: {}".format(e))
            
            # Set up IMU sensor if it doesn't exist
            if not hasattr(self, 'imu') or not self.imu.is_alive:
                try:
                    self.setup_imu()  # Setup IMU sensor
                    print("IMU sensor created")
                except Exception as e:
                    print("Error creating IMU sensor: {}".format(e))
            
            # Start processing thread if it doesn't exist or isn't alive
            if not self.imu_thread or not self.imu_thread.is_alive():
                self.imu_thread = threading.Thread(target=self.process_imu_queue)  # Create thread
                self.imu_thread.daemon = True  # Set as daemon thread
                self.imu_thread.start()  # Start thread
                print("IMU processing thread started")
        
        # If turned off, stop the sensor and close the socket
        else:
            # Destroy IMU sensor if it exists
            if hasattr(self, 'imu') and self.imu.is_alive:
                try:
                    self.imu.stop()  # Stop listening
                    self.actor_list.remove(self.imu)  # Remove from actor list
                    self.imu.destroy()  # Destroy sensor
                    delattr(self, 'imu')  # Remove attribute
                    print("IMU sensor destroyed")
                except Exception as e:
                    print("Error destroying IMU sensor: {}".format(e))
            
            # Close socket if it exists
            if hasattr(self, 'imu_socket'):
                try:
                    self.imu_socket.shutdown(socket.SHUT_RDWR)  # Shutdown socket
                    self.imu_socket.close()  # Close socket
                    delattr(self, 'imu_socket')  # Remove attribute
                    print("IMU socket closed")
                except Exception as e:
                    print("Error closing IMU socket: {}".format(e))

    def toggle_camera(self):
        """Toggle the camera sensor on/off"""
        self.camera_flag = not self.camera_flag  # Toggle flag
        print("Camera sensor toggled: {}".format(self.camera_flag))
        
        # If turned on, try to set up the socket and sensor if they don't exist
        if self.camera_flag:
            # Set up socket if it doesn't exist
            if not hasattr(self, 'camera_socket'):
                try:
                    self.camera_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Create TCP socket
                    self.camera_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Allow address reuse
                    self.camera_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # Disable Nagle's algorithm
                    try:
                        self.camera_socket.bind(('0.0.0.0', self.camera_port))  # Bind to all interfaces
                        self.camera_socket.listen(1)  # Listen for connections
                        self.camera_socket.settimeout(0.5)  # Non-blocking accept
                        print("Camera TCP server listening on port {}".format(self.camera_port))
                        self.camera_client = None  # No client connected yet
                    except Exception as e:
                        print("Error setting up camera server: {}".format(e))
                except Exception as e:
                    print("Error setting up camera socket: {}".format(e))
            
            # Set up camera sensor if it doesn't exist
            if not hasattr(self, 'camera') or not self.camera.is_alive:
                try:
                    self.setup_camera()  # Setup camera sensor
                    print("Camera sensor created")
                except Exception as e:
                    print("Error creating camera sensor: {}".format(e))
            
            # Start processing thread if it doesn't exist or isn't alive
            if not self.camera_thread or not self.camera_thread.is_alive():
                self.camera_thread = threading.Thread(target=self.process_camera_queue)  # Create thread
                self.camera_thread.daemon = True  # Set as daemon thread
                self.camera_thread.start()  # Start thread
                print("Camera processing thread started")
        
        # If turned off, stop the sensor and close the socket
        else:
            # Destroy camera sensor if it exists
            if hasattr(self, 'camera') and self.camera.is_alive:
                try:
                    self.camera.stop()  # Stop listening
                    self.actor_list.remove(self.camera)  # Remove from actor list
                    self.camera.destroy()  # Destroy sensor
                    delattr(self, 'camera')  # Remove attribute
                    print("Camera sensor destroyed")
                except Exception as e:
                    print("Error destroying camera sensor: {}".format(e))
            
            # Close socket if it exists
            if hasattr(self, 'camera_socket'):
                try:
                    if hasattr(self, 'camera_client') and self.camera_client:
                        self.camera_client.close()  # Close client connection
                    self.camera_socket.shutdown(socket.SHUT_RDWR)  # Shutdown socket
                    self.camera_socket.close()  # Close socket
                    delattr(self, 'camera_socket')  # Remove attribute
                    print("Camera socket closed")
                except Exception as e:
                    print("Error closing camera socket: {}".format(e))
    
    def cleanup(self):
        print("Cleaning up sensors and sockets...")
        self.running = False  # Signal threads to stop
        
        # Wait for processing threads to finish
        if self.lidar_thread and self.lidar_thread.is_alive():
            self.lidar_thread.join(timeout=1.0)  # Wait for LiDAR thread to finish
        if self.radar_thread and self.radar_thread.is_alive():
            self.radar_thread.join(timeout=1.0)  # Wait for RADAR thread to finish
        if self.imu_thread and self.imu_thread.is_alive():
            self.imu_thread.join(timeout=1.0)  # Wait for IMU thread to finish
        if self.camera_thread and self.camera_thread.is_alive():
            self.camera_thread.join(timeout=1.0)  # Wait for camera thread to finish
        if self.waypoint_thread and self.waypoint_thread.is_alive():
            self.waypoint_thread.join(timeout=1.0)  # Wait for waypoint thread to finish
        
        # Clean up actors
        for actor in self.actor_list:
            if actor is not None and actor.is_alive:
                actor.destroy()  # Destroy all sensor actors
        
        # Close sockets
        if hasattr(self, 'lidar_socket'):
            try:
                if self.lidar_flag:
                    self.lidar_socket.shutdown(socket.SHUT_RDWR)  # Shutdown LiDAR socket
                    self.lidar_socket.close()  # Close LiDAR socket
            except Exception as e:
                print("Error closing LiDAR socket: {0}".format(e))
            
        if hasattr(self, 'radar_socket'):
            try:
                if self.radar_flag:
                    self.radar_socket.shutdown(socket.SHUT_RDWR)  # Shutdown RADAR socket
                    self.radar_socket.close()  # Close RADAR socket
            except Exception as e:
                print("Error closing Radar socket: {0}".format(e))
                
        if hasattr(self, 'imu_socket'):
            try:
                if self.imu_flag:
                    self.imu_socket.shutdown(socket.SHUT_RDWR)  # Shutdown IMU socket
                    self.imu_socket.close()  # Close IMU socket
            except Exception as e:
                print("Error closing IMU socket: {0}".format(e))
        
        if hasattr(self, 'camera_socket'):
            try:
                if self.camera_flag:
                    if hasattr(self, 'camera_client') and self.camera_client:
                        self.camera_client.close()  # Close camera client connection
                    self.camera_socket.shutdown(socket.SHUT_RDWR)  # Shutdown camera socket
                    self.camera_socket.close()  # Close camera socket
            except Exception as e:
                print("Error closing Camera socket: {0}".format(e))
                
        if hasattr(self, 'waypoint_socket'):
            try:
                if self.waypoint_flag:
                    self.waypoint_socket.shutdown(socket.SHUT_RDWR)  # Shutdown waypoint socket
                    self.waypoint_socket.close()  # Close waypoint socket
            except Exception as e:
                print("Error closing Waypoint socket: {0}".format(e))
                
        print("Sensor cleanup complete")

class TrafficManager:
    def __init__(self, world, ego_vehicle, client):
        self.world = world  # CARLA world
        self.client = client  # CARLA client
        self.ego_vehicle = ego_vehicle  # Main player vehicle
        self.traffic_vehicles = []  # List of spawned traffic vehicles
        self.roaming_mode = False  # Flag to track if vehicles are roaming
        self.actor_list = []  # List to track all actors for cleanup
        self.port = 8000  # Port for traffic manager
        
        # Get traffic manager from client
        try:
            print("Initializing CARLA traffic manager...")

            try:
                self.tm = self.client.get_trafficmanager(self.port)  # Get traffic manager
            except AttributeError:
                # Fallback for older CARLA versions that might not have get_trafficmanager
                self.tm = None
                print("Traffic manager not available in this CARLA version")
                
            # Only set these if tm is available
            if self.tm:
                # Check which methods are available in this version
                try:
                    self.tm.set_global_distance_to_leading_vehicle(2.5)  # Set distance between vehicles
                except AttributeError:
                    print("set_global_distance_to_leading_vehicle not available")
                    
                try:
                    self.tm.set_synchronous_mode(True)  # Set synchronous mode
                except AttributeError:
                    print("set_synchronous_mode not available")
                    
                try:
                    self.tm.global_percentage_speed_difference(30.0)  # Set speed difference percentage
                except AttributeError:
                    print("global_percentage_speed_difference not available")
                
            print("Traffic manager initialized successfully")
        except Exception as e:
            print("Error initializing traffic manager: {}".format(e))
            self.tm = None
            
    def spawn_traffic_vehicles(self, num_vehicles=3):
        """Spawn traffic vehicles in front of the ego vehicle"""
        try:
            print("Spawning {} traffic vehicles...".format(num_vehicles))
            
            # Get spawn points
            spawn_points = self.world.get_map().get_spawn_points()  # Get all spawn points
            if not spawn_points:
                print("No spawn points available")
                return
                
            # Get ego vehicle transform
            ego_transform = self.ego_vehicle.get_transform()  # Get ego vehicle transform
            ego_location = ego_transform.location  # Get ego vehicle location
            ego_forward_vector = ego_transform.get_forward_vector()  # Get ego vehicle forward vector
            
            # Get vehicle blueprints
            blueprint_library = self.world.get_blueprint_library()  # Get blueprint library
            
            
            suv_blueprints = []
            for bp in blueprint_library.filter('vehicle.*'):
                
                if int(bp.get_attribute('number_of_wheels').as_int()) >= 4:
                    # Include only SUVs, trucks, vans, and other large vehicles
                    if any(tag in bp.id.lower() for tag in ['suv', 'offroad', 'truck', 'van', 'jeep', 'rubicon', 'patrol', 'cybertruck']):
                        suv_blueprints.append(bp)
            
            # If no SUVs were found, fallback to 4+ wheel vehicles
            if not suv_blueprints:
                print("No SUV blueprints found, falling back to 4+ wheel vehicles")
                suv_blueprints = [bp for bp in blueprint_library.filter('vehicle.*') 
                                if int(bp.get_attribute('number_of_wheels').as_int()) >= 4]
            
            # Spawn vehicles in front of the ego vehicle
            for i in range(num_vehicles):
                # Calculate spawn distance (increasing for each vehicle)
                spawn_distance = 20 + (i * 15)  # 20m, 35m, 50m, etc.
                
                # Calculate spawn location in front of ego vehicle
                spawn_location = carla.Location(
                    x=ego_location.x + ego_forward_vector.x * spawn_distance,  # X position ahead of vehicle
                    y=ego_location.y + ego_forward_vector.y * spawn_distance,  # Y position ahead of vehicle
                    z=ego_location.z + 0.5  # Slightly above ground
                )
                
                # Find closest spawn point to desired location
                closest_spawn_point = None
                min_distance = float('inf')
                for spawn_point in spawn_points:
                    dist = spawn_location.distance(spawn_point.location)  # Calculate distance
                    if dist < min_distance:
                        min_distance = dist
                        closest_spawn_point = spawn_point  # Find closest spawn point
                
                if not closest_spawn_point:
                    print("Could not find a valid spawn point for vehicle {}".format(i+1))
                    continue
                
                # Choose a random blueprint from SUVs
                vehicle_bp = random.choice(suv_blueprints)  # Select random SUV blueprint
                
                # Try to spawn the vehicle
                try:
                    vehicle = self.world.spawn_actor(vehicle_bp, closest_spawn_point)  # Spawn vehicle at spawn point
                    if vehicle:
                        self.traffic_vehicles.append(vehicle)  # Add to traffic vehicles list
                        print("Spawned {} at {}".format(vehicle.type_id, closest_spawn_point.location))
                        
                        # Set up basic autopilot - this should work in all CARLA versions
                        vehicle.set_autopilot(True)  # Enable autopilot for traffic vehicle
                        
                        # Configure TM settings only if available
                        if self.tm:
                            try:
                                
                                self.tm.vehicle_percentage_speed_difference(vehicle, random.uniform(-20, 10))  # Set random speed
                            except AttributeError:
                                # Handle case where the method isn't available
                                pass
                    else:
                        print("Failed to spawn vehicle {}".format(i+1))
                except Exception as e:
                    print("Failed to spawn vehicle {}".format(i+1))
                    print("Error spawning vehicle {}: {}".format(i+1, e))
            
            print("Successfully spawned {} traffic vehicles".format(len(self.traffic_vehicles)))
            
        except Exception as e:
            print("Error in spawn_traffic_vehicles: {}".format(e))
    
    def spawn_random_traffic_vehicles(self, num_vehicles=3):
        try:
            print("Spawning {} random traffic vehicles...".format(num_vehicles))
            
            # Clean up existing traffic vehicles first
            for vehicle in self.traffic_vehicles:
                if vehicle and vehicle.is_alive:
                    vehicle.destroy()  # Destroy existing vehicles
            self.traffic_vehicles = []  # Clear list
            
            # Get all available vehicle blueprints
            blueprints = self.world.get_blueprint_library().filter('vehicle.*')  # Get all vehicle blueprints
            
            # Filter for only SUVs and large vehicles
            suv_blueprints = []
            for bp in blueprints:
                # Filter out bicycles and motorcycles
                if int(bp.get_attribute('number_of_wheels').as_int()) >= 4:
                    # Include only SUVs, trucks, vans, and other large vehicles
                    if any(tag in bp.id.lower() for tag in ['suv', 'offroad', 'truck', 'van', 'jeep', 'rubicon', 'patrol', 'cybertruck']):
                        suv_blueprints.append(bp)
            
            # If no SUVs were found, fallback to 4+ wheel vehicles
            if not suv_blueprints:
                print("No SUV blueprints found, falling back to 4+ wheel vehicles")
                suv_blueprints = [bp for bp in blueprints if int(bp.get_attribute('number_of_wheels').as_int()) >= 4]
            
            # Get all spawn points
            spawn_points = self.world.get_map().get_spawn_points()  # Get all spawn points
            
            if not spawn_points:
                print("No spawn points available")
                return
                
            # Shuffle spawn points for randomness
            random.shuffle(spawn_points)  # Randomize spawn points
            
            # Try to spawn vehicles
            spawned_count = 0
            for i in range(num_vehicles):
                if i >= len(spawn_points):
                    break  # No more spawn points available
                
                # Select a random blueprint from SUVs
                blueprint = random.choice(suv_blueprints)  # Select random SUV blueprint
                
                # Try to spawn vehicle
                try:
                    # Set random color
                    if blueprint.has_attribute('color'):
                        color = random.choice(blueprint.get_attribute('color').recommended_values)  # Choose random color
                        blueprint.set_attribute('color', color)  # Set vehicle color
                    
                    # Set as ego to make it important
                    if blueprint.has_attribute('role_name'):
                        blueprint.set_attribute('role_name', 'traffic')  # Set role name
                    
                    # Get spawn point
                    spawn_point = spawn_points[i]  # Get spawn point
                    
                    # Spawn the vehicle
                    vehicle = self.world.spawn_actor(blueprint, spawn_point)  # Spawn vehicle
                    
                    if vehicle:
                        # Add to traffic vehicles list
                        self.traffic_vehicles.append(vehicle)  # Add to traffic vehicles list
                        self.actor_list.append(vehicle)  # Add to actor list for cleanup
                        spawned_count += 1
                        
                        # Set basic autopilot - this should work in all CARLA versions
                        vehicle.set_autopilot(True)  # Enable autopilot
                        
                        # Only attempt to use TM methods if TM is available
                        if self.tm:
                            try:
                                
                                self.tm.vehicle_percentage_speed_difference(vehicle, random.uniform(-20, 10))  # Set random speed
                            except AttributeError:
                                # Handle case where the method isn't available
                                pass
                        
                        print("Spawned {} at {}".format(vehicle.type_id, spawn_point.location))
                
                except Exception as e:
                    print("Failed to spawn vehicle: {}".format(e))
            
            print("Successfully spawned {} random traffic vehicles".format(spawned_count))
            
        except Exception as e:
            print("Error in spawn_random_traffic_vehicles: {}".format(e))
    
    def set_vehicles_to_roam(self):
        try:
            if not hasattr(self, 'traffic_vehicles') or len(self.traffic_vehicles) == 0:
                print("No traffic vehicles to control")
                return False
            
            # Toggle roaming mode state
            if not hasattr(self, 'roaming_mode'):
                self.roaming_mode = True
            else:
                self.roaming_mode = not self.roaming_mode  # Toggle roaming mode
            
            for vehicle in self.traffic_vehicles:
                if self.roaming_mode:
                    # Enable autopilot / roaming - basic autopilot should work in all versions
                    vehicle.set_autopilot(True)  # Enable autopilot
                    
                    # Try to configure TM settings only if available
                    if self.tm:
                        try:
                            
                            self.tm.vehicle_percentage_speed_difference(vehicle, random.uniform(-20, 10))  # Set random speed
                        except AttributeError:
                            # Handle case where method isn't available
                            pass
                else:
                    # Disable autopilot / roaming
                    vehicle.set_autopilot(False)  # Disable autopilot
                    
            return self.roaming_mode
        
        except Exception as e:
            print("Error setting vehicles to roam: {}".format(e))
            return False
            
    def remove_last_vehicle(self):
        if hasattr(self, 'traffic_vehicles') and len(self.traffic_vehicles) > 0:
            vehicle = self.traffic_vehicles.pop()  # Remove last vehicle from list
            if vehicle in self.actor_list:
                self.actor_list.remove(vehicle)  # Remove from actor list
            vehicle.destroy()  # Destroy vehicle
            print("Removed {}".format(vehicle.type_id))
            return True
        return False

    def cleanup(self):
        print("Cleaning up traffic...")
        
        if hasattr(self, 'traffic_vehicles'):
            for vehicle in self.traffic_vehicles:
                if vehicle and vehicle.is_alive:
                    vehicle.destroy()  # Destroy all traffic vehicles
        self.traffic_vehicles = []  # Clear traffic vehicles list

class CarlaControl:
    def __init__(self):
        print("\n=== Starting CarlaControl Initialization ===")
        try:
            print("Setting up Pygame...")
            self.setup_pygame()  # Initialize Pygame for GUI
            
            print("Setting up CARLA client...")
            # Control variables
            self.throttle = 0.0  # Throttle control value
            self.brake = 0.0  # Brake control value
            self.steer = 0.0  # Steering control value
            self.reverse = False  # Reverse gear state
            self.vehicle = None  # Player vehicle
            self.running = True  # Main loop control flag
            self.traffic_cars = []  # List to store traffic vehicles
            
            # Throttle control variables
            self.throttle_increment = 0.05  # Throttle increment per frame
            self.throttle_decrement = 0.03  # Throttle decrement per frame when released
            self.accelerating_forward = False  # Flag for W key pressed
            self.accelerating_reverse = False  # Flag for S key pressed
            
            # Initialize sensor manager and traffic manager references
            self.sensor_manager = None  # Will hold sensor manager instance
            self.traffic_manager = None  # Will hold traffic manager instance
            
            # TCP control server settings
            self.control_server_port = 12344  # Port for control server
            self.control_server_socket = None  # Server socket
            self.control_client_socket = None  # Client socket
            self.control_server_running = False  # Server running flag
            
            # Set up CARLA client and spawn vehicle
            self.setup_carla_client()  # Connect to CARLA
            
            # Set up TCP control server
            self.setup_control_server()  # Setup control server
            
            print("CarlaControl initialization complete")
            
        except Exception as e:
            print("ERROR in CarlaControl initialization: {0}".format(str(e)))
            self.cleanup()
            raise
            
    def setup_control_server(self):
        """Set up TCP server to receive control commands from external clients"""
        try:
            print("Setting up TCP control server on port {}...".format(self.control_server_port))
            self.control_server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Create TCP socket
            self.control_server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Allow address reuse
            self.control_server_socket.bind(('0.0.0.0', self.control_server_port))  # Bind to all interfaces
            self.control_server_socket.listen(1)  # Listen for connections
            self.control_server_running = True  # Set running flag
            
            # Start a thread to accept connections
            self.control_server_thread = threading.Thread(target=self.accept_control_connections)  # Create thread
            self.control_server_thread.daemon = True  # Set as daemon thread
            self.control_server_thread.start()  # Start thread
            print("TCP control server started successfully")
            
        except Exception as e:
            print("Error setting up TCP control server: {}".format(e))
            self.control_server_running = False
            
    def accept_control_connections(self):
        """Accept client connections for vehicle control"""
        while self.control_server_running:
            try:
                print("Waiting for control client connection...")
                # Set timeout to allow checking if server is still running
                self.control_server_socket.settimeout(1.0)  # Set timeout
                
                # Accept connection
                client_socket, client_address = self.control_server_socket.accept()  # Accept client connection
                print("Control client connected from {}".format(client_address))
                
                # Close previous client socket if exists
                if self.control_client_socket:
                    self.control_client_socket.close()
                    
                self.control_client_socket = client_socket
                
                # Start a thread to handle this client
                client_thread = threading.Thread(target=self.handle_client, args=(client_socket,))
                client_thread.daemon = True
                client_thread.start()
                
            except socket.timeout:
                # This is expected - just retry
                pass
            except Exception as e:
                if self.control_server_running:  # Only print error if we're still supposed to be running
                    print("Error accepting control connection: {}".format(e))
                    traceback.print_exc()
                time.sleep(1)  # Sleep to avoid tight loop
                
    def handle_client(self, client_socket):
        """Handle client connection and parse incoming control commands"""
        buffer = ""
        while self.control_server_running:
            try:
                data = client_socket.recv(1024)
                if not data:
                    print("Control client disconnected")
                    break
                    
                # Decode and add to buffer
                buffer += data.decode('utf-8')
                
                # Process complete commands (might receive multiple or partial commands)
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    self.parse_control_command(line.strip())
                    
            except Exception as e:
                print("Error handling control client: {}".format(e))
                break
                
        # Clean up
        try:
            client_socket.close()
        except:
            pass
            
    def parse_control_command(self, command):
        """Parse the command string in format 'throttle,steering,brake,reverse'"""
        try:
            parts = command.split(',')
            if len(parts) >= 3:  # Accept at least 3 parts
                self.throttle = float(parts[0])
                self.steer = float(parts[1])
                self.brake = float(parts[2])
                
                # Check if reverse flag is included
                if len(parts) >= 4:
                    self.reverse = bool(int(parts[3]))
                
                # Clamp values to valid ranges
                self.throttle = max(0.0, min(1.0, self.throttle))
                self.steer = max(-1.0, min(1.0, self.steer))
                self.brake = max(0.0, min(1.0, self.brake))
                
                print(f"Received control: Throttle={self.throttle:.2f}, Steering={self.steer:.2f}, Brake={self.brake:.2f}, Reverse={self.reverse}")
            else:
                print(f"Invalid command format: {command}")
                
        except Exception as e:
            print(f"Error parsing control command '{command}': {e}")
            traceback.print_exc()

    def cleanup(self):
        print("\n=== Starting Cleanup ===")
        try:
            print("Setting running flag to False...")
            self.running = False
            
            # Stop TCP control server
            print("Stopping TCP control server...")
            self.control_server_running = False
            
            if hasattr(self, 'control_client_socket') and self.control_client_socket:
                try:
                    self.control_client_socket.close()
                except:
                    pass
                    
            if hasattr(self, 'control_server_socket') and self.control_server_socket:
                try:
                    self.control_server_socket.close()
                except:
                    pass
            
            if hasattr(self, 'sensor_manager') and self.sensor_manager:
                print("Cleaning up sensor manager...")
                self.sensor_manager.cleanup()
                
            if hasattr(self, 'traffic_manager') and self.traffic_manager:
                print("Cleaning up traffic vehicles...")
                self.traffic_manager.cleanup()
            
            if hasattr(self, 'vehicle') and self.vehicle:
                print("Destroying vehicle...")
                self.vehicle.destroy()
            
            if hasattr(self, 'world'):
                print("Resetting world settings...")
                settings = self.world.get_settings()
                settings.synchronous_mode = False
                self.world.apply_settings(settings)
            
            print("Quitting Pygame...")
            pygame.quit()
            print("Cleanup complete")
            
        except Exception as e:
            print("ERROR during cleanup: {0}".format(str(e)))
            
    def setup_pygame(self):
        """Initialize pygame and create the control panel window"""
        try:
            pygame.init()
            pygame.font.init()
            
            # Set up display
            self.display_width = 800
            self.display_height = 600
            self.display = pygame.display.set_mode((self.display_width, self.display_height))
            pygame.display.set_caption("CARLA Control Panel")
            
            # Set up fonts
            self.font = pygame.font.SysFont('Arial', 20)
            self.small_font = pygame.font.SysFont('Arial', 16)
            self.large_font = pygame.font.SysFont('Arial', 24)
            self.title_font = pygame.font.SysFont('Arial', 30, True)  # Bold font for titles
            
            # Define colors
            self.BLACK = (0, 0, 0)
            self.WHITE = (255, 255, 255)
            self.GRAY = (100, 100, 100)
            self.LIGHT_GRAY = (200, 200, 200)
            self.RED = (255, 0, 0)
            self.GREEN = (0, 255, 0)
            self.BLUE = (0, 0, 255)
            self.YELLOW = (255, 255, 0)
            
            # UI state
            self.show_help = True  # Show help text by default
            
            print("Pygame setup complete")
            
        except Exception as e:
            print("ERROR in pygame setup: {0}".format(str(e)))
            raise
        
    def setup_fonts(self):
        try:
            print("Loading fonts...")
            self.font = pygame.font.Font(None, 36)
            self.small_font = pygame.font.Font(None, 24)
            print("Fonts loaded successfully")
        except Exception as e:
            print("ERROR in font setup: {0}".format(str(e)))
            raise
            
    def setup_carla_client(self):
        try:
            print("Connecting to CARLA...")
            self.client = carla.Client('localhost', 2000)
            self.client.set_timeout(10.0)
            print("Getting CARLA world...")
            self.world = self.client.get_world()
            print("Connected to CARLA world")
            print("Spawning vehicle...")
            self.spawn_vehicle()
        except Exception as e:
            print("ERROR in CARLA client setup: {0}".format(str(e)))
            raise
        
    def spawn_vehicle(self):
        try:
            print("Cleaning up existing vehicles...")
            vehicle_list = self.world.get_actors().filter('vehicle.*')
            for vehicle in vehicle_list:
                vehicle.destroy()
                
            print("Getting spawn points...")
            spawn_points = self.world.get_map().get_spawn_points()
            if not spawn_points:
                raise ValueError("No spawn points found!")
                
            spawn_point = spawn_points[0]
            spawn_point.location.z += 0.5
            
            print("Setting up vehicle blueprint...")
            blueprint_library = self.world.get_blueprint_library()
            
            # Filter for only SUVs and large vehicles
            suv_models = [
                'vehicle.audi.etron',
                'vehicle.chevrolet.blazer',
                'vehicle.jeep.wrangler_rubicon',
                'vehicle.lincoln.mkz2017',
                'vehicle.mercedes.coupe',
                'vehicle.mercedes.sprinter',
                'vehicle.mini.cooper_s',
                'vehicle.nissan.patrol',
                'vehicle.tesla.cybertruck',
                'vehicle.toyota.prius',
                'vehicle.volkswagen.t2'
            ]
            
            # Try to find one of the SUV models
            vehicle_bp = None
            for model in suv_models:
                try:
                    bp = blueprint_library.find(model)
                    if bp:
                        vehicle_bp = bp
                        print(f"Selected SUV model: {model}")
                        break
                except:
                    continue
            
            # If no specific SUV model was found, try a generic filter for SUVs
            if not vehicle_bp:
                suv_blueprints = []
                for bp in blueprint_library.filter('vehicle'):
                    if any(tag in bp.id for tag in ['suv', 'offroad', 'truck']):
                        suv_blueprints.append(bp)
                
                if suv_blueprints:
                    vehicle_bp = random.choice(suv_blueprints)
                    print(f"Selected generic SUV model: {vehicle_bp.id}")
                else:
                    # Fallback to any vehicle if no SUVs are found
                    vehicle_bp = blueprint_library.find('vehicle.tesla.cybertruck')
                    print("No SUV models found, using fallback vehicle")
            
            print("Spawning vehicle actor...")
            self.vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)
            if not self.vehicle:
                raise ValueError("Failed to spawn vehicle!")
                
            print("Vehicle spawned successfully")
            print("Initializing sensor manager...")
            self.sensor_manager = SensorManager(self.vehicle, self.world)
            
            if hasattr(self, 'sensor_manager'):
                print("Sensor manager created successfully")
                print("Sensor flags in manager - LIDAR: {}, RADAR: {}, IMU: {}, CAMERA: {}, WAYPOINT: {}".format(
                    self.sensor_manager.lidar_flag, self.sensor_manager.radar_flag, 
                    self.sensor_manager.imu_flag, self.sensor_manager.camera_flag,
                    self.sensor_manager.waypoint_flag))
            else:
                print("ERROR: Failed to create sensor manager!")
            
            # Set up traffic manager after main vehicle is spawned, but handle errors gracefully
            try:
                print("Setting up traffic manager...")
                self.traffic_manager = TrafficManager(self.world, self.vehicle, self.client)
                
                # Only try to spawn vehicles if the traffic manager was initialized
                if hasattr(self.traffic_manager, 'tm') and self.traffic_manager.tm is not None:
                    try:
                        self.traffic_manager.spawn_traffic_vehicles(3)  # Spawn 3 traffic vehicles
                        print("Successfully initialized traffic manager and spawned vehicles")
                    except Exception as e:
                        print("Error spawning initial traffic vehicles: {}".format(e))
                else:
                    print("Traffic manager initialized with limited functionality")
            except Exception as e:
                print("Error initializing traffic manager: {}".format(e))
                self.traffic_manager = None
            
            print("Vehicle setup complete")
            
        except Exception as e:
            print("ERROR in vehicle spawn: {}".format(str(e)))
            raise
            
    def process_input(self, event):
        """Process input events"""
        if event.type == pygame.QUIT:
            return True
        
        if event.type == pygame.KEYUP:
            if event.key == pygame.K_ESCAPE:
                return True
            
            # Toggle sensors with number keys
            elif event.key == pygame.K_1:
                if self.sensor_manager:
                    self.sensor_manager.toggle_lidar()
                    print("LIDAR toggled: {}".format(self.sensor_manager.lidar_flag))
            elif event.key == pygame.K_2:
                if self.sensor_manager:
                    self.sensor_manager.toggle_radar()
                    print("RADAR toggled: {}".format(self.sensor_manager.radar_flag))
            elif event.key == pygame.K_3:
                if self.sensor_manager:
                    self.sensor_manager.toggle_imu()
                    print("IMU toggled: {}".format(self.sensor_manager.imu_flag))
            elif event.key == pygame.K_4:
                if self.sensor_manager:
                    self.sensor_manager.toggle_camera()
                    print("Camera toggled: {}".format(self.sensor_manager.camera_flag))
            elif event.key == pygame.K_5:
                if self.sensor_manager:
                    self.sensor_manager.toggle_waypoint()
                    print("Waypoint toggled: {}".format(self.sensor_manager.waypoint_flag))
            
            # Reset vehicle controls when key is released
            elif event.key == pygame.K_w or event.key == pygame.K_UP:
                self.accelerating_forward = False  # Stop accelerating forward
                # Don't reset throttle immediately - it will gradually decrease
            elif event.key == pygame.K_s or event.key == pygame.K_DOWN:
                self.accelerating_reverse = False  # Stop accelerating reverse
                # Don't reset throttle immediately - it will gradually decrease
            elif event.key == pygame.K_a or event.key == pygame.K_LEFT or event.key == pygame.K_d or event.key == pygame.K_RIGHT:
                self.steer = 0.0
            
            # Toggle reverse with R key (kept for compatibility)
            elif event.key == pygame.K_r:
                self.reverse = not self.reverse
                print("Reverse: {}".format(self.reverse))
            
            # Traffic management keys - handle gracefully if traffic_manager is None
            elif event.key == pygame.K_t:
                # Add a single traffic vehicle
                if self.traffic_manager:
                    try:
                        self.traffic_manager.spawn_traffic_vehicles(1)
                        print("Added 1 traffic vehicle")
                    except Exception as e:
                        print("Error adding traffic vehicle: {}".format(e))
                else:
                    print("Traffic manager not available")
            elif event.key == pygame.K_u:
                # Add random traffic vehicles
                if self.traffic_manager:
                    try:
                        self.traffic_manager.spawn_random_traffic_vehicles(3)
                        print("Added 3 random traffic vehicles")
                    except Exception as e:
                        print("Error adding random traffic vehicles: {}".format(e))
                else:
                    print("Traffic manager not available")
            elif event.key == pygame.K_y:
                # Remove last traffic vehicle
                if self.traffic_manager:
                    try:
                        if self.traffic_manager.remove_last_vehicle():
                            print("Removed last traffic vehicle")
                        else:
                            print("No traffic vehicles to remove")
                    except Exception as e:
                        print("Error removing traffic vehicle: {}".format(e))
                else:
                    print("Traffic manager not available")
            elif event.key == pygame.K_m:
                # Toggle roaming mode for traffic vehicles
                if self.traffic_manager:
                    try:
                        is_roaming = self.traffic_manager.set_vehicles_to_roam()
                        print("Traffic vehicles roaming mode: {}".format("ON" if is_roaming else "OFF"))
                    except Exception as e:
                        print("Error toggling vehicle roaming: {}".format(e))
                else:
                    print("Traffic manager not available")
            
            # Help toggle
            elif event.key == pygame.K_h:
                self.show_help = not self.show_help
                print("Help: {}".format(self.show_help))
        
        if event.type == pygame.KEYDOWN:
            # Vehicle control with WASD or arrow keys
            if event.key == pygame.K_w or event.key == pygame.K_UP:
                self.accelerating_forward = True  # Start accelerating forward
                self.accelerating_reverse = False  # Stop accelerating reverse
                self.reverse = False  # Ensure reverse is off when accelerating forward
                self.brake = 0.0      # Ensure brake is off
            elif event.key == pygame.K_s or event.key == pygame.K_DOWN:
                self.accelerating_reverse = True  # Start accelerating reverse
                self.accelerating_forward = False  # Stop accelerating forward
                self.brake = 0.0      # Ensure brake is off
            elif event.key == pygame.K_a or event.key == pygame.K_LEFT:
                self.steer = max(-1.0, self.steer - 1.0)
            elif event.key == pygame.K_d or event.key == pygame.K_RIGHT:
                self.steer = min(1.0, self.steer + 1.0)
            # Add brake control with spacebar
            elif event.key == pygame.K_SPACE:
                self.brake = min(1.0, self.brake + 1.0)
                self.throttle = 0.0   # Ensure throttle is off when braking
                self.accelerating_forward = False  # Stop accelerating
                self.accelerating_reverse = False  # Stop accelerating
        
        return False

    def update_throttle(self):
        """Update throttle based on acceleration flags"""
        # Handle transition between forward and reverse
        # If accelerating forward but currently in reverse (negative throttle)
        if self.accelerating_forward and self.throttle < 0:
            # First bring throttle to zero before accelerating forward
            self.throttle = min(0.0, self.throttle + self.throttle_decrement)
            # Only switch to forward acceleration once throttle reaches zero
            if self.throttle == 0:
                self.reverse = False
            return
        
        # If accelerating reverse but currently moving forward (positive throttle)
        if self.accelerating_reverse and self.throttle > 0:
            # First bring throttle to zero before accelerating in reverse
            self.throttle = max(0.0, self.throttle - self.throttle_decrement)
            return
        
        # Normal acceleration handling
        if self.accelerating_forward:
            # Gradually increase forward throttle
            self.throttle = min(1.0, self.throttle + self.throttle_increment)
            self.reverse = False
        elif self.accelerating_reverse:
            # Gradually increase reverse throttle (negative value)
            self.throttle = max(-1.0, self.throttle - self.throttle_increment)
            self.reverse = self.throttle < 0  # Set reverse based on throttle sign
        else:
            # Gradually decrease throttle when no acceleration key is pressed
            if self.throttle > 0:
                self.throttle = max(0.0, self.throttle - self.throttle_decrement)
            elif self.throttle < 0:
                self.throttle = min(0.0, self.throttle + self.throttle_decrement)
            
            # Update reverse flag based on throttle
            self.reverse = self.throttle < 0

    def update_spectator(self):
        """Update the spectator camera to follow the vehicle"""
        try:
            if not self.vehicle or not self.vehicle.is_alive:
                return
                
            # Get the vehicle's transform
            vehicle_transform = self.vehicle.get_transform()
            
            # Calculate a position behind and above the vehicle
            camera_offset = carla.Location(x=-5, z=3)  # 5 meters behind, 3 meters above
            camera_location = vehicle_transform.transform(camera_offset)
            
            # Create a transform for the spectator
            spectator_transform = carla.Transform(
                camera_location,
                carla.Rotation(pitch=-15, yaw=vehicle_transform.rotation.yaw)
            )
            
            # Apply the transform to the spectator
            spectator = self.world.get_spectator()
            spectator.set_transform(spectator_transform)
            
        except Exception as e:
            print("ERROR in update_spectator: {}".format(e))

    def draw_control_panel(self):
        """Draw the control panel with vehicle controls and sensor status"""
        try:
            # Clear screen with black background
            self.display.fill(self.BLACK)
            
            # Panel sections
            PANEL_WIDTH = self.display_width
            PANEL_HEIGHT = self.display_height
            
            # Draw title and border
            title = self.title_font.render("CARLA Sensor Control Panel", True, self.YELLOW)
            title_rect = title.get_rect(center=(PANEL_WIDTH // 2, 20))
            self.display.blit(title, title_rect)
            pygame.draw.line(self.display, self.YELLOW, (20, 40), (PANEL_WIDTH - 20, 40), 2)
            
            # Left panel - Sensor status
            left_panel_x = 30
            left_panel_y = 60
            
            # Draw sensor status section title
            sensor_title = self.large_font.render("Sensor Status", True, self.WHITE)
            self.display.blit(sensor_title, (left_panel_x, left_panel_y))
            left_panel_y += 40
            
            # Check if sensor manager exists
            if self.sensor_manager:
                # LIDAR status
                lidar_status = "ON" if self.sensor_manager.lidar_flag else "OFF"
                lidar_color = self.GREEN if self.sensor_manager.lidar_flag else self.RED
                lidar_text = self.font.render("LIDAR: {}".format(lidar_status), True, lidar_color)
                self.display.blit(lidar_text, (left_panel_x, left_panel_y))
                left_panel_y += 30
                
                # RADAR status
                radar_status = "ON" if self.sensor_manager.radar_flag else "OFF"
                radar_color = self.GREEN if self.sensor_manager.radar_flag else self.RED
                radar_text = self.font.render("RADAR: {}".format(radar_status), True, radar_color)
                self.display.blit(radar_text, (left_panel_x, left_panel_y))
                left_panel_y += 30
                
                # IMU status
                imu_status = "ON" if self.sensor_manager.imu_flag else "OFF"
                imu_color = self.GREEN if self.sensor_manager.imu_flag else self.RED
                imu_text = self.font.render("IMU: {}".format(imu_status), True, imu_color)
                self.display.blit(imu_text, (left_panel_x, left_panel_y))
                left_panel_y += 30
                
                # Camera status
                camera_status = "ON" if self.sensor_manager.camera_flag else "OFF"
                camera_color = self.GREEN if self.sensor_manager.camera_flag else self.RED
                camera_text = self.font.render("Camera: {}".format(camera_status), True, camera_color)
                self.display.blit(camera_text, (left_panel_x, left_panel_y))
                left_panel_y += 30
                
                # Waypoint status
                waypoint_status = "ON" if self.sensor_manager.waypoint_flag else "OFF"
                waypoint_color = self.GREEN if self.sensor_manager.waypoint_flag else self.RED
                waypoint_text = self.font.render("Waypoint: {}".format(waypoint_status), True, waypoint_color)
                self.display.blit(waypoint_text, (left_panel_x, left_panel_y))
                left_panel_y += 30
            else:
                no_sensors_text = self.font.render("No sensors available", True, self.RED)
                self.display.blit(no_sensors_text, (left_panel_x, left_panel_y))
                left_panel_y += 50
            
            # Traffic vehicle information
            if hasattr(self, 'traffic_manager') and self.traffic_manager:
                traffic_title = self.large_font.render("Traffic Vehicles", True, self.WHITE)
                self.display.blit(traffic_title, (left_panel_x, left_panel_y))
                left_panel_y += 40
                
                vehicle_count = len(self.traffic_manager.traffic_vehicles) if hasattr(self.traffic_manager, 'traffic_vehicles') else 0
                traffic_text = self.font.render("Vehicles: {}".format(vehicle_count), True, self.WHITE)
                self.display.blit(traffic_text, (left_panel_x, left_panel_y))
                left_panel_y += 30
                
                roaming_status = "ON" if hasattr(self.traffic_manager, 'roaming_mode') and self.traffic_manager.roaming_mode else "OFF"
                roaming_color = self.GREEN if roaming_status == "ON" else self.GRAY
                roaming_text = self.font.render("Roaming Mode: {}".format(roaming_status), True, roaming_color)
                self.display.blit(roaming_text, (left_panel_x, left_panel_y))
            
            # Right panel - Vehicle controls
            right_panel_x = PANEL_WIDTH // 2 + 30
            right_panel_y = 60
            
            # Draw controls section title
            controls_title = self.large_font.render("Vehicle Controls", True, self.WHITE)
            self.display.blit(controls_title, (right_panel_x, right_panel_y))
            right_panel_y += 40
            
            # Throttle indicator
            throttle_text = self.font.render("Throttle: {:.2f}{}".format(
                abs(self.throttle), " (Reverse)" if self.reverse else ""), True, self.WHITE)
            self.display.blit(throttle_text, (right_panel_x, right_panel_y))
            pygame.draw.rect(self.display, self.GRAY, (right_panel_x + 150, right_panel_y, 100, 20), 1)
            pygame.draw.rect(self.display, self.GREEN if not self.reverse else self.BLUE, 
                            (right_panel_x + 150, right_panel_y, int(abs(self.throttle) * 100), 20))
            right_panel_y += 30
            
            # Brake indicator
            brake_text = self.font.render("Brake: {:.2f}".format(self.brake), True, self.WHITE)
            self.display.blit(brake_text, (right_panel_x, right_panel_y))
            pygame.draw.rect(self.display, self.GRAY, (right_panel_x + 150, right_panel_y, 100, 20), 1)
            pygame.draw.rect(self.display, self.RED, (right_panel_x + 150, right_panel_y, int(self.brake * 100), 20))
            right_panel_y += 30
            
            # Steering indicator
            steer_text = self.font.render("Steering: {:.2f}".format(self.steer), True, self.WHITE)
            self.display.blit(steer_text, (right_panel_x, right_panel_y))
            pygame.draw.rect(self.display, self.GRAY, (right_panel_x + 150, right_panel_y, 100, 20), 1)
            steer_center = right_panel_x + 150 + 50
            steer_pos = steer_center + int(self.steer * 50)
            pygame.draw.rect(self.display, self.BLUE, (steer_pos - 5, right_panel_y, 10, 20))
            pygame.draw.line(self.display, self.WHITE, (steer_center, right_panel_y), (steer_center, right_panel_y + 20), 1)
            right_panel_y += 30
            
            # Reverse indicator
            reverse_status = "ON" if self.reverse else "OFF"
            reverse_color = self.RED if self.reverse else self.GRAY
            reverse_text = self.font.render("Reverse: {}".format(reverse_status), True, reverse_color)
            self.display.blit(reverse_text, (right_panel_x, right_panel_y))
            right_panel_y += 50
            
            # Help section
            if self.show_help:
                help_y = PANEL_HEIGHT - 180
                help_title = self.large_font.render("Controls Help", True, self.WHITE)
                self.display.blit(help_title, (30, help_y))
                help_y += 30
                
                # Left column of help text
                help_texts_left = [
                    "W/Up: Accelerate",
                    "S/Down: Reverse",
                    "A/Left: Steer Left",
                    "D/Right: Steer Right",
                    "Space: Brake",
                    "ESC: Exit"
                ]
                
                # Right column of help text
                help_texts_right = [
                    "1-5: Toggle Sensors",
                    "T: Add Traffic Vehicle",
                    "U: Add Random Traffic",
                    "Y: Remove Traffic Vehicle",
                    "M: Toggle Roaming Mode",
                    "H: Toggle Help"
                ]
                
                # Draw help text in two columns
                for i, text in enumerate(help_texts_left):
                    help_text = self.small_font.render(text, True, self.LIGHT_GRAY)
                    self.display.blit(help_text, (30, help_y + i * 20))
                    
                for i, text in enumerate(help_texts_right):
                    help_text = self.small_font.render(text, True, self.LIGHT_GRAY)
                    self.display.blit(help_text, (PANEL_WIDTH // 2 + 30, help_y + i * 20))
            
        except Exception as e:
            print("ERROR in draw_control_panel: {0}".format(str(e)))
            traceback.print_exc()

    def run(self):
        try:
            print("\n=== Starting Main Loop ===")
            print("Setting up synchronous mode...")
            settings = self.world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05  # 20 FPS
            self.world.apply_settings(settings)
            
            print("Entering main loop...")
            while self.running:
                try:
                    # Process events
                    for event in pygame.event.get():
                        if self.process_input(event):  # Process keyboard/mouse input
                            self.running = False  # Exit if process_input returns True
                            break
                    
                    # Update throttle based on acceleration flags
                    self.update_throttle()
                    
                    # Tick the world
                    self.world.tick()  # Update CARLA world simulation
                    
                    # Update spectator camera
                    self.update_spectator()  # Update camera position
                    
                    # Apply control to vehicle
                    if self.vehicle and self.vehicle.is_alive:
                        # Use absolute value of throttle, reverse flag is already set based on throttle sign
                        control = carla.VehicleControl(
                            throttle=abs(self.throttle),  # Throttle control
                            steer=self.steer,  # Steering control
                            brake=self.brake,  # Brake control
                            hand_brake=False,  # No handbrake
                            reverse=self.reverse  # Reverse gear state
                        )
                        self.vehicle.apply_control(control)  # Apply control to vehicle
                    elif self.vehicle and not self.vehicle.is_alive:
                        print("Vehicle is not alive, respawning...")
                        self.spawn_vehicle()  # Respawn vehicle if destroyed
                    
                    # Update display
                    self.draw_control_panel()  # Draw GUI
                    pygame.display.flip()  # Update display
                    
                    # Cap the frame rate
                    pygame.time.Clock().tick(20)  # Limit to 20 FPS
                    
                except Exception as e:
                    print("ERROR in main loop iteration: {0}".format(str(e)))
                    traceback.print_exc()
            
            print("Main loop ended")
            
        except KeyboardInterrupt:
            print("KeyboardInterrupt received")
        except Exception as e:
            print("ERROR in run method: {0}".format(str(e)))
            traceback.print_exc()
        finally:
            print("Cleaning up...")
            self.cleanup()  # Clean up resources

def main():
    """
    Main entry point for the CARLA GUI Control application.
    Sets up the CARLA environment and starts the control interface.
    """
    try:
        carla_setup = CARLASetup()  # Initialize CARLA Python API
        control = CarlaControl()  # Create control interface
        control.run()  # Run main loop
    except Exception as e:
        print("Error:", str(e))
        sys.exit(1)  # Exit with error code

if __name__ == '__main__':
    main()  # Run main function when script is executed directly
