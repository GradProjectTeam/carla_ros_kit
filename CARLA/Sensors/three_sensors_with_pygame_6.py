#! /usr/local/bin/python3.5m
import sys
import glob
import os
import socket
import pickle
import numpy as np
import time
import json
import struct
import math
import random
import pygame
from pygame.locals import *
import threading
from queue import Queue

# Version 6: Added traffic vehicles in front of the main vehicle
# The traffic vehicles have no sensors and use CARLA's autopilot system
# The main vehicle remains user-controlled with sensors

class CARLASetup:
    def __init__(self):
        print("Starting CARLA setup...")
        self.carla_path = '/home/mostafa/ROS2andCarla/CARLA/CARLA_0.9.8/PythonAPI/carla/dist'
        self.setup_carla()
        
    def setup_carla(self):
        print("Looking for CARLA at:", self.carla_path)
        carla_eggs = glob.glob('{0}/carla-*{1}.{2}-{3}.egg'.format(
            self.carla_path,
            sys.version_info.major,
            sys.version_info.minor,
            "win-amd64" if os.name == "nt" else "linux-x86_64"
        ))
        sys.path.append(carla_eggs[0])
        print("Found CARLA egg:", carla_eggs[0])
        
        global carla
        import carla # type: ignore
        print("CARLA imported successfully")

class SensorManager:
    def __init__(self, vehicle, world):
        self.vehicle = vehicle
        self.world = world
        self.actor_list = []
        
        # Data queues with thread-safe implementation
        self.lidar_queue = Queue(maxsize=1)
        self.radar_queue = Queue(maxsize=1)
        self.imu_queue = Queue(maxsize=1)  # New IMU queue
        
        # TCP setup with different ports
        self.host_ip = '127.0.0.1'
        self.lidar_port = 12349
        self.radar_port = 12347
        self.imu_port = 12341  # New IMU port

        self.lidar_flag = True
        self.radar_flag = False
        self.imu_flag = False
        
        # Thread control
        self.running = True
        self.lidar_thread = None
        self.radar_thread = None
        self.imu_thread = None  # New IMU thread
        
        # Setup separate sockets for each sensor
        self.setup_tcp_sockets()
        self.setup_sensors()
        
        # Start processing threads
        self.start_processing_threads()
        
    def setup_tcp_sockets(self):
        # LiDAR socket

        if self.lidar_flag:
            self.lidar_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.lidar_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print("LiDAR TCP configured for {0}:{1}".format(self.host_ip, self.lidar_port))
        
        # Radar socket
        if self.radar_flag:
            self.radar_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.radar_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print("Radar TCP configured for {0}:{1}".format(self.host_ip, self.radar_port))
        
        # IMU socket
        if self.imu_flag:
            self.imu_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.imu_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print("IMU TCP configured for {0}:{1}".format(self.host_ip, self.imu_port))
        
    def start_processing_threads(self):
        if self.lidar_flag:
            self.lidar_thread = threading.Thread(target=self.process_lidar_queue)
        if self.radar_flag:
            self.radar_thread = threading.Thread(target=self.process_radar_queue)
        if self.imu_flag:
            self.imu_thread = threading.Thread(target=self.process_imu_queue)  # New IMU thread
        
        if self.lidar_flag:
            self.lidar_thread.daemon = True
        if self.radar_flag:
            self.radar_thread.daemon = True
        if self.imu_flag:
            self.imu_thread.daemon = True  # Set as daemon thread
        
        if self.lidar_flag:
            self.lidar_thread.start()
        if self.radar_flag:
            self.radar_thread.start()
        if self.imu_flag:
            self.imu_thread.start()  # Start IMU thread
        
    def process_lidar_queue(self):
        point_counter = 0  # Counter for debug output
        while self.running:
            try:
                if not self.lidar_queue.empty():
                    point_cloud = self.lidar_queue.get()
                    for point in point_cloud:
                        if not self.running:
                            break
                        
                        # Print debug info every 1000 points
                        point_counter += 1
                        if point_counter % 1000 == 0:
                            print("Sending LIDAR point #{0}: ({1:.2f}, {2:.2f}, {3:.2f})".format(point_counter, point.x, point.y, point.z))
                        
                        # Convert to network byte order (big-endian)
                        # Pack as float32 values in network byte order
                        point_data = struct.pack('!fff', point.x, point.y, point.z)
                        try:
                            if self.lidar_flag:
                                self.lidar_socket.send(point_data)
                        except socket.error as e:
                            print("LiDAR socket error: {0}".format(e))
                            break
                else:
                    time.sleep(0.001)  # Small sleep to prevent CPU hogging
            except Exception as e:
                print("Error in LiDAR processing thread: {0}".format(e))
                
    def process_radar_queue(self):
        while self.running:
            try:
                if not self.radar_queue.empty():
                    radar_data = self.radar_queue.get()
                    points = np.array([[det.altitude, det.azimuth, det.depth, det.velocity] 
                                    for det in radar_data], dtype=np.float32)
                    
                    for point in points:
                        if not self.running:
                            break
                        data = point.tobytes()
                        try:
                            if self.radar_flag:
                                self.radar_socket.sendall(data)
                        except socket.error as e:
                            print("Radar socket error: {0}".format(e))
                            break
                else:
                    time.sleep(0.001)  # Small sleep to prevent CPU hogging
            except Exception as e:
                print("Error in Radar processing thread: {0}".format(e))
    
    def process_imu_queue(self):
        while self.running:
            try:
                if not self.imu_queue.empty():
                    imu_data = self.imu_queue.get()
                    # Pack IMU data: acceleration (3 floats) + gyroscope (3 floats) + compass (1 float)
                    data = struct.pack('fffffff', 
                                     imu_data.accelerometer.x, imu_data.accelerometer.y, imu_data.accelerometer.z,
                                     imu_data.gyroscope.x, imu_data.gyroscope.y, imu_data.gyroscope.z,
                                     imu_data.compass)
                    try:
                        if self.imu_flag:
                            self.imu_socket.sendall(data)
                    except socket.error as e:
                        print("IMU socket error: {0}".format(e))
                else:
                    time.sleep(0.001)  # Small sleep to prevent CPU hogging
            except Exception as e:
                print("Error in IMU processing thread: {0}".format(e))
    
    def lidar_callback(self, point_cloud):
        try:
            if self.lidar_queue.full():
                try:
                    self.lidar_queue.get(block=False)  # Python 3.5 compatible
                except Queue.Empty:
                    pass
            self.lidar_queue.put(point_cloud, block=False)
        except Exception as e:
            print("Error in LiDAR callback: {0}".format(e))
        
    def radar_callback(self, radar_data):
        try:
            if self.radar_queue.full():
                try:
                    self.radar_queue.get(block=False)  # Python 3.5 compatible
                except Queue.Empty:
                    pass
            self.radar_queue.put(radar_data, block=False)
        except Exception as e:
            print("Error in Radar callback: {0}".format(e))
        
    def imu_callback(self, imu_data):
        try:
            if self.imu_queue.full():
                try:
                    self.imu_queue.get(block=False)  # Remove old data
                except Queue.Empty:
                    pass
            self.imu_queue.put(imu_data, block=False)
        except Exception as e:
            print("Error in IMU callback: {0}".format(e))
        
    def cleanup(self):
        print("Cleaning up sensors and sockets...")
        self.running = False  # Signal threads to stop
        
        # Wait for processing threads to finish
        if self.lidar_thread and self.lidar_thread.is_alive():
            self.lidar_thread.join(timeout=1.0)
        if self.radar_thread and self.radar_thread.is_alive():
            self.radar_thread.join(timeout=1.0)
        if self.imu_thread and self.imu_thread.is_alive():
            self.imu_thread.join(timeout=1.0)
        
        # Clean up actors
        for actor in self.actor_list:
            if actor is not None and actor.is_alive:
                actor.destroy()
        
        # Close sockets
        if hasattr(self, 'lidar_socket'):
            try:
                if self.lidar_flag:
                    self.lidar_socket.shutdown(socket.SHUT_RDWR)
                    self.lidar_socket.close()
            except Exception as e:
                print("Error closing LiDAR socket: {0}".format(e))
            
        if hasattr(self, 'radar_socket'):
            try:
                if self.radar_flag:
                    self.radar_socket.shutdown(socket.SHUT_RDWR)
                    self.radar_socket.close()
            except Exception as e:
                print("Error closing Radar socket: {0}".format(e))
                
        if hasattr(self, 'imu_socket'):
            try:
                if self.imu_flag:
                    self.imu_socket.shutdown(socket.SHUT_RDWR)
                    self.imu_socket.close()
            except Exception as e:
                print("Error closing IMU socket: {0}".format(e))
                
        print("Sensor cleanup complete")

    def setup_sensors(self):
        try:
            self.setup_lidar()
            self.setup_radar()
            self.setup_imu()  # Add IMU setup
            print("Sensors setup complete")
        except Exception as e:
            print("Error in setup_sensors: {0}".format(e))
            raise

    def setup_lidar(self):
        try:
            lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast')
            lidar_bp.set_attribute('channels', '32')
            lidar_bp.set_attribute('points_per_second', '100000')
            lidar_bp.set_attribute('rotation_frequency', '20')
            lidar_bp.set_attribute('range', '50.0')
            lidar_bp.set_attribute('upper_fov', '10.0')
            lidar_bp.set_attribute('lower_fov', '-30.0')
            
            # Mount on top of the car, slightly forward
            lidar_transform = carla.Transform(
                carla.Location(x=1.5, z=2.0),  # x: forward, z: up
                carla.Rotation()  # Default rotation (0,0,0) will inherit car's rotation
            )
            
            self.lidar = self.world.spawn_actor(lidar_bp, lidar_transform, attach_to=self.vehicle)
            self.actor_list.append(self.lidar)
            self.lidar.listen(self.lidar_callback)
            print("LiDAR sensor added")
            
            # Connect LiDAR socket
            if self.lidar_flag:
                self.lidar_socket.connect((self.host_ip, self.lidar_port))
                print("LiDAR TCP connected")
            
        except Exception as e:
            print("Error in LiDAR setup: {0}".format(str(e)))
            raise
        
    def setup_radar(self):
        try:
            radar_bp = self.world.get_blueprint_library().find('sensor.other.radar')
            radar_bp.set_attribute('horizontal_fov', '30.0')
            radar_bp.set_attribute('vertical_fov', '10.0')
            radar_bp.set_attribute('points_per_second', '1500')
            radar_bp.set_attribute('range', '50.0')
            
            # Mount on front of the car, same height as lidar
            radar_transform = carla.Transform(
                carla.Location(x=1.5, z=2.0),  # x: forward, z: up
                carla.Rotation()  # Default rotation (0,0,0) will inherit car's rotation
            )
            
            self.radar = self.world.spawn_actor(radar_bp, radar_transform, attach_to=self.vehicle)
            self.actor_list.append(self.radar)
            self.radar.listen(self.radar_callback)
            print("Radar sensor added")
            
            # Connect Radar socket
            if self.radar_flag:
                self.radar_socket.connect((self.host_ip, self.radar_port))
                print("Radar TCP connected")
            
        except Exception as e:
            print("Error in Radar setup: {0}".format(str(e)))
            raise

    def setup_imu(self):
        try:
            imu_bp = self.world.get_blueprint_library().find('sensor.other.imu')
            
            # Set IMU parameters
            imu_bp.set_attribute('sensor_tick', '0.05')  # 20Hz update rate
            
            # Mount at the center of the car
            imu_transform = carla.Transform(
                carla.Location(x=0.0, z=0.0),  # Center of the vehicle
                carla.Rotation()  # Default rotation
            )
            
            self.imu = self.world.spawn_actor(imu_bp, imu_transform, attach_to=self.vehicle)
            self.actor_list.append(self.imu)
            self.imu.listen(self.imu_callback)
            print("IMU sensor added")
            
            # Connect IMU socket
            if self.imu_flag:
                self.imu_socket.connect((self.host_ip, self.imu_port))
                print("IMU TCP connected")
            
        except Exception as e:
            print("Error in IMU setup: {0}".format(str(e)))
            raise

class CarlaControl:
    def __init__(self):
        print("\n=== Starting CarlaControl Initialization ===")
        try:
            print("Setting up Pygame...")
            self.setup_pygame()
            print("Setting up CARLA client...")
            
            # Control variables
            self.throttle = 0.0
            self.brake = 0.0
            self.steer = 0.0
            self.reverse = False  # Add reverse state
            self.vehicle = None
            self.running = True
            self.traffic_cars = []  # List to store traffic vehicles
            self.setup_carla_client()
            self.sensor_manager = None
            self.traffic_manager = None  # Traffic manager reference
            print("CarlaControl initialization complete")
            
        except Exception as e:
            print("ERROR in CarlaControl initialization: {0}".format(str(e)))
            self.cleanup()
            raise
        
    def setup_pygame(self):
        try:
            print("Initializing Pygame...")
            pygame.init()
            self.WINDOW_WIDTH = 800
            self.WINDOW_HEIGHT = 600
            print("Creating Pygame window...")
            self.screen = pygame.display.set_mode((self.WINDOW_WIDTH, self.WINDOW_HEIGHT))
            pygame.display.set_caption("CARLA Control Panel")
            self.clock = pygame.time.Clock()
            
            # Colors
            self.WHITE = (255, 255, 255)
            self.GREEN = (0, 255, 0)
            self.RED = (255, 0, 0)
            self.BLUE = (0, 0, 255)
            self.GRAY = (128, 128, 128)
            self.BLACK = (0, 0, 0)
            
            # Control panel positions
            self.THROTTLE_POS = (50, 250)
            self.BRAKE_POS = (150, 250)
            self.STEER_POS = (250, 200)
            self.BAR_WIDTH = 30
            self.BAR_HEIGHT = 100
            
            # Initialize fonts
            print("Setting up fonts...")
            self.font = pygame.font.Font(None, 36)
            self.small_font = pygame.font.Font(None, 24)
            
            # Draw initial screen
            self.screen.fill(self.BLACK)
            welcome_text = self.font.render("CARLA Control Panel", True, self.WHITE)
            text_rect = welcome_text.get_rect(center=(self.WINDOW_WIDTH/2, self.WINDOW_HEIGHT/2))
            self.screen.blit(welcome_text, text_rect)
            pygame.display.flip()
            
            print("Pygame setup complete")
            
        except Exception as e:
            print("ERROR in Pygame setup: {0}".format(str(e)))
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
            vehicle_bp = blueprint_library.find('vehicle.tesla.model3')
            
            print("Spawning vehicle actor...")
            self.vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)
            if not self.vehicle:
                raise ValueError("Failed to spawn vehicle!")
                
            print("Vehicle spawned successfully")
            print("Initializing sensor manager...")
            self.sensor_manager = SensorManager(self.vehicle, self.world)
            
            # Set up traffic manager after main vehicle is spawned
            print("Setting up traffic manager...")
            self.traffic_manager = TrafficManager(self.world, self.vehicle)
            self.traffic_manager.spawn_traffic_vehicles(3)  # Spawn 3 traffic vehicles
            
            print("Vehicle setup complete")
            
        except Exception as e:
            print("ERROR in vehicle spawn: {0}".format(str(e)))
            raise
            
    def process_input(self):
        try:
            print("\n=== Processing Input ===")
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    print("Quit event received")
                    return False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        print("Escape key pressed")
                        return False
                    elif event.key == pygame.K_r:
                        self.reverse = not self.reverse
                        print("REVERSE TOGGLED: {}".format(self.reverse))
                    elif event.key == pygame.K_t:
                        # Add more traffic vehicles when 'T' is pressed
                        if self.traffic_manager:
                            self.traffic_manager.spawn_traffic_vehicles(1)
                            print("Added additional traffic vehicle")
                    elif event.key == pygame.K_y:
                        # Remove traffic vehicles when 'Y' is pressed
                        if self.traffic_manager and self.traffic_manager.traffic_vehicles:
                            self.traffic_manager.remove_last_vehicle()
                            print("Removed last traffic vehicle")
            
            keys = pygame.key.get_pressed()
            
            # Throttle and Brake with smoother control
            if keys[pygame.K_UP] or keys[pygame.K_w]:
                self.throttle = min(2.0, self.throttle + 0.5)
                self.brake = 0.0
                print("THROTTLE UP: {:.2f}, BRAKE: {:.2f}, REVERSE: {}".format(
                    self.throttle, self.brake, self.reverse))
            elif keys[pygame.K_DOWN] or keys[pygame.K_s]:
                self.brake = min(1.0, self.brake + 0.5)
                self.throttle = 0.0
                print("BRAKE UP: {:.2f}, THROTTLE: {:.2f}, REVERSE: {}".format(
                    self.brake, self.throttle, self.reverse))
            else:
                self.throttle = max(0.0, self.throttle - 0.5)
                self.brake = max(0.0, self.brake - 0.5)
                print("COASTING - Throttle: {:.2f}, Brake: {:.2f}, REVERSE: {}".format(
                    self.throttle, self.brake, self.reverse))
            
            # Steering with improved response
            if keys[pygame.K_LEFT] or keys[pygame.K_a]:
                self.steer = max(-1.0, self.steer - 0.15)
                print("STEERING LEFT: {:.2f}".format(self.steer))
            elif keys[pygame.K_RIGHT] or keys[pygame.K_d]:
                self.steer = min(1.0, self.steer + 0.15)
                print("STEERING RIGHT: {:.2f}".format(self.steer))
            else:
                self.steer = self.steer * 0.7
                print("STEERING CENTER: {:.2f}".format(self.steer))
                
            # Apply control to vehicle
            if self.vehicle:
                try:
                    control = carla.VehicleControl(
                        throttle=self.throttle,
                        steer=self.steer,
                        brake=self.brake,
                        hand_brake=keys[pygame.K_SPACE],
                        reverse=self.reverse  # Add reverse state
                    )
                    
                    print("Applying control to vehicle:")
                    print("  - Throttle: {:.2f}".format(control.throttle))
                    print("  - Brake: {:.2f}".format(control.brake))
                    print("  - Steer: {:.2f}".format(control.steer))
                    print("  - Handbrake: {}".format(control.hand_brake))
                    print("  - Reverse: {}".format(control.reverse))
                    
                    if self.vehicle.is_alive:
                        self.vehicle.apply_control(control)
                    else:
                        print("ERROR: Vehicle is not alive!")
                        self.spawn_vehicle()
                except Exception as e:
                    print("ERROR applying vehicle control: {}".format(str(e)))
                    
            return True
            
        except Exception as e:
            print("ERROR in process_input: {0}".format(str(e)))
            return False
            
    def update_spectator(self):
        try:
            if self.vehicle:
                # Get vehicle transform
                vehicle_transform = self.vehicle.get_transform()
                
                # Calculate camera position behind and above vehicle
                camera_offset = carla.Location(x=-8, z=4)  # 8 meters behind, 4 meters up
                camera_location = vehicle_transform.transform(camera_offset)
                
                # Point camera at vehicle
                camera_rotation = carla.Rotation(
                    pitch=-15,  # Look down slightly
                    yaw=vehicle_transform.rotation.yaw  # Match vehicle direction
                )
                
                # Set spectator position and rotation
                spectator = self.world.get_spectator()
                spectator.set_transform(
                    carla.Transform(camera_location, camera_rotation)
                )
                print("Spectator updated - Following vehicle from behind")
                
        except Exception as e:
            print("ERROR in update_spectator: {0}".format(str(e)))
        
    def draw_control_panel(self):
        try:
            # Clear screen with black background
            self.screen.fill((0, 0, 0))
            
            # Colors
            WHITE = (255, 255, 255)
            GREEN = (0, 255, 0)
            RED = (255, 0, 0)
            BLUE = (0, 0, 255)
            GRAY = (128, 128, 128)
            
            # Draw throttle bar
            throttle_height = int(self.throttle * 100)
            pygame.draw.rect(self.screen, GRAY, (50, 250, 30, -100))  # Background
            pygame.draw.rect(self.screen, GREEN, (50, 250, 30, -throttle_height))
            text = self.font.render("Throttle: {throttle:.2f}".format(throttle=self.throttle), True, WHITE)
            self.screen.blit(text, (20, 260))
            
            # Draw brake bar
            brake_height = int(self.brake * 100)
            pygame.draw.rect(self.screen, GRAY, (150, 250, 30, -100))  # Background
            pygame.draw.rect(self.screen, RED, (150, 250, 30, -brake_height))
            text = self.font.render("Brake: {brake:.2f}".format(brake=self.brake), True, WHITE)
            self.screen.blit(text, (130, 260))
            
            # Draw steering indicator
            pygame.draw.rect(self.screen, GRAY, (250, 200, 100, 20))  # Steering background
            steer_pos = 300 + (self.steer * 45)  # Convert -1:1 to pixel position
            pygame.draw.circle(self.screen, BLUE, (int(steer_pos), 210), 10)
            text = self.font.render("Steer: {steer:.2f}".format(steer=self.steer), True, WHITE)
            self.screen.blit(text, (250, 260))
            
            # Draw sensor status
            if hasattr(self, 'sensor_manager') and self.sensor_manager:
                lidar_status = "LIDAR: Active" if not self.sensor_manager.lidar_queue.empty() else "LIDAR: Waiting"
                radar_status = "RADAR: Active" if not self.sensor_manager.radar_queue.empty() else "RADAR: Waiting"
                imu_status = "IMU: Active" if not self.sensor_manager.imu_queue.empty() else "IMU: Waiting"
                
                text = self.small_font.render(lidar_status, True, GREEN if "Active" in lidar_status else RED)
                self.screen.blit(text, (20, 20))
                text = self.small_font.render(radar_status, True, GREEN if "Active" in radar_status else RED)
                self.screen.blit(text, (20, 40))
                text = self.small_font.render(imu_status, True, GREEN if "Active" in imu_status else RED)
                self.screen.blit(text, (20, 60))
            
            # Draw traffic vehicle status
            if hasattr(self, 'traffic_manager') and self.traffic_manager:
                traffic_count = len(self.traffic_manager.traffic_vehicles)
                traffic_status = "Traffic Vehicles: {}".format(traffic_count)
                text = self.small_font.render(traffic_status, True, BLUE)
                self.screen.blit(text, (20, 80))
            
            # Draw controls help
            help_texts = [
                "Controls:",
                "W/UP: Accelerate",
                "S/DOWN: Brake",
                "A/LEFT: Turn Left",
                "D/RIGHT: Turn Right",
                "SPACE: Handbrake",
                "R: Toggle Reverse",
                "T: Add Traffic Vehicle",
                "Y: Remove Traffic Vehicle",
                "ESC: Quit"
            ]
            
            for i, text in enumerate(help_texts):
                help_surface = self.small_font.render(text, True, WHITE)
                self.screen.blit(help_surface, (20, 120 + i * 20))  # Moved down to make room for traffic status
            
            pygame.display.flip()
            
        except Exception as e:
            print("ERROR in draw_control_panel: {0}".format(str(e)))

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
                    if not self.process_input():
                        print("Process input returned False, breaking loop")
                        break
                    
                    self.world.tick()
                    self.update_spectator()
                    self.draw_control_panel()
                    self.clock.tick(20)
                    
                except Exception as e:
                    print("ERROR in main loop iteration: {0}".format(str(e)))
                    break
                    
        except KeyboardInterrupt:
            print("\nKeyboard interrupt received")
        except Exception as e:
            print("ERROR in run method: {0}".format(str(e)))
        finally:
            print("Initiating cleanup...")
            self.cleanup()
            
    def cleanup(self):
        print("\n=== Starting Cleanup ===")
        try:
            print("Setting running flag to False...")
            self.running = False
            
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

class TrafficManager:
    def __init__(self, world, ego_vehicle):
        self.world = world
        self.ego_vehicle = ego_vehicle  # Main player vehicle
        self.traffic_vehicles = []  # List of spawned traffic vehicles
        self.blueprint_library = self.world.get_blueprint_library()
        self.spawn_points = self.world.get_map().get_spawn_points()
        self.safety_distance = 20.0  # Distance in front of ego vehicle to start spawning traffic
        print("Traffic Manager initialized")
        
    def spawn_traffic_vehicles(self, num_vehicles=3):
        """Spawn a specified number of traffic vehicles in front of the ego vehicle"""
        print("Spawning {} traffic vehicles...".format(num_vehicles))
        
        # Get ego vehicle transform
        ego_transform = self.ego_vehicle.get_transform()
        ego_location = ego_transform.location
        ego_forward_vector = ego_transform.get_forward_vector()
        
        # Create a list of available vehicle blueprints (excluding bikes/motorcycles for stability)
        vehicle_blueprints = [bp for bp in self.blueprint_library.filter('vehicle.*') 
                            if int(bp.get_attribute('number_of_wheels')) >= 4]
        
        for i in range(num_vehicles):
            # Calculate spawn position ahead of ego vehicle
            # Each vehicle is placed further ahead than the previous one
            distance = self.safety_distance + (i * 10) + (len(self.traffic_vehicles) * 10)  # Increasing distance
            spawn_location = carla.Location(
                x=ego_location.x + (ego_forward_vector.x * distance),
                y=ego_location.y + (ego_forward_vector.y * distance),
                z=ego_location.z + 0.5
            )
            
            # Create the spawn transform at the calculated location
            spawn_transform = carla.Transform()
            spawn_transform.location = spawn_location
            spawn_transform.rotation = ego_transform.rotation  # Same direction as ego vehicle
            
            # Randomly select a vehicle blueprint
            bp = random.choice(vehicle_blueprints)
            
            # Try to spawn the vehicle
            try:
                vehicle = self.world.spawn_actor(bp, spawn_transform)
                if vehicle:
                    print("Spawned traffic vehicle {}: {}".format(len(self.traffic_vehicles)+1, bp.id))
                    self.traffic_vehicles.append(vehicle)
                    
                    # Set it to autopilot using CARLA's traffic manager
                    carla_tm = self.world.get_traffic_manager()
                    vehicle.set_autopilot(True, carla_tm.get_port())
                    
                    # Configure traffic manager behavior for this vehicle
                    carla_tm.ignore_lights_percentage(vehicle, 0)  # Obey traffic lights
                    carla_tm.keep_right_rule_percentage(vehicle, 90)  # Mostly keep right
                    carla_tm.distance_to_leading_vehicle(vehicle, 5.0)  # Follow distance
                    carla_tm.vehicle_percentage_speed_difference(vehicle, -10)  # Slightly slower than limit
            except Exception as e:
                print("Failed to spawn traffic vehicle {}: {}".format(i+1, e))
                
        print("Successfully spawned {} traffic vehicles".format(len(self.traffic_vehicles)))
        
    def remove_last_vehicle(self):
        """Remove the last traffic vehicle from the simulation"""
        if not self.traffic_vehicles:
            print("No traffic vehicles to remove")
            return
            
        vehicle = self.traffic_vehicles.pop()
        if vehicle and vehicle.is_alive:
            vehicle.destroy()
            print("Removed one traffic vehicle")
            return True
        return False
        
    def cleanup(self):
        """Destroy all spawned traffic vehicles"""
        print("Cleaning up {} traffic vehicles...".format(len(self.traffic_vehicles)))
        for vehicle in self.traffic_vehicles:
            if vehicle is not None and vehicle.is_alive:
                vehicle.destroy()
        self.traffic_vehicles.clear()
        print("Traffic vehicles cleanup complete")

def main():
    try:
        carla_setup = CARLASetup()
        control = CarlaControl()
        control.run()
    except Exception as e:
        print("Error:", str(e))
        sys.exit(1)

if __name__ == '__main__':
    main()