#!/usr/bin/python3
import socket
import sys
import time
import argparse

class VehicleControlClient:
    def __init__(self, host='localhost', port=12344):
        """
        Initialize the vehicle control client
        
        This client connects to a vehicle control server and sends control commands
        in the format "throttle,steering,brake,reverse" (comma-separated values).
        
        Args:
            host (str): Server host address
            port (int): Server port
        """
        self.host = host
        self.port = port
        self.socket = None
        self.connected = False
        
    def connect(self):
        """Connect to the vehicle control server"""
        try:
            print(f"Connecting to server at {self.host}:{self.port}...")
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host, self.port))
            self.connected = True
            print("Connected to server")
            return True
        except Exception as e:
            print(f"Error connecting to server: {e}")
            return False
            
    def disconnect(self):
        """Disconnect from the server"""
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
            self.socket = None
            self.connected = False
            print("Disconnected from server")
            
    def send_control(self, throttle, steering, brake):
        """
        Send control command to the server
        
        Args:
            throttle (float): Throttle value (-1.0 to 1.0, negative values activate reverse)
            steering (float): Steering value (-1.0 to 1.0, negative is left)
            brake (float): Brake value (0.0 to 1.0)
        
        Returns:
            bool: True if command was sent successfully
        """
        if not self.connected:
            print("Not connected to server")
            return False
        
        # Handle negative throttle as reverse
        reverse = False
        if throttle < 0:
            reverse = True
            throttle = abs(throttle)  # Use absolute value for throttle
        
        # Clamp values to valid ranges
        throttle = max(0.0, min(1.0, throttle))
        steering = max(-1.0, min(1.0, steering))
        brake = max(0.0, min(1.0, brake))
        
        # Format command using comma as delimiter
        command = f"{throttle:.4f},{steering:.4f},{brake:.4f},{1 if reverse else 0}\n"
        
        try:
            self.socket.sendall(command.encode('utf-8'))
            print(f"Sent control: Throttle={throttle:.2f}, Steering={steering:.2f}, Brake={brake:.2f}, Reverse={reverse}")
            return True
        except Exception as e:
            print(f"Error sending command: {e}")
            self.connected = False
            return False
            
    def interactive_mode(self):
        """Run an interactive mode to manually send commands"""
        print("\n=== Vehicle Control Client Interactive Mode ===")
        print("Enter control values as 'throttle steering brake' (space-separated)")
        print("Example: '0.5 0.0 0.0' for half throttle, no steering, no brake")
        print("Use negative throttle values (-0.5) for reverse")
        print("Type 'quit' or 'exit' to disconnect\n")
        
        while self.connected:
            try:
                user_input = input("Enter control values > ")
                
                if user_input.lower() in ['quit', 'exit', 'q']:
                    break
                    
                # Parse input
                try:
                    values = user_input.strip().split()
                    if len(values) == 3:
                        throttle = float(values[0])
                        steering = float(values[1])
                        brake = float(values[2])
                        self.send_control(throttle, steering, brake)
                    else:
                        print("Invalid input. Please enter 3 values: throttle steering brake")
                except ValueError:
                    print("Invalid input. Please enter numeric values")
                    
            except KeyboardInterrupt:
                break
                
        print("Exiting interactive mode")
        
    def demo_mode(self):
        """Run a demo sequence of control commands"""
        if not self.connected:
            print("Not connected to server")
            return
            
        print("\n=== Running Demo Sequence ===")
        
        # Sequence of (throttle, steering, brake, duration) commands
        sequence = [
            (0.5, 0.0, 0.0, 2.0),     # Accelerate straight
            (0.3, 0.5, 0.0, 3.0),     # Turn right
            (0.3, -0.5, 0.0, 3.0),    # Turn left
            (0.7, 0.0, 0.0, 2.0),     # Accelerate harder
            (0.0, 0.0, 0.8, 2.0),     # Brake
            (-0.3, 0.0, 0.0, 2.0),    # Reverse
            (-0.3, 0.5, 0.0, 2.0),    # Reverse right
            (0.0, 0.0, 0.0, 1.0)      # Stop
        ]
        
        try:
            for i, (throttle, steering, brake, duration) in enumerate(sequence):
                reverse_text = " (reverse)" if throttle < 0 else ""
                print(f"\nStep {i+1}/{len(sequence)}: Throttle={abs(throttle):.2f}{reverse_text}, Steering={steering:.2f}, Brake={brake:.2f}, Duration={duration:.1f}s")
                if not self.send_control(throttle, steering, brake):
                    print("Failed to send command, aborting demo")
                    break
                time.sleep(duration)
                
            print("\nDemo sequence completed")
            
        except KeyboardInterrupt:
            print("\nDemo interrupted by user")
        
def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="CARLA Vehicle Control Client")
    parser.add_argument("--host", default="localhost", help="Server host address")
    parser.add_argument("--port", type=int, default=12344, help="Server port")
    parser.add_argument("--demo", action="store_true", help="Run demo sequence")
    args = parser.parse_args()
    
    # Create and run client
    client = VehicleControlClient(args.host, args.port)
    
    try:
        if client.connect():
            if args.demo:
                client.demo_mode()
            else:
                client.interactive_mode()
    finally:
        client.disconnect()
        
if __name__ == "__main__":
    main() 