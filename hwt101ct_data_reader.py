#!/usr/bin/env python3
# coding: UTF-8

import serial
import time
import struct
import threading

class HWT101CTReader:
    def __init__(self, port="/dev/ttyUSB0", baud=230400):
        """Initialize connection to HWT101CT sensor"""
        self.port = port
        self.baud = baud
        self.ser = None
        self.running = False
        
        # Data storage
        self.gyro_z = 0.0  # Angular velocity Z (deg/s)
        self.yaw_angle = 0.0  # Yaw angle (degrees)
        self.data_rate = 0  # Current data rate (Hz)
        self.last_update_time = 0
        self.update_count = 0
        self.start_time = 0
        
    def connect(self):
        """Connect to the sensor"""
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            print(f"Connected to {self.port} at {self.baud} baud")
            time.sleep(0.1)  # Wait for connection to stabilize
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from sensor"""
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Disconnected from sensor")
    
    def parse_data(self, data):
        """Parse HWT101CT data format"""
        # Look for data packets (typically start with 0x55)
        for i in range(len(data) - 10):
            if data[i] == 0x55:
                packet_type = data[i + 1]
                
                # Angular velocity packet (0x52)
                if packet_type == 0x52 and i + 10 < len(data):
                    # Skip Y axis, get Z axis angular velocity
                    gyro_z_raw = struct.unpack('<h', data[i+6:i+8])[0]
                    self.gyro_z = gyro_z_raw / 32768.0 * 2000.0  # deg/s
                
                # Angle packet (0x53)
                elif packet_type == 0x53 and i + 10 < len(data):
                    # Skip X,Y axes, get Z axis angle (yaw)
                    yaw_raw = struct.unpack('<h', data[i+6:i+8])[0]
                    self.yaw_angle = yaw_raw / 32768.0 * 180.0  # degrees
                    
                    # Update timing for Hz calculation
                    current_time = time.time()
                    if self.start_time == 0:
                        self.start_time = current_time
                    
                    self.update_count += 1
                    elapsed_time = current_time - self.start_time
                    
                    # Calculate data rate every second
                    if elapsed_time >= 1.0:
                        self.data_rate = self.update_count / elapsed_time
                        self.update_count = 0
                        self.start_time = current_time
    
    def read_data_continuous(self):
        """Continuously read and parse data"""
        print("Starting continuous data reading...")
        print("Press Ctrl+C to stop\n")
        
        while self.running:
            try:
                if self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting)
                    self.parse_data(data)
                time.sleep(0.01)  # Small delay to prevent excessive CPU usage
            except Exception as e:
                print(f"Error reading data: {e}")
                break
    
    def display_data_continuous(self):
        """Display data continuously"""
        while self.running:
            try:
                # Clear line and print current values
                print(f"\rGyro Z: {self.gyro_z:+8.2f}°/s | Yaw: {self.yaw_angle:+8.2f}° | Rate: {self.data_rate:5.1f}Hz", end="", flush=True)
                time.sleep(0.1)  # Update display at 10Hz
            except KeyboardInterrupt:
                break
    
    def start_reading(self):
        """Start reading data in separate thread"""
        if not self.connect():
            return False
        
        self.running = True
        
        # Start reading thread
        read_thread = threading.Thread(target=self.read_data_continuous, daemon=True)
        read_thread.start()
        
        try:
            # Display data in main thread
            self.display_data_continuous()
        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            self.disconnect()
        
        return True
    
    def read_single_sample(self):
        """Read a single data sample"""
        if not self.connect():
            return False
        
        print("Reading single sample...")
        timeout_count = 0
        max_timeout = 50  # 5 seconds max
        
        try:
            while timeout_count < max_timeout:
                if self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting)
                    self.parse_data(data)
                    
                    # Check if we got both gyro and angle data
                    if self.gyro_z != 0 or self.yaw_angle != 0:
                        print(f"Gyro Z: {self.gyro_z:+8.2f}°/s")
                        print(f"Yaw:    {self.yaw_angle:+8.2f}°")
                        self.disconnect()
                        return True
                
                time.sleep(0.1)
                timeout_count += 1
            
            print("Timeout - no data received")
            self.disconnect()
            return False
            
        except Exception as e:
            print(f"Error: {e}")
            self.disconnect()
            return False

def main():
    import sys
    
    # Change this to your serial port
    SERIAL_PORT = "/dev/ttyUSB0"  # or "COM7" on Windows
    
    print("HWT101CT Gyro Yaw and Hz Reader")
    print("=" * 35)
    
    reader = HWT101CTReader(SERIAL_PORT)
    
    if len(sys.argv) > 1:
        command = sys.argv[1].lower()
        
        if command == "single":
            reader.read_single_sample()
        elif command == "continuous":
            reader.start_reading()
        else:
            print("Usage:")
            print("  python reader.py single     - Read single sample")
            print("  python reader.py continuous - Continuous reading")
    else:
        # Interactive menu
        print("Select mode:")
        print("1. Single sample")
        print("2. Continuous reading")
        print("3. Exit")
        
        choice = input("\nEnter choice (1-3): ").strip()
        
        if choice == "1":
            reader.read_single_sample()
        elif choice == "2":
            reader.start_reading()
        elif choice == "3":
            print("Goodbye!")
        else:
            print("Invalid choice!")

if __name__ == "__main__":
    main()