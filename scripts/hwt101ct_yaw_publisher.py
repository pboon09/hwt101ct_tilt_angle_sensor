#!/usr/bin/env python3
# coding: UTF-8

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import time
import math
import tf_transformations
import serial
import struct

class HWT101CTYawPublisher(Node):
    def __init__(self):
        super().__init__('hwt101ct_yaw_publisher')
        
        # Create publisher for IMU topic
        self.imu_publisher = self.create_publisher(Imu, '/hwt101ct_yaw_publisher', 10)
        
        # Serial configuration
        self.serial_port = "/dev/ttyUSB0"  # Change this to your port
        self.baud_rate = 230400
        
        # Initialize serial connection
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f'Connected to {self.serial_port} at {self.baud_rate} baud')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {str(e)}')
            return
        
        # Create timer to read data
        self.timer = self.create_timer(1.0 / 200.0, self.read_sensor_data)
        
        self.get_logger().info('HWT101CT-TTL IMU Publisher started')
        self.get_logger().info('Yaw data: CCW = positive, CW = negative, range: -180 to +180 degrees')

    def read_sensor_data(self):
        """Read data from serial port and parse HWT101CT format"""
        try:
            if self.ser.in_waiting > 0:
                data = self.ser.read(self.ser.in_waiting)
                self.parse_data(data)
        except Exception as e:
            self.get_logger().error(f'Error reading serial data: {str(e)}')

    def parse_data(self, data):
        """Parse HWT101CT data format"""
        # HWT101CT typically sends data in packets
        # This is a simplified parser - you might need to adjust based on your exact sensor format
        
        # Look for data packets (typically start with 0x55)
        for i in range(len(data) - 10):
            if data[i] == 0x55:
                packet_type = data[i + 1]
                
                # Angular velocity packet (0x52)
                if packet_type == 0x52 and i + 10 < len(data):
                    gyro_x = struct.unpack('<h', data[i+2:i+4])[0] / 32768.0 * 2000.0  # deg/s
                    gyro_y = struct.unpack('<h', data[i+4:i+6])[0] / 32768.0 * 2000.0  # deg/s
                    gyro_z = struct.unpack('<h', data[i+6:i+8])[0] / 32768.0 * 2000.0  # deg/s
                    
                    self.gyro_z = gyro_z
                
                # Angle packet (0x53)
                elif packet_type == 0x53 and i + 10 < len(data):
                    angle_x = struct.unpack('<h', data[i+2:i+4])[0] / 32768.0 * 180.0  # degrees
                    angle_y = struct.unpack('<h', data[i+4:i+6])[0] / 32768.0 * 180.0  # degrees
                    angle_z = struct.unpack('<h', data[i+6:i+8])[0] / 32768.0 * 180.0  # degrees
                    
                    self.angle_z = angle_z
                    
                    # When we get angle data, publish IMU message
                    if hasattr(self, 'gyro_z'):
                        self.publish_imu_data(self.gyro_z, self.angle_z)

    def publish_imu_data(self, gyro_z, angle_z):
        """Publish IMU data to ROS2 topic"""
        try:
            # Create IMU message
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "imu_link"
            
            # Convert angle from degrees to radians
            yaw_rad = math.radians(angle_z)
            
            # Convert Euler to quaternion using tf2
            quaternion = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw_rad)
            imu_msg.orientation.x = quaternion[0]
            imu_msg.orientation.y = quaternion[1]
            imu_msg.orientation.z = quaternion[2]
            imu_msg.orientation.w = quaternion[3]
            
            # Set angular velocity (convert deg/s to rad/s)
            imu_msg.angular_velocity.x = 0.0
            imu_msg.angular_velocity.y = 0.0
            imu_msg.angular_velocity.z = math.radians(gyro_z)
            
            # Set linear acceleration to zero (not used)
            imu_msg.linear_acceleration.x = 0.0
            imu_msg.linear_acceleration.y = 0.0
            imu_msg.linear_acceleration.z = 0.0
            
            # Set covariance matrices
            imu_msg.orientation_covariance[0] = 0.01
            imu_msg.angular_velocity_covariance[0] = 0.01
            imu_msg.linear_acceleration_covariance[0] = -1.0
            
            # Publish the message
            self.imu_publisher.publish(imu_msg)
            
            # Print data
            direction = "CCW" if angle_z > 0 else "CW" if angle_z < 0 else "CENTER"
            # print(f"Gyro Z: {gyro_z:.2f}°/s, Angle Z: {angle_z:.2f}° ({direction})")
            
        except Exception as e:
            self.get_logger().error(f'Error publishing IMU data: {str(e)}')

    def __del__(self):
        """Cleanup"""
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        imu_node = HWT101CTYawPublisher()
        rclpy.spin(imu_node)
    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        if 'imu_node' in locals():
            if hasattr(imu_node, 'ser') and imu_node.ser.is_open:
                imu_node.ser.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()