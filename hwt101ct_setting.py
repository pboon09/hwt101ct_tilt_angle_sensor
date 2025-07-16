#!/usr/bin/env python3
# coding: UTF-8

import serial
import time
import struct

class HWT101CTController:
    def __init__(self, port="/dev/ttyUSB0", baud=230400):
        """Initialize connection to HWT101CT sensor"""
        self.port = port
        self.baud = baud
        self.ser = None
        
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
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Disconnected from sensor")
    
    def unlock_sensor(self):
        """Unlock sensor for configuration changes"""
        try:
            # Unlock key: FF AA 69 88 B5
            unlock_cmd = bytearray([0xFF, 0xAA, 0x69, 0x88, 0xB5])
            self.ser.write(unlock_cmd)
            time.sleep(0.1)
            print("Sensor unlocked for configuration")
            return True
        except Exception as e:
            print(f"Failed to unlock sensor: {e}")
            return False
    
    def write_register(self, reg, value):
        """Write register command to HWT101CT"""
        try:
            # HWT101CT register write format: 0xFF 0xAA [reg] [value_low] [value_high]
            cmd = bytearray([0xFF, 0xAA, reg, value & 0xFF, (value >> 8) & 0xFF])
            self.ser.write(cmd)
            time.sleep(0.2)  # Longer wait for command to process
            print(f"Wrote register 0x{reg:02X} = 0x{value:04X}")
            return True
        except Exception as e:
            print(f"Failed to write register: {e}")
            return False
    
    def reset_yaw(self):
        """Reset Z-axis (yaw) angle to zero"""
        print("Resetting yaw angle to zero...")
        
        # Must unlock before writing
        if not self.unlock_sensor():
            return False
            
        success = self.write_register(0x76, 0x00)
        
        if success:
            # Save settings after change
            time.sleep(0.1)
            self.save_settings()
            
        return success
    
    def set_data_rate(self, rate_hz):
        """Set sensor data output rate"""
        rate_map = {
            1: 0x03, 2: 0x04, 5: 0x05, 10: 0x06, 
            20: 0x07, 50: 0x08, 100: 0x09, 200: 0x0B
        }
        
        if rate_hz in rate_map:
            print(f"Setting data rate to {rate_hz}Hz...")
            
            # Must unlock before writing
            if not self.unlock_sensor():
                return False
                
            success = self.write_register(0x03, rate_map[rate_hz])
            
            if success:
                # Save settings after change
                time.sleep(0.1)
                self.save_settings()
                
            return success
        else:
            print(f"Unsupported rate: {rate_hz}Hz")
            print(f"Supported rates: {list(rate_map.keys())}")
            return False
    

    
    def save_settings(self):
        """Save current settings to sensor memory"""
        print("Saving settings to sensor memory...")
        success = self.write_register(0x00, 0x00)
        time.sleep(0.5)  # Wait longer for save operation
        return success

# Standalone functions
def reset_yaw_only(port="/dev/ttyUSB0", baud=230400):
    """Reset yaw only"""
    controller = HWT101CTController(port, baud)
    
    if controller.connect():
        success = controller.reset_yaw()
        controller.disconnect()
        
        if success:
            print("✅ Yaw reset successful!")
        else:
            print("❌ Yaw reset failed!")
        
        return success
    else:
        print("❌ Could not connect to sensor")
        return False

def set_hz_only(port="/dev/ttyUSB0", baud=230400):
    """Set data rate only"""
    controller = HWT101CTController(port, baud)
    
    # Show available rates
    available_rates = [1, 2, 5, 10, 20, 50, 100, 200]
    print("Available data rates (Hz):")
    for i, rate in enumerate(available_rates, 1):
        print(f"{i}. {rate}Hz")
    
    try:
        choice = input("\nSelect rate (1-8): ").strip()
        choice_num = int(choice)
        
        if 1 <= choice_num <= len(available_rates):
            selected_rate = available_rates[choice_num - 1]
            
            if controller.connect():
                success = controller.set_data_rate(selected_rate)
                controller.disconnect()
                
                if success:
                    print(f"✅ Data rate set to {selected_rate}Hz successfully!")
                else:
                    print(f"❌ Failed to set data rate to {selected_rate}Hz!")
                
                return success
            else:
                print("❌ Could not connect to sensor")
                return False
        else:
            print("❌ Invalid choice!")
            return False
            
    except ValueError:
        print("❌ Invalid input! Please enter a number.")
        return False
    except Exception as e:
        print(f"❌ Error: {e}")
        return False

# Example usage
if __name__ == "__main__":
    import sys
    
    # Change this to your serial port
    SERIAL_PORT = "/dev/ttyUSB0"  # or "COM7" on Windows
    
    print("HWT101CT Controller - Two Settings")
    print("=" * 37)
    
    if len(sys.argv) > 1:
        command = sys.argv[1].lower()
        
        if command == "reset":
            reset_yaw_only(SERIAL_PORT)
        elif command == "hz":
            set_hz_only(SERIAL_PORT)
        else:
            print("Usage:")
            print("  python hwt101ct_setting.py reset      - Reset yaw only")
            print("  python hwt101ct_setting.py hz         - Set data rate only")
    else:
        # Interactive menu
        while True:
            print("\nSelect option:")
            print("1. Reset yaw")
            print("2. Set data rate (Hz)")
            print("3. Exit")
            
            choice = input("\nEnter choice (1-3): ").strip()
            
            if choice == "1":
                reset_yaw_only(SERIAL_PORT)
            elif choice == "2":
                set_hz_only(SERIAL_PORT)
            elif choice == "3":
                print("Goodbye!")
                break
            else:
                print("❌ Invalid choice! Please enter 1-3.")