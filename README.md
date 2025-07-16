# HWT101CT_ROS2
This package is designed to work with the [HWT101CT Z-Axis 1000HZ IP68 Z-Axis 0.1° Crystal Inclinometer, MEMS Tilt Angle Sensor](https://witmotion-sensor.com/products/hwt101ct-485-military-grade-z-axis-inclinometer-mems-tilt-sensor-built-in-highly-integrated-crystal-gyroscope-rotation-angle-attitude-sensor-kalman-filter-algorithm-automatic-data-storage-multi-cascade-support?srsltid=AfmBOop23uGmYKPOYSWU4np2T_F-v-4aGuQ0VW2XzHW-MNPmrIoRcxlW). It provides a ROS2 Node that publishes IMU messages with yaw data, configuration tools for sensor settings, and a data reader for monitoring sensor output.

## Features
- Direct communication with HWT101CT sensor using HWT101 protocol
- ROS2 publisher for standard IMU messages with orientation and angular velocity
- Configurable data rates from 1Hz to 200Hz
- Yaw angle reset functionality
- Real-time data monitoring and visualization
- High-precision Z-axis angular measurements (0.1° accuracy)

## Dependencies
- ROS2 (tested on Humble)
- Python 3.8+
- pyserial
- tf_transformations

## Hardware Setup and Configuration
### HWT101CT Sensor Specifications
- **Model**: HWT101CT Z-Axis Crystal Inclinometer
- **Accuracy**: 0.1° static, 0.1° dynamic
- **Range**: ±180° (yaw angle), ±400°/s (angular velocity)
- **Data Rate**: 1-200Hz configurable
- **Interface**: Serial TTL (default 230400 baud)
- **Power**: 5V-36V
- **Protection**: IP68 waterproof rating

### Connecting the Sensor
1. **Power Connection**
   - Connect VCC to 5V-36V power supply
   - Connect GND to ground
   
2. **Serial Connection**
   - Connect TX to your system's RX pin
   - Connect RX to your system's TX pin
   - Default baud rate: 230400
   - Use USB-to-Serial converter if connecting to computer

3. **Port Configuration**
   - Identify the serial port (e.g., `/dev/ttyUSB0` on Linux, `COM4` on Windows)
   - Ensure proper permissions for serial port access

## Protocol Message
The package uses the official HWT101 Protocol for communication with the sensor. The protocol documentation is available in the [Google Drive folder](https://drive.google.com/drive/u/1/folders/1g312yYHV4Fw_BxPk_sSVF-beAON5poL2) along with the complete datasheet.

### Key Protocol Features
- UBX-style packet format with 0x55 header
- Angular velocity packets (0x52) for gyroscope data
- Angle packets (0x53) for orientation data
- Configuration commands with unlock sequence
- Automatic checksum validation

## Installation
### Building the Package
```bash
mkdir -p ~/hwt101ct_ws/src
cd ~/hwt101ct_ws/src

git clone <your-repo-url> hwt101ct_ros2

cd ~/hwt101ct_ws
colcon build --symlink-install --packages-select hwt101ct_ros2

source ~/hwt101ct_ws/install/setup.bash
```

## Usage

### 1. Sensor Configuration
Before using the ROS2 node, configure your sensor settings using the configuration tool:

```bash
python3 hwt101ct_setting.py
```

**Interactive Menu Options:**
1. **Reset yaw** - Reset Z-axis angle to zero
2. **Set data rate (Hz)** - Configure sensor update rate
3. **Exit**

**Available Data Rates:**
- 1Hz, 2Hz, 5Hz, 10Hz, 20Hz, 50Hz, 100Hz, 200Hz

**Command Line Usage:**
```bash
# Reset yaw angle to zero
python3 hwt101ct_setting.py reset

# Set data rate interactively
python3 hwt101ct_setting.py hz
```

### 2. Data Monitoring
Monitor sensor data in real-time using the data reader:

```bash
python3 hwt101ct_data_reader.py
```

**Interactive Menu Options:**
1. **Single sample** - Read one data sample
2. **Continuous reading** - Live data display
3. **Exit**

**Command Line Usage:**
```bash
# Read single sample
python3 hwt101ct_data_reader.py single

# Continuous monitoring
python3 hwt101ct_data_reader.py continuous
```

**Continuous Display Format:**
```
Gyro Z: +12.34°/s | Yaw: -45.67° | Rate: 10.0Hz
```

### 3. ROS2 Publisher Node
Start the IMU publisher node to stream data to ROS2:

```bash
ros2 run hwt101ct_ros2 hwt101ct_yaw_publisher.py
```

This will publish IMU data to the `/hwt101ct_yaw_publisher` topic as `sensor_msgs/Imu` messages.

**Published Topics:**
- `/hwt101ct_yaw_publisher` (sensor_msgs/Imu) - IMU data with orientation and angular velocity

**Message Content:**
- **header.frame_id**: "imu_link"
- **orientation**: Quaternion from Z-axis rotation (yaw only)
- **angular_velocity**: Z-axis angular velocity in rad/s
- **linear_acceleration**: Set to zero (not measured by this sensor)

### 4. Viewing Published Data
Monitor the published IMU data:

```bash
# View topic info
ros2 topic info /hwt101ct_yaw_publisher

# Echo messages
ros2 topic echo /hwt101ct_yaw_publisher

# Check publishing rate
ros2 topic hz /hwt101ct_yaw_publisher
```

## Configuration Parameters

### Serial Port Configuration
Default settings (modify in script if needed):
- **Port**: `/dev/ttyUSB0` (Linux) or `COM4` (Windows)
- **Baud Rate**: 230400
- **Timeout**: 1 second

### ROS2 Node Configuration
- **Node Name**: `hwt101ct_yaw_publisher`
- **Topic Name**: `/hwt101ct_yaw_publisher`
- **Frame ID**: `imu_link`
- **Timer Frequency**: 200Hz (adjustable)
- **Message Type**: `sensor_msgs/Imu`

## Data Format and Coordinate System
- **Yaw Angle Range**: -180° to +180°
- **Angular Velocity Range**: ±400°/s
- **Coordinate System**: 
  - Positive yaw: Counter-clockwise rotation
  - Negative yaw: Clockwise rotation
  - Zero reference: Can be set using reset function

## Troubleshooting

### Common Issues
1. **"Failed to connect" Error**
   - Check serial port permissions: `sudo chmod 666 /dev/ttyUSB0`
   - Verify correct port name
   - Ensure sensor is powered and connected

2. **"No data received" Error**
   - Check wiring connections (TX/RX swap)
   - Verify baud rate (default: 230400)
   - Ensure sensor is not in sleep mode

3. **Settings Not Saving**
   - Sensor requires unlock command before configuration
   - Settings are automatically saved after each change
   - Power cycle sensor to verify settings persistence

4. **Low/Incorrect Data Rate**
   - Use configuration tool to set desired rate
   - Check actual rate using data reader
   - Verify serial port can handle the data throughput

### Hardware Troubleshooting
- **Power LED**: Verify sensor power indicator is on
- **Serial Communication**: Use oscilloscope to check TX/RX signals
- **Environmental**: Ensure sensor is within operating temperature range

## Technical Specifications
- **Sensor Type**: MEMS Crystal Gyroscope
- **Measurement Axis**: Z-axis (yaw) only
- **Static Accuracy**: 0.05°
- **Dynamic Accuracy**: 0.1°
- **Stability**: 0.01°
- **Update Rate**: 1-200Hz configurable
- **Response Time**: Fast (suitable for real-time applications)
- **Operating Temperature**: -40°C to +85°C

## Documentation and Resources
- **Complete Datasheet**: [Google Drive Folder](https://drive.google.com/drive/u/1/folders/1g312yYHV4Fw_BxPk_sSVF-beAON5poL2)
- **Protocol Documentation**: HWT101 Protocol (available in Google Drive)
- **Product Page**: [WitMotion HWT101CT](https://witmotion-sensor.com/products/hwt101ct-485-military-grade-z-axis-inclinometer-mems-tilt-sensor-built-in-highly-integrated-crystal-gyroscope-rotation-angle-attitude-sensor-kalman-filter-algorithm-automatic-data-storage-multi-cascade-support?srsltid=AfmBOop23uGmYKPOYSWU4np2T_F-beAON5poL2)

## Feedback
If you have any feedback, please create an issue and I will answer your questions there.