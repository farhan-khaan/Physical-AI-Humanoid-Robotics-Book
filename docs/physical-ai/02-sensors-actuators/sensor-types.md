---
title: Sensor Types
description: Comprehensive guide to sensors used in humanoid robotics
sidebar_position: 2
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
import ChapterActions from '@site/src/components/ChapterActions';

# Sensor Types

<ChapterActions 
  chapterId="02-sensors-actuators-sensor-types" 
  chapterTitle="Sensor Types"
/>

Sensors are the robot's window to the world. They provide the raw data that enables perception, state estimation, and decision-making. In this section, we'll explore the wide variety of sensors used in humanoid robotics.

## 🎯 Learning Outcomes

By the end of this section, you will be able to:

1. **Identify** different sensor types and their characteristics
2. **Explain** the difference between exteroceptive and proprioceptive sensors
3. **Select** appropriate sensors for specific robotics tasks
4. **Understand** sensor specifications like range, accuracy, and update rate
5. **Implement** code to read data from common sensors

## 📋 Prerequisites

- Understanding of the sense-think-act loop (Chapter 1)
- Basic Python programming
- Familiarity with coordinate systems and units

---

## 🔍 Two Categories of Sensors

Before diving into specific sensors, it's important to understand the two main categories:

### Exteroceptive Sensors (World-Sensing)

These sensors perceive the **external environment** around the robot:
- What objects are nearby?
- What does the scene look like?
- Are there obstacles in the path?
- What sounds are present?

### Proprioceptive Sensors (Self-Sensing)

These sensors perceive the **robot's own state**:
- What is my body position?
- How fast am I moving?
- What forces am I experiencing?
- How much current are my motors drawing?

:::tip Why Both Matter
A humanoid robot needs both types! Proprioceptive sensors tell you where your body is, while exteroceptive sensors tell you where everything else is. Together, they enable embodied intelligence.
:::

---

## 👁️ Exteroceptive Sensors

### 1. Cameras

Cameras provide rich visual information, making them one of the most versatile sensors in robotics.

#### RGB Cameras
**What they measure**: Color images of the environment  
**Typical resolution**: 640×480 to 4K (3840×2160)  
**Frame rate**: 30-120 Hz  
**Range**: Limited by lighting and optics (typically 0.5-10m effectively)

**Use cases**:
- Object detection and recognition
- Face tracking
- Visual servoing (using vision to guide motion)
- Scene understanding

<Tabs groupId="programming-language">
<TabItem value="python" label="Python" default>

```python
import cv2

# Open camera
camera = cv2.VideoCapture(0)

while True:
    # Capture frame
    ret, frame = camera.read()
    
    if ret:
        # Display frame
        cv2.imshow('Robot Camera View', frame)
        
        # Get frame dimensions
        height, width, channels = frame.shape
        print(f"Frame: {width}x{height}, {channels} channels")
    
    # Exit on 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

camera.release()
cv2.destroyAllWindows()
```

</TabItem>
<TabItem value="cpp" label="C++">

```cpp
#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // Open camera
    cv::VideoCapture camera(0);
    
    if (!camera.isOpened()) {
        std::cerr << "Cannot open camera" << std::endl;
        return -1;
    }
    
    cv::Mat frame;
    while (true) {
        // Capture frame
        camera >> frame;
        
        if (!frame.empty()) {
            // Display frame
            cv::imshow("Robot Camera View", frame);
            
            std::cout << "Frame: " << frame.cols << "x" 
                      << frame.rows << std::endl;
        }
        
        // Exit on 'q' key
        if (cv::waitKey(1) == 'q') break;
    }
    
    camera.release();
    cv::destroyAllWindows();
    return 0;
}
```

</TabItem>
</Tabs>

#### Depth Cameras

**What they measure**: Distance to each pixel in the image  
**Technologies**: Stereo vision, structured light (Kinect), Time-of-Flight (RealSense)  
**Range**: 0.3-10m (varies by technology)  
**Frame rate**: 30-90 Hz

**Use cases**:
- 3D scene reconstruction
- Obstacle detection
- Grasp planning
- SLAM (Simultaneous Localization and Mapping)

```python
import pyrealsense2 as rs
import numpy as np

# Configure RealSense depth camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

try:
    while True:
        # Wait for frames
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        
        if not depth_frame or not color_frame:
            continue
        
        # Convert to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        # Get distance at center pixel
        center_x, center_y = 320, 240
        distance = depth_frame.get_distance(center_x, center_y)
        print(f"Distance at center: {distance:.2f} meters")
        
finally:
    pipeline.stop()
```

:::note Depth Camera Technologies
- **Stereo**: Uses two cameras to compute depth (like human eyes)
- **Structured Light**: Projects patterns and analyzes deformation
- **Time-of-Flight**: Measures time for light to bounce back
:::

#### Event Cameras

**What they measure**: Per-pixel brightness changes asynchronously  
**Temporal resolution**: Microseconds  
**Dynamic range**: >120 dB (vs ~60 dB for traditional cameras)

**Use cases**:
- High-speed motion tracking
- Low-latency visual feedback
- Operating in challenging lighting

:::tip Emerging Technology
Event cameras are cutting-edge! They respond to motion instantly rather than capturing frames at fixed intervals, making them ideal for fast-moving humanoid robots.
:::

---

### 2. LIDAR (Light Detection and Ranging)

**What it measures**: Distance to objects using laser light  
**Range**: 0.1-100m+ (depending on model)  
**Accuracy**: ±2-5cm  
**Scan rate**: 5-40 Hz

**Types**:
- **2D LIDAR**: Single scanning plane (horizontal)
- **3D LIDAR**: Multiple planes or rotating head for full 3D

**Use cases**:
- Navigation and mapping
- Obstacle avoidance
- Perimeter detection
- Precise distance measurement

```python
# Example with RPLidar (popular 2D LIDAR)
from rplidar import RPLidar

lidar = RPLidar('/dev/ttyUSB0')

try:
    for scan in lidar.iter_scans():
        # Each scan is a list of (quality, angle, distance) tuples
        for quality, angle, distance in scan:
            if distance > 0:  # Valid measurement
                print(f"Angle: {angle:.1f}°, Distance: {distance:.1f}mm")
                
                # Check for obstacles in front (±10 degrees)
                if 350 <= angle or angle <= 10:
                    if distance < 500:  # Less than 50cm
                        print("⚠️ OBSTACLE AHEAD!")
                        
finally:
    lidar.stop()
    lidar.disconnect()
```

:::caution LIDAR Limitations
- Cannot detect transparent surfaces (glass)
- May struggle with very dark or reflective surfaces
- 2D LIDAR cannot see obstacles above/below scan plane
:::

---

### 3. Ultrasonic Sensors

**What they measure**: Distance using sound waves  
**Range**: 2cm-4m typically  
**Accuracy**: ±1cm  
**Update rate**: 10-50 Hz

**Use cases**:
- Close-range obstacle detection
- Floor detection
- Simple distance measurement
- Parking assistance

```python
import RPi.GPIO as GPIO
import time

# Setup pins
TRIG = 23  # Trigger pin
ECHO = 24  # Echo pin

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

def get_distance():
    """Measure distance with ultrasonic sensor."""
    # Send trigger pulse
    GPIO.output(TRIG, True)
    time.sleep(0.00001)  # 10 microseconds
    GPIO.output(TRIG, False)
    
    # Wait for echo
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()
    
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()
    
    # Calculate distance
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # Speed of sound / 2
    distance = round(distance, 2)
    
    return distance

# Measure distance
while True:
    dist = get_distance()
    print(f"Distance: {dist} cm")
    time.sleep(0.5)
```

---

### 4. Tactile and Force Sensors

**What they measure**: Physical contact and applied forces  
**Types**: FSR (Force Sensitive Resistors), capacitive, piezoelectric  
**Range**: 0-100N typically  
**Response time**: &lt;1ms

**Use cases**:
- Grasp force control
- Contact detection
- Surface texture sensing
- Safe human interaction

```python
import board
import busio
import adafruit_fxos8700

# Example: Reading force/torque sensor via I2C
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_fxos8700.FXOS8700(i2c)

while True:
    # Read accelerometer (can detect contact forces)
    accel_x, accel_y, accel_z = sensor.accelerometer
    
    # Calculate total force magnitude
    force_magnitude = (accel_x**2 + accel_y**2 + accel_z**2)**0.5
    
    print(f"Force: {force_magnitude:.2f} m/s²")
    
    # Detect contact
    if force_magnitude > 12.0:  # Threshold above gravity
        print("⚠️ CONTACT DETECTED!")
    
    time.sleep(0.1)
```

:::warning Safety Critical
Tactile sensors are essential for safe human-robot interaction. Always implement force limits to prevent injury!
:::

---

### 5. Microphones

**What they measure**: Sound pressure waves  
**Frequency range**: 20 Hz - 20 kHz (human hearing)  
**Sample rate**: 16-48 kHz typical

**Use cases**:
- Voice commands
- Sound localization
- Auditory scene analysis
- Human-robot interaction

```python
import pyaudio
import numpy as np

# Audio configuration
CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100

# Initialize PyAudio
p = pyaudio.PyAudio()
stream = p.open(format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                frames_per_chunk=CHUNK)

print("Listening...")

try:
    while True:
        # Read audio data
        data = stream.read(CHUNK)
        
        # Convert to numpy array
        audio_data = np.frombuffer(data, dtype=np.int16)
        
        # Calculate volume (RMS)
        volume = np.sqrt(np.mean(audio_data**2))
        
        print(f"Volume: {volume:.0f}", end='\r')
        
        # Detect loud sounds
        if volume > 1000:
            print("\n🔊 LOUD SOUND DETECTED!")
            
finally:
    stream.stop_stream()
    stream.close()
    p.terminate()
```

---

## 🤖 Proprioceptive Sensors

### 1. Encoders

**What they measure**: Joint position/angle or wheel rotation  
**Types**: Incremental (relative), Absolute (exact position)  
**Resolution**: 100-10,000+ counts per revolution  
**Accuracy**: ±0.1-1°

**Use cases**:
- Joint angle measurement
- Velocity calculation
- Position control
- Odometry

<Tabs groupId="programming-language">
<TabItem value="python" label="Python" default>

```python
import RPi.GPIO as GPIO

class RotaryEncoder:
    """Simple rotary encoder reader."""
    
    def __init__(self, pin_a, pin_b):
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.position = 0
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        GPIO.add_event_detect(pin_a, GPIO.BOTH, callback=self._update)
        
    def _update(self, channel):
        """Callback for encoder state change."""
        a_state = GPIO.input(self.pin_a)
        b_state = GPIO.input(self.pin_b)
        
        if a_state == b_state:
            self.position += 1
        else:
            self.position -= 1
    
    def get_position(self):
        """Get current encoder position."""
        return self.position
    
    def get_angle(self, counts_per_rev=1000):
        """Get angle in degrees."""
        return (self.position / counts_per_rev) * 360.0

# Usage
encoder = RotaryEncoder(pin_a=17, pin_b=18)

while True:
    position = encoder.get_position()
    angle = encoder.get_angle()
    print(f"Position: {position}, Angle: {angle:.2f}°")
    time.sleep(0.1)
```

</TabItem>
<TabItem value="cpp" label="C++">

```cpp
#include <iostream>
#include <wiringPi.h>

class RotaryEncoder {
private:
    int pin_a, pin_b;
    volatile int position;
    
    static void updateISR() {
        // Interrupt service routine
        // Handle encoder state change
    }
    
public:
    RotaryEncoder(int a, int b) : pin_a(a), pin_b(b), position(0) {
        wiringPiSetupGpio();
        pinMode(pin_a, INPUT);
        pinMode(pin_b, INPUT);
        pullUpDnControl(pin_a, PUD_UP);
        pullUpDnControl(pin_b, PUD_UP);
        
        wiringPiISR(pin_a, INT_EDGE_BOTH, &updateISR);
    }
    
    int getPosition() const {
        return position;
    }
    
    double getAngle(int countsPerRev = 1000) const {
        return (position / static_cast<double>(countsPerRev)) * 360.0;
    }
};
```

</TabItem>
</Tabs>

---

### 2. IMU (Inertial Measurement Unit)

An IMU combines multiple sensors to measure orientation and motion:

**Components**:
- **Accelerometer**: Measures linear acceleration (3 axes)
- **Gyroscope**: Measures angular velocity (3 axes)  
- **Magnetometer**: Measures magnetic field direction (3 axes)

**Update rate**: 100-1000 Hz  
**Accuracy**: Varies widely (consumer to aerospace grade)

**Use cases**:
- Balance control
- Orientation estimation
- Fall detection
- Motion tracking

<Tabs groupId="programming-language">
<TabItem value="python" label="Python" default>

```python
import board
import busio
import adafruit_bno055
import time

# Initialize I2C and BNO055 IMU
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)

print("IMU Calibration Status:")
while True:
    # Read calibration status (0-3 for each, 3 = fully calibrated)
    sys, gyro, accel, mag = sensor.calibration_status
    print(f"System: {sys}, Gyro: {gyro}, Accel: {accel}, Mag: {mag}")
    
    if sys == 3 and gyro == 3 and accel == 3 and mag == 3:
        print("✅ Fully calibrated!")
        break
    
    time.sleep(1)

print("\nReading IMU data:")
while True:
    # Read orientation (Euler angles)
    heading, roll, pitch = sensor.euler
    
    # Read angular velocity
    gyro_x, gyro_y, gyro_z = sensor.gyro
    
    # Read linear acceleration
    accel_x, accel_y, accel_z = sensor.acceleration
    
    print(f"Orientation - Heading: {heading:.1f}°, Roll: {roll:.1f}°, Pitch: {pitch:.1f}°")
    print(f"Gyro: X={gyro_x:.2f}, Y={gyro_y:.2f}, Z={gyro_z:.2f} rad/s")
    print(f"Accel: X={accel_x:.2f}, Y={accel_y:.2f}, Z={accel_z:.2f} m/s²")
    print("---")
    
    # Detect if robot is falling
    if abs(roll) > 30 or abs(pitch) > 30:
        print("⚠️ WARNING: Robot may be falling!")
    
    time.sleep(0.1)
```

</TabItem>
<TabItem value="cpp" label="C++">

```cpp
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class IMUReader {
private:
    ros::NodeHandle nh;
    ros::Subscriber imu_sub;
    
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        // Extract orientation (quaternion)
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w
        );
        
        // Convert to Euler angles
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        // Extract angular velocity
        double gyro_x = msg->angular_velocity.x;
        double gyro_y = msg->angular_velocity.y;
        double gyro_z = msg->angular_velocity.z;
        
        // Extract linear acceleration
        double accel_x = msg->linear_acceleration.x;
        double accel_y = msg->linear_acceleration.y;
        double accel_z = msg->linear_acceleration.z;
        
        ROS_INFO("Roll: %.2f, Pitch: %.2f, Yaw: %.2f", 
                 roll, pitch, yaw);
        
        // Detect falling
        if (std::abs(roll) > 0.52 || std::abs(pitch) > 0.52) {  // 30 degrees
            ROS_WARN("Robot may be falling!");
        }
    }
    
public:
    IMUReader() {
        imu_sub = nh.subscribe("/imu/data", 10, 
                               &IMUReader::imuCallback, this);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_reader");
    IMUReader reader;
    ros::spin();
    return 0;
}
```

</TabItem>
</Tabs>

:::tip IMU Calibration
Always calibrate your IMU! Move it in a figure-8 pattern to calibrate the magnetometer, and place it flat for accelerometer calibration.
:::

---

### 3. Force/Torque Sensors

**What they measure**: Forces (Fx, Fy, Fz) and torques (Tx, Ty, Tz) in 6 DOF  
**Range**: 1N-5000N depending on model  
**Resolution**: 0.01-1N  
**Frequency**: 1000-7000 Hz

**Use cases**:
- Grasp force control
- Contact force measurement
- Compliance control
- Tool interaction

```python
# Example with ATI Force/Torque sensor via network
import socket
import struct

class ForceTorqueSensor:
    """Read data from ATI Force/Torque sensor."""
    
    def __init__(self, ip='192.168.1.1', port=49152):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.connect((ip, port))
    
    def read(self):
        """Read force/torque data."""
        # Request data
        self.socket.send(b'\x12\x34')
        
        # Receive response (6 values: Fx, Fy, Fz, Tx, Ty, Tz)
        data = self.socket.recv(1024)
        values = struct.unpack('!6i', data[:24])
        
        # Convert to physical units (scale factors vary by sensor)
        scale = 1000000.0
        fx, fy, fz = [v/scale for v in values[0:3]]
        tx, ty, tz = [v/scale for v in values[3:6]]
        
        return {
            'force': (fx, fy, fz),
            'torque': (tx, ty, tz)
        }

# Usage
sensor = ForceTorqueSensor()

while True:
    data = sensor.read()
    fx, fy, fz = data['force']
    tx, ty, tz = data['torque']
    
    print(f"Force: X={fx:.2f}N, Y={fy:.2f}N, Z={fz:.2f}N")
    print(f"Torque: X={tx:.3f}Nm, Y={ty:.3f}Nm, Z={tz:.3f}Nm")
    
    # Check for excessive force
    force_magnitude = (fx**2 + fy**2 + fz**2)**0.5
    if force_magnitude > 50.0:
        print("⚠️ HIGH FORCE DETECTED!")
    
    time.sleep(0.01)
```

---

### 4. Current Sensors

**What they measure**: Electrical current drawn by motors  
**Range**: 0-30A typical for hobby servos, 0-100A+ for large motors  
**Accuracy**: ±1-5%

**Use cases**:
- Motor load estimation
- Collision detection (current spike)
- Power monitoring
- Thermal protection

```python
import board
import busio
from adafruit_ina219 import INA219

# Initialize I2C and INA219 current sensor
i2c = busio.I2C(board.SCL, board.SDA)
sensor = INA219(i2c)

print("Monitoring motor current...")

max_current = 0
while True:
    # Read current and voltage
    current_ma = sensor.current
    voltage_v = sensor.bus_voltage
    power_mw = sensor.power
    
    print(f"Current: {current_ma:.1f}mA, Voltage: {voltage_v:.2f}V, Power: {power_mw:.1f}mW")
    
    # Track peak current
    if current_ma > max_current:
        max_current = current_ma
        print(f"  New peak current: {max_current:.1f}mA")
    
    # Detect collision (current spike)
    if current_ma > 2000:  # 2A threshold
        print("⚠️ COLLISION DETECTED! (High current)")
        # Emergency stop would go here
    
    time.sleep(0.1)
```

---

## 📊 Sensor Comparison Table

| Sensor Type | Range | Accuracy | Update Rate | Cost | Power |
|-------------|-------|----------|-------------|------|-------|
| RGB Camera | 0.5-10m | High | 30-120 Hz | $ | Medium |
| Depth Camera | 0.3-10m | ±1-5cm | 30-90 Hz | $$ | Medium-High |
| LIDAR 2D | 0.1-40m | ±2-5cm | 5-40 Hz | $$$ | Medium |
| LIDAR 3D | 0.1-100m | ±2-5cm | 10-20 Hz | $$$$ | High |
| Ultrasonic | 0.02-4m | ±1cm | 10-50 Hz | $ | Low |
| IMU | N/A | Varies | 100-1000 Hz | $-$$ | Low |
| Encoder | N/A | ±0.1-1° | 1000+ Hz | $ | Low |
| Force/Torque | 0-1000N | ±0.5% | 1000+ Hz | $$$$ | Low |
| Tactile | 0-100N | ±5% | 100-1000 Hz | $-$$ | Low |

**Cost Legend**: $ = &lt;$50, $$ = $50-$200, $$$ = $200-$1000, $$$$ = &gt;$1000

---

## 🎯 Sensor Selection Guide

:::tip Choosing the Right Sensors

**For Navigation**:
- Primary: LIDAR (2D or 3D)
- Secondary: IMU, wheel encoders, cameras

**For Manipulation**:
- Primary: Cameras (RGB + depth), force/torque sensors
- Secondary: Tactile sensors, joint encoders

**For Balance**:
- Primary: IMU (high-frequency orientation)
- Secondary: Joint encoders, foot pressure sensors

**For Human Interaction**:
- Primary: Cameras, microphones, tactile sensors
- Secondary: Force sensors, proximity sensors

:::

### Key Considerations

1. **Task Requirements**: What does your robot need to perceive?
2. **Range**: How far away are the objects of interest?
3. **Accuracy**: How precise must the measurements be?
4. **Update Rate**: How fast is your control loop?
5. **Environmental Conditions**: Indoor/outdoor? Lighting? Weather?
6. **Cost**: Budget constraints?
7. **Power**: Battery life considerations?
8. **Integration**: Ease of interfacing and software support?

---

## ✅ Key Takeaways

1. **Exteroceptive sensors** perceive the environment; **proprioceptive sensors** perceive the robot's own state
2. **Cameras** provide rich visual data but require significant processing
3. **LIDAR** offers accurate distance measurements for navigation
4. **IMUs** are essential for balance and orientation estimation
5. **Encoders** track joint positions for precise control
6. **Force sensors** enable safe interaction and manipulation
7. **Sensor selection** depends on task, environment, and constraints
8. **Multiple sensors** working together (sensor fusion) provide robustness

---

## 🧪 Practice Exercise

:::note Exercise: Multi-Sensor Data Logger
**Difficulty**: Medium  
**Time**: 1 hour  
**Goal**: Create a data logger that reads from multiple sensors simultaneously

**Requirements**:
1. Read data from at least 2 different sensor types (e.g., IMU + ultrasonic)
2. Timestamp each sensor reading
3. Log data to a CSV file
4. Calculate and display update rates for each sensor
5. Detect and flag any sensor failures or anomalies

**Starter Code**:
```python
import time
import csv

class MultiSensorLogger:
    def __init__(self, sensors):
        self.sensors = sensors
        self.log_file = open('sensor_log.csv', 'w', newline='')
        self.writer = csv.writer(self.log_file)
        
        # Write header
        header = ['timestamp'] + [f'{name}_{field}' 
                                  for name, sensor in sensors.items()
                                  for field in sensor.get_fields()]
        self.writer.writerow(header)
    
    def log(self):
        """Log one sample from all sensors."""
        timestamp = time.time()
        row = [timestamp]
        
        for name, sensor in self.sensors.items():
            try:
                data = sensor.read()
                row.extend(data.values())
            except Exception as e:
                print(f"Error reading {name}: {e}")
                row.extend([None] * len(sensor.get_fields()))
        
        self.writer.writerow(row)
    
    def close(self):
        self.log_file.close()

# TODO: Implement this with your available sensors
```

**Challenge**: Add real-time plotting of sensor data using matplotlib!
:::

---

## 📚 Further Reading

- [ROS sensor_msgs documentation](http://docs.ros.org/en/api/sensor_msgs/html/index-msg.html)
- [Adafruit Sensor Learning Guides](https://learn.adafruit.com/category/sensors)
- "Probabilistic Robotics" by Thrun, Burgard, Fox (Chapter on Sensors)
- [OpenCV Camera Calibration Tutorial](https://docs.opencv.org/master/dc/dbb/tutorial_py_calibration.html)

---

## 🎓 Self-Check Questions

Before moving on, make sure you can answer:

1. What's the difference between exteroceptive and proprioceptive sensors?
2. When would you choose a LIDAR over a camera for obstacle detection?
3. What three sensors are typically combined in an IMU?
4. How can you detect a collision using motor current sensors?
5. Why is sensor calibration important?
6. What factors should you consider when selecting sensors for a robotics project?

---

**[← Previous: Chapter Introduction](./intro.md)** | **[Next: Actuator Types →](./actuator-types.md)**
