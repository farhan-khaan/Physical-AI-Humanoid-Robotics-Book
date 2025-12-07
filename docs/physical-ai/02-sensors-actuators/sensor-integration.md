---
title: Sensor Integration
description: Combining multiple sensors for robust perception through sensor fusion
sidebar_position: 4
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# Sensor Integration

Individual sensors are imperfect—they have noise, drift, limited range, and can fail. Sensor integration and fusion combine data from multiple sensors to create a more accurate, robust, and complete understanding of the robot's state and environment.

## 🎯 Learning Outcomes

By the end of this section, you will be able to:

1. **Explain** why sensor fusion is necessary for robust robotics
2. **Implement** complementary filters for IMU data fusion
3. **Understand** Kalman filtering concepts and applications
4. **Transform** data between different coordinate frames
5. **Synchronize** data from multiple sensors
6. **Design** multi-sensor architectures for humanoid robots

## 📋 Prerequisites

- Understanding of sensor types (previous sections)
- Basic linear algebra (vectors, matrices)
- Python programming
- Familiarity with coordinate systems

---

## 🤝 Why Sensor Fusion?

### The Problem with Single Sensors

Each sensor has limitations:

| Sensor | Strength | Weakness |
|--------|----------|----------|
| Camera | Rich visual data | Fails in darkness, computationally expensive |
| LIDAR | Accurate distances | Expensive, can't see through glass |
| IMU | Fast orientation | Drifts over time, no position |
| GPS | Absolute position | No indoor coverage, slow update |
| Encoders | Precise relative motion | Accumulates error (wheel slip) |

:::tip The Power of Fusion
By combining sensors, we can use each sensor's strengths to compensate for others' weaknesses!
:::

### Sensor Fusion Benefits

1. **Accuracy**: Combining measurements reduces overall error
2. **Robustness**: System still works if one sensor fails
3. **Completeness**: Different sensors measure different things
4. **Redundancy**: Cross-validation detects sensor faults
5. **Confidence**: Statistical fusion provides uncertainty estimates

---

## 🔄 Complementary Filters

The simplest sensor fusion technique: combine fast-changing data from one sensor with slow-changing data from another.

### IMU Example: Accelerometer + Gyroscope

**Problem**: 
- Gyroscope measures rotation rate (fast, no drift short-term, drifts long-term)
- Accelerometer measures orientation via gravity (slow, noisy, no drift long-term)

**Solution**: Complementary filter combines both!

<Tabs groupId="programming-language">
<TabItem value="python" label="Python" default>

```python
import numpy as np
import time

class ComplementaryFilter:
    """
    Fuse accelerometer and gyroscope for orientation estimation.
    """
    
    def __init__(self, alpha=0.98):
        """
        alpha: Complementary filter coefficient (0-1)
               0.98 means 98% gyro, 2% accel
        """
        self.alpha = alpha
        self.roll = 0.0
        self.pitch = 0.0
        self.last_time = time.time()
    
    def update(self, accel, gyro):
        """
        Update orientation estimate.
        
        accel: (ax, ay, az) in m/s²
        gyro: (gx, gy, gz) in rad/s
        """
        # Calculate time step
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Gyroscope integration (fast, drifts)
        self.roll += gyro[0] * dt
        self.pitch += gyro[1] * dt
        
        # Accelerometer angles (slow, noisy, no drift)
        accel_roll = np.arctan2(accel[1], accel[2])
        accel_pitch = np.arctan2(-accel[0], 
                                  np.sqrt(accel[1]**2 + accel[2]**2))
        
        # Complementary fusion
        self.roll = self.alpha * self.roll + (1 - self.alpha) * accel_roll
        self.pitch = self.alpha * self.pitch + (1 - self.alpha) * accel_pitch
        
        return self.roll, self.pitch
    
    def get_angles_degrees(self):
        """Get current angles in degrees."""
        return np.degrees(self.roll), np.degrees(self.pitch)


# Example usage with IMU
import board
import busio
import adafruit_bno055

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)

filter = ComplementaryFilter(alpha=0.98)

print("Fusing IMU data...")
for _ in range(100):
    # Read raw sensor data
    accel = sensor.acceleration  # (x, y, z) in m/s²
    gyro = sensor.gyro           # (x, y, z) in rad/s
    
    # Fuse data
    roll, pitch = filter.update(accel, gyro)
    roll_deg, pitch_deg = filter.get_angles_degrees()
    
    print(f"Roll: {roll_deg:6.2f}°, Pitch: {pitch_deg:6.2f}°")
    
    time.sleep(0.01)  # 100 Hz update rate
```

</TabItem>
<TabItem value="cpp" label="C++">

```cpp
#include <cmath>
#include <chrono>

class ComplementaryFilter {
private:
    double alpha;
    double roll, pitch;
    std::chrono::steady_clock::time_point last_time;
    
public:
    ComplementaryFilter(double alpha = 0.98) 
        : alpha(alpha), roll(0.0), pitch(0.0) {
        last_time = std::chrono::steady_clock::now();
    }
    
    void update(double accel[3], double gyro[3]) {
        // Calculate dt
        auto current_time = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(
            current_time - last_time).count();
        last_time = current_time;
        
        // Gyroscope integration
        roll += gyro[0] * dt;
        pitch += gyro[1] * dt;
        
        // Accelerometer angles
        double accel_roll = atan2(accel[1], accel[2]);
        double accel_pitch = atan2(-accel[0], 
            sqrt(accel[1]*accel[1] + accel[2]*accel[2]));
        
        // Complementary fusion
        roll = alpha * roll + (1 - alpha) * accel_roll;
        pitch = alpha * pitch + (1 - alpha) * accel_pitch;
    }
    
    double getRoll() const { return roll; }
    double getPitch() const { return pitch; }
};
```

</TabItem>
</Tabs>

:::note Alpha Parameter
- **α ≈ 1.0** (e.g., 0.98): Trust gyro more, smooth but may drift
- **α ≈ 0.5**: Equal trust, more responsive but noisier
- **α ≈ 0.0** (e.g., 0.02): Trust accel more, no drift but very noisy

Typical value: 0.96-0.98 for humanoid robots
:::

---

## 🎯 Kalman Filters

The Kalman filter is the gold standard for sensor fusion—it optimally combines measurements with a motion model.

### Kalman Filter Concept

1. **Predict**: Use motion model to predict next state
2. **Update**: Correct prediction using sensor measurements
3. **Repeat**: Continuously predict and update

### Simple 1D Position Example

```python
import numpy as np

class KalmanFilter1D:
    """
    Simple 1D Kalman filter for position tracking.
    """
    
    def __init__(self, process_variance=1e-5, measurement_variance=0.1):
        # State
        self.x = 0.0      # Position estimate
        self.P = 1.0      # Estimate uncertainty
        
        # Noise parameters
        self.Q = process_variance      # Process noise
        self.R = measurement_variance  # Measurement noise
    
    def predict(self, dt, velocity):
        """
        Predict next state based on velocity.
        """
        # State prediction: x = x + v * dt
        self.x = self.x + velocity * dt
        
        # Uncertainty increases
        self.P = self.P + self.Q
    
    def update(self, measurement):
        """
        Update estimate with new measurement.
        """
        # Kalman Gain
        K = self.P / (self.P + self.R)
        
        # Update estimate
        self.x = self.x + K * (measurement - self.x)
        
        # Update uncertainty
        self.P = (1 - K) * self.P
    
    def get_state(self):
        """Get current position estimate."""
        return self.x, self.P


# Example: Fuse noisy position sensor with velocity
kf = KalmanFilter1D(process_variance=1e-5, measurement_variance=0.1)

# Simulate robot moving at 1 m/s
true_position = 0.0
velocity = 1.0
dt = 0.1

for t in range(100):
    # Simulate movement
    true_position += velocity * dt
    
    # Noisy measurement
    measurement = true_position + np.random.normal(0, 0.3)
    
    # Kalman filter predict and update
    kf.predict(dt, velocity)
    kf.update(measurement)
    
    estimate, uncertainty = kf.get_state()
    
    print(f"t={t*dt:.1f}s: True={true_position:.2f}, "
          f"Measured={measurement:.2f}, "
          f"Estimate={estimate:.2f} ±{np.sqrt(uncertainty):.2f}")
```

### Extended Kalman Filter (EKF)

For nonlinear systems (most robots!), we use the Extended Kalman Filter:

```python
import numpy as np

class ExtendedKalmanFilter:
    """
    EKF for nonlinear state estimation.
    Example: 2D robot pose (x, y, theta)
    """
    
    def __init__(self):
        # State: [x, y, theta]
        self.x = np.zeros(3)
        
        # Covariance matrix
        self.P = np.eye(3) * 0.1
        
        # Process noise
        self.Q = np.diag([0.1, 0.1, 0.05])**2
        
        # Measurement noise (for GPS-like sensor)
        self.R = np.diag([0.5, 0.5])**2
    
    def predict(self, velocity, angular_velocity, dt):
        """
        Predict next state based on control inputs.
        """
        x, y, theta = self.x
        
        # Nonlinear motion model
        self.x[0] = x + velocity * np.cos(theta) * dt
        self.x[1] = y + velocity * np.sin(theta) * dt
        self.x[2] = theta + angular_velocity * dt
        
        # Jacobian of motion model
        F = np.array([
            [1, 0, -velocity * np.sin(theta) * dt],
            [0, 1,  velocity * np.cos(theta) * dt],
            [0, 0,  1]
        ])
        
        # Predict covariance
        self.P = F @ self.P @ F.T + self.Q
    
    def update(self, measurement):
        """
        Update with position measurement [x, y].
        """
        # Measurement model: we observe x and y directly
        H = np.array([
            [1, 0, 0],
            [0, 1, 0]
        ])
        
        # Innovation
        z_pred = H @ self.x
        y = measurement - z_pred
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state
        self.x = self.x + K @ y
        
        # Update covariance
        self.P = (np.eye(3) - K @ H) @ self.P
    
    def get_state(self):
        """Get current state estimate."""
        return self.x.copy()


# Example usage
ekf = ExtendedKalmanFilter()

# Simulate robot driving in a circle
dt = 0.1
for t in range(100):
    # Control inputs
    velocity = 1.0  # m/s
    angular_velocity = 0.2  # rad/s
    
    # Predict
    ekf.predict(velocity, angular_velocity, dt)
    
    # Every 10 steps, get a GPS measurement
    if t % 10 == 0:
        true_x = np.sin(0.2 * t * dt)
        true_y = 1 - np.cos(0.2 * t * dt)
        measurement = np.array([true_x, true_y]) + np.random.normal(0, 0.5, 2)
        ekf.update(measurement)
    
    state = ekf.get_state()
    print(f"t={t*dt:.1f}s: x={state[0]:.2f}, y={state[1]:.2f}, θ={state[2]:.2f}")
```

:::tip When to Use Kalman Filters
- You have a mathematical model of system dynamics
- Sensors have Gaussian noise
- You need optimal estimates with uncertainty quantification
- Real-time performance is required
:::

---

## 📐 Coordinate Frame Transformations

Different sensors measure in different coordinate frames. We must transform data to a common reference frame.

### Transform Types

```python
import numpy as np

def rotation_matrix_z(theta):
    """Rotation matrix around Z-axis."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c, -s, 0],
        [s,  c, 0],
        [0,  0, 1]
    ])

def transform_point(point, translation, rotation_matrix):
    """
    Transform a point from one frame to another.
    
    point: 3D point [x, y, z]
    translation: 3D translation [tx, ty, tz]
    rotation_matrix: 3x3 rotation matrix
    """
    return rotation_matrix @ point + translation


# Example: Camera on robot's head
# Robot is at (1, 2, 0) facing 45 degrees
# Camera sees object at (2, 0, 1) in camera frame
# Where is the object in world frame?

robot_position = np.array([1.0, 2.0, 0.0])
robot_heading = np.radians(45)
camera_offset = np.array([0.1, 0.0, 0.3])  # Camera is 10cm forward, 30cm up

object_in_camera = np.array([2.0, 0.0, 1.0])

# Transform camera offset to world frame
R_robot = rotation_matrix_z(robot_heading)
camera_position = robot_position + R_robot @ camera_offset

# Transform object position to world frame
object_in_world = camera_position + R_robot @ object_in_camera

print(f"Object in world frame: {object_in_world}")
```

### TF (Transform) Tree in ROS

For complex robots with many coordinate frames, ROS uses TF:

```python
import rospy
import tf2_ros
import geometry_msgs.msg

class MultiFrameTracker:
    """Track transformations between multiple coordinate frames."""
    
    def __init__(self):
        rospy.init_node('frame_tracker')
        
        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
    
    def get_transform(self, from_frame, to_frame):
        """Get transformation from one frame to another."""
        try:
            # Wait up to 1 second for transform
            transform = self.tf_buffer.lookup_transform(
                to_frame, from_frame, rospy.Time(0), rospy.Duration(1.0))
            return transform
        except (tf2_ros.LookupException, 
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Transform lookup failed: {e}")
            return None
    
    def transform_point(self, point, from_frame, to_frame):
        """Transform a point between frames."""
        # Create PointStamped message
        point_stamped = geometry_msgs.msg.PointStamped()
        point_stamped.header.frame_id = from_frame
        point_stamped.point.x = point[0]
        point_stamped.point.y = point[1]
        point_stamped.point.z = point[2]
        
        try:
            # Transform point
            transformed = self.tf_buffer.transform(point_stamped, to_frame)
            return [transformed.point.x, 
                    transformed.point.y, 
                    transformed.point.z]
        except Exception as e:
            rospy.logerr(f"Point transform failed: {e}")
            return None

# Usage
tracker = MultiFrameTracker()

# Transform object from camera frame to world frame
object_in_camera = [1.0, 0.5, 2.0]
object_in_world = tracker.transform_point(
    object_in_camera, 
    from_frame='camera_frame', 
    to_frame='world')

print(f"Object in world: {object_in_world}")
```

---

## ⏱️ Sensor Synchronization

Different sensors have different update rates. We need to synchronize their data.

```python
import time
from collections import deque
from threading import Thread, Lock

class SensorSynchronizer:
    """Synchronize data from multiple sensors with different rates."""
    
    def __init__(self, max_time_diff=0.1):
        self.max_time_diff = max_time_diff
        self.buffers = {}
        self.lock = Lock()
    
    def add_sensor(self, sensor_name):
        """Register a new sensor."""
        with self.lock:
            self.buffers[sensor_name] = deque(maxlen=100)
    
    def add_measurement(self, sensor_name, data):
        """Add a measurement with timestamp."""
        with self.lock:
            timestamp = time.time()
            self.buffers[sensor_name].append((timestamp, data))
    
    def get_synchronized_data(self, reference_time=None):
        """
        Get synchronized data from all sensors.
        Returns data closest to reference_time.
        """
        if reference_time is None:
            reference_time = time.time()
        
        synchronized = {}
        
        with self.lock:
            for sensor_name, buffer in self.buffers.items():
                if not buffer:
                    continue
                
                # Find closest measurement
                closest = min(buffer, 
                            key=lambda x: abs(x[0] - reference_time))
                
                time_diff = abs(closest[0] - reference_time)
                
                if time_diff <= self.max_time_diff:
                    synchronized[sensor_name] = {
                        'data': closest[1],
                        'timestamp': closest[0],
                        'time_diff': time_diff
                    }
        
        return synchronized


# Example usage
sync = SensorSynchronizer(max_time_diff=0.05)  # 50ms tolerance

# Register sensors
sync.add_sensor('imu')
sync.add_sensor('camera')
sync.add_sensor('lidar')

# Simulate sensors running at different rates
def imu_thread():
    """100 Hz IMU"""
    while True:
        imu_data = {'orientation': [0, 0, 0], 'accel': [0, 0, 9.8]}
        sync.add_measurement('imu', imu_data)
        time.sleep(0.01)

def camera_thread():
    """30 Hz camera"""
    while True:
        camera_data = {'image': 'frame_data'}
        sync.add_measurement('camera', camera_data)
        time.sleep(0.033)

def lidar_thread():
    """10 Hz LIDAR"""
    while True:
        lidar_data = {'scan': [1, 2, 3, 4, 5]}
        sync.add_measurement('lidar', lidar_data)
        time.sleep(0.1)

# Start sensor threads
Thread(target=imu_thread, daemon=True).start()
Thread(target=camera_thread, daemon=True).start()
Thread(target=lidar_thread, daemon=True).start()

time.sleep(1)  # Let sensors collect data

# Get synchronized snapshot
data = sync.get_synchronized_data()
print("Synchronized data:")
for sensor, info in data.items():
    print(f"  {sensor}: {info['time_diff']*1000:.1f}ms old")
```

---

## 🏗️ Multi-Sensor Architecture

### Layered Sensor Fusion

```
┌─────────────────────────────────────┐
│   High-Level Perception Layer       │
│   (Object recognition, SLAM)        │
└──────────────┬──────────────────────┘
               ↓
┌─────────────────────────────────────┐
│   Mid-Level Fusion Layer            │
│   (Kalman filters, particle filters)│
└──────────────┬──────────────────────┘
               ↓
┌─────────────────────────────────────┐
│   Low-Level Sensor Layer            │
│   (IMU, cameras, LIDAR, encoders)   │
└─────────────────────────────────────┘
```

```python
class HumanoidSensorFusion:
    """Complete sensor fusion system for humanoid robot."""
    
    def __init__(self):
        # Low-level sensors
        self.imu = IMUSensor()
        self.camera = CameraSensor()
        self.encoders = JointEncoders()
        self.force_sensors = ForceSensors()
        
        # Mid-level fusion
        self.orientation_filter = ComplementaryFilter()
        self.position_ekf = ExtendedKalmanFilter()
        
        # Synchronizer
        self.sync = SensorSynchronizer()
        self.sync.add_sensor('imu')
        self.sync.add_sensor('camera')
        self.sync.add_sensor('encoders')
        
    def update(self):
        """Main sensor fusion update loop."""
        # Get synchronized sensor data
        data = self.sync.get_synchronized_data()
        
        if 'imu' in data:
            # Fuse IMU data for orientation
            accel = data['imu']['data']['accel']
            gyro = data['imu']['data']['gyro']
            orientation = self.orientation_filter.update(accel, gyro)
        
        if 'encoders' in data:
            # Update position estimate from odometry
            joint_angles = data['encoders']['data']
            velocity = self.compute_velocity_from_joints(joint_angles)
            self.position_ekf.predict(velocity, dt=0.01)
        
        if 'camera' in data:
            # Visual odometry or landmark detection
            features = self.extract_features(data['camera']['data'])
            position_measurement = self.estimate_position(features)
            self.position_ekf.update(position_measurement)
        
        return self.get_robot_state()
    
    def get_robot_state(self):
        """Get complete robot state estimate."""
        return {
            'orientation': self.orientation_filter.get_angles_degrees(),
            'position': self.position_ekf.get_state(),
            'confidence': self.position_ekf.P
        }
```

---

## ✅ Key Takeaways

1. **Sensor fusion** combines multiple sensors for accuracy and robustness
2. **Complementary filters** are simple and effective for IMU fusion
3. **Kalman filters** optimally combine predictions with measurements
4. **Coordinate transforms** align data from different sensor frames
5. **Synchronization** handles sensors with different update rates
6. **Layered architectures** organize complex sensor systems
7. **Always validate** sensor data and handle failures gracefully

---

## 🧪 Practice Exercise

:::note Exercise: IMU-Based Balance Monitor
**Difficulty**: Medium  
**Time**: 1.5 hours  
**Goal**: Create a balance monitoring system using sensor fusion

**Requirements**:
1. Implement complementary filter for IMU data
2. Fuse accelerometer and gyroscope readings
3. Detect when robot is tilting beyond safe angles
4. Log data to file with timestamps
5. Visualize orientation in real-time (optional: 3D plot)

**Bonus Challenges**:
- Add Kalman filter implementation and compare results
- Fuse IMU with foot force sensors
- Implement fall detection algorithm
:::

---

## 📚 Further Reading

- "Probabilistic Robotics" by Thrun et al. (Chapters 2-3 on sensor models)
- "Modern Robotics" by Lynch & Park (Chapter on kinematics)
- [ROS TF2 Documentation](http://wiki.ros.org/tf2)
- ["Sensor Fusion on Android Devices"](https://www.youtube.com/watch?v=C7JQ7Rpwn2k) (concepts apply to robots)

---

**[← Previous: Actuator Types](./actuator-types.md)** | **[Next: Exercises →](./exercises.md)**
