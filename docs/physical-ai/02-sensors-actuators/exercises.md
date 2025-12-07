---
title: Exercises
description: Hands-on practice with sensors and actuators
sidebar_position: 5
---

# Chapter 2: Exercises

Put your knowledge into practice with these hands-on exercises! Each exercise builds on concepts from this chapter and prepares you for real-world robotics development.

## 🎯 Exercise Overview

| Exercise | Difficulty | Time | Focus Area |
|----------|-----------|------|------------|
| 1. IMU Data Logger | ⭐⭐☆☆☆ | 30 min | Sensors, Data Collection |
| 2. Servo Control System | ⭐⭐⭐☆☆ | 1 hour | Actuators, Control |
| 3. Sensor Fusion | ⭐⭐⭐⭐☆ | 2 hours | Integration, Filtering |
| 4. Multi-Sensor Logger | ⭐⭐⭐☆☆ | 1 hour | Synchronization |

---

## Exercise 1: IMU Data Visualization

:::note Exercise Details
**Difficulty**: ⭐⭐☆☆☆ Easy  
**Time**: 30 minutes  
**Concepts**: Sensor reading, data visualization  
**Hardware**: IMU sensor (or simulated data)
:::

### Goal
Read IMU data and create real-time plots of orientation, acceleration, and angular velocity.

### Requirements
1. Read data from an IMU sensor at 100 Hz
2. Display roll, pitch, and yaw angles
3. Plot acceleration on 3 axes
4. Create real-time updating graphs
5. Save data to CSV file

### Starter Code

```python
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

class IMUVisualizer:
    def __init__(self):
        self.times = []
        self.roll_data = []
        self.pitch_data = []
        self.yaw_data = []
        
        # Setup plot
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 8))
        
    def update_plot(self, frame):
        # TODO: Read IMU data here
        # imu_data = read_imu()
        
        # TODO: Update data lists
        # self.times.append(time.time())
        # self.roll_data.append(imu_data['roll'])
        
        # Plot orientation
        self.ax1.clear()
        self.ax1.plot(self.times, self.roll_data, label='Roll')
        self.ax1.plot(self.times, self.pitch_data, label='Pitch')
        self.ax1.legend()
        self.ax1.set_ylabel('Angle (degrees)')
        
        # Plot acceleration
        self.ax2.clear()
        # TODO: Plot acceleration data
        
    def run(self):
        ani = FuncAnimation(self.fig, self.update_plot, interval=50)
        plt.show()

# TODO: Complete the implementation
```

### Success Criteria
- [ ] IMU data reads successfully at 100 Hz
- [ ] Plots update in real-time without lag
- [ ] Data is saved to CSV file
- [ ] Graphs are properly labeled with units

### Bonus Challenge
- Add a 3D orientation visualization using a cube or arrow
- Implement data smoothing/filtering
- Detect and flag sensor anomalies

---

## Exercise 2: Servo Control System

:::note Exercise Details
**Difficulty**: ⭐⭐⭐☆☆ Medium  
**Time**: 1 hour  
**Concepts**: Actuator control, interpolation, position limits  
**Hardware**: 3+ servo motors
:::

### Goal
Create a multi-servo control system with smooth motion and safety limits.

### Requirements
1. Control at least 3 servos simultaneously
2. Implement smooth interpolation between positions
3. Add software position limits to prevent damage
4. Create a sequence of 5 predefined poses
5. Display current vs target positions for each servo

### Starter Code

```python
from adafruit_servokit import ServoKit
import time
import numpy as np

class ServoController:
    def __init__(self, num_servos=3):
        self.kit = ServoKit(channels=16)
        self.num_servos = num_servos
        
        # Safety limits (min, max) for each servo
        self.limits = [(0, 180) for _ in range(num_servos)]
        
        # Current positions
        self.current_positions = [90] * num_servos
    
    def set_limits(self, servo_id, min_angle, max_angle):
        """Set safety limits for a servo."""
        # TODO: Implement limit setting
        pass
    
    def move_to(self, target_positions, duration=1.0):
        """
        Smoothly move servos to target positions.
        target_positions: list of target angles
        duration: time to complete motion (seconds)
        """
        # TODO: Implement smooth interpolation
        # Use linear or cubic interpolation
        steps = 50
        dt = duration / steps
        
        for step in range(steps + 1):
            # TODO: Calculate intermediate positions
            # intermediate = interpolate(current, target, step/steps)
            # self.set_positions(intermediate)
            time.sleep(dt)
    
    def set_positions(self, positions):
        """Set servo positions with safety checks."""
        for i, pos in enumerate(positions):
            # TODO: Check limits and set position
            pass
    
    def get_positions(self):
        """Get current servo positions."""
        return self.current_positions.copy()

# Predefined poses
POSES = {
    'rest': [90, 90, 90],
    'reach_left': [45, 90, 135],
    'reach_right': [135, 90, 45],
    'wave': [90, 45, 135],
    'point': [90, 135, 90]
}

# TODO: Create servo controller and execute pose sequence
```

### Success Criteria
- [ ] All servos move smoothly without jerking
- [ ] Safety limits prevent out-of-range commands
- [ ] Pose sequence executes correctly
- [ ] System handles invalid inputs gracefully

### Bonus Challenge
- Add keyboard control for manual servo adjustment
- Implement collision avoidance between servos
- Record and playback motion sequences

---

## Exercise 3: Sensor Fusion

:::note Exercise Details
**Difficulty**: ⭐⭐⭐⭐☆ Hard  
**Time**: 2 hours  
**Concepts**: Complementary filter, Kalman filter, sensor fusion  
**Hardware**: IMU + encoder OR IMU + GPS
:::

### Goal
Fuse data from multiple sensors for accurate state estimation.

### Requirements
1. Implement complementary filter for IMU
2. Implement simple Kalman filter
3. Compare filtered vs raw sensor data
4. Plot estimation error over time
5. Handle sensor dropouts gracefully

### Starter Code

```python
import numpy as np

class SensorFusionComparison:
    def __init__(self):
        self.comp_filter = ComplementaryFilter(alpha=0.98)
        self.kalman_filter = KalmanFilter1D()
        
        self.raw_data = []
        self.comp_filtered = []
        self.kalman_filtered = []
        self.ground_truth = []
    
    def run_comparison(self, num_samples=1000):
        """Run comparison between filtering methods."""
        for i in range(num_samples):
            # TODO: Generate or read sensor data
            # accel, gyro = read_imu()
            # position_measurement = read_encoder()
            
            # TODO: Apply filters
            # comp_result = self.comp_filter.update(accel, gyro)
            # self.kalman_filter.predict(dt, velocity)
            # self.kalman_filter.update(position_measurement)
            
            # TODO: Store results for comparison
            pass
    
    def plot_results(self):
        """Plot comparison of filtering methods."""
        import matplotlib.pyplot as plt
        
        plt.figure(figsize=(12, 8))
        
        # TODO: Create comparison plots
        # - Raw sensor data
        # - Complementary filter output
        # - Kalman filter output
        # - Ground truth (if available)
        # - Estimation error for each method
        
        plt.show()
    
    def compute_metrics(self):
        """Compute RMSE and other metrics."""
        # TODO: Calculate:
        # - Root Mean Square Error
        # - Maximum error
        # - Convergence time
        pass

# TODO: Complete implementation and run comparison
```

### Success Criteria
- [ ] Both filters implemented correctly
- [ ] Complementary filter reduces noise
- [ ] Kalman filter provides optimal estimates
- [ ] Results are visualized and compared
- [ ] Metrics show quantitative improvement

### Bonus Challenge
- Implement Extended Kalman Filter
- Add particle filter implementation
- Test with real sensor failures
- Compare computational performance

---

## Exercise 4: Multi-Sensor Data Logger

:::note Exercise Details
**Difficulty**: ⭐⭐⭐☆☆ Medium  
**Time**: 1 hour  
**Concepts**: Multi-threading, synchronization, data logging  
**Hardware**: 2+ different sensor types
:::

### Goal
Create a synchronized data logger for multiple sensors with different update rates.

### Requirements
1. Read from at least 2 sensor types simultaneously
2. Timestamp all measurements accurately
3. Synchronize data from different rates
4. Log to CSV with proper formatting
5. Display real-time update rates for each sensor

### Starter Code

```python
import threading
import time
import csv
from queue import Queue

class MultiSensorLogger:
    def __init__(self, sensors):
        self.sensors = sensors
        self.data_queue = Queue()
        self.running = False
        self.threads = []
        
    def sensor_thread(self, sensor_name, sensor_obj, rate_hz):
        """Thread function to read from one sensor."""
        dt = 1.0 / rate_hz
        while self.running:
            try:
                # TODO: Read sensor data
                # data = sensor_obj.read()
                
                # Add to queue with timestamp
                timestamp = time.time()
                # self.data_queue.put((timestamp, sensor_name, data))
                
                time.sleep(dt)
            except Exception as e:
                print(f"Error in {sensor_name}: {e}")
    
    def logger_thread(self, filename):
        """Thread function to write data to file."""
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            
            # TODO: Write header
            # writer.writerow(['timestamp', 'sensor', 'data'])
            
            while self.running:
                if not self.data_queue.empty():
                    # TODO: Get data from queue and write to file
                    pass
    
    def start(self):
        """Start all sensor threads."""
        self.running = True
        
        # TODO: Create and start threads for each sensor
        # TODO: Create and start logger thread
        pass
    
    def stop(self):
        """Stop all threads."""
        self.running = False
        for thread in self.threads:
            thread.join()

# TODO: Setup sensors and run logger
```

### Success Criteria
- [ ] Multiple sensors read simultaneously
- [ ] Data is properly timestamped
- [ ] CSV file format is correct
- [ ] No data loss or corruption
- [ ] Clean shutdown on Ctrl+C

### Bonus Challenge
- Add real-time data rate monitoring
- Implement data buffer to handle rate bursts
- Add data validation and error checking
- Create synchronized data snapshots

---

## 📚 Additional Challenges

### Challenge A: Emergency Stop System
Create a safety system that monitors sensors and triggers emergency stop when:
- IMU detects falling (tilt > 30°)
- Force sensors detect collision (force > threshold)
- Any sensor reports error/timeout

### Challenge B: Calibration Tool
Build a tool to automatically calibrate:
- IMU magnetometer (figure-8 motion)
- Servo center positions
- Camera intrinsics

### Challenge C: Sensor Dashboard
Create a web-based dashboard showing:
- Real-time sensor values
- Historical data plots
- System health indicators
- Calibration status

---

## ✅ Submission Checklist

When completing exercises, ensure:

- [ ] Code runs without errors
- [ ] Comments explain key logic
- [ ] Results are documented (screenshots, plots)
- [ ] Edge cases are handled
- [ ] Code follows style guidelines
- [ ] README explains how to run

---

## 💡 Tips for Success

:::tip Testing Strategy
1. **Start simple**: Get basic functionality working first
2. **Test incrementally**: Verify each component separately
3. **Use simulated data**: Don't need hardware to start coding
4. **Add logging**: Print debug info to understand what's happening
5. **Visualize**: Plots help identify issues quickly
:::

:::warning Common Pitfalls
- Not handling sensor initialization failures
- Forgetting to close files/ports on exit
- Blocking operations causing lag
- Integer overflow in calculations
- Not validating sensor data ranges
:::

---

**[← Previous: Sensor Integration](./sensor-integration.md)** | **[Next Chapter: Simulation →](../03-simulation/intro.md)**
