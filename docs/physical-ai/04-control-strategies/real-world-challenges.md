---
title: Real-World Challenges
description: Handling noise, failures, and uncertainties in real robotics
sidebar_position: 6
---

# Real-World Challenges

Real robots face challenges simulation doesn't capture: sensor noise, actuator limits, communication delays, and unexpected failures. This section covers practical techniques for robust real-world operation.

## 🎯 Learning Outcomes

1. **Handle** sensor noise and outliers
2. **Deal with** actuator saturation
3. **Compensate** for latency
4. **Implement** failure detection and recovery
5. **Ensure** safety in all conditions

## 📋 Prerequisites

- Previous control strategies knowledge
- Understanding of sensors and actuators

---

## 📊 Dealing with Sensor Noise

### Outlier Rejection

```python
class OutlierFilter:
    """Remove sensor outliers."""
    
    def __init__(self, window_size=10, threshold=3.0):
        self.window = []
        self.window_size = window_size
        self.threshold = threshold
    
    def filter(self, measurement):
        """Filter out outliers."""
        self.window.append(measurement)
        if len(self.window) > self.window_size:
            self.window.pop(0)
        
        if len(self.window) < 3:
            return measurement
        
        mean = np.mean(self.window)
        std = np.std(self.window)
        
        if abs(measurement - mean) > self.threshold * std:
            return mean  # Return mean instead of outlier
        
        return measurement
```

### Moving Average Filter

```python
class MovingAverageFilter:
    """Simple moving average filter."""
    
    def __init__(self, window_size=5):
        self.window = []
        self.window_size = window_size
    
    def filter(self, measurement):
        """Apply moving average."""
        self.window.append(measurement)
        if len(self.window) > self.window_size:
            self.window.pop(0)
        return np.mean(self.window)
```

---

## ⚡ Handling Actuator Saturation

### Anti-Windup for PID

```python
class PIDWithAntiWindup:
    """PID controller with anti-windup."""
    
    def __init__(self, kp, ki, kd, output_limits):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min, self.output_max = output_limits
        
        self.integral = 0
        self.last_error = 0
    
    def update(self, error, dt):
        """Update with anti-windup."""
        p_term = self.kp * error
        self.integral += error * dt
        i_term = self.ki * self.integral
        d_term = self.kd * (error - self.last_error) / dt
        
        output = p_term + i_term + d_term
        
        # Anti-windup
        if output > self.output_max:
            output = self.output_max
            self.integral -= error * dt  # Don't accumulate
        elif output < self.output_min:
            output = self.output_min
            self.integral -= error * dt
        
        self.last_error = error
        return output
```

---

## ⏱️ Latency Compensation

### Predictive Control

```python
class LatencyCompensator:
    """Compensate for control latency."""
    
    def __init__(self, latency_steps=3):
        self.latency_steps = latency_steps
        self.command_history = []
    
    def predict_future_state(self, current_state, dt):
        """Predict state after latency."""
        predicted_state = current_state.copy()
        
        for cmd in self.command_history[-self.latency_steps:]:
            predicted_state = self.simulate_step(predicted_state, cmd, dt)
        
        return predicted_state
    
    def send_command(self, command):
        """Send command and store in history."""
        self.command_history.append(command)
        if len(self.command_history) > 10:
            self.command_history.pop(0)
        
        return command
```

---

## 🚨 Failure Detection and Recovery

### Watchdog Timer

```python
class WatchdogTimer:
    """Detect communication failures."""
    
    def __init__(self, timeout=1.0):
        self.timeout = timeout
        self.last_update = time.time()
    
    def feed(self):
        """Reset watchdog."""
        self.last_update = time.time()
    
    def check(self):
        """Check if timeout occurred."""
        if time.time() - self.last_update > self.timeout:
            return True  # Timeout!
        return False
```

### Safe Degradation

```python
class SafetyMonitor:
    """Monitor robot safety."""
    
    def __init__(self):
        self.emergency_stop = False
    
    def check_safety(self, state):
        """Check safety conditions."""
        if abs(state['tilt']) > 30:
            self.trigger_emergency_stop("Excessive tilt")
        
        if state['battery'] < 0.1:
            self.trigger_emergency_stop("Low battery")
        
        return not self.emergency_stop
    
    def trigger_emergency_stop(self, reason):
        """Emergency stop."""
        print(f"🚨 EMERGENCY STOP: {reason}")
        self.emergency_stop = True
        disable_all_motors()
```

---

## ✅ Key Takeaways

1. **Filter sensor noise** with outlier rejection and smoothing
2. **Handle saturation** with anti-windup
3. **Compensate latency** with prediction
4. **Detect failures** with watchdogs and monitors
5. **Fail safely** with emergency stops

---

**[← Previous: Learned Control](./learned-control.md)** | **[Next Chapter: Capstone Project →](../05-capstone/intro.md)**
