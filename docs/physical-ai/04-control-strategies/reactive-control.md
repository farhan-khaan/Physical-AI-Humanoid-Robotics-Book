---
title: Reactive Control
description: Fast, sensor-driven control strategies for immediate responses
sidebar_position: 2
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# Reactive Control

Reactive control responds directly to sensor inputs without complex planning. These controllers are fast, predictable, and essential for real-time robotics applications.

## 🎯 Learning Outcomes

By the end of this section, you will be able to:

1. **Implement** PID controllers for position and velocity control
2. **Tune** PID parameters using systematic methods
3. **Design** state machines for behavior switching
4. **Create** reflex behaviors for safety and responsiveness
5. **Apply** impedance control for compliant interaction

## 📋 Prerequisites

- Understanding of sensors and actuators (Chapter 2)
- Basic calculus (derivatives, integrals)
- Python programming

---

## 🎛️ PID Control

**PID (Proportional-Integral-Derivative)** control is the workhorse of robotics. It's simple, effective, and used everywhere from servos to spacecraft.

### The PID Equation

```
u(t) = Kp * e(t) + Ki * ∫e(τ)dτ + Kd * de(t)/dt
```

Where:
- `u(t)` = control output
- `e(t)` = error (desired - actual)
- `Kp` = proportional gain
- `Ki` = integral gain
- `Kd` = derivative gain

### Understanding Each Term

**Proportional (P)**: React to current error
- Large error → large correction
- Problem: Can't eliminate steady-state error

**Integral (I)**: Accumulate past errors
- Eliminates steady-state error
- Problem: Can cause overshoot, oscillation

**Derivative (D)**: Predict future error
- Provides damping
- Problem: Amplifies noise

<Tabs groupId="programming-language">
<TabItem value="python" label="Python" default>

```python
import time
import numpy as np

class PIDController:
    """
    PID controller implementation.
    """
    
    def __init__(self, kp, ki, kd, output_limits=None):
        """
        Initialize PID controller.
        
        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            output_limits: Tuple of (min, max) output
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        
        # State variables
        self.last_error = 0
        self.integral = 0
        self.last_time = time.time()
    
    def update(self, setpoint, measured_value):
        """
        Calculate PID output.
        
        Args:
            setpoint: Desired value
            measured_value: Current measured value
            
        Returns:
            Control output
        """
        # Calculate dt
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Calculate error
        error = setpoint - measured_value
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # Derivative term
        derivative = (error - self.last_error) / dt if dt > 0 else 0
        d_term = self.kd * derivative
        
        # Total output
        output = p_term + i_term + d_term
        
        # Apply output limits
        if self.output_limits:
            output = np.clip(output, self.output_limits[0], self.output_limits[1])
            
            # Anti-windup: Stop integrating if saturated
            if output == self.output_limits[0] or output == self.output_limits[1]:
                self.integral -= error * dt  # Undo integration
        
        # Save for next iteration
        self.last_error = error
        
        return output
    
    def reset(self):
        """Reset controller state."""
        self.last_error = 0
        self.integral = 0
        self.last_time = time.time()


# Example: Position control of a joint
def joint_position_control():
    """Control joint to reach target position."""
    
    # Create PID controller
    pid = PIDController(
        kp=10.0,   # Proportional gain
        ki=0.1,    # Integral gain
        kd=1.0,    # Derivative gain
        output_limits=(-50, 50)  # Torque limits in Nm
    )
    
    target_position = 1.57  # 90 degrees
    
    for _ in range(1000):
        # Read current position
        current_position = get_joint_position()
        
        # Calculate control output
        torque = pid.update(target_position, current_position)
        
        # Apply torque to joint
        apply_joint_torque(torque)
        
        # Print debug info
        error = target_position - current_position
        print(f"Position: {current_position:.3f}, Error: {error:.3f}, Torque: {torque:.2f}")
        
        time.sleep(0.01)  # 100 Hz control loop
```

</TabItem>
<TabItem value="cpp" label="C++">

```cpp
#include <chrono>
#include <algorithm>

class PIDController {
private:
    double kp, ki, kd;
    double last_error;
    double integral;
    std::chrono::steady_clock::time_point last_time;
    double output_min, output_max;
    
public:
    PIDController(double kp, double ki, double kd, 
                  double out_min = -1e6, double out_max = 1e6)
        : kp(kp), ki(ki), kd(kd), 
          output_min(out_min), output_max(out_max),
          last_error(0), integral(0) {
        last_time = std::chrono::steady_clock::now();
    }
    
    double update(double setpoint, double measured_value) {
        auto current_time = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(
            current_time - last_time).count();
        last_time = current_time;
        
        // Error
        double error = setpoint - measured_value;
        
        // P term
        double p_term = kp * error;
        
        // I term
        integral += error * dt;
        double i_term = ki * integral;
        
        // D term
        double derivative = (dt > 0) ? (error - last_error) / dt : 0;
        double d_term = kd * derivative;
        
        // Output
        double output = p_term + i_term + d_term;
        output = std::clamp(output, output_min, output_max);
        
        // Anti-windup
        if (output == output_min || output == output_max) {
            integral -= error * dt;
        }
        
        last_error = error;
        return output;
    }
    
    void reset() {
        last_error = 0;
        integral = 0;
        last_time = std::chrono::steady_clock::now();
    }
};
```

</TabItem>
</Tabs>

### PID Tuning Methods

#### Method 1: Manual Tuning

```python
def manual_tuning_guide():
    """
    Step-by-step manual tuning procedure.
    """
    print("Manual PID Tuning Guide:")
    print()
    print("Step 1: Set Ki=0, Kd=0. Increase Kp until oscillation occurs.")
    print("  - Too low Kp: Slow response, large steady-state error")
    print("  - Too high Kp: Oscillation, overshoot")
    print()
    print("Step 2: Set Kp to 50% of oscillation value.")
    print()
    print("Step 3: Increase Ki to eliminate steady-state error.")
    print("  - Too low Ki: Steady-state error remains")
    print("  - Too high Ki: Overshoot, slow settling")
    print()
    print("Step 4: Increase Kd to reduce overshoot.")
    print("  - Too low Kd: Overshoot remains")
    print("  - Too high Kd: Noise amplification, instability")
```

#### Method 2: Ziegler-Nichols

```python
def ziegler_nichols_tuning():
    """
    Ziegler-Nichols tuning method.
    """
    # Step 1: Find ultimate gain (Ku) and period (Tu)
    # Set Ki=0, Kd=0, increase Kp until sustained oscillation
    
    Ku = 20.0  # Ultimate gain (from testing)
    Tu = 0.5   # Ultimate period (from testing)
    
    # Step 2: Calculate PID gains
    # Classic PID
    Kp = 0.6 * Ku
    Ki = 1.2 * Ku / Tu
    Kd = 0.075 * Ku * Tu
    
    print(f"Ziegler-Nichols PID gains:")
    print(f"  Kp = {Kp:.2f}")
    print(f"  Ki = {Ki:.2f}")
    print(f"  Kd = {Kd:.2f}")
    
    return Kp, Ki, Kd

# Alternative: Pessen Integral Rule (less overshoot)
def pessen_integral_rule(Ku, Tu):
    Kp = 0.7 * Ku
    Ki = 1.75 * Ku / Tu
    Kd = 0.105 * Ku * Tu
    return Kp, Ki, Kd
```

:::tip PID Tuning Tips
1. **Start with P-only** control, get basic response
2. **Add I** to eliminate steady-state error
3. **Add D** to reduce overshoot and oscillation
4. **Test thoroughly** with different setpoints
5. **Monitor actuator saturation** - adjust limits if needed
:::

---

## 🔄 State Machines

State machines organize complex behaviors into discrete states with transitions.

### Finite State Machine Example

```python
from enum import Enum

class RobotState(Enum):
    IDLE = 0
    STANDING = 1
    WALKING = 2
    FALLING = 3
    RECOVERING = 4

class HumanoidStateMachine:
    """
    State machine for humanoid robot control.
    """
    
    def __init__(self):
        self.state = RobotState.IDLE
        self.state_entry_time = time.time()
    
    def update(self, sensor_data):
        """
        Update state based on sensor data.
        """
        # Get current state duration
        time_in_state = time.time() - self.state_entry_time
        
        # State transition logic
        if self.state == RobotState.IDLE:
            if sensor_data['battery'] > 0.2:
                self.transition_to(RobotState.STANDING)
        
        elif self.state == RobotState.STANDING:
            if self.is_balanced(sensor_data):
                if sensor_data['command'] == 'walk':
                    self.transition_to(RobotState.WALKING)
            else:
                self.transition_to(RobotState.FALLING)
        
        elif self.state == RobotState.WALKING:
            if not self.is_balanced(sensor_data):
                self.transition_to(RobotState.FALLING)
            elif sensor_data['command'] == 'stop':
                self.transition_to(RobotState.STANDING)
        
        elif self.state == RobotState.FALLING:
            # Execute safe fall behavior
            self.safe_fall()
            if time_in_state > 2.0:  # After 2 seconds
                self.transition_to(RobotState.RECOVERING)
        
        elif self.state == RobotState.RECOVERING:
            if self.stand_up_complete():
                self.transition_to(RobotState.STANDING)
        
        # Execute current state behavior
        self.execute_state()
    
    def transition_to(self, new_state):
        """Transition to new state."""
        print(f"State transition: {self.state.name} → {new_state.name}")
        self.state = new_state
        self.state_entry_time = time.time()
    
    def execute_state(self):
        """Execute behavior for current state."""
        if self.state == RobotState.IDLE:
            self.idle_behavior()
        elif self.state == RobotState.STANDING:
            self.standing_behavior()
        elif self.state == RobotState.WALKING:
            self.walking_behavior()
        elif self.state == RobotState.FALLING:
            self.falling_behavior()
        elif self.state == RobotState.RECOVERING:
            self.recovering_behavior()
    
    def is_balanced(self, sensor_data):
        """Check if robot is balanced."""
        roll, pitch = sensor_data['orientation']
        return abs(roll) < 15 and abs(pitch) < 15
    
    def idle_behavior(self):
        """Idle state behavior."""
        # Disable motors, low power
        pass
    
    def standing_behavior(self):
        """Standing balance control."""
        # Active balance control
        pass
    
    def walking_behavior(self):
        """Walking gait."""
        # Execute walking controller
        pass
    
    def falling_behavior(self):
        """Safe falling."""
        # Protect joints, prepare for impact
        pass
    
    def recovering_behavior(self):
        """Stand up from ground."""
        # Execute stand-up sequence
        pass
```

:::note State Machine Benefits
- **Clear structure**: Easy to understand and maintain
- **Predictable**: Known behavior in each state
- **Safe**: Explicit handling of all situations
- **Debuggable**: Can log state transitions
:::

---

## ⚡ Reflex Behaviors

Reflexes are fast, instinctive responses to specific stimuli.

### Obstacle Avoidance Reflex

```python
class ObstacleAvoidanceReflex:
    """
    Fast obstacle avoidance reflex.
    """
    
    def __init__(self, safety_distance=0.5):
        self.safety_distance = safety_distance
    
    def check_and_avoid(self, lidar_scan, current_velocity):
        """
        Check for obstacles and adjust velocity.
        
        Args:
            lidar_scan: List of (angle, distance) tuples
            current_velocity: Current [vx, vy, omega]
            
        Returns:
            Adjusted velocity command
        """
        # Find closest obstacle in front
        front_obstacles = [
            (angle, dist) for angle, dist in lidar_scan
            if -30 < angle < 30  # ±30 degrees in front
        ]
        
        if not front_obstacles:
            return current_velocity  # No obstacles
        
        min_distance = min(dist for _, dist in front_obstacles)
        
        # Reflex activation
        if min_distance < self.safety_distance:
            # Calculate avoidance strength
            danger = 1.0 - (min_distance / self.safety_distance)
            
            # Slow down
            vx = current_velocity[0] * (1.0 - danger)
            
            # Turn away from obstacle
            obstacles_left = [d for a, d in front_obstacles if a < 0]
            obstacles_right = [d for a, d in front_obstacles if a > 0]
            
            avg_left = np.mean(obstacles_left) if obstacles_left else float('inf')
            avg_right = np.mean(obstacles_right) if obstacles_right else float('inf')
            
            # Turn towards more open side
            turn_rate = 1.0 * danger * (1 if avg_left > avg_right else -1)
            
            return [vx, 0, turn_rate]
        
        return current_velocity
```

### Fall Detection Reflex

```python
class FallDetectionReflex:
    """
    Detect falling and trigger protective reflex.
    """
    
    def __init__(self):
        self.falling = False
        self.fall_threshold_angle = 25  # degrees
        self.fall_threshold_accel = 5.0  # m/s²
    
    def check_falling(self, imu_data):
        """
        Check if robot is falling.
        
        Args:
            imu_data: Dict with 'orientation' and 'accel'
            
        Returns:
            True if falling detected
        """
        roll, pitch = imu_data['orientation']
        accel = imu_data['accel']
        
        # Check orientation
        angle_exceeded = (abs(roll) > self.fall_threshold_angle or 
                         abs(pitch) > self.fall_threshold_angle)
        
        # Check vertical acceleration (falling = low vertical accel)
        vertical_accel = accel[2]  # Z-axis
        freefall = abs(vertical_accel - 9.8) > self.fall_threshold_accel
        
        if angle_exceeded or freefall:
            if not self.falling:
                print("⚠️ FALL DETECTED! Activating protective reflex.")
                self.trigger_fall_reflex()
            self.falling = True
            return True
        
        self.falling = False
        return False
    
    def trigger_fall_reflex(self):
        """
        Execute protective falling behavior.
        """
        # 1. Relax joints to absorb impact
        set_joint_compliance(high=True)
        
        # 2. Protect head (tuck chin)
        move_head_down()
        
        # 3. Extend arms to catch fall
        extend_arms()
        
        # 4. Disable walking controller
        disable_walking_controller()
```

---

## 🤝 Impedance Control

Impedance control makes robots compliant, allowing safe interaction with environments and humans.

### Concept

Instead of controlling position directly, control the relationship between force and motion:

```
F = K(x_desired - x) + B(v_desired - v)
```

Where:
- `F` = applied force
- `K` = virtual stiffness
- `B` = virtual damping
- `x_desired` = desired position
- `x` = actual position
- `v` = velocity

```python
class ImpedanceController:
    """
    Impedance controller for compliant behavior.
    """
    
    def __init__(self, stiffness=100, damping=20):
        """
        Args:
            stiffness: Virtual spring constant (N/m)
            damping: Virtual damping constant (N·s/m)
        """
        self.stiffness = stiffness
        self.damping = damping
    
    def compute_force(self, desired_pos, actual_pos, 
                     desired_vel, actual_vel):
        """
        Compute force for impedance control.
        
        Returns:
            Force/torque command
        """
        # Position error
        pos_error = desired_pos - actual_pos
        
        # Velocity error
        vel_error = desired_vel - actual_vel
        
        # Impedance force
        force = (self.stiffness * pos_error + 
                self.damping * vel_error)
        
        return force
    
    def set_compliance(self, compliance_level):
        """
        Adjust compliance (0=rigid, 1=very compliant).
        """
        max_stiffness = 1000
        max_damping = 200
        
        self.stiffness = max_stiffness * (1 - compliance_level)
        self.damping = max_damping * (1 - compliance_level)

# Example: Compliant reaching
def compliant_reaching():
    """
    Reach to target with compliance.
    """
    controller = ImpedanceController(stiffness=50, damping=10)
    
    target_position = [0.5, 0.3, 0.8]  # XYZ in meters
    
    while True:
        # Read current state
        current_pos = get_end_effector_position()
        current_vel = get_end_effector_velocity()
        
        # Compute compliant force
        force = controller.compute_force(
            desired_pos=target_position,
            actual_pos=current_pos,
            desired_vel=[0, 0, 0],
            actual_vel=current_vel
        )
        
        # Apply force (robot yields if it hits obstacle)
        apply_end_effector_force(force)
        
        time.sleep(0.01)
```

---

## ✅ Key Takeaways

1. **PID control** is simple and effective for position/velocity control
2. **Tuning PID** requires systematic approach (manual or Ziegler-Nichols)
3. **State machines** organize complex behaviors clearly
4. **Reflexes** provide fast responses to dangers
5. **Impedance control** enables safe, compliant interaction

---

## 🧪 Practice Exercise

:::note Exercise: Balance Controller
**Difficulty**: Medium  
**Time**: 1.5 hours  
**Goal**: Implement PID-based balance control

**Requirements**:
1. Create PID controller for robot balance
2. Use IMU data to measure tilt
3. Control ankle/hip joints to maintain balance
4. Tune PID gains for stable standing
5. Test with external disturbances

**Bonus**: Add state machine for sit-stand transitions
:::

---

**[← Previous: Chapter Introduction](./intro.md)** | **[Next: Deliberative Control →](./deliberative-control.md)**
