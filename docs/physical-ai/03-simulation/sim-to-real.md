---
title: Sim-to-Real Transfer
description: Bridging the gap between simulation and reality
sidebar_position: 5
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# Sim-to-Real Transfer

The **reality gap** is one of robotics' biggest challenges: controllers that work perfectly in simulation often fail on real hardware. This section explores techniques to bridge this gap and successfully transfer from simulation to reality.

## 🎯 Learning Outcomes

By the end of this section, you will be able to:

1. **Explain** the reality gap and its causes
2. **Apply** domain randomization techniques
3. **Implement** system identification for better models
4. **Use** sim-to-real best practices
5. **Validate** controllers incrementally
6. **Handle** the transition from sim to real hardware

## 📋 Prerequisites

- Understanding of simulation platforms (previous sections)
- Experience running simulations
- Basic understanding of probability and statistics
- Familiarity with robot control

---

## 🌉 The Reality Gap

### What is the Reality Gap?

The **reality gap** is the difference between simulated and real-world behavior caused by:

```
Simulation = Simplified Model + Perfect Sensors + Ideal Actuators
Reality = Complex Physics + Noisy Sensors + Imperfect Actuators
```

### Why Does It Exist?

| Aspect | Simulation | Reality |
|--------|-----------|---------|
| **Physics** | Simplified contact dynamics | Complex, chaotic interactions |
| **Sensors** | Perfect, no noise | Noisy, drift, failures |
| **Actuators** | Instant response, no backlash | Delays, friction, wear |
| **Environment** | Known, controlled | Unknown, variable |
| **Modeling** | Exact parameters | Approximations, unknowns |
| **Computation** | Unlimited precision | Floating-point errors |

### Example: The Walking Robot

```python
# Simulation: Works perfectly
def simulation_walk():
    for step in range(1000):
        apply_forces([10, 10, 10, 10])  # Perfect forces
        step_physics()  # Perfect physics
        state = get_state()  # Perfect sensing
        # Robot walks smoothly!

# Reality: Falls over
def reality_walk():
    for step in range(10):  # Falls after 10 steps!
        apply_forces([10, 10, 10, 10])  # But actual forces vary ±20%
        # Physics has unmodeled effects (flexibility, slip)
        state = get_state()  # Noisy, delayed measurements
        # Robot unstable and falls!
```

:::warning Classic Pitfall
A controller with 100% success rate in simulation might have 0% success rate in reality without proper sim-to-real techniques!
:::

---

## 🎲 Domain Randomization

**Domain randomization** varies simulation parameters during training to create robust policies that work across different conditions.

### Concept

Instead of one perfect simulation:
```
Train on: Sim₁ (perfect model)
Deploy to: Reality (different)
Result: Failure ❌
```

Use many randomized simulations:
```
Train on: Sim₁, Sim₂, Sim₃, ..., Simₙ (varied models)
Deploy to: Reality (within range)
Result: Success ✓
```

### What to Randomize

#### 1. Physics Parameters

<Tabs groupId="programming-language">
<TabItem value="python" label="Python" default>

```python
import random
import numpy as np

class PhysicsRandomizer:
    """Randomize physics parameters for robust training."""
    
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.base_params = self.save_current_params()
    
    def randomize_mass(self, variation=0.2):
        """Randomize link masses by ±variation."""
        num_links = p.getNumJoints(self.robot_id)
        
        for link_idx in range(-1, num_links):  # -1 = base
            # Get current mass
            dynamics = p.getDynamicsInfo(self.robot_id, link_idx)
            base_mass = dynamics[0]
            
            # Randomize ±20%
            new_mass = base_mass * random.uniform(1-variation, 1+variation)
            
            p.changeDynamics(
                self.robot_id,
                link_idx,
                mass=new_mass
            )
    
    def randomize_friction(self):
        """Randomize friction coefficients."""
        num_links = p.getNumJoints(self.robot_id)
        
        for link_idx in range(-1, num_links):
            # Lateral friction: 0.5 to 1.5
            lateral = random.uniform(0.5, 1.5)
            # Spinning friction: 0.001 to 0.01
            spinning = random.uniform(0.001, 0.01)
            
            p.changeDynamics(
                self.robot_id,
                link_idx,
                lateralFriction=lateral,
                spinningFriction=spinning
            )
    
    def randomize_joint_properties(self):
        """Randomize joint damping and friction."""
        num_joints = p.getNumJoints(self.robot_id)
        
        for joint_idx in range(num_joints):
            damping = random.uniform(0.1, 1.0)
            friction = random.uniform(0.0, 0.5)
            
            p.changeDynamics(
                self.robot_id,
                joint_idx,
                jointDamping=damping,
                jointFriction=friction
            )
    
    def randomize_all(self):
        """Apply all randomizations."""
        self.randomize_mass(variation=0.2)
        self.randomize_friction()
        self.randomize_joint_properties()
        
        # Also randomize gravity slightly
        base_gravity = -9.81
        gravity_var = random.uniform(-0.1, 0.1)
        p.setGravity(0, 0, base_gravity + gravity_var)

# Usage
randomizer = PhysicsRandomizer(robot_id)

for episode in range(1000):
    # Reset with new random parameters
    p.resetSimulation()
    robot_id = p.loadURDF("robot.urdf")
    randomizer = PhysicsRandomizer(robot_id)
    randomizer.randomize_all()
    
    # Train your controller
    train_episode()
```

</TabItem>
<TabItem value="cpp" label="C++">

```cpp
class PhysicsRandomizer {
private:
    int robot_id;
    std::default_random_engine generator;
    
public:
    void randomizeMass(double variation = 0.2) {
        std::uniform_real_distribution<double> dist(1-variation, 1+variation);
        
        int num_joints = p.getNumJoints(robot_id);
        for (int i = -1; i < num_joints; i++) {
            b3DynamicsInfo info;
            p.getDynamicsInfo(robot_id, i, &info);
            
            double new_mass = info.mass * dist(generator);
            p.changeDynamics(robot_id, i, new_mass);
        }
    }
    
    void randomizeAll() {
        randomizeMass();
        randomizeFriction();
        randomizeJointProperties();
    }
};
```

</TabItem>
</Tabs>

#### 2. Sensor Noise

```python
class SensorNoiseModel:
    """Add realistic noise to simulated sensors."""
    
    def __init__(self):
        pass
    
    def add_imu_noise(self, clean_imu_data):
        """Add noise to IMU readings."""
        accel = clean_imu_data['accel']
        gyro = clean_imu_data['gyro']
        
        # Accelerometer noise (Gaussian)
        accel_noise = np.random.normal(0, 0.02, 3)  # 0.02 m/s² std
        noisy_accel = accel + accel_noise
        
        # Gyroscope noise and bias
        gyro_noise = np.random.normal(0, 0.001, 3)  # 0.001 rad/s std
        gyro_bias = np.random.normal(0, 0.0001, 3)  # Small bias
        noisy_gyro = gyro + gyro_noise + gyro_bias
        
        return {
            'accel': noisy_accel,
            'gyro': noisy_gyro
        }
    
    def add_camera_noise(self, clean_image):
        """Add noise to camera image."""
        # Gaussian noise
        noise = np.random.normal(0, 5, clean_image.shape)  # 5 intensity units
        noisy_image = np.clip(clean_image + noise, 0, 255)
        
        # Motion blur (occasionally)
        if random.random() < 0.1:
            kernel_size = random.randint(3, 7)
            kernel = np.ones((kernel_size, kernel_size)) / (kernel_size**2)
            noisy_image = cv2.filter2D(noisy_image, -1, kernel)
        
        return noisy_image.astype(np.uint8)
    
    def add_force_sensor_noise(self, true_force):
        """Add noise to force sensor."""
        # Proportional noise (±2% of reading)
        noise = np.random.normal(0, 0.02 * abs(true_force))
        
        # Quantization (12-bit ADC)
        max_force = 100.0
        quantized = np.round(true_force / max_force * 4096) / 4096 * max_force
        
        return quantized + noise

# Usage in simulation loop
noise_model = SensorNoiseModel()

while simulating:
    # Get clean sensor data from simulator
    clean_imu = simulator.get_imu()
    clean_camera = simulator.get_camera()
    clean_force = simulator.get_force()
    
    # Add realistic noise
    noisy_imu = noise_model.add_imu_noise(clean_imu)
    noisy_camera = noise_model.add_camera_noise(clean_camera)
    noisy_force = noise_model.add_force_sensor_noise(clean_force)
    
    # Use noisy data for control
    control = controller.update(noisy_imu, noisy_camera, noisy_force)
```

#### 3. Actuator Imperfections

```python
class ActuatorModel:
    """Model real actuator imperfections."""
    
    def __init__(self):
        self.delays = {}  # Command delays
        self.backlash = {}  # Gear backlash
        self.command_queue = []
    
    def add_delay(self, joint_id, command, delay_steps=3):
        """Add communication/actuation delay."""
        self.command_queue.append({
            'joint': joint_id,
            'command': command,
            'execute_step': current_step + delay_steps
        })
    
    def apply_backlash(self, joint_id, command, backlash=0.02):
        """Model gear backlash (dead zone)."""
        if not hasattr(self, 'last_direction'):
            self.last_direction = {}
        
        current_pos = get_joint_position(joint_id)
        direction = np.sign(command - current_pos)
        
        # If direction changed, add backlash offset
        if joint_id in self.last_direction:
            if direction != self.last_direction[joint_id]:
                command += direction * backlash
        
        self.last_direction[joint_id] = direction
        return command
    
    def add_torque_limit(self, commanded_torque, max_torque=50.0):
        """Apply realistic torque limits."""
        return np.clip(commanded_torque, -max_torque, max_torque)
    
    def add_velocity_limit(self, commanded_velocity, max_velocity=3.0):
        """Apply realistic velocity limits."""
        return np.clip(commanded_velocity, -max_velocity, max_velocity)
    
    def model_motor_dynamics(self, commanded_position, current_position, dt):
        """Model motor response (not instantaneous)."""
        # Simple first-order lag
        tau = 0.05  # Time constant (50ms)
        alpha = dt / (tau + dt)
        
        new_position = current_position + alpha * (commanded_position - current_position)
        return new_position

# Usage
actuator_model = ActuatorModel()

def send_command(joint_id, target_position):
    # Apply imperfections
    target_position = actuator_model.apply_backlash(joint_id, target_position)
    actuator_model.add_delay(joint_id, target_position, delay_steps=3)

def apply_queued_commands():
    """Execute delayed commands."""
    for cmd in actuator_model.command_queue:
        if cmd['execute_step'] == current_step:
            apply_to_simulator(cmd['joint'], cmd['command'])
            actuator_model.command_queue.remove(cmd)
```

#### 4. Visual Randomization

```python
class VisualRandomizer:
    """Randomize visual appearance for vision-based controllers."""
    
    def randomize_lighting(self):
        """Randomize light position and intensity."""
        # Random light position
        light_pos = [
            random.uniform(-5, 5),
            random.uniform(-5, 5),
            random.uniform(2, 8)
        ]
        
        # Random intensity
        intensity = random.uniform(0.5, 1.5)
        
        # Apply to simulator (PyBullet doesn't have great lighting control)
        # This is conceptual - Isaac Sim has better lighting APIs
        pass
    
    def randomize_textures(self, object_id):
        """Randomize object textures/colors."""
        # Random color
        color = [random.random() for _ in range(3)] + [1.0]
        
        p.changeVisualShape(
            object_id,
            -1,
            rgbaColor=color
        )
    
    def randomize_camera_params(self):
        """Randomize camera intrinsics slightly."""
        base_fov = 60
        fov = base_fov + random.uniform(-5, 5)
        
        # Recompute projection matrix
        return fov
```

---

## 🔬 System Identification

**System identification** measures real robot parameters to improve simulation accuracy.

### Measuring Mass and Inertia

```python
class SystemIdentification:
    """Identify real system parameters."""
    
    def measure_link_mass(self, joint_id):
        """Estimate link mass from torque measurements."""
        # Apply known torque
        applied_torque = 10.0  # Nm
        
        # Measure resulting acceleration
        initial_vel = get_joint_velocity(joint_id)
        apply_torque(joint_id, applied_torque)
        time.sleep(0.1)
        final_vel = get_joint_velocity(joint_id)
        
        acceleration = (final_vel - initial_vel) / 0.1
        
        # Estimate inertia: τ = I * α
        estimated_inertia = applied_torque / acceleration
        
        return estimated_inertia
    
    def measure_friction(self, joint_id):
        """Measure joint friction coefficient."""
        velocities = []
        torques = []
        
        # Test at different velocities
        for target_vel in np.linspace(0, 2.0, 20):
            set_joint_velocity(joint_id, target_vel)
            time.sleep(0.5)  # Let it stabilize
            
            actual_vel = get_joint_velocity(joint_id)
            required_torque = get_joint_torque(joint_id)
            
            velocities.append(actual_vel)
            torques.append(required_torque)
        
        # Fit linear model: τ = τ_static + b * v
        coeffs = np.polyfit(velocities, torques, 1)
        friction_coeff = coeffs[0]
        static_friction = coeffs[1]
        
        return {
            'viscous_friction': friction_coeff,
            'static_friction': static_friction
        }
    
    def measure_motor_response_time(self, joint_id):
        """Measure actuator response time."""
        # Send step command
        target = 1.0
        t0 = time.time()
        set_joint_position(joint_id, target)
        
        # Wait for 90% of target
        while True:
            pos = get_joint_position(joint_id)
            if abs(pos - target) < 0.1 * target:
                break
            time.sleep(0.001)
        
        response_time = time.time() - t0
        return response_time

# Use identified parameters in simulation
sysid = SystemIdentification()
friction_params = sysid.measure_friction(joint_id=0)

# Update simulation
p.changeDynamics(
    robot_id, 
    0,
    jointDamping=friction_params['viscous_friction']
)
```

---

## 📋 Sim-to-Real Best Practices

### 1. Start Simple

```python
# Progressive complexity
def sim_to_real_progression():
    # Step 1: Test in simulation
    test_in_simulation()
    
    # Step 2: Test simple motion on real robot
    test_single_joint_motion()
    
    # Step 3: Test with safety constraints
    test_with_position_limits()
    
    # Step 4: Test full controller slowly
    test_full_controller(speed=0.5)
    
    # Step 5: Full speed deployment
    deploy_full_speed()
```

### 2. Use Conservative Parameters

```python
# In simulation: Aggressive
sim_params = {
    'max_velocity': 5.0,
    'max_acceleration': 10.0,
    'control_frequency': 1000
}

# On real robot: Conservative
real_params = {
    'max_velocity': 2.0,  # 40% of sim
    'max_acceleration': 4.0,  # 40% of sim
    'control_frequency': 100  # Lower initially
}
```

### 3. Implement Safety Monitors

```python
class SafetyMonitor:
    """Monitor robot state and trigger emergency stops."""
    
    def __init__(self):
        self.limits = {
            'max_joint_velocity': 3.0,
            'max_joint_torque': 50.0,
            'max_tilt_angle': 30.0,
            'min_battery_voltage': 10.0
        }
        self.emergency_stop = False
    
    def check_safety(self, robot_state):
        """Check if robot is in safe state."""
        # Check joint velocities
        for vel in robot_state['joint_velocities']:
            if abs(vel) > self.limits['max_joint_velocity']:
                self.trigger_emergency_stop("Excessive joint velocity")
                return False
        
        # Check orientation
        roll, pitch = robot_state['orientation']
        if abs(roll) > self.limits['max_tilt_angle'] or \
           abs(pitch) > self.limits['max_tilt_angle']:
            self.trigger_emergency_stop("Robot tilting")
            return False
        
        # Check battery
        if robot_state['battery'] < self.limits['min_battery_voltage']:
            self.trigger_emergency_stop("Low battery")
            return False
        
        return True
    
    def trigger_emergency_stop(self, reason):
        """Emergency stop all motors."""
        print(f"🚨 EMERGENCY STOP: {reason}")
        self.emergency_stop = True
        disable_all_motors()
```

### 4. Log Everything

```python
class DataLogger:
    """Log all data for analysis."""
    
    def __init__(self, filename):
        self.file = open(filename, 'w')
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            'timestamp', 'joint_positions', 'joint_velocities',
            'joint_torques', 'imu_data', 'commands', 'errors'
        ])
    
    def log(self, data):
        """Log timestamped data."""
        row = [
            time.time(),
            data['positions'],
            data['velocities'],
            data['torques'],
            data['imu'],
            data['commands'],
            data.get('errors', '')
        ]
        self.writer.writerow(row)
```

---

## ✅ Key Takeaways

1. **Reality gap** exists due to modeling simplifications
2. **Domain randomization** creates robust policies
3. **System identification** improves simulation accuracy
4. **Start simple** and progress gradually
5. **Safety first** always on real hardware
6. **Log data** to understand sim-reality differences
7. **Iterate** between sim and real

---

## 🧪 Practice Exercise

:::note Exercise: Domain Randomization
**Difficulty**: Hard  
**Time**: 2 hours  
**Goal**: Implement domain randomization for robust control

**Tasks**:
1. Create baseline controller in simulation
2. Implement physics randomization
3. Implement sensor noise addition
4. Train/test with randomization
5. Compare robustness with and without randomization

**Bonus**: Test on real robot if available!
:::

---

**[← Previous: Setting Up Simulation](./setting-up-simulation.md)** | **[Next Chapter: Control Strategies →](../04-control-strategies/intro.md)**
