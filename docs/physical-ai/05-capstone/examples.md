---
title: Example Projects
description: Detailed examples of successful capstone projects
sidebar_position: 5
---

# Example Projects

Learn from successful capstone projects. These examples show what a complete project looks like, including challenges faced and lessons learned.

---

## Example 1: Balance Controller ⭐⭐☆☆☆

### Project Overview

**Student**: Alex Chen  
**Duration**: 2 weeks  
**Platform**: PyBullet simulation  
**Difficulty**: Beginner

**Goal**: Implement PID-based balance control for humanoid robot to maintain upright stance.

### Technical Approach

**Sensors Used**:
- IMU (orientation, angular velocity)
- Joint encoders (ankle, hip positions)

**Control Strategy**:
- PID controller for ankle joints
- PID controller for hip joints
- Cascade control architecture

**Architecture**:
```
IMU → Balance Controller → Joint Controllers → Robot
                ↓
         State Machine
         (Standing/Recovering)
```

### Implementation Highlights

```python
class BalanceController:
    def __init__(self):
        self.ankle_pid = PIDController(kp=50, ki=1, kd=10)
        self.hip_pid = PIDController(kp=30, ki=0.5, kd=5)
    
    def update(self, imu_data):
        roll, pitch = imu_data['orientation']
        
        # Ankle control for pitch
        ankle_torque = self.ankle_pid.update(0, pitch)
        
        # Hip control for roll
        hip_torque = self.hip_pid.update(0, roll)
        
        return ankle_torque, hip_torque
```

### Results

**Success Metrics**:
- **Balance time**: 60+ seconds
- **Recovery angle**: Up to 15° disturbance
- **Control frequency**: 100 Hz

**Key Achievements**:
- ✅ Maintained balance indefinitely
- ✅ Recovered from push disturbances
- ✅ Smooth, stable control

### Challenges & Solutions

**Challenge 1**: Oscillation at high gains
- **Solution**: Reduced Kp, increased Kd for damping

**Challenge 2**: Drift over time
- **Solution**: Added small Ki term to eliminate steady-state error

**Challenge 3**: Slow response to disturbances
- **Solution**: Implemented derivative kick prevention

### Lessons Learned

1. **Start simple**: P-only control first, then add I and D
2. **Tune systematically**: Use Ziegler-Nichols as starting point
3. **Test extensively**: Try various disturbances
4. **Log everything**: Data helps debug issues
5. **Iterate quickly**: Small changes, test immediately

### Demo Video Timestamps

- 0:00-0:30: System overview
- 0:30-1:00: Steady-state balance
- 1:00-1:30: Recovery from push
- 1:30-2:00: Parameter tuning demo

---

## Example 2: Object Grasping System ⭐⭐⭐☆☆

### Project Overview

**Student**: Maria Rodriguez  
**Duration**: 4 weeks  
**Platform**: Gazebo + ROS  
**Difficulty**: Intermediate

**Goal**: Detect objects with camera and grasp them using robotic arm.

### Technical Approach

**Sensors Used**:
- RGB-D camera (object detection, depth)
- Force/torque sensor (grasp feedback)
- Joint encoders (arm position)

**Control Strategy**:
- Computer vision for object detection
- Inverse kinematics for reaching
- Force control for grasping

**Pipeline**:
```
Camera → Object Detection → 3D Localization → IK Solver → 
Path Planning → Execution → Force Control → Grasp
```

### Implementation Highlights

**Object Detection**:
```python
def detect_object(rgb_image, depth_image):
    # Color-based segmentation
    mask = cv2.inRange(rgb_image, lower_red, upper_red)
    
    # Find centroid
    M = cv2.moments(mask)
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    
    # Get 3D position from depth
    depth = depth_image[cy, cx]
    position_3d = pixel_to_3d(cx, cy, depth)
    
    return position_3d
```

**Grasp Control**:
```python
class GraspController:
    def __init__(self):
        self.force_threshold = 5.0  # Newtons
        self.target_force = 10.0
    
    def grasp(self, force_sensor):
        current_force = force_sensor.read()
        
        if current_force < self.force_threshold:
            return "close_gripper"
        elif current_force > self.target_force:
            return "hold"
        else:
            return "adjust"
```

### Results

**Success Metrics**:
- **Detection accuracy**: 95% (100 trials)
- **Grasp success rate**: 88% (50 trials)
- **Average time**: 12 seconds per grasp

**Performance Analysis**:
| Metric | Value |
|--------|-------|
| Detection time | 0.2s |
| Planning time | 0.8s |
| Execution time | 8s |
| Grasp time | 3s |

### Challenges & Solutions

**Challenge 1**: Lighting variations affecting detection
- **Solution**: HSV color space, adaptive thresholding

**Challenge 2**: IK solver failing for some positions
- **Solution**: Added reachability check, fallback positions

**Challenge 3**: Dropping objects during transport
- **Solution**: Implemented force feedback, adjusted grip strength

### Lessons Learned

1. **Vision is hard**: Spend time on robust detection
2. **Test edge cases**: Occlusions, poor lighting, etc.
3. **Failure recovery**: Always have fallback behaviors
4. **Sensor fusion helps**: Combine vision and force
5. **Real-world messier**: Simulation worked better than hardware

---

## Example 3: Bipedal Walking ⭐⭐⭐⭐☆

### Project Overview

**Student**: James Park  
**Duration**: 8 weeks  
**Platform**: MuJoCo + Python  
**Difficulty**: Advanced

**Goal**: Implement stable bipedal walking using ZMP-based control.

### Technical Approach

**Control Strategy**:
- Zero-Moment Point (ZMP) planning
- Preview control for stability
- Joint-level PD control
- State machine for gait phases

**Gait Cycle**:
```
Double Support → Right Swing → Double Support → Left Swing → Repeat
```

### Implementation Highlights

**ZMP Planner**:
```python
class ZMPPlanner:
    def __init__(self, robot_height, com_height):
        self.g = 9.81
        self.h = com_height
        self.T = np.sqrt(self.h / self.g)  # Time constant
    
    def plan_com_trajectory(self, zmp_ref, duration):
        """Plan CoM trajectory to track ZMP reference."""
        # Use preview control to generate stable CoM trajectory
        # that keeps ZMP within support polygon
        pass
```

**Footstep Planner**:
```python
def plan_footsteps(start, goal, step_length=0.2):
    """Generate sequence of footsteps."""
    footsteps = []
    current = start
    
    while distance(current, goal) > step_length:
        # Alternate left and right foot
        next_step = current + direction * step_length
        footsteps.append(next_step)
        current = next_step
    
    return footsteps
```

### Results

**Success Metrics**:
- **Walking speed**: 0.3 m/s
- **Stability margin**: 2cm ZMP error
- **Continuous walking**: 50+ steps
- **Turning**: ±30° per step

**Video Results**:
- Forward walking on flat ground
- Walking with turns
- Recovery from small pushes
- Walking on slight incline (5°)

### Challenges & Solutions

**Challenge 1**: Falling during swing phase
- **Solution**: Adjusted CoM trajectory timing, increased support margin

**Challenge 2**: Foot scuffing ground
- **Solution**: Higher foot lift, better swing trajectory

**Challenge 3**: Instability at walking transitions
- **Solution**: Smooth velocity ramping, longer double support

**Challenge 4**: Sim-to-real gap
- **Solution**: Domain randomization, conservative parameters

### Lessons Learned

1. **Stability first**: Get standing stable before walking
2. **Small steps**: Start with slow, small steps
3. **Debug visualization**: Plot ZMP, CoM, support polygon
4. **Parameter sensitivity**: Small changes have big effects
5. **Patience required**: Walking is hard!

---

## Example 4: RL-Based Manipulation ⭐⭐⭐⭐⭐

### Project Overview

**Student**: Sarah Kim  
**Duration**: 10 weeks  
**Platform**: Isaac Sim + PyTorch  
**Difficulty**: Expert

**Goal**: Learn object manipulation policy using reinforcement learning.

### Technical Approach

**RL Algorithm**: Proximal Policy Optimization (PPO)

**State Space** (42-dim):
- Arm joint positions (7)
- Arm joint velocities (7)
- End-effector pose (6)
- Object pose (6)
- Goal pose (6)
- Gripper state (2)
- Object-gripper relative pose (6)
- Contact forces (2)

**Action Space** (8-dim):
- Joint position targets (7)
- Gripper command (1)

**Reward Function**:
```python
reward = -distance_to_goal * 10.0 \
         + success_bonus * 100.0 \
         - action_magnitude * 0.1 \
         + progress_reward * 5.0
```

### Training Process

**Domain Randomization**:
- Object mass: ±30%
- Object size: ±20%
- Friction: 0.5-1.5
- Initial pose: randomized
- Visual randomization: colors, lighting

**Training Stats**:
- Total timesteps: 10M
- Training time: 48 hours (4x RTX 3090)
- Parallel envs: 4096
- Success rate: 85% (final)

### Results

**Performance**:
- **Pick success**: 90%
- **Place accuracy**: ±2cm
- **Average time**: 8 seconds
- **Generalization**: Works on novel objects

### Lessons Learned

1. **Domain randomization crucial**: Enables sim-to-real
2. **Reward shaping matters**: Carefully design rewards
3. **Curriculum learning helps**: Start easy, increase difficulty
4. **Parallelization essential**: Train on many environments
5. **Patience and compute**: RL takes time and resources

---

## 🎓 Key Takeaways from Examples

### Common Success Factors

1. **Clear goals**: Well-defined success criteria
2. **Iterative development**: Start simple, add complexity
3. **Extensive testing**: Test edge cases and failures
4. **Good documentation**: Explain decisions and results
5. **Video demonstrations**: Show system working

### Common Pitfalls to Avoid

1. **Overambitious scope**: Start smaller than you think
2. **Poor time management**: Leave time for integration
3. **Weak testing**: Don't just test happy path
4. **Late documentation**: Document as you go
5. **Ignoring failures**: Learn from what doesn't work

---

## 📚 More Resources

- **Project templates**: github.com/physical-ai/project-templates
- **Example code**: github.com/physical-ai/examples
- **Video tutorials**: youtube.com/physical-ai-channel
- **Community forum**: forum.physical-ai.org

---

**[← Previous: Evaluation Rubric](./evaluation-rubric.md)** | **[Back to Physical AI Home →](../index.md)**
