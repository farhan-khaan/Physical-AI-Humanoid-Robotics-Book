---
title: The Sense-Think-Act Loop
description: Understanding the fundamental cycle of robot perception, reasoning, and action
sidebar_position: 3
---

# The Sense-Think-Act Loop

The **sense-think-act loop** is the fundamental operating cycle of all embodied AI systems. It's the continuous process by which robots interact with their environment.

## 🔄 The Basic Loop

```
     ┌─────────┐
     │  SENSE  │  ← Gather information from environment
     └────┬────┘
          │ Sensor Data
          ↓
     ┌─────────┐
     │  THINK  │  ← Process information, make decisions
     └────┬────┘
          │ Commands
          ↓
     ┌─────────┐
     │   ACT   │  ← Execute physical actions
     └────┬────┘
          │ Changes Environment
          ↓
     (Loop back to SENSE)
```

This isn't a one-time process - it's a **continuous loop** running at high frequency (often 10-1000 Hz depending on the task).

## 👁️ Phase 1: SENSE

**Goal**: Perceive the current state of the world and the robot's body.

### What Gets Sensed?

Robots use two types of sensors:

#### Exteroceptive Sensors (World-facing)
These sense the external environment:

- **Cameras**: Visual information (RGB, depth, infrared)
- **LIDAR**: Distance measurements using lasers
- **Microphones**: Sound and audio signals
- **Tactile sensors**: Touch and pressure on skin/fingers
- **Force/Torque sensors**: External forces applied to the robot

#### Proprioceptive Sensors (Body-facing)
These sense the robot's own state:

- **Joint encoders**: Current position of each joint
- **IMU (Inertial Measurement Unit)**: Orientation, acceleration, angular velocity
- **Current sensors**: Motor current (indicates effort/load)
- **Velocity sensors**: Speed of joint movement

### Example: Robot Reaching for a Cup

```python
# SENSE: Gather current state
def sense_environment(robot):
    # External perception
    camera_image = robot.camera.capture()
    cup_position = vision_system.detect_cup(camera_image)
    
    # Internal perception  
    arm_joint_angles = robot.arm.get_joint_positions()
    hand_position = robot.arm.forward_kinematics(arm_joint_angles)
    gripper_state = robot.gripper.get_opening_width()
    
    return {
        'cup_position': cup_position,
        'hand_position': hand_position,
        'arm_angles': arm_joint_angles,
        'gripper_width': gripper_state
    }
```

### Challenges in Sensing

:::warning Sensor Limitations
- **Noise**: Sensor readings are never perfect
- **Latency**: There's a delay between reality and measurement
- **Limited range**: Cameras can't see through walls, encoders can't detect external forces
- **Occlusion**: Objects can block sensors' view
- **Calibration drift**: Sensors become less accurate over time
:::

## 🧠 Phase 2: THINK

**Goal**: Process sensor information and decide what action to take.

This is where the AI "brain" operates. The complexity varies dramatically:

### Simple Reactive Control
Direct mapping from sensors to actions (no reasoning about the future):

```python
# Simple example: Obstacle avoidance
def think_reactive(sensor_data):
    distance_to_obstacle = sensor_data['lidar_front']
    
    if distance_to_obstacle < 0.5:  # Less than 50cm
        return {'action': 'turn_right', 'speed': 0.3}
    else:
        return {'action': 'move_forward', 'speed': 0.5}
```

**Pros**: Fast, reliable, easy to understand  
**Cons**: No planning, can't handle complex situations

### Deliberative Planning
Build a model of the world, consider future outcomes:

```python
# Complex example: Path planning
def think_deliberative(sensor_data, goal):
    # Build world model
    world_map = build_map_from_sensors(sensor_data)
    current_position = sensor_data['robot_position']
    
    # Plan sequence of actions
    path = a_star_search(
        start=current_position,
        goal=goal,
        obstacles=world_map.obstacles
    )
    
    # Return first action in plan
    next_waypoint = path[0]
    return compute_motion_to_waypoint(current_position, next_waypoint)
```

**Pros**: Can solve complex problems, finds optimal solutions  
**Cons**: Slow, can fail if world model is wrong

### Hybrid Architectures
Combine reactive and deliberative approaches:

```python
def think_hybrid(sensor_data, goal, world_model):
    # High-level deliberative planning (runs slowly)
    if time_to_replan():
        global_plan = plan_path_to_goal(world_model, goal)
    
    # Low-level reactive control (runs fast)
    immediate_obstacle = sensor_data['proximity_sensor']
    if immediate_obstacle < DANGER_THRESHOLD:
        return emergency_avoid(immediate_obstacle)  # React!
    else:
        return follow_plan(global_plan, sensor_data)  # Deliberate
```

### Modern Learned Control
Use neural networks trained on data:

```python
# Learning-based approach
def think_learned(sensor_data, goal):
    # Neural network directly maps observations to actions
    observation = preprocess_sensors(sensor_data)
    goal_encoding = encode_goal(goal)
    
    # Forward pass through trained network
    action = policy_network.forward(observation, goal_encoding)
    
    return action
```

**Pros**: Can learn complex behaviors, handles uncertainty well  
**Cons**: Requires lots of training data, hard to interpret

## 🦾 Phase 3: ACT

**Goal**: Execute the decided action by commanding actuators.

### Types of Actions

#### Joint-Space Control
Command individual joint positions, velocities, or torques:

```python
# Example: Move arm to specific joint angles
def act_joint_space(action):
    target_angles = action['joint_positions']
    robot.arm.set_joint_positions(target_angles)
```

#### Task-Space Control
Command end-effector (hand) position/orientation:

```python
# Example: Move hand to specific 3D position
def act_task_space(action):
    target_hand_position = action['end_effector_position']
    
    # Convert to joint angles using inverse kinematics
    joint_angles = robot.arm.inverse_kinematics(target_hand_position)
    
    # Command joints
    robot.arm.set_joint_positions(joint_angles)
```

#### Force Control
Command forces instead of positions:

```python
# Example: Apply gentle force when grasping
def act_force_control(action):
    target_grip_force = action['grasp_force']
    robot.gripper.set_force(target_grip_force)
```

### Control Frequency

Different tasks require different control rates:

| Task | Typical Frequency | Why? |
|------|------------------|------|
| Balance control | 100-1000 Hz | Fast corrections needed to prevent falling |
| Manipulation | 10-100 Hz | Object interaction requires responsive control |
| Navigation | 1-10 Hz | Environment changes slowly relative to robot |
| High-level planning | 0.1-1 Hz | Strategic decisions don't need rapid updates |

## 🔗 Closing the Loop

The magic happens when you close the loop: your actions change the world, which changes what you sense, which affects your next action.

### Example: Balancing on One Foot

```python
# Simplified balance control loop
def balance_control_loop():
    while robot.is_active():
        # SENSE: Am I tilting?
        imu_data = robot.imu.read()
        tilt_angle = imu_data['pitch']
        tilt_rate = imu_data['pitch_rate']
        
        # THINK: How should I correct?
        # PID controller
        correction_torque = (
            Kp * tilt_angle +           # Proportional to current tilt
            Kd * tilt_rate              # Proportional to tilt velocity
        )
        
        # ACT: Apply corrective torque
        robot.ankle.set_torque(-correction_torque)
        
        # Wait for next control cycle
        sleep(0.01)  # 100 Hz control loop
```

This creates a **feedback loop**: 
1. Sense tilt → 
2. Compute correction → 
3. Apply torque → 
4. Body moves → 
5. Tilt changes → 
6. Sense new tilt → (repeat)

## ⚡ Real-Time Constraints

The sense-think-act loop must run fast enough to keep up with the dynamics of the task.

:::danger Timing is Critical
If your control loop runs too slowly:
- Balance controller too slow → robot falls over
- Obstacle avoidance too slow → robot crashes
- Catching controller too slow → ball slips through fingers
:::

### Example: Why Speed Matters

```python
# Scenario: Robot trying to catch a falling ball

# SLOW LOOP (10 Hz = 0.1 second per cycle)
t = 0.0
while True:
    ball_position = sense()           # t = 0.0s
    hand_command = think(ball_position)  # t = 0.05s
    act(hand_command)                 # t = 0.1s
    sleep(0.1)                        # Wait until t = 0.2s
    
# Problem: Ball has moved significantly in 0.2 seconds!
# At falling speed of 5 m/s, ball moves 1 meter in this time

# FAST LOOP (100 Hz = 0.01 second per cycle)
t = 0.0
while True:
    ball_position = sense()           # t = 0.0s
    hand_command = think(ball_position)  # t = 0.005s
    act(hand_command)                 # t = 0.01s
    sleep(0.01)                       # Wait until t = 0.02s
    
# Much better: Ball only moves 0.1 meters between updates
```

## 🎯 Complete Example: Humanoid Walking

Let's see how all three phases work together for a humanoid robot walking:

```python
class WalkingController:
    def __init__(self, robot):
        self.robot = robot
        self.target_velocity = [0, 0]  # [forward, sideways] m/s
        
    def control_loop(self):
        """Main sense-think-act loop for walking"""
        while True:
            # ===== SENSE =====
            state = self.sense()
            
            # ===== THINK =====
            actions = self.think(state)
            
            # ===== ACT =====
            self.act(actions)
            
            # Run at 100 Hz
            sleep(0.01)
    
    def sense(self):
        """Gather all relevant sensor data"""
        return {
            # Body state
            'imu': self.robot.imu.read(),
            'joint_angles': self.robot.get_all_joint_positions(),
            'joint_velocities': self.robot.get_all_joint_velocities(),
            'foot_contact': [
                self.robot.left_foot.force_sensor.read(),
                self.robot.right_foot.force_sensor.read()
            ],
            
            # Environment perception
            'ground_height': self.robot.lidar.measure_ground(),
            'obstacles': self.robot.camera.detect_obstacles()
        }
    
    def think(self, state):
        """Decide on leg movements for walking"""
        # Determine which phase of gait we're in
        gait_phase = self.compute_gait_phase(state['foot_contact'])
        
        # Plan footstep locations
        next_footstep = self.plan_footstep(
            state['joint_angles'],
            self.target_velocity,
            state['obstacles']
        )
        
        # Compute balance-maintaining torso adjustments
        balance_adjustment = self.compute_balance(
            state['imu'],
            state['foot_contact']
        )
        
        # Generate joint trajectories
        target_joint_positions = self.inverse_kinematics(
            next_footstep,
            balance_adjustment
        )
        
        return {
            'joint_positions': target_joint_positions,
            'gait_phase': gait_phase
        }
    
    def act(self, actions):
        """Execute the computed actions"""
        self.robot.set_joint_positions(actions['joint_positions'])
```

## 📊 Visualizing the Loop

For a humanoid robot walking forward, here's what's happening at each moment:

| Time | Sense | Think | Act |
|------|-------|-------|-----|
| t=0.00s | Left foot touching ground, tilting forward 2° | Need to swing right leg forward, adjust torso to maintain balance | Command right hip to flex 30°, ankle to adjust 5° |
| t=0.01s | Right foot lifting, tilt now 1.5° | Continue right leg swing, balance is improving | Continue hip flexion, adjust arms for momentum |
| t=0.02s | Right foot in air, tilt 1° | Prepare for right foot landing, predict impact | Begin extending right knee for landing |
| t=0.03s | Right foot about to contact, tilt 0.5° | Stiffen right leg, prepare to transfer weight | Increase right leg stiffness, shift balance right |

This continues at 100 Hz - 100 sense-think-act cycles every second!

## ✅ Key Takeaways

1. The sense-think-act loop is the fundamental operating cycle of embodied AI
2. **Sense**: Gather information using exteroceptive and proprioceptive sensors
3. **Think**: Process information and decide on actions (reactive, deliberative, or hybrid)
4. **Act**: Execute actions through actuators (joint, task, or force control)
5. The loop runs continuously, creating feedback between robot and environment
6. Control frequency must match task dynamics - faster tasks need faster loops
7. Real-time constraints are critical - slow loops lead to failure

## 🧪 Hands-On Exercise

Try implementing a simple sense-think-act loop in simulation:

```python
# Exercise: Implement a simple obstacle avoider
# Complete the think() function

def sense(robot):
    return {
        'front_distance': robot.lidar.read_front(),
        'left_distance': robot.lidar.read_left(),
        'right_distance': robot.lidar.read_right()
    }

def think(sensor_data):
    # TODO: Implement your logic here
    # If obstacle ahead, turn away from it
    # Otherwise, move forward
    pass

def act(robot, command):
    robot.set_velocity(command['forward'], command['turn'])

# Main loop
while True:
    state = sense(robot)
    command = think(state)
    act(robot, command)
    sleep(0.1)  # 10 Hz
```

**[Next: Applications in Humanoid Robotics →](./applications.md)**
