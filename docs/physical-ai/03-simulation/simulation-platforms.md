---
title: Simulation Platforms
description: Comprehensive comparison of robotics simulation platforms
sidebar_position: 3
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# Simulation Platforms

Choosing the right simulation platform is crucial for your robotics project. Each simulator has different strengths, and understanding them helps you make the best choice for your needs.

## 🎯 Learning Outcomes

By the end of this section, you will be able to:

1. **Compare** major robotics simulation platforms
2. **Select** the right simulator for your project
3. **Understand** each platform's strengths and limitations
4. **Set up** basic simulations in multiple platforms
5. **Evaluate** trade-offs between simulators

## 📋 Prerequisites

- Understanding of digital twins (previous section)
- Basic Python programming
- Familiarity with robot file formats (URDF helpful)

---

## 🏆 Platform Comparison Overview

| Platform | Best For | Difficulty | Physics | Graphics | Cost |
|----------|----------|------------|---------|----------|------|
| **PyBullet** | Learning, quick prototyping | Easy | Good | Basic | Free |
| **Gazebo** | ROS integration, research | Medium | Excellent | Good | Free |
| **Isaac Sim** | ML training, photorealism | Hard | Excellent | Excellent | Free |
| **MuJoCo** | Fast physics, research | Medium | Excellent | Basic | Free |
| **Webots** | Education, cross-platform | Medium | Good | Good | Free |
| **CoppeliaSim** | Multi-robot, education | Medium | Good | Good | Free/Paid |

---

## 🐍 PyBullet

### Overview
PyBullet is a Python module for physics simulation, perfect for learning and rapid prototyping.

**Developer**: Erwin Coumans (Bullet Physics)  
**Language**: Python (C++ backend)  
**First Release**: 2016

### Strengths ✅
- **Easy to learn**: Simple Python API
- **Fast setup**: `pip install pybullet` and you're ready
- **Good documentation**: Many examples and tutorials
- **Lightweight**: Runs on laptops without GPU
- **Interactive**: GUI for debugging and visualization
- **Versatile**: Supports URDF, SDF, MJCF formats

### Limitations ❌
- Basic graphics (not photorealistic)
- Limited sensor models compared to Gazebo
- Single-threaded physics
- Smaller community than Gazebo/ROS

<Tabs groupId="simulator">
<TabItem value="pybullet" label="PyBullet Example" default>

### Installation

```bash
pip install pybullet
```

### Basic Example

```python
import pybullet as p
import pybullet_data
import time

# Connect to physics server
physicsClient = p.connect(p.GUI)  # or p.DIRECT for no graphics

# Set search path for URDF files
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Configure simulation
p.setGravity(0, 0, -9.8)
p.setTimeStep(1./240.)

# Load environment and robot
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("humanoid/humanoid.urdf", [0, 0, 1])

# Get information about robot
num_joints = p.getNumJoints(robotId)
print(f"Robot has {num_joints} joints")

# Run simulation
for i in range(10000):
    # Example: Control joint
    p.setJointMotorControl2(
        robotId,
        jointIndex=0,
        controlMode=p.POSITION_CONTROL,
        targetPosition=0.5,
        force=500
    )
    
    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()
```

### Camera and Sensors

```python
# Get camera image
width, height = 640, 480
view_matrix = p.computeViewMatrixFromYawPitchRoll(
    cameraTargetPosition=[0, 0, 0],
    distance=2,
    yaw=45,
    pitch=-30,
    roll=0,
    upAxisIndex=2
)
projection_matrix = p.computeProjectionMatrixFOV(
    fov=60, aspect=width/height, nearVal=0.1, farVal=100
)

img = p.getCameraImage(
    width, height,
    viewMatrix=view_matrix,
    projectionMatrix=projection_matrix
)

# img[2] = RGB array, img[3] = depth array
```

</TabItem>
</Tabs>

:::tip Best Use Cases for PyBullet
- Learning robotics simulation
- Rapid prototyping of controllers
- Reinforcement learning research
- Projects without ROS dependency
- Lightweight applications
:::

---

## 🚀 Gazebo

### Overview
Gazebo is the industry-standard simulator for ROS (Robot Operating System), offering excellent physics and extensive robot models.

**Developer**: Open Robotics  
**Language**: C++ with ROS bindings  
**First Release**: 2004 (Gazebo Classic), 2019 (Gazebo Ignition/New)

### Strengths ✅
- **ROS integration**: Seamless ROS/ROS2 support
- **Extensive models**: Large library of robots and environments
- **Plugin ecosystem**: Customize with C++ plugins
- **Realistic sensors**: Detailed sensor simulations
- **Multi-robot**: Easily simulate swarms
- **Active community**: Large user base, good support

### Limitations ❌
- Steeper learning curve
- More resource-intensive
- Setup can be complex (especially on non-Linux)
- Graphics less advanced than Isaac Sim

<Tabs groupId="simulator">
<TabItem value="gazebo" label="Gazebo Example">

### Installation (Ubuntu)

```bash
# Gazebo Classic (with ROS Noetic)
sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control

# Or Gazebo Ignition (standalone)
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt update
sudo apt install ignition-fortress
```

### Launch File Example

```xml
<!-- my_robot.launch -->
<launch>
  <!-- Start Gazebo with empty world -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
    <arg name="headless" value="false"/>
    <arg name="debug" value="false"/>
  </include>

  <!-- Spawn robot in Gazebo -->
  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model"
        args="-file $(find my_robot)/urdf/robot.urdf -urdf -model my_robot -z 0.5"
        output="screen"/>
</launch>
```

### Python Control Example

```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class GazeboController:
    def __init__(self):
        rospy.init_node('robot_controller')
        
        # Publishers for joint commands
        self.joint_pub = rospy.Publisher(
            '/robot/joint_position_controller/command',
            Float64, queue_size=10
        )
        
        # Subscriber for joint states
        rospy.Subscriber('/robot/joint_states', JointState, 
                        self.joint_callback)
        
        self.current_position = 0.0
    
    def joint_callback(self, msg):
        """Receive current joint states."""
        if msg.name:
            self.current_position = msg.position[0]
    
    def control_loop(self):
        """Main control loop."""
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            # Command joint position
            target = 1.57  # 90 degrees
            self.joint_pub.publish(target)
            
            print(f"Target: {target:.2f}, Current: {self.current_position:.2f}")
            rate.sleep()

if __name__ == '__main__':
    controller = GazeboController()
    controller.control_loop()
```

</TabItem>
</Tabs>

:::tip Best Use Cases for Gazebo
- ROS/ROS2 projects
- Multi-robot simulations
- Research with published models
- Realistic sensor simulation
- Integration with existing ROS ecosystem
:::

---

## 🎮 NVIDIA Isaac Sim

### Overview
Isaac Sim is NVIDIA's photorealistic simulator built on Omniverse, designed for robotics AI and machine learning.

**Developer**: NVIDIA  
**Language**: Python, C++  
**First Release**: 2020

### Strengths ✅
- **Photorealistic graphics**: Ray-traced rendering
- **GPU acceleration**: Massive parallelization
- **ML integration**: Built-in for training neural networks
- **Domain randomization**: Automatic variation for training
- **ROS/ROS2 support**: Bridge to ROS ecosystem
- **Synthetic data generation**: Perfect for computer vision

### Limitations ❌
- Requires NVIDIA RTX GPU
- Large installation (~40GB)
- Steeper learning curve
- Higher system requirements
- Newer platform (smaller community)

<Tabs groupId="simulator">
<TabItem value="isaac" label="Isaac Sim Example">

### Installation

1. Download from [NVIDIA Isaac Sim](https://developer.nvidia.com/isaac-sim)
2. Requires NVIDIA GPU (RTX series recommended)
3. Follow official installation guide

### Basic Example

```python
from omni.isaac.kit import SimulationApp

# Launch Isaac Sim
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.stage import add_reference_to_stage

# Create world
world = World()
world.scene.add_default_ground_plane()

# Add robot
add_reference_to_stage(
    usd_path="/Isaac/Robots/Humanoid/humanoid.usd",
    prim_path="/World/Humanoid"
)

robot = world.scene.add(Robot(prim_path="/World/Humanoid"))

# Reset world
world.reset()

# Simulation loop
for i in range(1000):
    # Step physics
    world.step(render=True)
    
    # Control robot
    if i % 100 == 0:
        # Apply joint commands
        robot.set_joint_positions([0.5] * robot.num_dof)

simulation_app.close()
```

### Camera and Rendering

```python
from omni.isaac.core.utils.render import RenderProduct
from omni.isaac.synthetic_utils import SyntheticDataHelper

# Create camera
camera_path = "/World/Camera"
camera = world.scene.add(Camera(prim_path=camera_path))

# Setup synthetic data helper
sd_helper = SyntheticDataHelper()
sd_helper.initialize(sensor_names=[camera_path])

# Get photorealistic image
world.step(render=True)
rgb = sd_helper.get_rgb(camera_path)
depth = sd_helper.get_depth(camera_path)
```

</TabItem>
</Tabs>

:::tip Best Use Cases for Isaac Sim
- Training vision-based AI/ML models
- Generating synthetic datasets
- Parallel simulation (train 100s of policies)
- Photorealistic visualization
- Large-scale reinforcement learning
:::

---

## ⚡ MuJoCo

### Overview
MuJoCo (Multi-Joint dynamics with Contact) is a fast physics engine popular in reinforcement learning research.

**Developer**: DeepMind (formerly Roboti LLC)  
**Language**: C, Python bindings  
**First Release**: 2012, Open-sourced 2021

### Strengths ✅
- **Fast physics**: Optimized contact dynamics
- **Accurate**: High-quality physics simulation
- **Efficient**: Great for RL training
- **Well-documented**: Clear API and examples
- **Free since 2021**: Previously commercial

### Limitations ❌
- Basic graphics
- Limited sensor models
- Smaller ecosystem than Gazebo
- Requires MJCF format (not URDF native)

<Tabs groupId="simulator">
<TabItem value="mujoco" label="MuJoCo Example">

### Installation

```bash
pip install mujoco
```

### Basic Example

```python
import mujoco
import mujoco.viewer

# Load model
model = mujoco.MjModel.from_xml_path('humanoid.xml')
data = mujoco.MjData(model)

# Launch viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    # Simulation loop
    for _ in range(10000):
        # Apply control
        data.ctrl[0] = 0.5  # Set actuator command
        
        # Step simulation
        mujoco.mj_step(model, data)
        
        # Update viewer
        viewer.sync()
```

### Advanced Control

```python
import numpy as np

# PD controller for joint
kp = 100.0  # Proportional gain
kd = 10.0   # Derivative gain

target_position = np.pi / 4  # 45 degrees

for _ in range(1000):
    # Current joint state
    current_pos = data.qpos[0]
    current_vel = data.qvel[0]
    
    # PD control
    error = target_position - current_pos
    torque = kp * error - kd * current_vel
    
    # Apply torque
    data.ctrl[0] = torque
    
    mujoco.mj_step(model, data)
```

</TabItem>
</Tabs>

:::tip Best Use Cases for MuJoCo
- Reinforcement learning research
- Fast physics simulation
- Contact-rich tasks
- Locomotion research
- Academic publications
:::

---

## 🌍 Platform Selection Guide

### Choose PyBullet if you want:
- ✅ Easiest learning curve
- ✅ Quick prototyping
- ✅ Python-only development
- ✅ Minimal setup time
- ✅ Good physics without complexity

### Choose Gazebo if you want:
- ✅ ROS/ROS2 integration
- ✅ Large model library
- ✅ Multi-robot scenarios
- ✅ Realistic sensors
- ✅ Industry-standard tools

### Choose Isaac Sim if you want:
- ✅ Photorealistic rendering
- ✅ ML/AI training at scale
- ✅ Computer vision applications
- ✅ GPU acceleration
- ✅ Synthetic data generation

### Choose MuJoCo if you want:
- ✅ Fastest physics engine
- ✅ RL research
- ✅ Academic work
- ✅ Contact-rich manipulation
- ✅ Efficient simulation

---

## ✅ Key Takeaways

1. **Different simulators** excel at different tasks
2. **PyBullet**: Easy to learn, great for prototyping
3. **Gazebo**: Industry standard, ROS integration
4. **Isaac Sim**: ML training, photorealism
5. **MuJoCo**: Fast physics, RL research
6. **Selection depends** on your project requirements and constraints

---

**[← Previous: Digital Twins](./digital-twins.md)** | **[Next: Setting Up Simulation →](./setting-up-simulation.md)**
