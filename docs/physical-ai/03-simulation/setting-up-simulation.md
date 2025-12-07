---
title: Setting Up Simulation
description: Step-by-step guide to setting up robot simulations
sidebar_position: 4
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# Setting Up Simulation

This section provides hands-on guidance for setting up robot simulations. We'll walk through installation, configuration, and running your first simulations in multiple platforms.

## 🎯 Learning Outcomes

By the end of this section, you will be able to:

1. **Install** simulation platforms on your system
2. **Load** robot models into simulation
3. **Configure** physics parameters appropriately
4. **Add** sensors and actuators to simulated robots
5. **Run** basic simulation loops
6. **Debug** common simulation issues

## 📋 Prerequisites

- Python 3.8+ installed
- Basic command line familiarity
- 4GB+ RAM, graphics card recommended
- Understanding of simulation platforms (previous section)

---

## 🐍 PyBullet Setup

### System Requirements

- **OS**: Windows, macOS, Linux
- **Python**: 3.7+
- **RAM**: 2GB minimum
- **GPU**: Optional (CPU rendering available)

### Installation

<Tabs groupId="os">
<TabItem value="linux" label="Linux" default>

```bash
# Install PyBullet
pip install pybullet

# Verify installation
python -c "import pybullet as p; print(p.getVersionInfo())"

# Install additional tools (optional)
pip install numpy matplotlib
```

</TabItem>
<TabItem value="mac" label="macOS">

```bash
# Install PyBullet
pip3 install pybullet

# Verify installation
python3 -c "import pybullet as p; print(p.getVersionInfo())"

# Install additional tools (optional)
pip3 install numpy matplotlib
```

</TabItem>
<TabItem value="windows" label="Windows">

```powershell
# Install PyBullet
pip install pybullet

# Verify installation
python -c "import pybullet as p; print(p.getVersionInfo())"

# Install additional tools (optional)
pip install numpy matplotlib
```

</TabItem>
</Tabs>

### Your First PyBullet Simulation

```python
import pybullet as p
import pybullet_data
import time

def first_simulation():
    """Run your first PyBullet simulation."""
    
    # Step 1: Connect to physics server
    physics_client = p.connect(p.GUI)  # Use p.DIRECT for headless
    print("Connected to PyBullet")
    
    # Step 2: Set up search paths
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    # Step 3: Configure physics
    p.setGravity(0, 0, -9.8)
    p.setTimeStep(1./240.)  # 240 Hz
    p.setRealTimeSimulation(0)  # Step manually
    
    # Step 4: Load environment
    plane_id = p.loadURDF("plane.urdf")
    print(f"Loaded plane with ID: {plane_id}")
    
    # Step 5: Load robot
    start_pos = [0, 0, 1]
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    robot_id = p.loadURDF("r2d2.urdf", start_pos, start_orientation)
    print(f"Loaded robot with ID: {robot_id}")
    
    # Step 6: Get robot information
    num_joints = p.getNumJoints(robot_id)
    print(f"Robot has {num_joints} joints")
    
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot_id, i)
        print(f"Joint {i}: {joint_info[1].decode('utf-8')}")
    
    # Step 7: Run simulation loop
    print("Running simulation...")
    for step in range(2000):
        p.stepSimulation()
        
        # Every 240 steps = 1 second
        if step % 240 == 0:
            pos, orn = p.getBasePositionAndOrientation(robot_id)
            print(f"Time {step/240:.1f}s: Robot at {pos}")
        
        time.sleep(1./240.)  # Real-time
    
    # Step 8: Clean up
    p.disconnect()
    print("Simulation ended")

if __name__ == "__main__":
    first_simulation()
```

:::tip Success Check
You should see a GUI window with a robot falling and landing on the ground. The console shows robot position updates every second.
:::

### Loading Custom URDF

```python
def load_custom_robot():
    """Load a custom robot from URDF file."""
    
    p.connect(p.GUI)
    p.setGravity(0, 0, -9.8)
    
    # Load your custom URDF
    robot_id = p.loadURDF(
        "path/to/your/robot.urdf",
        basePosition=[0, 0, 0.5],
        baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
        useFixedBase=False,  # True for fixed base robots
        flags=p.URDF_USE_SELF_COLLISION  # Enable self-collision
    )
    
    # Print joint info
    for i in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, i)
        print(f"{info[1].decode()}: {info[2]}")  # Name, type
    
    return robot_id
```

### Configuring Physics

```python
def configure_physics():
    """Configure physics parameters for better simulation."""
    
    p.connect(p.GUI)
    
    # Time step (smaller = more accurate, slower)
    p.setTimeStep(1./240.)
    
    # Gravity
    p.setGravity(0, 0, -9.81)
    
    # Solver parameters
    p.setPhysicsEngineParameter(
        numSolverIterations=50,  # More = more stable
        numSubSteps=1,
        contactBreakingThreshold=0.001,
        erp=0.1,  # Error reduction parameter
        contactERP=0.1,
        frictionERP=0.1
    )
    
    # Load robot
    robot_id = p.loadURDF("humanoid.urdf", [0, 0, 1])
    
    # Change dynamics of specific link
    p.changeDynamics(
        robot_id,
        linkIndex=-1,  # -1 for base
        mass=50.0,
        lateralFriction=1.0,
        spinningFriction=0.1,
        rollingFriction=0.0,
        restitution=0.1,  # Bounciness
        linearDamping=0.04,
        angularDamping=0.04
    )
```

---

## 🚀 Gazebo Setup

### System Requirements

- **OS**: Ubuntu (official), macOS/Windows (limited)
- **RAM**: 4GB minimum, 8GB recommended
- **GPU**: Recommended for good performance

### Installation

<Tabs groupId="gazebo-version">
<TabItem value="classic" label="Gazebo Classic (ROS 1)" default>

```bash
# Ubuntu 20.04 with ROS Noetic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update

# Install ROS Noetic with Gazebo
sudo apt install ros-noetic-desktop-full

# Install Gazebo ROS packages
sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control

# Source ROS
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify installation
gazebo --version
```

</TabItem>
<TabItem value="ignition" label="Gazebo Ignition (Standalone)">

```bash
# Install Ignition Fortress
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt update
sudo apt install ignition-fortress

# Verify installation
ign gazebo --version
```

</TabItem>
</Tabs>

### Your First Gazebo Simulation

#### Launch Empty World

```bash
# Launch Gazebo with empty world
gazebo
```

#### Create Launch File

```xml
<!-- my_first_sim.launch -->
<launch>
  <!-- Start Gazebo with empty world -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find gazebo_ros)/worlds/empty.world"/>
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
    <arg name="headless" value="false"/>
    <arg name="debug" value="false"/>
    <arg name="verbose" value="true"/>
  </include>

  <!-- Spawn robot -->
  <param name="robot_description" 
         command="$(find xacro)/xacro $(find my_robot)/urdf/robot.xacro"/>
  
  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model"
        args="-urdf -model my_robot -param robot_description -z 1.0"
        output="screen"/>
</launch>
```

#### Run Launch File

```bash
roslaunch my_package my_first_sim.launch
```

### Adding Sensors in Gazebo

```xml
<!-- Camera sensor in URDF/Xacro -->
<gazebo reference="camera_link">
  <sensor type="camera" name="camera1">
    <update_rate>30.0</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>800</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>0.0</updateRate>
      <cameraName>robot/camera1</cameraName>
      <imageTopicName>image_raw</imageTopicName>
      <cameraInfoTopicName>camera_info</cameraInfoTopicName>
      <frameName>camera_link</frameName>
    </plugin>
  </sensor>
</gazebo>
```

---

## 🎮 Isaac Sim Setup

### System Requirements

- **OS**: Ubuntu 20.04/22.04, Windows 10/11
- **GPU**: NVIDIA RTX series (2000+)
- **RAM**: 32GB recommended
- **Storage**: 50GB free space
- **NVIDIA Driver**: Latest

### Installation

1. **Download Isaac Sim**
   - Visit [NVIDIA Isaac Sim](https://developer.nvidia.com/isaac-sim)
   - Create NVIDIA account (free)
   - Download Omniverse Launcher

2. **Install via Omniverse Launcher**
   ```bash
   # Linux: Run the downloaded AppImage
   chmod +x omniverse-launcher-linux.AppImage
   ./omniverse-launcher-linux.AppImage
   ```

3. **Install Isaac Sim from Launcher**
   - Open Omniverse Launcher
   - Navigate to "Exchange" tab
   - Find "Isaac Sim" and click Install
   - Wait for ~40GB download

### Your First Isaac Sim Script

```python
from omni.isaac.kit import SimulationApp

# Launch Isaac Sim
simulation_app = SimulationApp({
    "headless": False,  # Show GUI
    "width": 1280,
    "height": 720
})

# Import Isaac Sim modules (must be after SimulationApp creation)
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.prims import XFormPrim

# Create world
world = World()
world.scene.add_default_ground_plane()

# Add a cube
cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/Cube",
        name="cube",
        position=[0, 0, 1.0],
        size=0.5,
        color=[0, 0, 1]  # Blue
    )
)

# Reset world
world.reset()

# Simulation loop
for i in range(1000):
    world.step(render=True)
    
    if i % 100 == 0:
        position, _ = cube.get_world_pose()
        print(f"Cube position: {position}")

# Close
simulation_app.close()
```

### Loading Robots in Isaac Sim

```python
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.stage import add_reference_to_stage

# Add robot from USD file
add_reference_to_stage(
    usd_path="/Isaac/Robots/Franka/franka.usd",
    prim_path="/World/Franka"
)

# Create robot instance
robot = world.scene.add(
    Robot(prim_path="/World/Franka", name="franka")
)

# Get robot info
print(f"DOF: {robot.num_dof}")
print(f"Joint names: {robot.dof_names}")

# Control robot
robot.set_joint_positions([0.0] * robot.num_dof)
```

---

## ⚡ MuJoCo Setup

### Installation

```bash
# Install MuJoCo
pip install mujoco

# Install viewer
pip install mujoco-viewer

# Verify
python -c "import mujoco; print(mujoco.__version__)"
```

### Your First MuJoCo Simulation

```python
import mujoco
import mujoco.viewer
import numpy as np

def first_mujoco_sim():
    """Run first MuJoCo simulation."""
    
    # Load model (MJCF format)
    model = mujoco.MjModel.from_xml_path('humanoid.xml')
    data = mujoco.MjData(model)
    
    print(f"Model has {model.nq} positions, {model.nv} velocities")
    print(f"Model has {model.nu} actuators")
    
    # Interactive viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Simulation loop
        for _ in range(10000):
            # Set actuator controls
            data.ctrl[:] = 0.0
            
            # Step simulation
            mujoco.mj_step(model, data)
            
            # Sync viewer
            viewer.sync()

if __name__ == "__main__":
    first_mujoco_sim()
```

---

## 🔧 Common Configuration Tasks

### 1. Setting Physics Parameters

```python
# PyBullet
p.setPhysicsEngineParameter(
    fixedTimeStep=1./240.,
    numSolverIterations=150,
    useSplitImpulse=True,
    splitImpulsePenetrationThreshold=-0.02,
    numSubSteps=1
)

# Gazebo (in world file)
"""
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
  <gravity>0 0 -9.81</gravity>
</physics>
"""

# MuJoCo (in XML)
"""
<option timestep="0.002" iterations="50" solver="Newton" gravity="0 0 -9.81"/>
"""
```

### 2. Adding Sensors

```python
# PyBullet: Camera
def setup_camera(robot_id):
    # Attach camera to robot head
    camera_link = 5  # Head link index
    
    view_matrix = p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=[0, 0, 0],
        distance=2.0,
        yaw=0,
        pitch=-30,
        roll=0,
        upAxisIndex=2
    )
    
    projection_matrix = p.computeProjectionMatrixFOV(
        fov=60,
        aspect=1.0,
        nearVal=0.1,
        farVal=100
    )
    
    # Get image
    width, height = 640, 480
    img = p.getCameraImage(
        width, height,
        viewMatrix=view_matrix,
        projectionMatrix=projection_matrix,
        renderer=p.ER_BULLET_HARDWARE_OPENGL
    )
    
    return img[2]  # RGB array
```

### 3. Controlling Joints

```python
def control_joint(robot_id, joint_index, target_position):
    """Control a joint with PD control."""
    p.setJointMotorControl2(
        bodyUniqueId=robot_id,
        jointIndex=joint_index,
        controlMode=p.POSITION_CONTROL,
        targetPosition=target_position,
        targetVelocity=0,
        force=500,
        positionGain=0.1,  # Kp
        velocityGain=0.1   # Kd
    )
```

---

## 🐛 Troubleshooting

### PyBullet Issues

:::warning GUI Won't Open
**Problem**: GUI window doesn't appear  
**Solution**:
```python
# Try different connection modes
p.connect(p.GUI)  # Direct OpenGL
p.connect(p.GUI_SERVER)  # Server mode
p.connect(p.SHARED_MEMORY)  # Shared memory
```
:::

:::warning Robot Explodes
**Problem**: Robot parts fly apart  
**Solution**:
```python
# Reduce time step
p.setTimeStep(1./500.)

# Increase solver iterations
p.setPhysicsEngineParameter(numSolverIterations=200)

# Check URDF for joint limits and damping
```
:::

### Gazebo Issues

:::warning Gazebo Won't Start
**Problem**: `gazebo: command not found`  
**Solution**:
```bash
# Source ROS environment
source /opt/ros/noetic/setup.bash

# Add to ~/.bashrc for permanent fix
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```
:::

:::warning Model Sinks Through Ground
**Problem**: Robot falls through plane  
**Solution**:
```xml
<!-- Increase collision surface parameters -->
<collision>
  <surface>
    <contact>
      <ode>
        <kp>1000000.0</kp>
        <kd>100.0</kd>
      </ode>
    </contact>
  </surface>
</collision>
```
:::

---

## ✅ Key Takeaways

1. **Installation** varies by platform; follow official guides
2. **PyBullet**: Fastest to set up, great for learning
3. **Gazebo**: More complex but powerful with ROS
4. **Isaac Sim**: Requires high-end GPU, amazing graphics
5. **Physics parameters**: Critical for stable simulation
6. **Troubleshooting**: Start simple, add complexity gradually

---

## 🧪 Practice Exercise

:::note Exercise: Multi-Platform Setup
**Difficulty**: Medium  
**Time**: 2 hours  
**Goal**: Set up the same robot in multiple simulators

**Tasks**:
1. Install PyBullet and Gazebo (or MuJoCo)
2. Find or create a simple robot URDF
3. Load robot in both simulators
4. Apply same joint commands to both
5. Compare behavior and performance

**Deliverable**: Report documenting differences observed
:::

---

**[← Previous: Simulation Platforms](./simulation-platforms.md)** | **[Next: Sim-to-Real Transfer →](./sim-to-real.md)**
