---
title: Sensors and Actuators
description: Understanding the robotic nervous system - how robots perceive and act in the physical world
sidebar_position: 1
---

# Chapter 2: Sensors and Actuators

Welcome to the "nervous system" of robotics! In this chapter, you'll learn how robots sense their environment and interact with the physical world through sensors and actuators.

## 🎯 Learning Outcomes

By the end of this chapter, you will be able to:

1. **Identify** and categorize different types of sensors used in humanoid robots
2. **Explain** how various sensor modalities provide complementary information
3. **Describe** actuator types and their characteristics (motors, servos, hydraulics)
4. **Understand** the trade-offs between sensor accuracy, speed, and cost
5. **Integrate** sensor data in practical control applications
6. **Select** appropriate sensors and actuators for specific robotics tasks

## 📋 Prerequisites

- Completion of Chapter 1 (Embodied Intelligence)
- Basic understanding of the sense-think-act loop
- Familiarity with coordinate systems and units (meters, radians, etc.)

## 🗺️ Chapter Overview

This chapter covers:

1. **[Sensor Types](./sensor-types.md)** - Comprehensive overview of robotic sensors
2. **[Actuator Types](./actuator-types.md)** - Motors, servos, and actuation mechanisms
3. **[Sensor Integration](./sensor-integration.md)** - Combining sensor data for robust perception
4. **[Exercises](./exercises.md)** - Hands-on practice with sensors and actuators

## 🧠 The Robot's Interface to Reality

Remember from Chapter 1 that embodied intelligence requires:
- A **body** to interact with the world
- **Sensors** to perceive the environment and body state
- **Actuators** to execute physical actions

In this chapter, we focus on sensors and actuators - the hardware that makes embodied intelligence possible.

## 🔌 Two Types of Sensing

### Exteroceptive Sensors (World-Sensing)
These sensors perceive the external environment:
- 👁️ Cameras - visual information
- 📡 LIDAR - distance measurements
- 🎤 Microphones - sound
- 🤲 Tactile sensors - touch and pressure

### Proprioceptive Sensors (Self-Sensing)
These sensors perceive the robot's own state:
- 📐 Encoders - joint positions
- 🧭 IMU - orientation and acceleration
- ⚡ Current sensors - motor load
- 📏 Force/torque sensors - applied forces

Both are essential for the sense-think-act loop!

## 🦾 The Actuation Challenge

Actuators convert electrical energy into mechanical motion. Key challenges:
- **Precision**: Accurate position control
- **Force**: Sufficient torque for manipulation
- **Speed**: Fast response times
- **Compliance**: Safe interaction with environment
- **Efficiency**: Minimize energy consumption

## 💡 Preview: What You'll Build

By the end of this chapter, you'll understand how to build systems like:

```python
# Simple sensor-actuator loop
class RobotArm:
    def reach_to_target(self, target_position):
        while not self.reached_target(target_position):
            # SENSE: Where am I now?
            current_position = self.encoders.read_positions()
            distance_to_target = self.camera.measure_distance(target_position)
            
            # THINK: How should I move?
            movement = self.compute_motion(current_position, target_position)
            
            # ACT: Command motors
            self.motors.set_velocities(movement)
```

## 🚀 Ready to Begin?

Let's start by exploring the wide variety of sensors that give robots their perception capabilities.

**[Start with Sensor Types →](./sensor-types.md)**

---

:::note Coming Up
In Chapter 3, you'll learn how to test these sensors and actuators in simulation before deploying to real hardware!
:::
