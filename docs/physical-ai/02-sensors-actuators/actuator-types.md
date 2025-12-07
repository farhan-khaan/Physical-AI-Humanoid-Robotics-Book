---
title: Actuator Types
description: Motors, servos, and actuation mechanisms for humanoid robots
sidebar_position: 3
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# Actuator Types

Actuators are the muscles of a robot—they convert electrical energy into mechanical motion. Understanding actuators is crucial for controlling humanoid robots effectively and safely.

## 🎯 Learning Outcomes

By the end of this section, you will be able to:

1. **Identify** different types of actuators and their characteristics
2. **Compare** DC motors, servos, steppers, and advanced actuators
3. **Select** appropriate actuators for specific tasks
4. **Implement** control code for common actuator types
5. **Understand** power, torque, and speed relationships

## 📋 Prerequisites

- Understanding of sensors (previous section)
- Basic electrical concepts (voltage, current, power)
- Python or C++ programming

---

## ⚙️ Core Actuator Types

### 1. DC Motors

DC motors are the foundation of robotic actuation. They spin continuously when powered.

#### Brushed DC Motors

**How they work**: Current flows through brushes and commutator to create rotating magnetic field

**Characteristics**:
- Simple control (apply voltage, motor spins)
- Speed proportional to voltage
- Inexpensive
- Brushes wear out over time

**Use cases**: Wheels, fans, simple continuous rotation

<Tabs groupId="programming-language">
<TabItem value="python" label="Python" default>

```python
import RPi.GPIO as GPIO
import time

# Motor control pins
MOTOR_PWM_PIN = 18
MOTOR_DIR_PIN = 23

# Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTOR_PWM_PIN, GPIO.OUT)
GPIO.setup(MOTOR_DIR_PIN, GPIO.OUT)

# Create PWM instance (1000 Hz)
pwm = GPIO.PWM(MOTOR_PWM_PIN, 1000)
pwm.start(0)

def set_motor_speed(speed):
    """
    Set motor speed.
    speed: -100 to 100 (negative = reverse)
    """
    if speed >= 0:
        GPIO.output(MOTOR_DIR_PIN, GPIO.HIGH)
        pwm.ChangeDutyCycle(min(abs(speed), 100))
    else:
        GPIO.output(MOTOR_DIR_PIN, GPIO.LOW)
        pwm.ChangeDutyCycle(min(abs(speed), 100))

# Demo: Ramp up speed, then reverse
for speed in range(0, 101, 10):
    set_motor_speed(speed)
    print(f"Speed: {speed}%")
    time.sleep(0.5)

time.sleep(1)

for speed in range(0, -101, -10):
    set_motor_speed(speed)
    print(f"Speed: {speed}%")
    time.sleep(0.5)

# Cleanup
pwm.stop()
GPIO.cleanup()
```

</TabItem>
<TabItem value="cpp" label="C++">

```cpp
#include <wiringPi.h>
#include <softPwm.h>
#include <iostream>

class DCMotor {
private:
    int pwm_pin;
    int dir_pin;
    
public:
    DCMotor(int pwm, int dir) : pwm_pin(pwm), dir_pin(dir) {
        pinMode(pwm_pin, OUTPUT);
        pinMode(dir_pin, OUTPUT);
        softPwmCreate(pwm_pin, 0, 100);
    }
    
    void setSpeed(int speed) {
        // speed: -100 to 100
        if (speed >= 0) {
            digitalWrite(dir_pin, HIGH);
            softPwmWrite(pwm_pin, std::min(abs(speed), 100));
        } else {
            digitalWrite(dir_pin, LOW);
            softPwmWrite(pwm_pin, std::min(abs(speed), 100));
        }
    }
    
    void stop() {
        softPwmWrite(pwm_pin, 0);
    }
};
```

</TabItem>
</Tabs>

#### Brushless DC Motors (BLDC)

**How they work**: Electronic controller switches current through coils (no brushes)

**Advantages over brushed**:
- No brush wear → longer life
- Higher efficiency
- Better power-to-weight ratio
- Higher speeds possible

**Disadvantages**:
- Requires ESC (Electronic Speed Controller)
- More expensive
- More complex control

**Use cases**: Drones, high-performance applications, continuous operation

:::tip BLDC Power
Brushless motors are the standard for drones and high-performance robots. They can run for years without maintenance!
:::

---

### 2. Servo Motors

Servos are position-controlled motors with built-in feedback and control circuitry.

#### Standard Servos (Hobby Servos)

**Characteristics**:
- Position control (typically 0-180°)
- Built-in PID controller
- Simple 3-wire interface (power, ground, signal)
- PWM control signal (1-2ms pulse width)

**Specifications**:
- Torque: 2-20 kg·cm
- Speed: 0.1-0.2 sec/60°
- Position accuracy: ±1-2°

<Tabs groupId="programming-language">
<TabItem value="python" label="Python" default>

```python
from adafruit_servokit import ServoKit
import time

# Initialize 16-channel PWM driver
kit = ServoKit(channels=16)

# Configure servo parameters
kit.servo[0].actuation_range = 180
kit.servo[0].set_pulse_width_range(500, 2500)

def move_servo(channel, angle):
    """Move servo to specified angle."""
    angle = max(0, min(180, angle))  # Clamp to valid range
    kit.servo[channel].angle = angle
    print(f"Servo {channel} moved to {angle}°")

# Demo: Sweep servo back and forth
print("Sweeping servo...")
for angle in range(0, 181, 10):
    move_servo(0, angle)
    time.sleep(0.1)

for angle in range(180, -1, -10):
    move_servo(0, angle)
    time.sleep(0.1)

# Set to neutral position
move_servo(0, 90)
```

</TabItem>
<TabItem value="cpp" label="C++">

```cpp
#include <wiringPi.h>
#include <softPwm.h>
#include <iostream>

class ServoMotor {
private:
    int pin;
    int min_pulse = 50;   // 0.5ms
    int max_pulse = 250;  // 2.5ms
    
public:
    ServoMotor(int p) : pin(p) {
        pinMode(pin, PWM_OUTPUT);
        pwmSetMode(PWM_MODE_MS);
        pwmSetRange(2000);
        pwmSetClock(192);
    }
    
    void setAngle(int angle) {
        // Map angle (0-180) to pulse width
        angle = std::max(0, std::min(180, angle));
        int pulse = min_pulse + 
                    (angle * (max_pulse - min_pulse)) / 180;
        pwmWrite(pin, pulse);
    }
};

int main() {
    wiringPiSetupGpio();
    ServoMotor servo(18);
    
    // Sweep
    for (int angle = 0; angle <= 180; angle += 10) {
        servo.setAngle(angle);
        delay(100);
    }
    
    return 0;
}
```

</TabItem>
</Tabs>

#### Digital/Smart Servos

**Examples**: Dynamixel, Herkulex, Robotis servos

**Features**:
- Daisy-chain communication (one wire for many servos)
- Read position, speed, temperature, load
- Advanced control modes (position, velocity, torque)
- Higher precision and reliability

```python
from dynamixel_sdk import *

# Control table addresses (Dynamixel XM430)
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

# Protocol and settings
PROTOCOL_VERSION = 2.0
DXL_ID = 1
BAUDRATE = 57600
DEVICENAME = '/dev/ttyUSB0'

# Initialize PortHandler and PacketHandler
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Opened port")
else:
    print("Failed to open port")
    quit()

# Set baudrate
portHandler.setBaudRate(BAUDRATE)

# Enable torque
packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 1)

# Move to position
goal_position = 2048  # Center position
packetHandler.write4ByteTxRx(portHandler, DXL_ID, 
                              ADDR_GOAL_POSITION, goal_position)

# Read current position
present_position, _, _ = packetHandler.read4ByteTxRx(
    portHandler, DXL_ID, ADDR_PRESENT_POSITION)

print(f"Goal: {goal_position}, Current: {present_position}")

# Cleanup
packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 0)
portHandler.closePort()
```

:::tip Dynamixel for Humanoids
Dynamixel servos are industry-standard for humanoid robots. They offer excellent feedback and can be daisy-chained, reducing wiring complexity.
:::

---

### 3. Stepper Motors

**How they work**: Move in discrete steps by energizing coils in sequence

**Characteristics**:
- Precise positioning without feedback
- Hold torque when stationary
- No cumulative error
- Can lose steps if overloaded

**Step resolution**: 1.8° (200 steps/rev) to 0.9° (400 steps/rev) typical

**Use cases**: 3D printers, CNC machines, precise positioning

```python
import RPi.GPIO as GPIO
import time

class StepperMotor:
    """Control a stepper motor using GPIO pins."""
    
    def __init__(self, pins, steps_per_rev=200):
        self.pins = pins
        self.steps_per_rev = steps_per_rev
        self.step_sequence = [
            [1, 0, 0, 1],
            [1, 0, 0, 0],
            [1, 1, 0, 0],
            [0, 1, 0, 0],
            [0, 1, 1, 0],
            [0, 0, 1, 0],
            [0, 0, 1, 1],
            [0, 0, 0, 1]
        ]
        self.step_index = 0
        
        GPIO.setmode(GPIO.BCM)
        for pin in pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, 0)
    
    def step(self, direction=1):
        """Take one step (direction: 1=forward, -1=reverse)."""
        for i, pin in enumerate(self.pins):
            GPIO.output(pin, self.step_sequence[self.step_index][i])
        
        self.step_index += direction
        self.step_index %= len(self.step_sequence)
    
    def rotate(self, degrees, speed=0.001):
        """Rotate by specified degrees."""
        steps = int(abs(degrees) * self.steps_per_rev / 360)
        direction = 1 if degrees > 0 else -1
        
        for _ in range(steps):
            self.step(direction)
            time.sleep(speed)

# Usage
motor = StepperMotor([17, 18, 27, 22])

# Rotate 90 degrees clockwise
motor.rotate(90)
time.sleep(0.5)

# Rotate 90 degrees counter-clockwise
motor.rotate(-90)

GPIO.cleanup()
```

---

## 🦾 Advanced Actuators

### Series Elastic Actuators (SEA)

**Concept**: Spring between motor and load for compliance and force control

**Advantages**:
- Shock absorption
- Accurate force control
- Safe for human interaction
- Energy storage and return

**Use cases**: Legged robots, rehabilitation devices, collaborative robots

:::note Research-Grade
SEAs are common in research humanoids like MIT's Cheetah and NASA's Valkyrie. They enable dynamic, energy-efficient locomotion.
:::

---

### Pneumatic Actuators

**How they work**: Compressed air drives pistons or artificial muscles

**Advantages**:
- High power-to-weight ratio
- Naturally compliant
- Safe failure mode (no power = soft)

**Disadvantages**:
- Requires air compressor
- Noisy
- Difficult precise control

**Use cases**: Soft robotics, grippers, large-scale motion

---

### Hydraulic Actuators

**How they work**: Pressurized fluid drives pistons

**Advantages**:
- Extremely high force/torque
- Precise force control
- High power density

**Disadvantages**:
- Heavy and complex
- Requires pump and reservoir
- Potential fluid leaks
- Expensive

**Use cases**: Large humanoid robots (Atlas, HRP-5), heavy-duty applications

:::warning Hydraulics Safety
Hydraulic systems operate at very high pressures (3000+ PSI). Always follow safety protocols and use proper fittings!
:::

---

## 📊 Actuator Comparison

| Actuator Type | Torque | Speed | Control | Cost | Complexity |
|---------------|--------|-------|---------|------|------------|
| Brushed DC | Medium | High | Simple | $ | Low |
| Brushless DC | High | Very High | Medium | $$ | Medium |
| Hobby Servo | Low-Med | Medium | Simple | $ | Low |
| Smart Servo | Medium | Medium | Advanced | $$$ | Medium |
| Stepper | Medium | Low-Med | Medium | $$ | Medium |
| SEA | High | Medium | Complex | $$$$ | High |
| Pneumatic | High | Medium | Complex | $$$ | High |
| Hydraulic | Very High | Medium | Complex | $$$$ | Very High |

---

## ⚡ Power and Control Concepts

### Torque vs Speed

All motors have a torque-speed curve:

```
Torque ^
       |
   Max |●
       | \
       |  \
       |   \
       |    \
       |     ●___________
       +--------------> Speed
       0              Max
```

- **High torque** = slow speed
- **Low torque** = high speed
- **Gear reduction** trades speed for torque

### Gear Ratios

```python
def calculate_output_specs(motor_speed_rpm, motor_torque_nm, gear_ratio):
    """Calculate output specs after gearing."""
    output_speed = motor_speed_rpm / gear_ratio
    output_torque = motor_torque_nm * gear_ratio * 0.9  # 90% efficiency
    
    print(f"Input: {motor_speed_rpm} RPM, {motor_torque_nm} Nm")
    print(f"Gear Ratio: {gear_ratio}:1")
    print(f"Output: {output_speed:.1f} RPM, {output_torque:.2f} Nm")
    
    return output_speed, output_torque

# Example: 100:1 gearbox
calculate_output_specs(motor_speed_rpm=6000, 
                       motor_torque_nm=0.05, 
                       gear_ratio=100)
# Output: 60 RPM, 4.5 Nm
```

### Power Consumption

Power (Watts) = Voltage (V) × Current (A)

```python
def estimate_battery_life(battery_capacity_mah, voltage_v, 
                         motor_current_a, num_motors):
    """Estimate battery life with given motors."""
    battery_capacity_ah = battery_capacity_mah / 1000
    total_current_a = motor_current_a * num_motors
    
    hours = battery_capacity_ah / total_current_a
    minutes = hours * 60
    
    total_power_w = voltage_v * total_current_a
    
    print(f"Battery: {battery_capacity_mah}mAh @ {voltage_v}V")
    print(f"Motors: {num_motors} × {motor_current_a}A = {total_current_a}A")
    print(f"Total Power: {total_power_w}W")
    print(f"Estimated Runtime: {minutes:.1f} minutes")
    
    return hours

# Example: humanoid with 18 servos
estimate_battery_life(battery_capacity_mah=5000,
                     voltage_v=11.1,  # 3S LiPo
                     motor_current_a=0.5,
                     num_motors=18)
```

:::warning Heat Management
Motors convert some energy to heat. Running at high current continuously can damage motors. Use duty cycles and heat sinks when needed.
:::

---

## 🎯 Actuator Selection Guide

### For Humanoid Arms
- **Joints**: Smart servos (Dynamixel) for precise control
- **Grippers**: Small servos or pneumatic fingers
- **Torque requirements**: 5-50 Nm depending on size

### For Humanoid Legs
- **Hip/Knee**: High-torque servos or brushless motors with gearbox
- **Ankles**: Medium-torque servos
- **Torque requirements**: 20-200 Nm depending on robot mass

### For Mobile Base
- **Wheels**: Brushless DC motors with encoders
- **Speed**: 1-2 m/s typical
- **Torque**: Depends on weight and terrain

---

## ✅ Key Takeaways

1. **DC motors** provide continuous rotation; servos provide position control
2. **Brushless motors** are more efficient and durable than brushed
3. **Smart servos** offer advanced features like feedback and daisy-chain communication
4. **Gear reduction** increases torque but decreases speed
5. **Power management** is critical for battery-powered robots
6. **Safety** requires current limiting, emergency stops, and proper mechanical design
7. **Actuator selection** depends on required torque, speed, precision, and cost

---

## 🧪 Practice Exercise

:::note Exercise: Servo Control System
**Difficulty**: Medium  
**Time**: 1 hour  
**Goal**: Create a multi-servo control system with smooth motion

**Requirements**:
1. Control at least 3 servos simultaneously
2. Implement smooth interpolation between positions
3. Add position limits to prevent mechanical damage
4. Create a sequence of predefined poses
5. Monitor and display current servo positions

**Bonus**: Add keyboard control to manually adjust servo positions
:::

---

**[← Previous: Sensor Types](./sensor-types.md)** | **[Next: Sensor Integration →](./sensor-integration.md)**
