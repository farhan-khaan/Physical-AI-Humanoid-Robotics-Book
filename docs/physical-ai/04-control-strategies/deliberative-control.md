---
title: Deliberative Control
description: Planning-based control strategies for complex tasks
sidebar_position: 3
---

# Deliberative Control

Deliberative control involves reasoning, planning, and optimization to achieve complex goals. While slower than reactive control, it enables sophisticated behaviors like navigation, manipulation, and multi-step task execution.

## 🎯 Learning Outcomes

By the end of this section, you will be able to:

1. **Implement** path planning algorithms (A*, RRT)
2. **Apply** inverse kinematics for reaching tasks
3. **Design** task planning systems
4. **Optimize** trajectories for smooth motion
5. **Combine** planning with reactive execution

## 📋 Prerequisites

- Understanding of reactive control (previous section)
- Basic graph algorithms
- Linear algebra (matrices, vectors)
- Python programming

---

## 🗺️ Path Planning

Path planning finds collision-free paths from start to goal.

### A* Algorithm

A* is a popular path planning algorithm that finds optimal paths efficiently.

```python
import heapq
import numpy as np

class AStarPlanner:
    """
    A* path planning algorithm.
    """
    
    def __init__(self, grid_map):
        """
        Args:
            grid_map: 2D numpy array (0=free, 1=obstacle)
        """
        self.grid_map = grid_map
        self.height, self.width = grid_map.shape
    
    def heuristic(self, a, b):
        """Euclidean distance heuristic."""
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    def get_neighbors(self, pos):
        """Get valid neighbor cells."""
        x, y = pos
        neighbors = []
        
        # 8-connected grid
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                
                nx, ny = x + dx, y + dy
                
                # Check bounds
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    # Check obstacle
                    if self.grid_map[ny, nx] == 0:
                        neighbors.append((nx, ny))
        
        return neighbors
    
    def plan(self, start, goal):
        """
        Find path from start to goal using A*.
        
        Args:
            start: (x, y) tuple
            goal: (x, y) tuple
            
        Returns:
            List of (x, y) waypoints, or None if no path
        """
        # Priority queue: (f_score, counter, position)
        counter = 0
        open_set = [(0, counter, start)]
        
        # Track best path
        came_from = {}
        
        # Cost from start
        g_score = {start: 0}
        
        # Estimated total cost
        f_score = {start: self.heuristic(start, goal)}
        
        while open_set:
            _, _, current = heapq.heappop(open_set)
            
            # Goal reached
            if current == goal:
                return self.reconstruct_path(came_from, current)
            
            # Explore neighbors
            for neighbor in self.get_neighbors(current):
                # Cost to reach neighbor
                tentative_g = g_score[current] + self.heuristic(current, neighbor)
                
                # Better path found
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                    
                    counter += 1
                    heapq.heappush(open_set, 
                                 (f_score[neighbor], counter, neighbor))
        
        # No path found
        return None
    
    def reconstruct_path(self, came_from, current):
        """Reconstruct path from came_from chain."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

# Example usage
def path_planning_example():
    # Create grid map (0=free, 1=obstacle)
    grid = np.zeros((50, 50))
    grid[10:40, 20:22] = 1  # Vertical wall
    grid[20:22, 10:40] = 1  # Horizontal wall
    
    planner = AStarPlanner(grid)
    
    start = (5, 5)
    goal = (45, 45)
    
    path = planner.plan(start, goal)
    
    if path:
        print(f"Path found with {len(path)} waypoints")
        for i, waypoint in enumerate(path[::10]):  # Every 10th point
            print(f"  Waypoint {i}: {waypoint}")
    else:
        print("No path found!")
```

### RRT (Rapidly-exploring Random Tree)

RRT is excellent for high-dimensional spaces like robot arms.

```python
class RRTPlanner:
    """
    RRT path planning for high-dimensional spaces.
    """
    
    def __init__(self, start, goal, bounds, obstacle_check_fn, 
                 step_size=0.5, max_iterations=5000):
        """
        Args:
            start: Starting configuration (list/array)
            goal: Goal configuration
            bounds: List of (min, max) for each dimension
            obstacle_check_fn: Function that returns True if config is valid
            step_size: Maximum distance to extend
            max_iterations: Maximum planning iterations
        """
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.bounds = bounds
        self.obstacle_check = obstacle_check_fn
        self.step_size = step_size
        self.max_iterations = max_iterations
        
        # Tree
        self.nodes = [self.start]
        self.parent = {tuple(self.start): None}
    
    def sample_random_config(self):
        """Sample random configuration in space."""
        config = []
        for min_val, max_val in self.bounds:
            config.append(np.random.uniform(min_val, max_val))
        return np.array(config)
    
    def nearest_node(self, config):
        """Find nearest node in tree to config."""
        distances = [np.linalg.norm(node - config) for node in self.nodes]
        nearest_idx = np.argmin(distances)
        return self.nodes[nearest_idx]
    
    def steer(self, from_node, to_config):
        """Steer from from_node towards to_config by step_size."""
        direction = to_config - from_node
        distance = np.linalg.norm(direction)
        
        if distance < self.step_size:
            return to_config
        
        direction = direction / distance
        new_node = from_node + direction * self.step_size
        return new_node
    
    def plan(self):
        """
        Run RRT planning.
        
        Returns:
            Path as list of configurations, or None
        """
        for i in range(self.max_iterations):
            # Sample random config (bias towards goal 10% of time)
            if np.random.random() < 0.1:
                random_config = self.goal
            else:
                random_config = self.sample_random_config()
            
            # Find nearest node
            nearest = self.nearest_node(random_config)
            
            # Steer towards random config
            new_node = self.steer(nearest, random_config)
            
            # Check if valid
            if self.obstacle_check(new_node):
                self.nodes.append(new_node)
                self.parent[tuple(new_node)] = tuple(nearest)
                
                # Check if goal reached
                if np.linalg.norm(new_node - self.goal) < self.step_size:
                    self.nodes.append(self.goal)
                    self.parent[tuple(self.goal)] = tuple(new_node)
                    return self.extract_path()
            
            if i % 500 == 0:
                print(f"Iteration {i}/{self.max_iterations}")
        
        print("Max iterations reached, no path found")
        return None
    
    def extract_path(self):
        """Extract path from tree."""
        path = [self.goal]
        current = tuple(self.goal)
        
        while current is not None:
            path.append(np.array(current))
            current = self.parent[current]
        
        path.reverse()
        return path

# Example: Arm planning
def arm_planning_example():
    # 3-DOF arm
    start = [0, 0, 0]
    goal = [1.5, 1.0, -0.5]
    bounds = [(-3.14, 3.14)] * 3  # Joint limits
    
    def is_valid_config(config):
        """Check if configuration is collision-free."""
        # Simple collision check (implement based on environment)
        return True  # Placeholder
    
    planner = RRTPlanner(start, goal, bounds, is_valid_config)
    path = planner.plan()
    
    if path:
        print(f"Path found with {len(path)} waypoints")
```

---

## 🦾 Inverse Kinematics

Inverse kinematics (IK) computes joint angles to reach a desired end-effector pose.

### Jacobian-Based IK

```python
class JacobianIK:
    """
    Inverse kinematics using Jacobian pseudo-inverse.
    """
    
    def __init__(self, robot, max_iterations=100, tolerance=0.01):
        self.robot = robot
        self.max_iterations = max_iterations
        self.tolerance = tolerance
    
    def solve(self, target_position, initial_joints=None):
        """
        Solve IK for target end-effector position.
        
        Args:
            target_position: Desired [x, y, z]
            initial_joints: Starting joint configuration
            
        Returns:
            Joint angles that reach target, or None
        """
        if initial_joints is None:
            joints = np.zeros(self.robot.num_joints)
        else:
            joints = np.array(initial_joints)
        
        for iteration in range(self.max_iterations):
            # Forward kinematics
            current_pos = self.robot.forward_kinematics(joints)
            
            # Error
            error = target_position - current_pos
            error_magnitude = np.linalg.norm(error)
            
            # Check convergence
            if error_magnitude < self.tolerance:
                print(f"IK converged in {iteration} iterations")
                return joints
            
            # Compute Jacobian
            J = self.robot.compute_jacobian(joints)
            
            # Pseudo-inverse
            J_pinv = np.linalg.pinv(J)
            
            # Update joints
            delta_joints = J_pinv @ error
            joints += delta_joints * 0.1  # Step size
            
            # Enforce joint limits
            joints = self.robot.clip_to_limits(joints)
        
        print("IK did not converge")
        return None

# Simple 2D arm example
class Simple2DArm:
    """Simple 2-link planar arm."""
    
    def __init__(self, l1=1.0, l2=1.0):
        self.l1 = l1  # Link 1 length
        self.l2 = l2  # Link 2 length
        self.num_joints = 2
    
    def forward_kinematics(self, joints):
        """Compute end-effector position."""
        q1, q2 = joints
        x = self.l1 * np.cos(q1) + self.l2 * np.cos(q1 + q2)
        y = self.l1 * np.sin(q1) + self.l2 * np.sin(q1 + q2)
        return np.array([x, y])
    
    def compute_jacobian(self, joints):
        """Compute Jacobian matrix."""
        q1, q2 = joints
        
        J = np.array([
            [-self.l1*np.sin(q1) - self.l2*np.sin(q1+q2), 
             -self.l2*np.sin(q1+q2)],
            [self.l1*np.cos(q1) + self.l2*np.cos(q1+q2), 
             self.l2*np.cos(q1+q2)]
        ])
        
        return J
    
    def clip_to_limits(self, joints):
        """Enforce joint limits."""
        return np.clip(joints, -np.pi, np.pi)

# Example usage
arm = Simple2DArm(l1=1.0, l2=1.0)
ik_solver = JacobianIK(arm)

target = [1.5, 0.5]
solution = ik_solver.solve(target)

if solution is not None:
    print(f"Joint angles: {np.degrees(solution)}")
    reached = arm.forward_kinematics(solution)
    print(f"Reached: {reached}, Target: {target}")
```

---

## 📋 Task Planning

Task planning breaks down high-level goals into sequences of actions.

### Simple Task Planner

```python
from collections import deque

class TaskPlanner:
    """
    Simple STRIPS-style task planner.
    """
    
    def __init__(self):
        self.actions = {}
    
    def add_action(self, name, preconditions, effects):
        """
        Add an action to planner.
        
        Args:
            name: Action name
            preconditions: Set of required states
            effects: Dict of state changes
        """
        self.actions[name] = {
            'preconditions': set(preconditions),
            'effects': effects
        }
    
    def plan(self, initial_state, goal_state):
        """
        Find sequence of actions to reach goal.
        
        Args:
            initial_state: Dict of initial conditions
            goal_state: Dict of desired conditions
            
        Returns:
            List of action names, or None
        """
        # BFS search
        queue = deque([(initial_state, [])])
        visited = {self.state_to_key(initial_state)}
        
        while queue:
            current_state, plan = queue.popleft()
            
            # Check if goal reached
            if self.is_goal_satisfied(current_state, goal_state):
                return plan
            
            # Try all actions
            for action_name, action in self.actions.items():
                # Check preconditions
                if self.can_apply(action, current_state):
                    # Apply action
                    new_state = self.apply_action(action, current_state)
                    state_key = self.state_to_key(new_state)
                    
                    if state_key not in visited:
                        visited.add(state_key)
                        new_plan = plan + [action_name]
                        queue.append((new_state, new_plan))
        
        return None  # No plan found
    
    def can_apply(self, action, state):
        """Check if action preconditions are met."""
        for precond in action['preconditions']:
            key, value = precond.split('=')
            if state.get(key) != value:
                return False
        return True
    
    def apply_action(self, action, state):
        """Apply action effects to state."""
        new_state = state.copy()
        new_state.update(action['effects'])
        return new_state
    
    def is_goal_satisfied(self, state, goal):
        """Check if goal conditions are met."""
        for key, value in goal.items():
            if state.get(key) != value:
                return False
        return True
    
    def state_to_key(self, state):
        """Convert state dict to hashable key."""
        return frozenset(state.items())

# Example: Robot manipulation task
def manipulation_task_example():
    planner = TaskPlanner()
    
    # Define actions
    planner.add_action(
        'pickup_object',
        preconditions=['hand=empty', 'object=on_table'],
        effects={'hand': 'holding_object', 'object': 'in_hand'}
    )
    
    planner.add_action(
        'place_object',
        preconditions=['hand=holding_object'],
        effects={'hand': 'empty', 'object': 'on_table'}
    )
    
    planner.add_action(
        'open_drawer',
        preconditions=['hand=empty', 'drawer=closed'],
        effects={'drawer': 'open'}
    )
    
    planner.add_action(
        'place_in_drawer',
        preconditions=['hand=holding_object', 'drawer=open'],
        effects={'hand': 'empty', 'object': 'in_drawer'}
    )
    
    # Initial state
    initial = {
        'hand': 'empty',
        'object': 'on_table',
        'drawer': 'closed'
    }
    
    # Goal state
    goal = {
        'object': 'in_drawer'
    }
    
    # Plan
    plan = planner.plan(initial, goal)
    
    if plan:
        print("Task plan:")
        for i, action in enumerate(plan, 1):
            print(f"  {i}. {action}")
    else:
        print("No plan found")
```

---

## 🎯 Trajectory Optimization

Trajectory optimization finds smooth, efficient paths while satisfying constraints.

### Minimum Jerk Trajectory

```python
class TrajectoryGenerator:
    """
    Generate smooth trajectories.
    """
    
    @staticmethod
    def minimum_jerk(start, goal, duration, dt=0.01):
        """
        Generate minimum jerk trajectory.
        
        Args:
            start: Starting position
            goal: Goal position
            duration: Time duration (seconds)
            dt: Time step
            
        Returns:
            positions, velocities, accelerations
        """
        t = np.arange(0, duration, dt)
        tau = t / duration  # Normalized time [0, 1]
        
        # Minimum jerk polynomial (5th order)
        s = 10*tau**3 - 15*tau**4 + 6*tau**5
        s_dot = (30*tau**2 - 60*tau**3 + 30*tau**4) / duration
        s_ddot = (60*tau - 180*tau**2 + 120*tau**3) / (duration**2)
        
        # Scale to start/goal
        positions = start + (goal - start) * s
        velocities = (goal - start) * s_dot
        accelerations = (goal - start) * s_ddot
        
        return positions, velocities, accelerations
    
    @staticmethod
    def via_points(waypoints, durations):
        """
        Generate trajectory through multiple waypoints.
        
        Args:
            waypoints: List of positions
            durations: List of durations between waypoints
            
        Returns:
            Complete trajectory
        """
        all_positions = []
        all_velocities = []
        all_accelerations = []
        
        for i in range(len(waypoints) - 1):
            start = waypoints[i]
            goal = waypoints[i + 1]
            duration = durations[i]
            
            pos, vel, acc = TrajectoryGenerator.minimum_jerk(
                start, goal, duration
            )
            
            all_positions.extend(pos)
            all_velocities.extend(vel)
            all_accelerations.extend(acc)
        
        return all_positions, all_velocities, all_accelerations

# Example usage
start_pos = 0.0
goal_pos = 1.5
duration = 2.0

positions, velocities, accelerations = TrajectoryGenerator.minimum_jerk(
    start_pos, goal_pos, duration
)

print(f"Generated {len(positions)} trajectory points")
print(f"Max velocity: {max(abs(velocities)):.3f}")
print(f"Max acceleration: {max(abs(accelerations)):.3f}")
```

---

## ✅ Key Takeaways

1. **Path planning** finds collision-free paths (A*, RRT)
2. **Inverse kinematics** computes joint angles for desired poses
3. **Task planning** sequences high-level actions
4. **Trajectory optimization** creates smooth, efficient motion
5. **Combine deliberative and reactive** for robust execution

---

## 🧪 Practice Exercise

:::note Exercise: Navigation System
**Difficulty**: Hard  
**Time**: 2 hours  
**Goal**: Implement complete navigation system

**Requirements**:
1. Use A* to plan path in grid world
2. Generate smooth trajectory through waypoints
3. Add reactive obstacle avoidance
4. Handle dynamic obstacles
5. Visualize planned path and execution

**Bonus**: Implement RRT for arm motion planning
:::

---

**[← Previous: Reactive Control](./reactive-control.md)** | **[Next: Hybrid Architectures →](./hybrid-architectures.md)**
