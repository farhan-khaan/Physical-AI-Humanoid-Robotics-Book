---
title: Hybrid Architectures
description: Combining reactive and deliberative control for robust behavior
sidebar_position: 4
---

# Hybrid Architectures

Hybrid architectures combine the speed of reactive control with the intelligence of deliberative planning. This combination creates robust, adaptive robots that can handle both planned tasks and unexpected situations.

## 🎯 Learning Outcomes

By the end of this section, you will be able to:

1. **Design** layered control architectures
2. **Implement** behavior trees for decision-making
3. **Create** subsumption architectures
4. **Build** hierarchical state machines
5. **Integrate** planning and execution

## 📋 Prerequisites

- Understanding of reactive control
- Understanding of deliberative control
- Python programming

---

## 🏗️ Three-Layer Architecture

The classic hybrid architecture has three layers:

```
┌─────────────────────────────────────┐
│   DELIBERATIVE LAYER                │
│   (Planning, reasoning, goals)      │
└──────────────┬──────────────────────┘
               ↓
┌─────────────────────────────────────┐
│   EXECUTIVE LAYER                   │
│   (Task decomposition, scheduling)  │
└──────────────┬──────────────────────┘
               ↓
┌─────────────────────────────────────┐
│   REACTIVE LAYER                    │
│   (Fast responses, safety)          │
└──────────────┬──────────────────────┘
               ↓
         Robot Hardware
```

### Implementation

```python
from abc import ABC, abstractmethod
import time

class ThreeLayerArchitecture:
    """
    Three-layer hybrid control architecture.
    """
    
    def __init__(self):
        self.deliberative = DeliberativeLayer()
        self.executive = ExecutiveLayer()
        self.reactive = ReactiveLayer()
    
    def run(self):
        """Main control loop."""
        while True:
            # Deliberative layer (slow, runs occasionally)
            if time.time() % 10 < 0.1:  # Every 10 seconds
                goal = self.deliberative.update()
                self.executive.set_goal(goal)
            
            # Executive layer (medium speed)
            if time.time() % 1 < 0.01:  # Every second
                task = self.executive.update()
                self.reactive.set_task(task)
            
            # Reactive layer (fast, every cycle)
            self.reactive.update()  # 100 Hz
            
            time.sleep(0.01)

class DeliberativeLayer:
    """High-level planning and reasoning."""
    
    def __init__(self):
        self.world_model = WorldModel()
        self.planner = TaskPlanner()
        self.current_goal = None
    
    def update(self):
        """Update plans based on world model."""
        # Update world understanding
        self.world_model.update()
        
        # Check if replanning needed
        if self.needs_replan():
            print("🧠 Deliberative: Replanning...")
            self.current_goal = self.planner.plan(
                self.world_model.get_state(),
                self.world_model.get_goal()
            )
        
        return self.current_goal
    
    def needs_replan(self):
        """Check if replanning is necessary."""
        # Replan if goal changed or plan failed
        return (self.current_goal is None or 
                self.world_model.environment_changed())

class ExecutiveLayer:
    """Task decomposition and sequencing."""
    
    def __init__(self):
        self.current_goal = None
        self.task_queue = []
        self.current_task = None
    
    def set_goal(self, goal):
        """Receive goal from deliberative layer."""
        self.current_goal = goal
        self.decompose_goal()
    
    def decompose_goal(self):
        """Break goal into executable tasks."""
        if self.current_goal:
            # Example: Navigation goal → waypoint following tasks
            self.task_queue = [
                {'type': 'move_to', 'position': wp}
                for wp in self.current_goal.get('waypoints', [])
            ]
    
    def update(self):
        """Select and monitor current task."""
        # Get next task if current is complete
        if self.current_task is None or self.is_task_complete():
            if self.task_queue:
                self.current_task = self.task_queue.pop(0)
                print(f"📋 Executive: Starting task {self.current_task['type']}")
        
        return self.current_task
    
    def is_task_complete(self):
        """Check if current task is done."""
        # Implementation depends on task type
        return False  # Placeholder

class ReactiveLayer:
    """Fast reactive behaviors."""
    
    def __init__(self):
        self.current_task = None
        self.obstacle_avoider = ObstacleAvoidance()
        self.balance_controller = BalanceController()
    
    def set_task(self, task):
        """Receive task from executive layer."""
        self.current_task = task
    
    def update(self):
        """Execute reactive behaviors."""
        # Read sensors
        sensors = read_sensors()
        
        # Safety reflexes (highest priority)
        if self.obstacle_avoider.detect_danger(sensors):
            command = self.obstacle_avoider.avoid()
            send_motor_commands(command)
            return
        
        # Balance control
        self.balance_controller.update(sensors)
        
        # Task execution
        if self.current_task:
            self.execute_task(self.current_task, sensors)
    
    def execute_task(self, task, sensors):
        """Execute current task reactively."""
        if task['type'] == 'move_to':
            # Reactive navigation to waypoint
            target = task['position']
            # Implement reactive control to target
            pass
```

---

## 🌳 Behavior Trees

Behavior trees provide a modular, hierarchical way to organize robot behaviors.

### Node Types

1. **Action**: Leaf node that does something
2. **Condition**: Leaf node that checks something
3. **Sequence**: Runs children in order (fails if any fail)
4. **Selector**: Tries children until one succeeds
5. **Parallel**: Runs children simultaneously

### Implementation

```python
from enum import Enum

class NodeStatus(Enum):
    SUCCESS = 1
    FAILURE = 2
    RUNNING = 3

class BehaviorNode(ABC):
    """Base class for behavior tree nodes."""
    
    @abstractmethod
    def tick(self):
        """Execute node and return status."""
        pass

class ActionNode(BehaviorNode):
    """Leaf node that executes an action."""
    
    def __init__(self, action_fn):
        self.action_fn = action_fn
    
    def tick(self):
        return self.action_fn()

class ConditionNode(BehaviorNode):
    """Leaf node that checks a condition."""
    
    def __init__(self, condition_fn):
        self.condition_fn = condition_fn
    
    def tick(self):
        return NodeStatus.SUCCESS if self.condition_fn() else NodeStatus.FAILURE

class SequenceNode(BehaviorNode):
    """Runs children in sequence until one fails."""
    
    def __init__(self, children):
        self.children = children
        self.current_child = 0
    
    def tick(self):
        while self.current_child < len(self.children):
            status = self.children[self.current_child].tick()
            
            if status == NodeStatus.RUNNING:
                return NodeStatus.RUNNING
            elif status == NodeStatus.FAILURE:
                self.current_child = 0  # Reset
                return NodeStatus.FAILURE
            
            # SUCCESS: move to next child
            self.current_child += 1
        
        # All children succeeded
        self.current_child = 0
        return NodeStatus.SUCCESS

class SelectorNode(BehaviorNode):
    """Tries children until one succeeds."""
    
    def __init__(self, children):
        self.children = children
        self.current_child = 0
    
    def tick(self):
        while self.current_child < len(self.children):
            status = self.children[self.current_child].tick()
            
            if status == NodeStatus.RUNNING:
                return NodeStatus.RUNNING
            elif status == NodeStatus.SUCCESS:
                self.current_child = 0  # Reset
                return NodeStatus.SUCCESS
            
            # FAILURE: try next child
            self.current_child += 1
        
        # All children failed
        self.current_child = 0
        return NodeStatus.FAILURE

class ParallelNode(BehaviorNode):
    """Runs all children simultaneously."""
    
    def __init__(self, children, success_threshold=None):
        self.children = children
        self.success_threshold = success_threshold or len(children)
    
    def tick(self):
        success_count = 0
        failure_count = 0
        running_count = 0
        
        for child in self.children:
            status = child.tick()
            
            if status == NodeStatus.SUCCESS:
                success_count += 1
            elif status == NodeStatus.FAILURE:
                failure_count += 1
            else:
                running_count += 1
        
        if success_count >= self.success_threshold:
            return NodeStatus.SUCCESS
        elif failure_count > len(self.children) - self.success_threshold:
            return NodeStatus.FAILURE
        else:
            return NodeStatus.RUNNING

# Example: Humanoid behavior tree
def create_humanoid_behavior_tree():
    """Create behavior tree for humanoid robot."""
    
    # Define actions
    def check_battery():
        battery = get_battery_level()
        return battery > 0.2
    
    def navigate_to_charger():
        # Implementation
        return NodeStatus.SUCCESS
    
    def charge_battery():
        # Implementation
        return NodeStatus.RUNNING
    
    def check_task_available():
        return has_pending_task()
    
    def execute_task():
        # Implementation
        return NodeStatus.SUCCESS
    
    def patrol():
        # Default behavior
        return NodeStatus.SUCCESS
    
    # Build tree
    root = SelectorNode([
        # Priority 1: Low battery → charge
        SequenceNode([
            ConditionNode(lambda: not check_battery()),
            ActionNode(navigate_to_charger),
            ActionNode(charge_battery)
        ]),
        
        # Priority 2: Task available → execute
        SequenceNode([
            ConditionNode(check_task_available),
            ActionNode(execute_task)
        ]),
        
        # Priority 3: Default → patrol
        ActionNode(patrol)
    ])
    
    return root

# Run behavior tree
tree = create_humanoid_behavior_tree()

while True:
    status = tree.tick()
    print(f"Tree status: {status}")
    time.sleep(0.1)
```

:::tip Behavior Tree Benefits
- **Modular**: Easy to add/remove behaviors
- **Reusable**: Subtrees can be shared
- **Visual**: Can be designed graphically
- **Debuggable**: Clear execution trace
:::

---

## 🔄 Subsumption Architecture

Subsumption uses layered behaviors where higher layers can suppress lower layers.

```python
class SubsumptionArchitecture:
    """
    Subsumption architecture with layered behaviors.
    """
    
    def __init__(self):
        self.layers = []
    
    def add_layer(self, behavior, priority):
        """
        Add behavior layer.
        
        Args:
            behavior: Behavior object
            priority: Higher number = higher priority
        """
        self.layers.append((priority, behavior))
        self.layers.sort(key=lambda x: x[0], reverse=True)
    
    def run(self):
        """Run subsumption architecture."""
        while True:
            command = None
            
            # Check behaviors from highest to lowest priority
            for priority, behavior in self.layers:
                if behavior.is_active():
                    command = behavior.get_command()
                    print(f"Layer {priority} active: {behavior.name}")
                    break  # Highest priority behavior wins
            
            # Execute command
            if command:
                execute_command(command)
            
            time.sleep(0.01)

class Behavior:
    """Base class for subsumption behaviors."""
    
    def __init__(self, name):
        self.name = name
    
    @abstractmethod
    def is_active(self):
        """Check if behavior should be active."""
        pass
    
    @abstractmethod
    def get_command(self):
        """Get motor command."""
        pass

# Example behaviors
class AvoidObstacleBehavior(Behavior):
    """Highest priority: avoid obstacles."""
    
    def __init__(self):
        super().__init__("Avoid Obstacles")
        self.danger_distance = 0.5
    
    def is_active(self):
        sensors = read_sensors()
        min_distance = min(sensors['lidar'])
        return min_distance < self.danger_distance
    
    def get_command(self):
        # Turn away from obstacle
        return {'velocity': 0, 'turn_rate': 1.0}

class NavigateToGoalBehavior(Behavior):
    """Medium priority: navigate to goal."""
    
    def __init__(self, goal):
        super().__init__("Navigate to Goal")
        self.goal = goal
    
    def is_active(self):
        position = get_position()
        distance = np.linalg.norm(position - self.goal)
        return distance > 0.1
    
    def get_command(self):
        # Move towards goal
        position = get_position()
        direction = self.goal - position
        return {'velocity': 0.5, 'heading': np.arctan2(direction[1], direction[0])}

class WanderBehavior(Behavior):
    """Lowest priority: wander randomly."""
    
    def __init__(self):
        super().__init__("Wander")
    
    def is_active(self):
        return True  # Always active as fallback
    
    def get_command(self):
        # Random wandering
        return {'velocity': 0.2, 'turn_rate': np.random.uniform(-0.5, 0.5)}

# Create subsumption system
subsumption = SubsumptionArchitecture()
subsumption.add_layer(AvoidObstacleBehavior(), priority=3)
subsumption.add_layer(NavigateToGoalBehavior([10, 10]), priority=2)
subsumption.add_layer(WanderBehavior(), priority=1)

subsumption.run()
```

---

## 🎚️ Hierarchical State Machines

Hierarchical state machines (HSMs) nest states within states for better organization.

```python
class HierarchicalState:
    """State that can contain substates."""
    
    def __init__(self, name):
        self.name = name
        self.substates = {}
        self.current_substate = None
        self.entry_action = None
        self.exit_action = None
    
    def add_substate(self, state, is_initial=False):
        """Add a substate."""
        self.substates[state.name] = state
        if is_initial:
            self.current_substate = state.name
    
    def on_entry(self, action):
        """Set entry action."""
        self.entry_action = action
        return self
    
    def on_exit(self, action):
        """Set exit action."""
        self.exit_action = action
        return self
    
    def enter(self):
        """Enter this state."""
        if self.entry_action:
            self.entry_action()
        
        # Enter initial substate
        if self.current_substate:
            self.substates[self.current_substate].enter()
    
    def exit(self):
        """Exit this state."""
        # Exit current substate
        if self.current_substate:
            self.substates[self.current_substate].exit()
        
        if self.exit_action:
            self.exit_action()
    
    def update(self):
        """Update current substate."""
        if self.current_substate:
            self.substates[self.current_substate].update()

# Example: Humanoid locomotion HSM
def create_locomotion_hsm():
    """Create hierarchical state machine for locomotion."""
    
    # Top-level states
    root = HierarchicalState("Root")
    
    # Stationary state
    stationary = HierarchicalState("Stationary")
    idle = HierarchicalState("Idle")
    sitting = HierarchicalState("Sitting")
    
    stationary.add_substate(idle, is_initial=True)
    stationary.add_substate(sitting)
    
    # Moving state
    moving = HierarchicalState("Moving")
    walking = HierarchicalState("Walking")
    running = HierarchicalState("Running")
    
    moving.add_substate(walking, is_initial=True)
    moving.add_substate(running)
    
    # Add to root
    root.add_substate(stationary, is_initial=True)
    root.add_substate(moving)
    
    # Define actions
    stationary.on_entry(lambda: print("Entering stationary mode"))
    moving.on_entry(lambda: print("Starting to move"))
    
    return root

hsm = create_locomotion_hsm()
hsm.enter()
```

---

## ✅ Key Takeaways

1. **Hybrid architectures** combine reactive and deliberative strengths
2. **Three-layer architecture** separates planning, sequencing, execution
3. **Behavior trees** provide modular, hierarchical behavior organization
4. **Subsumption** uses priority-based behavior selection
5. **HSMs** organize complex state logic hierarchically

---

## 🧪 Practice Exercise

:::note Exercise: Delivery Robot
**Difficulty**: Hard  
**Time**: 3 hours  
**Goal**: Build hybrid controller for delivery robot

**Requirements**:
1. Implement three-layer architecture OR behavior tree
2. Deliberative: Plan route to delivery location
3. Executive: Break route into waypoints
4. Reactive: Navigate with obstacle avoidance
5. Handle: low battery, blocked paths, new orders

**Bonus**: Visualize behavior tree execution
:::

---

**[← Previous: Deliberative Control](./deliberative-control.md)** | **[Next: Learned Control →](./learned-control.md)**
