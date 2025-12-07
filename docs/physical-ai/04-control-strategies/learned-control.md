---
title: Learned Control
description: Using machine learning for robot control
sidebar_position: 5
---

# Learned Control

Machine learning enables robots to learn control policies from data, discover strategies humans might not program, and adapt to new situations. This section covers reinforcement learning and imitation learning for robotics.

## 🎯 Learning Outcomes

1. **Understand** reinforcement learning fundamentals
2. **Implement** simple RL algorithms
3. **Apply** imitation learning techniques
4. **Train** policies in simulation
5. **Transfer** learned controllers to real robots

## 📋 Prerequisites

- Python programming
- Basic understanding of neural networks
- Familiarity with control concepts

---

## 🎮 Reinforcement Learning Basics

**Reinforcement Learning (RL)** learns by trial and error, receiving rewards for good behavior.

### Key Concepts

- **State (s)**: Robot's current situation
- **Action (a)**: What robot can do
- **Reward (r)**: Feedback on action quality
- **Policy (π)**: Mapping from states to actions
- **Value function (V)**: Expected future reward

### Simple Q-Learning Example

```python
import numpy as np

class QLearningAgent:
    """Simple Q-learning agent."""
    
    def __init__(self, n_states, n_actions, learning_rate=0.1, 
                 discount=0.99, epsilon=0.1):
        self.q_table = np.zeros((n_states, n_actions))
        self.lr = learning_rate
        self.gamma = discount
        self.epsilon = epsilon
    
    def select_action(self, state):
        """Epsilon-greedy action selection."""
        if np.random.random() < self.epsilon:
            return np.random.randint(self.q_table.shape[1])
        return np.argmax(self.q_table[state])
    
    def update(self, state, action, reward, next_state):
        """Update Q-value."""
        best_next_action = np.argmax(self.q_table[next_state])
        td_target = reward + self.gamma * self.q_table[next_state, best_next_action]
        td_error = td_target - self.q_table[state, action]
        self.q_table[state, action] += self.lr * td_error
    
    def train(self, env, episodes=1000):
        """Train the agent."""
        for ep in range(episodes):
            state = env.reset()
            total_reward = 0
            
            for step in range(100):
                action = self.select_action(state)
                next_state, reward, done = env.step(action)
                
                self.update(state, action, reward, next_state)
                
                total_reward += reward
                state = next_state
                
                if done:
                    break
            
            if ep % 100 == 0:
                print(f"Episode {ep}, Reward: {total_reward}")
```

---

## 🧠 Deep RL for Continuous Control

For complex robots with continuous states/actions, we use deep neural networks.

### PPO (Proximal Policy Optimization)

```python
import torch
import torch.nn as nn
import torch.optim as optim

class PolicyNetwork(nn.Module):
    """Neural network policy."""
    
    def __init__(self, state_dim, action_dim):
        super().__init__()
        self.network = nn.Sequential(
            nn.Linear(state_dim, 64),
            nn.Tanh(),
            nn.Linear(64, 64),
            nn.Tanh(),
            nn.Linear(64, action_dim)
        )
    
    def forward(self, state):
        return torch.tanh(self.network(state))

class PPOAgent:
    """Simple PPO implementation."""
    
    def __init__(self, state_dim, action_dim):
        self.policy = PolicyNetwork(state_dim, action_dim)
        self.optimizer = optim.Adam(self.policy.parameters(), lr=3e-4)
    
    def select_action(self, state):
        """Sample action from policy."""
        state_tensor = torch.FloatTensor(state)
        with torch.no_grad():
            action = self.policy(state_tensor)
        return action.numpy()
    
    def train_step(self, states, actions, rewards, advantages):
        """Perform one training step."""
        states = torch.FloatTensor(states)
        actions = torch.FloatTensor(actions)
        advantages = torch.FloatTensor(advantages)
        
        # Policy gradient
        pred_actions = self.policy(states)
        loss = -torch.mean(advantages * torch.sum((pred_actions - actions)**2, dim=1))
        
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()
        
        return loss.item()

# Training loop
agent = PPOAgent(state_dim=10, action_dim=4)

for episode in range(1000):
    states, actions, rewards = collect_rollout(agent)
    advantages = compute_advantages(rewards)
    loss = agent.train_step(states, actions, rewards, advantages)
```

---

## 👨‍🏫 Imitation Learning

Learn from expert demonstrations instead of trial-and-error.

### Behavioral Cloning

```python
class BehavioralCloningAgent:
    """Learn policy by imitating expert."""
    
    def __init__(self, state_dim, action_dim):
        self.policy = PolicyNetwork(state_dim, action_dim)
        self.optimizer = optim.Adam(self.policy.parameters(), lr=1e-3)
        self.criterion = nn.MSELoss()
    
    def train_from_demonstrations(self, expert_data, epochs=100):
        """Train from expert demonstrations."""
        states = torch.FloatTensor([d['state'] for d in expert_data])
        actions = torch.FloatTensor([d['action'] for d in expert_data])
        
        for epoch in range(epochs):
            pred_actions = self.policy(states)
            loss = self.criterion(pred_actions, actions)
            
            self.optimizer.zero_grad()
            loss.backward()
            self.optimizer.step()
            
            if epoch % 10 == 0:
                print(f"Epoch {epoch}, Loss: {loss.item():.4f}")
```

---

## ✅ Key Takeaways

1. **RL learns** through trial-and-error with rewards
2. **Deep RL** handles high-dimensional states/actions
3. **Imitation learning** leverages expert demonstrations
4. **Simulation** is essential for safe RL training
5. **Sim-to-real transfer** requires domain randomization

---

**[← Previous: Hybrid Architectures](./hybrid-architectures.md)** | **[Next: Real-World Challenges →](./real-world-challenges.md)**
