---
title: Project Requirements
description: What your capstone project must include
sidebar_position: 3
---

# Project Requirements

All capstone projects must demonstrate mastery of Physical AI concepts through comprehensive implementation and documentation.

## 🎯 Mandatory Requirements

### 1. Technical Implementation

**Embodied Intelligence (Required)**
- [ ] Demonstrate sense-think-act loop
- [ ] Show interaction with physical world (simulated or real)
- [ ] React to environment changes

**Sensor Integration (Minimum 2 types)**
- [ ] Read from sensors reliably
- [ ] Handle sensor noise/failures
- [ ] Fuse multiple sensor streams

**Actuator Control**
- [ ] Control motors/servos
- [ ] Implement safety limits
- [ ] Handle actuator saturation

**Control Strategy (Choose 1+)**
- [ ] Reactive control (PID, reflexes, state machines)
- [ ] Deliberative control (planning, IK, trajectory optimization)
- [ ] Hybrid architecture (behavior trees, subsumption)
- [ ] Learned control (RL, imitation learning)

**Simulation Testing**
- [ ] Test in simulation environment
- [ ] Demonstrate repeatable results
- [ ] Show robustness to variations

---

## 📚 Documentation Requirements

### System Architecture Document
- Block diagram of system components
- Data flow between modules
- Hardware/software stack

### Design Decisions
- Why you chose specific algorithms
- Trade-offs considered
- Alternatives rejected and why

### Code Documentation
- Comments explaining key logic
- Docstrings for functions/classes
- README with setup instructions

### Test Results
- Quantitative metrics (success rate, accuracy, time)
- Plots/graphs of performance
- Video demonstrations
- Failure case analysis

### User Guide
- How to run your system
- Dependencies and installation
- Configuration parameters
- Troubleshooting common issues

---

## 📦 Deliverables Checklist

- [ ] **Source code** (well-commented, organized)
- [ ] **README.md** with project overview
- [ ] **requirements.txt** or equivalent
- [ ] **Architecture diagram**
- [ ] **Technical report** (3-5 pages)
- [ ] **Demo video** (2-5 minutes)
- [ ] **Test results** (data, plots, analysis)
- [ ] **Presentation slides** (10-15 slides)

---

## 💻 Code Quality Standards

### Organization
```
my-capstone-project/
├── README.md
├── requirements.txt
├── src/
│   ├── sensors/
│   ├── actuators/
│   ├── control/
│   └── main.py
├── tests/
├── docs/
├── data/
└── videos/
```

### Style Guidelines
- Follow PEP 8 (Python) or equivalent
- Meaningful variable names
- Functions < 50 lines
- Classes with clear responsibility
- No hardcoded magic numbers

### Error Handling
- Try-except blocks for I/O
- Graceful failure modes
- Informative error messages
- Logging for debugging

---

## ⏱️ Timeline

### Week 1-2: Design & Setup
- Choose project and scope
- Set up development environment
- Create architecture diagram
- Write initial code structure

### Week 3-5: Implementation
- Core functionality
- Sensor/actuator integration
- Control algorithms
- Basic testing

### Week 6-7: Testing & Refinement
- Comprehensive testing
- Bug fixes and improvements
- Performance optimization
- Robustness testing

### Week 8: Documentation & Presentation
- Write technical report
- Create demo video
- Prepare presentation
- Final code cleanup

---

## ✅ Success Criteria

Your project is successful if:
- ✅ Meets all mandatory requirements
- ✅ Demonstrates working system
- ✅ Shows Physical AI concepts clearly
- ✅ Code is clean and documented
- ✅ Results are reproducible
- ✅ Documentation is comprehensive

---

**[← Previous: Project Ideas](./project-ideas.md)** | **[Next: Evaluation Rubric →](./evaluation-rubric.md)**
