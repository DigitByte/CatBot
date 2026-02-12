# CatBot: Introduction to Robotics (12 Weeks)

Prepared by D. Hoskia


**Course level:** Undergraduate

**Format:** Lecture + lab each week

**Platform:** CatBot simulation for all students; hardware for selected teams

**Instructor:** Damien (drdelgado9@outlook.com)

---

## Course description
This course introduces the core mathematics and engineering ideas that make robots move, sense, and act. Students learn spatial description, kinematics, Jacobians, dynamics, trajectory generation, and control. CatBot, a quadruped robotics platform, provides a concrete system for weekly labs and a final capstone demo.

---

## Learning outcomes
By the end of the course, students will be able to:
- Describe rigid‑body motion using coordinate frames and transformations
- Derive forward and inverse kinematics for a legged robot
- Use Jacobians to relate joint motion to end‑effector motion
- Model dynamics and reason about forces and stability
- Generate smooth trajectories in joint and task space
- Implement and tune feedback control for walking behaviors
- Integrate sensing and navigation for a basic autonomy task

---

## Grading
- Weekly labs (10): 60%
- Capstone project: 30%
- Participation / quizzes: 10%

---

## Weekly schedule (aligned to the 7‑chapter notes)

**Week 1 — Course Launch + Tools**
- ROS basics, CatBot simulation, workflow setup
- Lab 1: Simulation bring‑up + URDF inspection

**Weeks 2–3 — Chapter 1: Spatial Description**
- Frames, transforms, rotation representations
- Lab 2: Coordinate frames in RViz
- Lab 3: Transform composition and verification

**Week 4 — Chapter 2: Direct Kinematics**
- FK for a leg chain
- Lab 4: FK derivation + simulated verification

**Week 5 — Chapter 3: Inverse Kinematics**
- IK strategies and constraints
- Lab 5: Foot placement IK

**Weeks 6–7 — Chapter 4: The Jacobian**
- Velocity mapping, singularities, force relationships
- Lab 6: Jacobian computation
- Lab 7: Jacobian velocity test in sim

**Weeks 8–9 — Chapter 5: Dynamics**
- Newton–Euler and Lagrange formulations
- Lab 8: Load/torque estimation
- Lab 9: Dynamic stability analysis

**Week 10 — Chapter 6: Trajectory Generation**
- Polynomial trajectories, path planning
- Lab 10: Smooth gait trajectory

**Week 11 — Chapter 7: Manipulator Control**
- PD control, robustness, tracking
- Capstone prep + design review

**Week 12 — Capstone Demo + Report**
- Live demo and technical report

---

## Required tools
- Ubuntu 20.04 LTS
- ROS Noetic
- Catkin workspace
- CatBot repository (simulation + notebooks)

---

## References
- Course notes (HTML) structured by the 7 chapters
- CatBot notebooks and ROS packages in this repository

