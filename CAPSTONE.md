# CatBot Capstone Project

Prepared by D. Hoskia


## Overview
The capstone is a student‑defined CatBot behavior or autonomy demo. It is the final assessment of the course and must be demonstrated in simulation, with optional hardware validation.

## Required elements
- A clear goal (e.g., patrol a room, obstacle avoidance, stable trot over uneven terrain)
- A technical approach (algorithms, control, sensors)
- A working CatBot demo
- A short report and live demo

## Deliverables
- **Demo:** live or recorded run
- **Report:** 3–5 pages
- **Code:** committed to the course repo

---

## Rubric (100 pts)

**1) Technical correctness (30 pts)**
- Correct use of kinematics/dynamics/control (15)
- Robustness and stability (15)

**2) System integration (20 pts)**
- Sensor integration or simulation realism (10)
- Clean ROS topic/service use (10)

**3) Performance (20 pts)**
- Achieves task goal (10)
- Repeatable behavior (10)

**4) Documentation (20 pts)**
- Clear report (10)
- Plots, figures, or metrics (10)

**5) Presentation (10 pts)**
- Demo clarity and explanation

---

## Example capstone submissions

### Example A — Room Patrol (Simulation)
**Goal:** CatBot patrols a room map and avoids walls.

**Approach:**
- Use LIDAR or map image to generate a safe path
- Use velocity control + waypoint tracking

**Evidence:**
- 60‑second demo video
- Map with trajectory overlay
- Error plot vs target path

**Score guide:** 85–95/100 if stable and repeatable.

---

### Example B — Stable Trot on Uneven Terrain
**Goal:** Maintain stable trot while terrain height varies.

**Approach:**
- Adjust foot height using feedback
- Use Jacobian‑based velocity adjustment

**Evidence:**
- Simulation run with height map
- Plots of body roll/pitch

**Score guide:** 90–100/100 if stable and smooth.

---

### Example C — Obstacle Avoidance Demo
**Goal:** Navigate to a target while avoiding obstacles.

**Approach:**
- Simple obstacle detection + reactive control
- Gait switching based on proximity

**Evidence:**
- Demo video
- Histogram of minimum clearance

**Score guide:** 80–90/100 if avoidance is consistent.

