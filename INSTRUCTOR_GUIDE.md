# CatBot Instructor Guide (12 Weeks)

Prepared by D. Hoskia


This guide provides weekly objectives, lab setup, grading rubrics, and instructor notes. It aligns to the 7‑chapter HTML notes and the CatBot lab sequence.

---

## Course rhythm
- **Lecture (60–75 min):** core theory and worked examples
- **Lab (60–90 min):** CatBot simulation or hardware validation
- **Deliverable:** short report + plots + code commit

---

## Weekly lab rubrics (10 labs)
Each lab is graded on a 10‑point scale unless noted.

### Lab 1 — Simulation bring‑up and URDF inspection
**Objective:** verify environment and interpret the robot model.

Rubric:
- 3 pts: ROS workspace builds without error
- 3 pts: RViz shows CatBot model with correct frame tree
- 2 pts: short explanation of URDF structure
- 2 pts: screenshot + launch command in report

Instructor notes:
- Pre‑check: students can run `gait_control_test.launch`.
- Common issue: missing ROS packages; use rosdep.

---

### Lab 2 — Coordinate frames and transforms
**Objective:** visualize frames and validate transforms.

Rubric:
- 4 pts: frame diagram with at least 3 frames
- 3 pts: computed transform between two frames
- 3 pts: verification in RViz

Instructor notes:
- Encourage consistent axis conventions.

---

### Lab 3 — Transform composition
**Objective:** combine transforms and verify a point mapping.

Rubric:
- 4 pts: correct transform chain derivation
- 3 pts: numerical verification with a point in two frames
- 3 pts: clean plots or tables

---

### Lab 4 — Forward kinematics (leg chain)
**Objective:** compute end‑effector pose from joint angles.

Rubric:
- 4 pts: correct FK derivation
- 3 pts: numeric test against simulation
- 3 pts: plot of foot position over joint sweep

---

### Lab 5 — Inverse kinematics (foot placement)
**Objective:** solve joint angles for target foot positions.

Rubric:
- 4 pts: IK method described (analytic or numeric)
- 3 pts: successful pose reach in simulation
- 3 pts: error analysis for unreachable poses

---

### Lab 6 — Jacobian computation
**Objective:** derive Jacobian for a leg chain.

Rubric:
- 4 pts: Jacobian matrix correctly derived
- 3 pts: verified with finite differences
- 3 pts: discussion of singular configuration

---

### Lab 7 — Jacobian velocity mapping
**Objective:** map joint velocity to foot velocity.

Rubric:
- 4 pts: velocity mapping demonstrated in sim
- 3 pts: comparison between predicted and measured velocity
- 3 pts: discussion of limitations

---

### Lab 8 — Dynamics (load/torque estimation)
**Objective:** estimate joint torques for a stance pose.

Rubric:
- 4 pts: correct use of mass/inertia terms
- 3 pts: torque plots or tables
- 3 pts: stability interpretation

---

### Lab 9 — Dynamic stability
**Objective:** compute support polygon and stability margin.

Rubric:
- 4 pts: correct support polygon calculation
- 3 pts: stability margin computed and plotted
- 3 pts: discussion of gait implications

---

### Lab 10 — Trajectory generation
**Objective:** generate smooth leg or body trajectories.

Rubric:
- 4 pts: polynomial trajectory defined
- 3 pts: smoothness verified (velocity/acceleration)
- 3 pts: simulation demo

---

## Capstone (Week 12)
**Goal:** Student‑defined CatBot behavior or autonomy demo.

**Rubric summary:** see `CAPSTONE.md`

---

## Grading guidance
- 10 labs × 6% each = 60%
- Capstone = 30%
- Participation/quizzes = 10%

---

## Common student issues
- ROS environment not sourced
- Wrong `/dev/input` device for controller
- Catkin workspace not built or not sourced

---

## Recommended instructor assets
- Use `CONTROLLER_CHEATSHEET.md` in lab handouts
- Show example plots from the notebooks for faster onboarding

