# CatBot: Intro to Robotics (12 Weeks)

This course is structured around the learning flow in the provided HTML notes: spatial description, kinematics, Jacobians, dynamics, trajectory generation, and control. Each week pairs theory with hands-on work in this repository.

# Why robotics\nRobotics blends mechanics, electronics, and software into systems that can sense, decide, and act. This course focuses on the core ideas that make that possible, and uses CatBot as a concrete, testable platform.\n\n## How to succeed\n- Read the short theory notes before lab.\n- Run the notebooks to confirm the math with numbers.\n- Test each change in simulation before hardware.\n- Keep a lab log of what you tried and what you observed.\n\n## Course goals
- Explain and use coordinate frames, rotations, and transforms
- Derive forward and inverse kinematics for a legged robot
- Use Jacobians to connect joint motion to end-effector motion
- Model dynamics and reason about forces and stability
- Generate smooth trajectories in joint and task space
- Implement feedback control and test it in simulation
- Integrate sensing and navigation for basic autonomy

# How to use this repository
- Read the week plan below before each lab.
- Use the notebooks for math practice and quick experiments.
- Use the ROS packages for simulation and real-time testing.
- Keep notes in your own lab notebook or report template.

# Weekly plan

| Week | Focus | Practice | Repo resources |
| --- | --- | --- | --- |
| 1 | Course overview, ROS basics, safety, toolchain | Run a first simulation and explore topics | `setup.md`, `catbot_visualization` |
| 2 | Spatial description and frames | Visualize frames and links in RViz | `catbot_description`, `catbot_visualization` |
| 3 | Direct kinematics | Compute link transforms and verify in simulation | `notebooks/robot_kinematics.ipynb` |
| 4 | Inverse kinematics | Solve for joint angles in simple poses | `notebooks/robot_kinematics.ipynb`, `catbot_pybullet` |
| 5 | Jacobians | Map joint velocity to foot velocity | `notebooks/Jacobian.ipynb` |
| 6 | Dynamics | Build intuition for forces, inertia, and stability | `notebooks/robot_model.ipynb` |
| 7 | Trajectory generation | Create smooth joint trajectories | `notebooks/predictive_model_control.ipynb` |
| 8 | Control | Tune a walking controller | `gait_control` |
| 9 | State estimation | Fuse sensor data for stable motion | `notebooks/robot_state_estimation.ipynb` |
| 10 | Perception | LIDAR and camera data for environment awareness | `catbot_lidar`, `catbot_vision` |
| 11 | Navigation | 2D mapping and navigation basics | `catbot_2dnav`, `catbot_gazebo` |
| 12 | Capstone | Demonstrate a behavior or project of your choice | All packages |

# Assessment ideas
- Weekly labs or quizzes
- Midterm: kinematics and Jacobian application
- Final project: a motion or autonomy demo using CatBot

Author: Damien
