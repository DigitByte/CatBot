# CatBot

ROS-based quadruped robotics platform for simulation, controls, teleoperation, and learning-by-building.

## Overview

CatBot is a robotics workspace built around a quadruped platform and the software needed to study how that platform moves, senses, and is controlled. The repository combines ROS packages, simulation environments, calibration utilities, and course materials so the project can be used both as a development platform and as a structured learning environment.

The software stack is the primary focus of this repository. Hardware kits, replacement CAD assets, and trademark guidance are tracked separately where noted below.

## What This Repository Includes

- Core gait and control packages for CatBot bring-up and teleoperation.
- Simulation environments in both PyBullet and Gazebo.
- Robot description, calibration, and visualization tooling.
- Navigation, lidar, and vision packages for perception experiments.
- Notebooks, handouts, and course documents for guided learning.

## Repository Layout

```text
CatBot/
├── gait_control/           core gait and control workflows
├── catbot_description/     URDF and robot model assets
├── catbot_visualization/   visualization and teleoperation helpers
├── catbot_calibration/     servo and hardware calibration tools
├── catbot_pybullet/        PyBullet simulation package
├── catbot_gazebo/          Gazebo simulation package
├── catbot_2dnav/           2D navigation experiments
├── catbot_lidar/           lidar integration
├── catbot_vision/          camera and vision experiments
├── catbot_msgs/            custom ROS messages
├── arduino/                embedded support for physical hardware
├── electronics/            electronics references and assets
├── notebooks/              computational notebooks and exercises
├── handouts/               course handouts and supporting material
└── *.md                    setup, course, hardware, and instructor docs
```

## Key Documents

- [setup.md](./setup.md): environment setup, build steps, simulation bring-up, and hardware notes.
- [COURSE.md](./COURSE.md): 12-week learning path built around CatBot.
- [SYLLABUS.md](./SYLLABUS.md): course description, learning outcomes, and weekly schedule.
- [INSTRUCTOR_GUIDE.md](./INSTRUCTOR_GUIDE.md): teaching notes, lab rubrics, and grading guidance.
- [components.md](./components.md): hardware bill-of-materials reference and sourcing context.
- [HARDWARE.md](./HARDWARE.md): hardware availability status.
- [CONTROLLER_CHEATSHEET.md](./CONTROLLER_CHEATSHEET.md): quick reference for PS5 DualSense control.

## Quick Start

1. Create the development environment with `conda env create -f ros_tf.yml`.
2. Place the repository inside your catkin workspace `src` directory.
3. Build the workspace with `catkin_make`.
4. Launch the simulation with `roslaunch gait_control gait_control_test.launch`.
5. Change gait profiles from a second terminal with `rosservice call /catbot/set_gait "gait_filename: 'diagonal_fast'"`.

Detailed setup instructions, controller mapping, and hardware bring-up are documented in [setup.md](./setup.md).

## Platform Coverage

| Area | Packages / Docs |
| --- | --- |
| Control | `gait_control`, `catbot_msgs`, `catbot_calibration` |
| Robot model | `catbot_description`, `catbot_visualization` |
| Simulation | `catbot_pybullet`, `catbot_gazebo` |
| Perception | `catbot_lidar`, `catbot_vision`, `catbot_2dnav` |
| Learning material | `notebooks/`, `handouts/`, `COURSE.md`, `SYLLABUS.md` |
| Hardware support | `arduino/`, `electronics/`, `components.md`, `HARDWARE.md` |

## Hardware Note

This repository is intended to remain useful even without a physical robot. The simulation and coursework can be run on a standard ROS workstation, while hardware kits and updated CAD assets are handled separately from the core software repository.

## License

CatBot is released under the terms of the [MIT License](./LICENSE).
