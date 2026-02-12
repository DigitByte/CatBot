# Environment setup

You can run all course labs without a physical robot. The simulation stack works on a standard computer with ROS.

# Recommended: ROS Noetic (current)
This is the primary environment for the course. It has been tested on Ubuntu 20.04 LTS and newer.

1. Install Anaconda3:
   https://www.anaconda.com/

2. Clone the repository (GitHub URL pending):
   `git clone https://github.com/DigitByte/CatBot catbot`

3. Create the ROS environment from the provided file:
   `conda env create -f ros_tf.yml`

4. Activate the environment:
   `conda activate ros_tf`

5. Optional: auto-activate the environment on new terminals by adding this line to `~/.bashrc`:
   `conda activate ros_tf`

6. Create a catkin workspace:
   http://wiki.ros.org/catkin/Tutorials/create_a_workspace

7. Move the CatBot folder into the workspace `src` directory:
   `mv ~/catbot/ YOUR-CATKIN-WORKSPACE/src/catbot`

8. Build the workspace:
   `cd ~/YOUR-CATKIN-WORKSPACE/`
   `catkin_make`

# Legacy: ROS Melodic (deprecated)
This branch is kept for historical reference and older Ubuntu versions. Use ROS Noetic if possible.

1. Install ROS Melodic:
   http://wiki.ros.org/melodic/Installation/Ubuntu

2. Create a catkin workspace:
   http://wiki.ros.org/catkin/Tutorials/create_a_workspace

3. Clone and switch to the legacy branch:
   `git clone https://github.com/DigitByte/CatBot catbot`
   `git checkout ros_melodic`

4. Build:
   `cd ..`
   `catkin_make`

# Run the simulation
1. Launch the test file:
   `roslaunch gait_control gait_control_test.launch`

2. Two windows should appear: RViz and a control terminal.

3. Switch gait in a second terminal:
   `rosservice call /catbot/set_gait "gait_filename: 'diagonal_fast'"`

4. Use the control terminal keys to move the robot:
   `w`, `a`, `s`, `d`, `q`, `e`

# PS5 controller (wireless)
CatBot can be driven with a DualSense/PS5 controller using the gamepad node.

1. Find your controller device path:
   `python catbot_visualization/scripts/find_bluetooth_port.py`

2. Launch the controller node (update the device path if needed):
   `rosrun catbot_visualization xbox_controller_node.py _controller_port:=/dev/input/event12`

3. If buttons or axes feel wrong, update the mapping in:
   `catbot_visualization/scripts/settings.py`
   See `CONTROLLER_MAPPING.md` for the default layout.
   For a student-friendly summary, see `CONTROLLER_CHEATSHEET.md`.

# Hardware setup (optional)
If you are building the physical CatBot, you will need a Teensy 4.0 and a BNO080 IMU.

1. Copy Arduino libraries:
   `cp -r /catbot/arduino/libraries/ ~/Arduino/libraries/`

2. Install Teensyduino:
   https://www.pjrc.com/teensy/teensyduino.html

3. Upload the firmware from:
   `/catbot/arduino/catbot_arduino/`

4. Launch the hardware control stack:
   `roslaunch gait_control gait_control_with_visualization.launch`

If you see `rosserial` errors, install it:
`sudo apt install ros-noetic-rosserial`

If you see a serial port error, update the port in:
`/catbot/catbot_calibration/launch/connect_teensy.launch`
