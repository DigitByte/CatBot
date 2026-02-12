# CatBot Gamepad Mapping (PS5 DualSense)

This course uses a PS5 DualSense controller over Bluetooth. The gamepad node reads standard Linux `evdev` codes and should work on most modern controllers.

# Default mapping
- Left stick up/down: forward/backward velocity
- Right stick left/right: yaw rate
- Right stick up/down: body roll
- L2 trigger: strafe left
- R2 trigger: strafe right
- D-pad up/down: body height (z)
- D-pad left/right: body pitch
- L1: connecting servos
- R1: disconnecting servos
- Triangle: stop gait
- Square: diagonal gait
- Cross: diagonal fast gait
- Circle: trot gait
- Share: in‑place gait

# Find your controller device path
Run:
`python catbot_visualization/scripts/find_bluetooth_port.py`

Then launch the controller node:
`rosrun catbot_visualization xbox_controller_node.py _controller_port:=/dev/input/event12`

# Adjusting mappings
If buttons or axes do not match your controller, run the test tool:
`python catbot_visualization/scripts/test.py`

Update the constants in:
`catbot_visualization/scripts/settings.py`

