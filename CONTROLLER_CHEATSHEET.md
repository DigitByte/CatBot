# CatBot Controller Cheat Sheet (PS5 DualSense)

## Motion
- Left stick up/down: forward/backward
- Right stick left/right: yaw
- Right stick up/down: body roll
- L2: strafe left
- R2: strafe right
- D-pad up/down: body height (z)
- D-pad left/right: body pitch

## Actions
- L1: connecting servos
- R1: disconnecting servos
- Triangle: stop gait
- Square: diagonal gait
- Cross: diagonal fast gait
- Circle: trot gait
- Share: in-place gait

## Connect the controller
1. Find the device path:
   `python catbot_visualization/scripts/find_bluetooth_port.py`
2. Launch the node:
   `rosrun catbot_visualization xbox_controller_node.py _controller_port:=/dev/input/event12`

## If buttons feel wrong
1. Run the tester:
   `python catbot_visualization/scripts/test.py`
2. Update mappings here:
   `catbot_visualization/scripts/settings.py`

