
from evdev import ecodes

############ GAMEPAD MAPPING ###############
# These mappings use standard Linux evdev codes and work with most
# modern controllers (Xbox, DualSense/PS5). If your controller uses
# different codes, run `catbot_visualization/scripts/test.py` and
# update the constants below.

# # Forward and backwards axis
CODE_FORWARD_AXIS = ecodes.ABS_Y

MAX_FORWARD_VEL = 0.6
MIN_FORWARD_VEL = -0.4

# # theta axis
CODE_TH_AXIS = ecodes.ABS_RX

MAX_TH_VEL = -0.5
MIN_TH_VEL = 0.5


# # roll axis
CODE_ROLL_AXIS = ecodes.ABS_RY

MAX_ROLL_VEL = -10
MIN_ROLL_VEL = 10

# # pitch axis (D-pad left/right)
CODE_PITCH_AXIS       = ecodes.ABS_HAT0X
PITCH_AXIS_MAX_VALUE  = 1
PITCH_AXIS_MIN_VALUE  = -1
PITCH_AXIS_REST_VALUE = 0
MAX_PITCH_VEL         = -10
MIN_PITCH_VEL         = 10


# # side axis (left/right triggers)
CODE_SIDE_LEFT_AXIS       = ecodes.ABS_Z

MAX_SIDE_LEFT_VEL = 0.4
MIN_SIDE_LEFT_VEL = 0

CODE_SIDE_RIGHT_AXIS       = ecodes.ABS_RZ

MAX_SIDE_RIGHT_VEL = -0.4
MIN_SIDE_RIGHT_VEL = 0

# # z axis (D-pad up/down)
CODE_Z_AXIS       = ecodes.ABS_HAT0Y

MAX_Z_VEL = -0.02
MIN_Z_VEL = 0.02

################ BUTTONS #############
# Shoulder buttons
CODE_CONNECT_SERVOS_BUTTON    = ecodes.BTN_TL
CODE_DISCONNECT_SERVOS_BUTTON = ecodes.BTN_TR

# Face buttons (South/East/West/North)
CODE_STOP_SERVOS_BUTTON          = ecodes.BTN_NORTH
CODE_DIAGONAL_SERVOS_BUTTON      = ecodes.BTN_WEST
CODE_DIAGONAL_FAST_SERVOS_BUTTON = ecodes.BTN_SOUTH
CODE_TROT_SERVOS_BUTTON          = ecodes.BTN_EAST
CODE_INPLACE_SERVOS_BUTTON       = ecodes.BTN_SELECT
