# globals.py
"""Configuration for Facade production."""

# Robot namespace
ROBOT_NAME = "/rob1"

# Project header
PROJECT_NAME = "Facade FS26"

# Tools
TOOL_SPIKE = 't_HSLU_Spike'
TOOL_GRIPPER = 't_HSLU_GripperZimmer'

# Workobjects
# W_OBJ_PICK = defined in wood storage file
W_OBJ_CUT = 'ob_HSLU_Cut'
W_OBJ_GLUE = 'ob_HSLU_Glue'
W_OBJ_PLACE = 'ob_HSLU_Place'
OFFSET_TRACK_W_OBJ_PLACE = 593  # mm

# Speeds
SPEED_GLUE = 1000  # mm/s
SPEED_NO_MEMBER = 500  # mm/s
SPEED_WITH_MEMBER = 300  # mm/s
SPEED_APPROACH = 100  # mm/s
SPEED_PRECISE = 25  # mm/s
SPEED_CUT = 25  # mm/s

# Acceleration, Override, and Max Speed
ACC = 100  # Unit [%]
RAMP = 100  # Unit [%]
OVERRIDE = 100  # Unit [%]
MAX_TCP = 1000  # Unit [mm/s]

# Facade-specific constants
FRAME_WIDTH = 600  # mm (Y-direction)
FRAME_LENGTH = 2500  # mm (X-direction)
BEAM_SECTION = 25  # mm (cross-section of production beams)
FRAME_SECTION = 40  # mm (cross-section of base frame beams)
MAX_LAYERS = 2  # Maximum number of layers
