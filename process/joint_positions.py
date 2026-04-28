# joint_positions.py
"""Predefined joint target positions for the Facade production cell.

Each Jointtarget stores 6 robot axis values (robax, in degrees) and
1 external axis value (extax, track position in mm).

These positions were taught on the real robot using the FlexPendant
and serve as safe intermediate waypoints for station transitions.
The coordinated move system (custom_motion.py) interpolates between
these positions to move the track and robot simultaneously.

Joint axes:
    robax[0..5] = J1..J6 in degrees
    extax[0] = Track position in mm (0 = far end, 2900 = near end)
"""


class Jointtarget:
    """Simple container for a joint target (robot axes + external axes)."""
    def __init__(self, robax, extax):
        self.robax = robax  # List of 6 floats (degrees)
        self.extax = extax  # List of 1 float (mm)


# ==============================================================================
# Joint limits (from controller configuration)
# ==============================================================================
JOINT_LIMITS = {
    'robax_min': [-270, -180, -225, -180, -180, -270],
    'robax_max': [270, 180, 85, 180, 180, 270],
    'extax_min': [0.0],
    'extax_max': [2900.0],
}


# ==============================================================================
# Standard positions
# ==============================================================================
jp_calib = Jointtarget([0, 0, 0, 0, 0, 0], [0.0])           # Calibration (all zeros)
jp_home = Jointtarget([-40, 20, 0, 0, 70, -40], [500.0])    # Home / safe pose at process start and end
jp_park = Jointtarget([-90, -30, 55, 0, -25, 90], [1500.0])     # Park (compact, track retracted)


# ==============================================================================
# Station entry positions
# ==============================================================================
# These are the joint positions the robot moves to before entering each station.
# The track position (extax) is set to bring the robot near the station.
jp_pick = Jointtarget([-53.13, 44.33, 9.19, 0.38, 36.05, -53.1], [1000.0])
jp_cut = Jointtarget([-50, 20, 40, 0, 30, -50], [500.0])
jp_glue = Jointtarget([-33.61, 7.22, 58.23, 0.0, 24.55, -33.61], [500.0])
# Glue rotation transit: TCP points -Z and the beam is held high above the
# robot arm, so J6 can rotate freely without self-collision. Used as the
# entry/exit pose around each glue plane (see d_glue_station._do_glue_sequence).
jp_glue_rot = Jointtarget([-70, 35, -15, 179, 110, -20], [1000.0])
# Intermediate waypoint between jp_glue and jp_glue_rot. The direct path
# between them would sweep the held beam through the robot arm; this pose
# splits the trajectory into two safe steps. Used at glue station entry/exit.
jp_glue_transit = Jointtarget([-75, 25, 20, 100, 80, -135], [1000.0])
jp_pre_app_place = Jointtarget([-30, 0, 25, 0, 65, -30], [500.0])  # TODO: verify
jp_app_place = Jointtarget([15, 0, 25, 0, 65, 15], [500.0])        # TODO: verify
jp_place = Jointtarget([50, 0, 30, 0, 65, 45], [700.0])            # TODO: verify
