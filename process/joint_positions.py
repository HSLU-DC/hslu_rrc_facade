# joint_positions.py
"""Joint target positions for all stations."""


class Jointtarget:
    def __init__(self, robax, extax):
        self.robax = robax
        self.extax = extax


# ===========================================
# Joint limits
# ===========================================

JOINT_LIMITS = {
    'robax_min': [-270, -180, -225, -180, -180, -270],
    'robax_max': [270, 180, 85, 180, 180, 270],
    'extax_min': [0.0],
    'extax_max': [2900.0],
}


# ===========================================
# Standard positions
# ===========================================

jp_calib = Jointtarget([0, 0, 0, 0, 0, 0], [0.0])
jp_home = Jointtarget([90, 0, 0, 0, 90, 90], [1000.0])
jp_park = Jointtarget([180, -30, 75, 0, 45, 90], [0.0])


# ===========================================
# Station positions
# ===========================================

jp_pick = Jointtarget([-53.13, 44.33, 9.19, 0.38, 36.05, -53.1], [1000.0])
jp_cut = Jointtarget([-50, 20, 40, 0, 30, -50], [500.0])
jp_glue = Jointtarget([-33.61, 7.22, 58.23, 0.0, 24.55, -33.61], [200.0])
jp_pre_app_place = Jointtarget([-30, 0, 25, 0, 65, -30], [500.0])  # TODO: verify
jp_app_place = Jointtarget([15, 0, 25, 0, 65, 15], [500.0])  # TODO: verify
jp_place = Jointtarget([50, 0, 30, 0, 65, 45], [700.0])  # TODO: verify
