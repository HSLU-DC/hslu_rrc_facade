# e_place_station.py
"""Place station: Precision placement of beams on facade frame.

Approach frames (pre-app, app, rot) are computed from place_position,
so students only need to provide the final placement frame.
"""
import sys
import os
_parent = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _parent not in sys.path:
    sys.path.insert(0, _parent)

import math
import compas_rrc as rrc
from compas_rrc import Motion
from compas.geometry import Frame, Vector, Rotation

import _skills.custom_motion as cm
from _skills.fabdata import load_data, get_element
from _skills.gripper import gripper_open

from globals import (
    ROBOT_NAME, TOOL_GRIPPER,
    SPEED_APPROACH, SPEED_PRECISE, SPEED_NO_MEMBER, SPEED_WITH_MEMBER,
    W_OBJ_PLACE, OFFSET_TRACK_W_OBJ_PLACE,
)
from joint_positions import jp_glue

# Coordinated move time
COORD_MOVE_TIME = 1

# Dynamic offset configuration (linear interpolation)
X_REF_MIN = 0
X_REF_MAX = 2500
OFFSET_AT_X_MIN = -800
OFFSET_AT_X_MAX = 200

# Joint positions for approach (POS/NEG depending on place_frame.xaxis)
JOINTS = {
    "app": {
        "pos": {"robax": [-30, 20, 40, 0, 30, -35], "extax": [500]},   # TODO: verify
        "neg": {"robax": [-30, 20, 40, 0, 30, 145], "extax": [500]},   # TODO: verify
    },
    "place": {
        "pos": {"robax": [27.17, 39.31, 25.5, 178.96, -25.42, 207.17], "extax": []},  # TODO: verify
        "neg": {"robax": [27.17, 39.31, 25.5, 178.96, -25.42, 27.17], "extax": []},   # TODO: verify
    },
}

# Rotation after placement (slight twist to ensure contact)
PLACE_ROTATION_DEG = 1.5


# ==============================================================================
# Helpers
# ==============================================================================

def is_xaxis_positive(frame, threshold=0.0):
    """True if frame.xaxis points more towards world +X than -X."""
    return frame.xaxis.dot(Vector(1, 0, 0)) >= threshold


def set_frame_x(frame, x):
    """Return a copy of a frame with point.x set to x."""
    f = frame.copy()
    f.point.x = x
    return f


def calculate_dynamic_offset(x):
    """Linear interpolation of track offset based on X coordinate."""
    if X_REF_MAX == X_REF_MIN:
        return OFFSET_AT_X_MIN
    t = (x - X_REF_MIN) / (X_REF_MAX - X_REF_MIN)
    return OFFSET_AT_X_MIN + t * (OFFSET_AT_X_MAX - OFFSET_AT_X_MIN)


def get_jointset(name, sign):
    """Return (robax, extax) for given pose name and sign ('pos'/'neg')."""
    js = JOINTS[name][sign]
    return list(js["robax"]), list(js["extax"])


def rotate_frame_about_local_z(frame, deg_cw):
    """Rotate frame about its own local z-axis by deg_cw clockwise."""
    angle = math.radians(-deg_cw)
    R = Rotation.from_axis_and_angle(frame.zaxis, angle, point=frame.point)
    return frame.transformed(R)


def create_approach_frame(place_frame, sign):
    """Create approach frame 150mm above place_frame with standard orientation.

    Uses a standard orientation (no wrist rotation) to prevent J6 spinning
    on the way from previous station.
    """
    point = place_frame.point.copy()
    point.z += 150

    if sign == "pos":
        xaxis = Vector(1, 0, 0)
        yaxis = Vector(0, -1, 0)
    else:
        xaxis = Vector(-1, 0, 0)
        yaxis = Vector(0, 1, 0)

    return Frame(point, xaxis, yaxis)


def create_intermediate_frames(place_frame, sign, offset):
    """Compute app and rot frames from place_position.

    These were separate GH outputs in Swissbau but are now computed
    automatically to reduce student error potential.

    Args:
        place_frame: Final placement frame (with X already set to offset)
        sign: "pos" or "neg"
        offset: X offset value

    Returns:
        (app_frame, rot_frame) - intermediate approach frames
    """
    # App frame: 80mm above place, slightly offset in Y
    app_frame = place_frame.copy()
    app_frame.point.z += 80
    app_frame.point.y -= 20

    # Rot frame: 30mm above place (orientation transition)
    rot_frame = place_frame.copy()
    rot_frame.point.z += 30

    return app_frame, rot_frame


# ==============================================================================
# Station
# ==============================================================================

def e_place_station(r1, data, i, *, layer_idx=0, dry_run=False):
    """Place beam at target position on facade frame.

    The student provides only place_position. All intermediate frames
    (approach, rotation) are computed automatically.

    Args:
        r1: ABB robot client
        data: Loaded fab_data
        i: Element index
        layer_idx: Layer index (0 or 1)
        dry_run: If True, only print what would happen
    """
    element = get_element(data, i, layer_idx=layer_idx)
    place_frame = element["place_position"]

    original_x = place_frame.point.x
    offset = calculate_dynamic_offset(original_x)

    # Track position
    trackpos = place_frame.point.x - offset + OFFSET_TRACK_W_OBJ_PLACE

    # POS/NEG based on x-axis direction
    sign = "pos" if is_xaxis_positive(place_frame) else "neg"

    # Joint positions
    app_robax, app_extax = get_jointset("app", sign)

    # Offset X for all frames
    place_frame = set_frame_x(place_frame, offset)

    # Compute intermediate frames from place_position
    app_frame, rot_frame = create_intermediate_frames(place_frame, sign, offset)

    # Approach frame: 150mm above with standard orientation
    approach_above_place = create_approach_frame(place_frame, sign)

    if dry_run:
        print(f"[PLACE] i={i} layer={layer_idx}")
        print(f"  original_x: {original_x:.1f}")
        print(f"  offset:     {offset:.1f}")
        print(f"  sign:       {sign}")
        print(f"  trackpos:   {trackpos:.1f}")
        print(f"  approach:   X={approach_above_place.point.x:.1f} Y={approach_above_place.point.y:.1f} Z={approach_above_place.point.z:.1f}")
        print(f"  app_frame:  X={app_frame.point.x:.1f} Y={app_frame.point.y:.1f} Z={app_frame.point.z:.1f}")
        print(f"  rot_frame:  X={rot_frame.point.x:.1f} Y={rot_frame.point.y:.1f} Z={rot_frame.point.z:.1f}")
        print(f"  place:      X={place_frame.point.x:.1f} Y={place_frame.point.y:.1f} Z={place_frame.point.z:.1f}")
        return

    # === MOTION SEQUENCE ===

    r1.send_and_wait(cm.MoveToJoints(app_robax, app_extax, COORD_MOVE_TIME, rrc.Zone.Z1))

    r1.send(rrc.SetWorkObject(W_OBJ_PLACE))

    # Coordinated move: frame + track
    r1.send_and_wait(cm.MoveToRobtarget(
        frame=approach_above_place,
        ext_axes=[trackpos],
        time=COORD_MOVE_TIME,
        zone=rrc.Zone.FINE,
        motion_type=Motion.LINEAR
    ))
    print("At place station (150mm above).")

    # Down through app -> rot -> place
    r1.send_and_wait(rrc.MoveToFrame(app_frame, SPEED_APPROACH, rrc.Zone.Z10, rrc.Motion.LINEAR))
    r1.send(rrc.MoveToFrame(rot_frame, SPEED_APPROACH, rrc.Zone.Z10, rrc.Motion.LINEAR))
    r1.send_and_wait(rrc.MoveToFrame(place_frame, SPEED_PRECISE, rrc.Zone.FINE, rrc.Motion.LINEAR))

    # Slight rotation for press contact
    deg_cw = -PLACE_ROTATION_DEG if sign == "pos" else PLACE_ROTATION_DEG
    place_frame_rot = rotate_frame_about_local_z(place_frame, deg_cw=deg_cw)
    r1.send_and_wait(rrc.MoveToFrame(place_frame_rot, SPEED_PRECISE, rrc.Zone.FINE, rrc.Motion.LINEAR))

    r1.send(rrc.WaitTime(5))

    # Release
    gripper_open(r1, dry_run=dry_run, wait=True)

    # Deactivate gripper load
    r1.send_and_wait(rrc.CustomInstruction('r_RRC_CI_GripLoad', ['Off'], []))
    r1.send(rrc.WaitTime(1))

    # Retract
    place_retract = place_frame_rot.copy()
    place_retract.point.z += 50
    r1.send(rrc.MoveToFrame(place_retract, SPEED_APPROACH, rrc.Zone.Z1, rrc.Motion.LINEAR))
    r1.send_and_wait(rrc.MoveToFrame(approach_above_place, SPEED_NO_MEMBER, rrc.Zone.Z1, rrc.Motion.LINEAR))

    # Reset J6
    retract_robax, retract_extax = r1.send_and_wait(rrc.GetJoints())
    retract_robax[5] = -40
    r1.send(rrc.MoveToJoints(retract_robax, retract_extax, SPEED_NO_MEMBER, rrc.Zone.Z1))

    print("Left place station.")


if __name__ == "__main__":
    DATA = load_data()

    ros = rrc.RosClient()
    ros.run()

    r1 = rrc.AbbClient(ros, ROBOT_NAME)
    print("Connected.")

    r1.send(rrc.SetTool(TOOL_GRIPPER))

    e_place_station(r1, DATA, i=0, layer_idx=0, dry_run=False)

    print("Finished")
    ros.close()
    ros.terminate()
