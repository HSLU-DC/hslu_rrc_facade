# e_place_station.py
"""Station E: Place beam onto facade frame.

The final station places the beam at its target position on the facade frame.
The place position comes from Grasshopper fab_data and requires careful
coordination between the track and robot to reach positions across the
full 2.5m frame while avoiding collisions with the enclosure.

Dynamic track offset:
    The frame spans 2500mm in X. The robot arm has limited reach, so
    the track position is calculated dynamically based on where in X the
    beam needs to go. A linear interpolation maps X position to a track
    offset, ensuring the robot can always reach the target without the
    track or beam hitting the safety enclosure walls.

    offset = lerp(X_REF_MIN -> OFFSET_AT_X_MIN, X_REF_MAX -> OFFSET_AT_X_MAX)
    trackpos = original_x - offset + OFFSET_TRACK_W_OBJ_PLACE

POS vs NEG:
    Like the glue station, beams face either direction. The approach
    joints and final orientation differ based on whether the beam's
    X-axis points towards world +X (POS) or -X (NEG).

Student input simplification:
    Students provide ONLY place_position in fab_data; the intermediate
    approach (app, rot) frames are computed automatically by
    create_intermediate_frames() to reduce student error potential.
    The 150mm-above approach uses a fixed standard orientation
    (create_approach_frame) so axis 6 doesn't spin on the way in from
    the previous station.

Motion sequence (Layer 0 & 1, only layers in facade project):
    1. Joint move to approach config (POS/NEG dependent)
    2. Coordinated move to 150mm above place position (track + frame)
    3. Linear approach: app_frame -> rot_frame -> place_frame
    4. Wait 2s, open gripper, optionally release SimBeam geometry
    5. Deactivate GripLoad
    6. Retract vertically, exit via approach, reset J6
"""

# ==============================
# Imports
# ==============================
import sys
import os
_parent = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _parent not in sys.path:
    sys.path.insert(0, _parent)

import compas_rrc as rrc
from compas_rrc import Motion
from compas.geometry import Frame, Vector

import _skills.custom_motion as cm
from _skills.fabdata import load_data, get_element
from _skills.gripper import gripper_open
from _skills.SimBeam import sim_beam_release

from globals import (
    ROBOT_NAME, TOOL_GRIPPER,
    SPEED_APPROACH, SPEED_PRECISE, SPEED_NO_MEMBER, SPEED_WITH_MEMBER,
    W_OBJ_PLACE, OFFSET_TRACK_W_OBJ_PLACE,
)
from joint_positions import jp_glue

# Duration in seconds for coordinated (track + robot) moves
COORD_MOVE_TIME = 1


# ==============================================================================
# Dynamic track offset configuration
# ==============================================================================
# Linear interpolation: maps beam X position to a track offset.
# This ensures the robot can reach all positions across the 2.5m frame
# without the track or beam colliding with the safety enclosure.
X_REF_MIN = 0            # X coordinate for minimum offset
X_REF_MAX = 2500         # X coordinate for maximum offset
OFFSET_AT_X_MIN = -800   # Track offset when beam is at X=0
OFFSET_AT_X_MAX = 200    # Track offset when beam is at X=2500


# ==============================================================================
# Joint configurations for POS/NEG approach
# ==============================================================================
# Different joint configs depending on beam orientation (xaxis direction).
# These were taught on the real robot to avoid singularities and collisions.
JOINTS = {
    "app": {
        "pos": {"robax": [-30, 20, 40, 0, 30, -35], "extax": [500]},   # TODO: verify
        "neg": {"robax": [-30, 20, 40, 0, 30, 145], "extax": [500]},   # TODO: verify
    },
}


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
    """Calculate the dynamic track offset based on the beam's X coordinate.

    Linear interpolation between (X_REF_MIN, OFFSET_AT_X_MIN) and
    (X_REF_MAX, OFFSET_AT_X_MAX).
    """
    if X_REF_MAX == X_REF_MIN:
        return OFFSET_AT_X_MIN
    t = (x - X_REF_MIN) / (X_REF_MAX - X_REF_MIN)
    return OFFSET_AT_X_MIN + t * (OFFSET_AT_X_MAX - OFFSET_AT_X_MIN)


def get_jointset(name, sign):
    """Return (robax, extax) for given pose name and sign ('pos'/'neg')."""
    js = JOINTS[name][sign]
    return list(js["robax"]), list(js["extax"])


def create_approach_frame(place_frame, sign):
    """Create an approach frame 150mm above place_frame with standard orientation.

    Uses a standard orientation (no wrist rotation) to prevent J6 from
    spinning on the way from the previous station to the approach frame.
    """
    point = place_frame.point.copy()
    point.z += 150

    if sign == "pos":
        # X-axis points towards world +X, Z down
        xaxis = Vector(1, 0, 0)
        yaxis = Vector(0, -1, 0)
    else:
        # X-axis points towards world -X, Z down
        xaxis = Vector(-1, 0, 0)
        yaxis = Vector(0, 1, 0)

    return Frame(point, xaxis, yaxis)


def create_intermediate_frames(place_frame, sign, offset):
    """Compute app and rot frames automatically from place_position.

    These were separate GH outputs in the Swissbau26 project but are now
    computed automatically to reduce student error potential — students
    only need to provide place_position.

    Args:
        place_frame: Final placement frame (with X already set to offset)
        sign: "pos" or "neg"
        offset: X offset value

    Returns:
        (app_frame, rot_frame): Intermediate approach frames
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

def e_place_station(r1, data, i, *, layer_idx=0, dry_run=False, sim_beams=False):
    """Place beam at target position on facade frame.

    The student provides only place_position. All intermediate frames
    (approach, rotation) are computed automatically.

    Args:
        r1: AbbClient instance (or None for dry_run)
        data: Loaded fab_data dict
        i: Element index within the layer
        layer_idx: Layer index (0 or 1)
        dry_run: If True, prints planned moves without robot connection
        sim_beams: If True, releases SimBeam geometry at place position
                   after the gripper opens (virtual controller only)
    """
    element = get_element(data, i, layer_idx=layer_idx)
    place_frame = element["place_position"]

    original_x = place_frame.point.x
    offset = calculate_dynamic_offset(original_x)

    # Track position: world X minus dynamic offset plus place wobj offset
    trackpos = place_frame.point.x - offset + OFFSET_TRACK_W_OBJ_PLACE

    # POS/NEG based on x-axis direction of original place_frame
    sign = "pos" if is_xaxis_positive(place_frame) else "neg"

    # Joint position for the approach
    app_robax, app_extax = get_jointset("app", sign)

    # Offset X for all frames (move the place position by the dynamic offset)
    place_frame = set_frame_x(place_frame, offset)

    # Compute intermediate frames from place_position
    app_frame, rot_frame = create_intermediate_frames(place_frame, sign, offset)

    # Approach frame: 150mm above place with standard orientation
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

    # 1. Joint move to approach config
    r1.send_and_wait(cm.MoveToJoints(app_robax, app_extax, COORD_MOVE_TIME, rrc.Zone.Z1))

    # Set work object BEFORE MoveToRobtarget (needs WObj for frame interpretation)
    r1.send(rrc.SetWorkObject(W_OBJ_PLACE))

    # 2. Coordinated move to 150mm above place position (frame + track)
    r1.send_and_wait(cm.MoveToRobtarget(
        frame=approach_above_place,
        ext_axes=[trackpos],
        time=COORD_MOVE_TIME,
        zone=rrc.Zone.FINE,
        motion_type=Motion.LINEAR
    ))
    print("At place station (150mm above).")

    # 3. Linear approach: app -> rot -> place
    r1.send_and_wait(rrc.MoveToFrame(app_frame, SPEED_APPROACH, rrc.Zone.Z10, rrc.Motion.LINEAR))
    r1.send(rrc.MoveToFrame(rot_frame, SPEED_APPROACH, rrc.Zone.Z10, rrc.Motion.LINEAR))
    r1.send_and_wait(rrc.MoveToFrame(place_frame, SPEED_PRECISE, rrc.Zone.FINE, rrc.Motion.LINEAR))

    # 4. Settle, release
    r1.send(rrc.WaitTime(2))
    gripper_open(r1, dry_run=dry_run, wait=True)

    # Detach beam geometry in simulation (stays at place position)
    if sim_beams:
        sim_beam_release(r1, dry_run=dry_run)

    # 5. Deactivate gripper load (no more beam in gripper)
    r1.send_and_wait(rrc.CustomInstruction('r_RRC_CI_GripLoad', ['Off'], []))
    r1.send(rrc.WaitTime(1))

    # 6. Retract vertically + back to approach + reset J6
    place_retract = place_frame.copy()
    place_retract.point.z += 50
    r1.send(rrc.MoveToFrame(place_retract, SPEED_APPROACH, rrc.Zone.Z1, rrc.Motion.LINEAR))
    r1.send_and_wait(rrc.MoveToFrame(approach_above_place, SPEED_NO_MEMBER, rrc.Zone.Z1, rrc.Motion.LINEAR))

    # Reset J6 to a neutral angle (avoid carrying wrist twist into next pick)
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
