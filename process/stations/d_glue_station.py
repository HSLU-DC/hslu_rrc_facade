# d_glue_station.py
"""Station D: Apply adhesive to beam surfaces.

The glue system uses a Robatech hot-melt adhesive melter controlled via
a Beckhoff PLC. A safety handshake (glue_on/glue_off) ensures the safety
cell is closed before enabling the glue system.

Each beam has 1-2 student-provided glue planes (glue_position_a, optional
glue_position_b) on the top face where it will contact the beam above.
The robot drives a predefined glue path pattern at each plane: a zigzag
fill of N parallel lines covering a 15x15 mm active area (5 mm margin
on each side of the 25x25 mm beam top).

Glue application is executed entirely in RAPID (r_RRC_CI_GlueLine) to
avoid Python-ROS-RAPID latency — this ensures consistent glue bead quality.
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
from compas.geometry import Frame

import _skills.custom_motion as cm
from _skills.fabdata import load_data, get_element
from _skills.GlueLine.glue_line import glue_line
from _skills.GluePLC.glue_plc import glue_on, glue_off

from globals import (
    ROBOT_NAME, TOOL_GRIPPER,
    SPEED_WITH_MEMBER, SPEED_APPROACH, SPEED_GLUE, SPEED_NO_MEMBER,
    W_OBJ_GLUE,
)
from joint_positions import jp_glue, jp_glue_rot, jp_glue_transit

# Duration in seconds for coordinated (track + robot) moves
COORD_MOVE_TIME = 1


# ==============================================================================
# Helpers
# ==============================================================================

def _computed_zaxis(frame):
    """Robust z-axis: x cross y. Used for dry_run debug output."""
    z = frame.xaxis.cross(frame.yaxis)
    if z.length > 1e-6:
        z.unitize()
    return z


def _build_offset_frames(start_frame):
    """Build the intermediate helper frames around the glue start position.

    Returns:
        pre: Pre-approach frame (Z+300, X+300, Y-50 from start)
        app: Approach frame (Z+50, X+50, Y-50 from start)
        ret: Retract frame (Z+100, X+100, Y-50 from start)
    """
    pre = start_frame.copy()
    pre.point.z += 300
    pre.point.x += 300
    #pre.point.y -= 50

    app = start_frame.copy()
    app.point.z += 50
    app.point.x += 50
    #app.point.y -= 50

    ret = start_frame.copy()
    ret.point.z += 100
    ret.point.x += 100
    #ret.point.y -= 50

    return pre, app, ret


def _run_glue_line(r1, glue_frame, *, glue_valve_enabled=True,
                   line_length=15, y_step=7.5, num_lines=3,
                   pulse_on_ms=20, pulse_off_ms=20, accel_dist=0):
    """Execute a zigzag fill of parallel glue lines around glue_frame.

    Lines run along the frame's local X. The pattern is centered on glue_frame:
      - Each line is ``line_length`` mm long, from -L/2 to +L/2 in X.
      - ``num_lines`` parallel lines are spaced ``y_step`` mm apart, symmetric
        about Y=0.
      - Direction alternates each line (zigzag); shifts in Y between lines run
        without glue.

    Defaults give a 15x15 mm zigzag fill (3 lines at Y=-7.5/0/+7.5, each 15 mm)
    on the 25x25 mm beam top, leaving a 5 mm margin on every side.

    Args:
        glue_frame: TCP frame at the centre of the active glue area (beam
                    centred under the nozzle).
        glue_valve_enabled: If True, pulses the glue valve during each line.
        line_length: Length of each line along local X in mm.
        y_step: Spacing between adjacent lines in mm.
        num_lines: Number of parallel lines.
        pulse_on_ms / pulse_off_ms / accel_dist: Pass-through to glue_line.
    """
    half_x = line_length / 2.0
    y_offsets = [(i - (num_lines - 1) / 2.0) * y_step for i in range(num_lines)]

    for idx, y_off in enumerate(y_offsets):
        going_neg = (idx % 2 == 0)
        x_offset = -line_length if going_neg else +line_length
        x_start = +half_x if going_neg else -half_x

        line_start = glue_frame.copy()
        line_start.point.x += x_start
        line_start.point.y += y_off

        # Position TCP at line start (no glue). FINE on the first line so the
        # bead begins from rest; Z1 on subsequent shifts to keep it smooth.
        zone_in = rrc.Zone.FINE if idx == 0 else rrc.Zone.Z1
        speed_in = SPEED_APPROACH if idx == 0 else SPEED_GLUE
        r1.send_and_wait(rrc.MoveToFrame(line_start, speed_in, zone_in, rrc.Motion.LINEAR))

        if glue_valve_enabled:
            glue_line(r1, line_start, x_offset=x_offset, speed=SPEED_GLUE,
                      pulse_on_ms=pulse_on_ms, pulse_off_ms=pulse_off_ms,
                      accel_dist=accel_dist)
        else:
            line_end = line_start.copy()
            line_end.point.x += x_offset
            r1.send_and_wait(rrc.MoveToFrame(line_end, SPEED_GLUE, rrc.Zone.FINE, rrc.Motion.LINEAR))


# ==============================================================================
# Glue sequence
# ==============================================================================

def _do_glue_sequence(r1, glue_frame, *, dry_run=False, tag="", glue_valve_enabled=True):
    """app -> start -> glue line -> ret -> jp_glue_rot.

    Precondition: caller MUST position the robot at jp_glue_rot before calling.
    Postcondition: robot is left at jp_glue_rot, ready for the next plane or exit.

    Args:
        r1: AbbClient instance
        glue_frame: Start frame for the glue line
        dry_run: If True, prints planned moves only
        tag: Label used in print statements ("A" or "B")
        glue_valve_enabled: If True, opens/closes glue valve during path
    """
    pre, app, ret = _build_offset_frames(glue_frame)

    prefix = f"[GLUE {tag}] " if tag else "[GLUE] "

    if dry_run:
        z_calc = _computed_zaxis(glue_frame)
        print(f"{prefix}z_calc={z_calc}")
        print(f"{prefix}  rot joints: {jp_glue_rot.robax} extax={jp_glue_rot.extax}")
        print(f"{prefix}  pre:   X={pre.point.x:.0f} Y={pre.point.y:.0f} Z={pre.point.z:.0f}")
        print(f"{prefix}  app:   X={app.point.x:.0f} Y={app.point.y:.0f} Z={app.point.z:.0f}")
        print(f"{prefix}  start: X={glue_frame.point.x:.0f} Y={glue_frame.point.y:.0f} Z={glue_frame.point.z:.0f}")
        print(f"{prefix}  ret:   X={ret.point.x:.0f} Y={ret.point.y:.0f} Z={ret.point.z:.0f}")
        return

    print(f"{prefix}Executing glue sequence.")

    # Caller has already positioned us at jp_glue_rot (a safe rotation pose
    # where J6 can rotate freely without beam-arm collision). After A's
    # sequence ends at jp_glue_rot, B's sequence reuses that state without
    # an extra coord move.

    # Reorient TCP at jp_glue_rot's position to glue_frame's orientation BEFORE
    # translating. A combined rotate+translate move makes the held beam swing
    # during the approach, which can collide with the glue station; rotating in
    # place first means the subsequent move to `app` is pure translation.
    current_tcp = r1.send_and_wait(rrc.GetFrame())
    rot_oriented = Frame(current_tcp.point, glue_frame.xaxis, glue_frame.yaxis)
    r1.send_and_wait(rrc.MoveToFrame(rot_oriented, SPEED_APPROACH, rrc.Zone.FINE, rrc.Motion.LINEAR))

    # Move to approach (pure translation now — orientations match)
    r1.send_and_wait(cm.MoveToRobtarget(app, jp_glue.extax, 2, rrc.Zone.Z10, rrc.Motion.LINEAR))

    # Execute zigzag glue pattern (handles its own move-to-line-start)
    _run_glue_line(r1, glue_frame, glue_valve_enabled=glue_valve_enabled)

    # Retract + go back to pre position
    r1.send(rrc.MoveToFrame(ret, SPEED_WITH_MEMBER, rrc.Zone.Z10, rrc.Motion.LINEAR))
    #r1.send_and_wait(rrc.MoveToFrame(pre, SPEED_WITH_MEMBER, rrc.Zone.Z50, rrc.Motion.LINEAR))

    # Return to safe rotation pose. Replaces the old J6=0 reset: the beam ends
    # up high above the arm, ready for the next glue plane or station exit.
    r1.send_and_wait(cm.MoveToJoints(jp_glue_rot.robax, jp_glue_rot.extax, COORD_MOVE_TIME, rrc.Zone.FINE))


# ==============================================================================
# Main station entry point
# ==============================================================================

def d_glue_station(r1, data, i, *, layer_idx=0, dry_run=False, glue_valve_enabled=True):
    """Apply adhesive to beam at glue positions A and (optionally) B.

    The PLC safety handshake (glue_on/glue_off) brackets the entire operation —
    the glue system is only active while the robot is in the glue station.

    Args:
        r1: AbbClient instance (or None for dry_run)
        data: Loaded fab_data dict
        i: Element index within the layer
        layer_idx: Layer index (0 or 1)
        dry_run: If True, prints planned moves without robot connection
        glue_valve_enabled: If True, pulses the glue valve during motion.
                            If False, robot moves through the glue path
                            without dispensing (for position testing).
    """
    element = get_element(data, i, layer_idx=layer_idx)
    glue_a_frame = element.get("glue_position_a")
    glue_b_frame = element.get("glue_position_b")

    # Skip if no glue positions are defined
    if not glue_a_frame and not glue_b_frame:
        print(f"[GLUE] Skipping element {i} - no glue positions defined")
        return

    if dry_run:
        print(f"[GLUE] i={i} layer={layer_idx}")
        glue_on(r1, dry_run=True)
        if glue_a_frame:
            _do_glue_sequence(r1, glue_a_frame, dry_run=True, tag="A", glue_valve_enabled=glue_valve_enabled)
        if glue_b_frame:
            _do_glue_sequence(r1, glue_b_frame, dry_run=True, tag="B", glue_valve_enabled=glue_valve_enabled)
        glue_off(r1, dry_run=True)
        return

    # Move to glue station
    r1.send(
        cm.MoveToJoints(jp_glue.robax, jp_glue.extax, COORD_MOVE_TIME, rrc.Zone.FINE)
    )
    # Transit waypoint on the way to jp_glue_rot. The direct path would sweep
    # the held beam through the robot arm.
    r1.send_and_wait(
        cm.MoveToJoints(jp_glue_transit.robax, jp_glue_transit.extax, COORD_MOVE_TIME, rrc.Zone.FINE)
    )
    # Position once at jp_glue_rot here so each _do_glue_sequence call can
    # skip the redundant entry move (A leaves us at rot, B reuses it).
    r1.send_and_wait(
        cm.MoveToJoints(jp_glue_rot.robax, jp_glue_rot.extax, COORD_MOVE_TIME, rrc.Zone.FINE)
    )
    print("At glue position.")

    r1.send(rrc.SetWorkObject(W_OBJ_GLUE))

    # PLC Safety Handshake: wait for glue system ready
    glue_on(r1)
    print("Glue system enabled.")

    # A = first call
    if glue_a_frame:
        _do_glue_sequence(r1, glue_a_frame, tag="A", glue_valve_enabled=glue_valve_enabled)

    # B = second call
    if glue_b_frame:
        _do_glue_sequence(r1, glue_b_frame, tag="B", glue_valve_enabled=glue_valve_enabled)

    # Turn glue system off
    glue_off(r1)
    print("Glue system disabled.")

    # Leave station - transit first, then back to jp_glue
    r1.send_and_wait(
        cm.MoveToJoints(jp_glue_transit.robax, jp_glue_transit.extax, COORD_MOVE_TIME, rrc.Zone.FINE)
    )
    r1.send_and_wait(
        cm.MoveToJoints(jp_glue.robax, jp_glue.extax, COORD_MOVE_TIME, rrc.Zone.FINE)
    )
    print("Left glue station.")


if __name__ == "__main__":
    DATA = load_data()

    ros = rrc.RosClient()
    ros.run()

    r1 = rrc.AbbClient(ros, ROBOT_NAME)
    print("Connected.")

    r1.send(rrc.SetTool(TOOL_GRIPPER))

    d_glue_station(r1, DATA, i=0, layer_idx=0, dry_run=False, glue_valve_enabled=False)

    print("Finished")
    ros.close()
    ros.terminate()
