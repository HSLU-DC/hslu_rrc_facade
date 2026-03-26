# d_glue_station.py
"""Glue station: Apply glue at student-provided planes with predefined path pattern."""
import sys
import os
_parent = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _parent not in sys.path:
    sys.path.insert(0, _parent)

import compas_rrc as rrc
from compas.geometry import Vector

import _skills.custom_motion as cm
from _skills.fabdata import load_data, get_element
from _skills.GlueLine.glue_line import glue_line
from _skills.GluePLC.glue_plc import glue_on, glue_off

from globals import (
    ROBOT_NAME, TOOL_GRIPPER,
    SPEED_WITH_MEMBER, SPEED_APPROACH, SPEED_GLUE, SPEED_NO_MEMBER,
    W_OBJ_GLUE,
)
from joint_positions import jp_glue

# Coordinated move time
COORD_MOVE_TIME = 2

# Reference direction for POS/NEG dispatch
SEQ_REF = Vector(0, 1, 0)
EPS = 1e-6


# ==============================================================================
# Helpers
# ==============================================================================

def _computed_zaxis(frame):
    """Robust z-axis calculation via x cross y."""
    z = frame.xaxis.cross(frame.yaxis)
    if z.length > EPS:
        z.unitize()
    return z


def _pick_sequence_by_z(frame, *, ref=SEQ_REF):
    """Decide POS vs NEG based on computed z-axis direction."""
    z = _computed_zaxis(frame)
    d = z.dot(ref)
    return ("POS", z, d) if d >= 0 else ("NEG", z, d)


def _build_offset_frames(start_frame):
    """Build pre-approach, approach, and retract frames."""
    pre = start_frame.copy()
    pre.point.z += 300
    pre.point.x += 300
    pre.point.y -= 50

    app = start_frame.copy()
    app.point.z += 50
    app.point.x += 50
    app.point.y -= 50

    ret = start_frame.copy()
    ret.point.z += 100
    ret.point.x += 100
    ret.point.y -= 50

    return pre, app, ret


def _run_glue_line(r1, glue_frame, *, glue_valve_enabled=True, x_offset=-30,
                   y_offset_return=10, do_return_pass=True,
                   pulse_on_ms=20, pulse_off_ms=20, accel_dist=10):
    """Execute a glue line with optional return pass.

    Pattern:
    1. Forward pass: Start -> End (x_offset direction, with glue)
    2. Shift Y (y_offset_return)
    3. Return pass: End+Y -> Start+Y (with glue)
    """
    end_frame = glue_frame.copy()
    end_frame.point.x += x_offset

    if glue_valve_enabled:
        glue_line(r1, glue_frame, x_offset=x_offset, speed=SPEED_GLUE,
                  pulse_on_ms=pulse_on_ms, pulse_off_ms=pulse_off_ms, accel_dist=accel_dist)

        if do_return_pass:
            return_start = end_frame.copy()
            return_start.point.y += y_offset_return

            r1.send_and_wait(rrc.MoveToFrame(return_start, SPEED_GLUE, rrc.Zone.Z1, rrc.Motion.LINEAR))

            glue_line(r1, return_start, x_offset=-x_offset, speed=SPEED_GLUE,
                      pulse_on_ms=pulse_on_ms, pulse_off_ms=pulse_off_ms, accel_dist=accel_dist)
    else:
        # Dry move without glue (for testing)
        r1.send(rrc.MoveToFrame(glue_frame, SPEED_GLUE, rrc.Zone.Z10, rrc.Motion.LINEAR))
        r1.send_and_wait(rrc.MoveToFrame(end_frame, SPEED_GLUE, rrc.Zone.FINE, rrc.Motion.LINEAR))

        if do_return_pass:
            return_start = end_frame.copy()
            return_start.point.y += y_offset_return
            return_end = glue_frame.copy()
            return_end.point.y += y_offset_return

            r1.send(rrc.MoveToFrame(return_start, SPEED_GLUE, rrc.Zone.Z1, rrc.Motion.LINEAR))
            r1.send_and_wait(rrc.MoveToFrame(return_end, SPEED_GLUE, rrc.Zone.FINE, rrc.Motion.LINEAR))


# ==============================================================================
# POS / NEG Sequences
# ==============================================================================

def _do_glue_sequence_pos(r1, glue_frame, *, dry_run=False, tag="", glue_valve_enabled=True):
    """POS glue sequence."""
    pre, app, ret = _build_offset_frames(glue_frame)

    prefix = f"[GLUE {tag}] " if tag else "[GLUE] "

    if dry_run:
        z_calc = _computed_zaxis(glue_frame)
        print(f"{prefix}seq=POS | z_calc={z_calc}")
        print(f"{prefix}  pre:   X={pre.point.x:.0f} Y={pre.point.y:.0f} Z={pre.point.z:.0f}")
        print(f"{prefix}  app:   X={app.point.x:.0f} Y={app.point.y:.0f} Z={app.point.z:.0f}")
        print(f"{prefix}  start: X={glue_frame.point.x:.0f} Y={glue_frame.point.y:.0f} Z={glue_frame.point.z:.0f}")
        return

    print(f"{prefix}Executing POS glue sequence.")

    r1.send(rrc.MoveToFrame(pre, SPEED_WITH_MEMBER, rrc.Zone.Z200, rrc.Motion.LINEAR))
    r1.send(rrc.MoveToFrame(app, SPEED_WITH_MEMBER, rrc.Zone.Z10, rrc.Motion.LINEAR))
    r1.send_and_wait(rrc.MoveToFrame(glue_frame, SPEED_APPROACH, rrc.Zone.FINE, rrc.Motion.LINEAR))

    _run_glue_line(r1, glue_frame, glue_valve_enabled=glue_valve_enabled)

    r1.send(rrc.MoveToFrame(ret, SPEED_WITH_MEMBER, rrc.Zone.Z10, rrc.Motion.LINEAR))
    r1.send(rrc.MoveToFrame(pre, SPEED_WITH_MEMBER, rrc.Zone.Z50, rrc.Motion.LINEAR))

    # Reset wrist rotation (J6)
    robax, extax = r1.send_and_wait(rrc.GetJoints())
    robax[5] = 0
    r1.send(rrc.MoveToJoints(robax, [], SPEED_WITH_MEMBER, rrc.Zone.Z50))


def _do_glue_sequence_neg(r1, glue_frame, *, dry_run=False, tag="", is_second=False, glue_valve_enabled=True):
    """NEG glue sequence."""
    pre, app, ret = _build_offset_frames(glue_frame)

    prefix = f"[GLUE {tag}] " if tag else "[GLUE] "

    if dry_run:
        z_calc = _computed_zaxis(glue_frame)
        print(f"{prefix}seq=NEG | z_calc={z_calc}")
        print(f"{prefix}  pre:   X={pre.point.x:.0f} Y={pre.point.y:.0f} Z={pre.point.z:.0f}")
        print(f"{prefix}  start: X={glue_frame.point.x:.0f} Y={glue_frame.point.y:.0f} Z={glue_frame.point.z:.0f}")
        return

    print(f"{prefix}Executing NEG glue sequence.")

    r1.send(rrc.MoveToFrame(pre, SPEED_WITH_MEMBER, rrc.Zone.Z200, rrc.Motion.LINEAR))
    r1.send(rrc.MoveToFrame(app, SPEED_WITH_MEMBER, rrc.Zone.Z10, rrc.Motion.LINEAR))
    r1.send_and_wait(rrc.MoveToFrame(glue_frame, SPEED_APPROACH, rrc.Zone.FINE, rrc.Motion.LINEAR))

    _run_glue_line(r1, glue_frame, glue_valve_enabled=glue_valve_enabled)

    r1.send(rrc.MoveToFrame(ret, SPEED_WITH_MEMBER, rrc.Zone.Z10, rrc.Motion.LINEAR))

    if is_second:
        r1.send(rrc.MoveToJoints([-40, -5, 40, 0, 55, -130], [], SPEED_WITH_MEMBER, rrc.Zone.Z10))
    else:
        r1.send(rrc.MoveToJoints([-40, -5, 40, 0, 55, 50], [], SPEED_WITH_MEMBER, rrc.Zone.Z10))
        r1.send(rrc.MoveToJoints([-40, -5, 40, 0, 55, -130], [], SPEED_WITH_MEMBER, rrc.Zone.Z10))


def _do_glue_sequence_auto(r1, glue_frame, *, dry_run=False, tag="", is_second=False, glue_valve_enabled=True):
    """Auto-dispatch: decide POS vs NEG based on z-axis direction."""
    seq, z, d = _pick_sequence_by_z(glue_frame)

    if dry_run:
        prefix = f"[GLUE {tag}] " if tag else "[GLUE] "
        print(f"{prefix}DISPATCH seq={seq} | dot={d:.3f} | valve={glue_valve_enabled}")

    if seq == "POS":
        return _do_glue_sequence_pos(r1, glue_frame, dry_run=dry_run, tag=tag, glue_valve_enabled=glue_valve_enabled)
    return _do_glue_sequence_neg(r1, glue_frame, dry_run=dry_run, tag=tag, is_second=is_second, glue_valve_enabled=glue_valve_enabled)


# ==============================================================================
# Main station entry point
# ==============================================================================

def d_glue_station(r1, data, i, *, layer_idx=0, dry_run=False, glue_valve_enabled=True):
    """Glue station: apply glue at 1-2 student-provided planes.

    The student provides glue plane positions. The robot drives a predefined
    glue path pattern (forward + return pass with Y offset) at each plane.

    Args:
        r1: ABB robot client
        data: Loaded fab_data
        i: Element index
        layer_idx: Layer index (0 or 1)
        dry_run: If True, only print what would happen
        glue_valve_enabled: If True, activates glue valve
    """
    element = get_element(data, i, layer_idx=layer_idx)
    glue_a_frame = element.get("glue_position_a")
    glue_b_frame = element.get("glue_position_b")

    if not glue_a_frame and not glue_b_frame:
        print(f"[GLUE] Skipping element {i} - no glue positions defined")
        return

    if dry_run:
        print(f"[GLUE] i={i} layer={layer_idx}")
        glue_on(r1, dry_run=True)
        if glue_a_frame:
            _do_glue_sequence_auto(r1, glue_a_frame, dry_run=True, tag="A", is_second=False, glue_valve_enabled=glue_valve_enabled)
        if glue_b_frame:
            _do_glue_sequence_auto(r1, glue_b_frame, dry_run=True, tag="B", is_second=True, glue_valve_enabled=glue_valve_enabled)
        glue_off(r1, dry_run=True)
        return

    # Move to glue station
    r1.send_and_wait(
        cm.MoveToJoints(jp_glue.robax, jp_glue.extax, COORD_MOVE_TIME, rrc.Zone.Z30)
    )
    print("At glue position.")

    r1.send(rrc.SetWorkObject(W_OBJ_GLUE))

    # PLC Safety Handshake
    glue_on(r1)
    print("Glue system enabled.")

    # Glue A
    if glue_a_frame:
        _do_glue_sequence_auto(r1, glue_a_frame, tag="A", is_second=False, glue_valve_enabled=glue_valve_enabled)

    # Glue B
    if glue_b_frame:
        _do_glue_sequence_auto(r1, glue_b_frame, tag="B", is_second=True, glue_valve_enabled=glue_valve_enabled)

    # Turn glue off
    glue_off(r1)
    print("Glue system disabled.")

    # Leave station
    r1.send_and_wait(
        cm.MoveToJoints(jp_glue.robax, jp_glue.extax, COORD_MOVE_TIME, rrc.Zone.Z30)
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
