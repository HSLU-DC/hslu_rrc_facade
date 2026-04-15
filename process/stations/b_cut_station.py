# b_cut_station.py
"""Cut station: Dual cuts (1D Gehrungsschnitte only, no Schifterschnitte)."""
import sys
import os
_parent = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _parent not in sys.path:
    sys.path.insert(0, _parent)

import compas_rrc as rrc
from compas.geometry import Frame, Point

import _skills.custom_motion as cm
from _skills.fabdata import load_data, get_element
from _skills.SimBeam import sim_swap_cut_a, sim_swap_cut_b
from _skills.WoodStorage.wood_storage import WoodStorage

from globals import (
    ROBOT_NAME, TOOL_GRIPPER,
    SPEED_WITH_MEMBER, SPEED_APPROACH, SPEED_CUT,
    W_OBJ_CUT,
)
from joint_positions import jp_cut

# Storage instance for extax lookup
storage = WoodStorage()

# Coordinated move time
COORD_MOVE_TIME = 2

# Track position for cut station
EXTAX_CUT = 500


def _get_rotation_frame(current_point, cut_frame):
    """Create rotation frame: current position + cut_frame orientation."""
    return Frame(current_point, cut_frame.xaxis, cut_frame.yaxis)


def _get_original_rotation_frame(cut_frame):
    """Create original rotation frame: offset from cut_frame position."""
    return Frame(
        Point(
            cut_frame.point.x - 100,
            cut_frame.point.y - 150,
            cut_frame.point.z + 300
        ),
        cut_frame.xaxis,
        cut_frame.yaxis
    )


def _do_cut_sequence(r1, cut_frame, rotation_point=None, *, dry_run=False,
                     saw_on=False, saw_off=False, skip_initial_move=False,
                     use_original_rotation=False, on_arrived=None):
    """Run approach + cut + retract sequence for one cut frame.

    Args:
        cut_frame: Target cut position frame
        rotation_point: Point for rotation frame
        saw_on: Turn saw ON before cutting
        saw_off: Turn saw OFF after cutting
        skip_initial_move: Skip move to rotation_frame (already there)
        use_original_rotation: Use offset-based rotation frame
        on_arrived: Callable invoked right after the robot reaches cut_frame
    """
    if dry_run:
        print(f"  [CUT-SEQ] frame: X={cut_frame.point.x:.1f} Y={cut_frame.point.y:.1f} Z={cut_frame.point.z:.1f} | saw_on={saw_on} | saw_off={saw_off}")
        return

    # Rotation frame
    if use_original_rotation:
        rotation_frame = _get_original_rotation_frame(cut_frame)
    else:
        rotation_frame = _get_rotation_frame(rotation_point, cut_frame)

    # Move to rotation frame
    if not skip_initial_move:
        r1.send(rrc.MoveToFrame(rotation_frame, SPEED_WITH_MEMBER, rrc.Zone.Z50, rrc.Motion.JOINT))

    # Approach: 80mm above cut
    cut_approach = cut_frame.copy()
    cut_approach.point.z += 80
    r1.send(rrc.MoveToFrame(cut_approach, SPEED_WITH_MEMBER, rrc.Zone.Z10, rrc.Motion.LINEAR))

    # Saw on
    if saw_on:
        r1.send_and_wait(rrc.CustomInstruction("r_HSLU_SawOn", [], []))

    # Cut move
    r1.send(rrc.MoveToFrame(cut_frame, SPEED_CUT, rrc.Zone.FINE, rrc.Motion.LINEAR))

    # Trigger swap/hook the moment the robot arrives at cut_frame
    if on_arrived is not None:
        on_arrived()

    # Retract slightly in -X
    cut_retract = cut_frame.copy()
    cut_retract.point.x -= 30
    r1.send(rrc.MoveToFrame(cut_retract, SPEED_CUT, rrc.Zone.FINE, rrc.Motion.LINEAR))

    # Saw off
    if saw_off:
        r1.send_and_wait(rrc.CustomInstruction("r_HSLU_SawOff", [], []))

    # Retract to rotation frame
    r1.send(rrc.MoveToFrame(rotation_frame, SPEED_WITH_MEMBER, rrc.Zone.Z50, rrc.Motion.LINEAR))


def b_cut_station(r1, data, i, *, layer_idx=0, dry_run=False, saw_enabled=True, sim_beams=False):
    """Execute dual cuts on element.

    Both cuts are always executed (both ends of the beam).
    Only 1D miter cuts (Gehrungsschnitte) are supported.

    Args:
        r1: ABB robot client
        data: Loaded fab_data
        i: Element index
        layer_idx: Layer index (0 or 1)
        dry_run: If True, only print what would happen
        saw_enabled: If True, controls saw on/off
    """
    element = get_element(data, i, layer_idx=layer_idx)
    cut_a_frame = element["cut_position_a"]
    cut_b_frame = element["cut_position_b"]

    beam_size = element.get("beam_size", "small").strip('"').strip("'")
    pick_extax = storage.get_extax(beam_size)

    # Track compensation
    track_delta = pick_extax - EXTAX_CUT

    if dry_run:
        print(f"[CUT] i={i} layer={layer_idx} | saw={saw_enabled}")
        print(f"  beam_size: {beam_size} | pick_extax: {pick_extax} | delta: {track_delta}")
        _do_cut_sequence(None, cut_a_frame, dry_run=True, saw_on=saw_enabled, saw_off=False)
        _do_cut_sequence(None, cut_b_frame, dry_run=True, saw_on=False, saw_off=saw_enabled)
        return

    # Set work object
    r1.send(rrc.SetWorkObject(W_OBJ_CUT))

    # Read current position in cut wobj coordinates
    current_frame = r1.send_and_wait(rrc.GetFrame())
    print(f"Current position in cut wobj: {current_frame.point}")

    # Adjust X for track movement compensation
    rotation_point = Point(
        current_frame.point.x - track_delta,
        current_frame.point.y,
        current_frame.point.z
    )
    print(f"Adjusted rotation point (delta={track_delta}): {rotation_point}")

    # Coordinated move to cut station with orientation change
    start_frame_a = Frame(rotation_point, cut_a_frame.xaxis, cut_a_frame.yaxis)

    r1.send_and_wait(cm.MoveToRobtarget(
        frame=start_frame_a,
        ext_axes=[EXTAX_CUT],
        time=COORD_MOVE_TIME,
        zone=rrc.Zone.Z1,
        motion_type=rrc.Motion.LINEAR
    ))
    print("At cut station (coordinated move).")

    # Swap hooks: fire the moment the robot arrives at the cut frame
    on_arrived_a = (lambda: sim_swap_cut_a(r1, dry_run=dry_run)) if sim_beams else None
    on_arrived_b = (lambda: sim_swap_cut_b(r1, dry_run=dry_run)) if sim_beams else None

    # Cut A: saw ON, stays on
    _do_cut_sequence(r1, cut_a_frame, rotation_point,
                     saw_on=saw_enabled, saw_off=False, skip_initial_move=True,
                     on_arrived=on_arrived_a)

    # Cut B: saw stays on, OFF at end
    _do_cut_sequence(r1, cut_b_frame,
                     saw_on=False, saw_off=saw_enabled,
                     skip_initial_move=False, use_original_rotation=True,
                     on_arrived=on_arrived_b)

    # Leave station
    r1.send_and_wait(rrc.MoveToJoints(jp_cut.robax, [], SPEED_WITH_MEMBER, rrc.Zone.Z30))
    print("Left cut station.")


if __name__ == "__main__":
    DATA = load_data()

    ros = rrc.RosClient()
    ros.run()

    r1 = rrc.AbbClient(ros, ROBOT_NAME)
    print("Connected.")

    r1.send(rrc.SetTool(TOOL_GRIPPER))

    b_cut_station(r1, DATA, i=0, layer_idx=0, dry_run=False, saw_enabled=False)

    print("Finished")
    ros.close()
    ros.terminate()
