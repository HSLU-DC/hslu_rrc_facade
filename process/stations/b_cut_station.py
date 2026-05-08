# b_cut_station.py
"""Station B: Cut beam to length with circular saw.

Each beam receives 2 cuts (cut_position_a and cut_position_b) from the
Grasshopper-exported fab_data. The saw is a fixed circular saw — the robot
moves the beam through the blade.

Only 1D miter cuts (Gehrungsschnitte) are supported on the facade project;
no Schifterschnitte (compound miter cuts).

The saw stays ON between cut A and cut B intentionally (faster cycle time,
blade is already spinning).

Cut strategy depends on stock length (see _cut_strategy):
    - 400/550 mm: from-above drop, wobj ob_HSLU_Cut_Top
    - 750/1000 mm: from-behind L-feed, wobj ob_HSLU_Cut_Back

Motion sequence (Layer 0 & 1, horizontal beams only):
    - Robot arrives from pick station holding the beam
    - Track moves from pick position to cut position (EXTAX_CUT=500mm)
    - X-compensation is applied: when the track moves, the robot must
      compensate in X to keep the beam at the same world position
    - Two cuts are executed with approach -> cut -> retract sequences
    - Between cuts, the robot rotates to the second cut orientation
    - Optional SimBeam swap hooks fire when the robot reaches each cut frame

RAPID instructions used:
    - r_HSLU_SawOn / r_HSLU_SawOff: Control saw motor via digital outputs
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
from compas.geometry import Frame, Point

import _skills.custom_motion as cm
from _skills.fabdata import load_data, get_element
from _skills.SimBeam import sim_swap_cut_a, sim_swap_cut_b
from _skills.WoodStorage.wood_storage import WoodStorage

from globals import (
    ROBOT_NAME, TOOL_GRIPPER,
    SPEED_WITH_MEMBER, SPEED_APPROACH, SPEED_CUT,
    W_OBJ_CUT_TOP, W_OBJ_CUT_BACK,
)
from joint_positions import jp_cut

# Storage instance for looking up track position by beam size
storage = WoodStorage()

# Duration in seconds for coordinated (track + robot) moves
COORD_MOVE_TIME = 2

# Fixed track position for the cut station (mm)
EXTAX_CUT = 500

# Beam categories that use the from-above (TOP) cut. Everything else goes
# through the from-behind (BACK) sequence. Short stock can drop vertically
# without collision risk; long stock needs the horizontal feed.
CUT_TOP_CATEGORIES = {"400", "550"}


def _cut_strategy(beam_size):
    """Return (wobj, sequence_func) for a beam_size string.

    Splits the cut station into two physical setups:
    - 400/550 -> ob_HSLU_Cut_Top + _do_cut_sequence_top  (drop from above)
    - 750/1000 -> ob_HSLU_Cut_Back + _do_cut_sequence_back (feed from -Y)
    """
    if beam_size in CUT_TOP_CATEGORIES:
        return W_OBJ_CUT_TOP, _do_cut_sequence_top
    return W_OBJ_CUT_BACK, _do_cut_sequence_back


def _get_rotation_frame(current_point, cut_frame):
    """Create a rotation frame: keeps the robot's current XYZ position but
    adopts the orientation (xaxis/yaxis) of the target cut frame.

    Used for the first cut (A) where the robot needs to rotate the beam
    into the cut orientation while staying at the track-compensated position.
    """
    return Frame(
        current_point,
        cut_frame.xaxis,
        cut_frame.yaxis
    )


def _get_original_rotation_frame(cut_frame):
    """Create a safe rotation frame offset from the cut position.

    Used for the second cut (B) where the robot has already released
    from the first rotation frame and needs a new safe position to
    rotate into the second cut orientation.

    Offset: X-100, Y-150, Z+300 from cut_frame (above and behind the saw).
    """
    return Frame(
        Point(
            cut_frame.point.x - 100,
            cut_frame.point.y - 150,
            cut_frame.point.z + 300
        ),
        cut_frame.xaxis,
        cut_frame.yaxis
    )


def _resolve_rotation_frame(cut_frame, rotation_point, use_original_rotation):
    if use_original_rotation:
        return _get_original_rotation_frame(cut_frame)
    return _get_rotation_frame(rotation_point, cut_frame)


def _do_cut_sequence_top(r1, cut_frame, rotation_point=None, *, dry_run=False,
                         saw_on=False, saw_off=False, skip_initial_move=False,
                         use_original_rotation=False, on_arrived=None):
    """From-above cut sequence (short stock 400/550 with W_OBJ_CUT_TOP).

    Approach 70mm above cut_frame, drop straight down through the blade at
    SPEED_CUT, retract -30mm in X. Original Swissbau-style sequence.
    """
    if dry_run:
        print(f"  [CUT-TOP] frame: X={cut_frame.point.x:.1f} Y={cut_frame.point.y:.1f} Z={cut_frame.point.z:.1f} | saw_on={saw_on} | saw_off={saw_off}")
        return

    rotation_frame = _resolve_rotation_frame(cut_frame, rotation_point, use_original_rotation)

    if not skip_initial_move:
        r1.send(rrc.MoveToFrame(rotation_frame, SPEED_WITH_MEMBER, rrc.Zone.Z50, rrc.Motion.JOINT))

    # Approach 70mm above the cut
    cut_approach = cut_frame.copy()
    cut_approach.point.z += 70
    r1.send(rrc.MoveToFrame(cut_approach, SPEED_WITH_MEMBER, rrc.Zone.Z10, rrc.Motion.LINEAR))

    if saw_on:
        r1.send_and_wait(rrc.CustomInstruction("r_HSLU_SawOn", [], []))

    # Drop straight down through the blade
    r1.send(rrc.MoveToFrame(cut_frame, SPEED_CUT, rrc.Zone.FINE, rrc.Motion.LINEAR))

    if on_arrived is not None:
        on_arrived()

    # Retract sideways in -X to clear the blade
    cut_retract = cut_frame.copy()
    cut_retract.point.x -= 30
    r1.send(rrc.MoveToFrame(cut_retract, SPEED_CUT, rrc.Zone.FINE, rrc.Motion.LINEAR))

    if saw_off:
        r1.send_and_wait(rrc.CustomInstruction("r_HSLU_SawOff", [], []))

    r1.send(rrc.MoveToFrame(rotation_frame, SPEED_WITH_MEMBER, rrc.Zone.Z50, rrc.Motion.LINEAR))


def _do_cut_sequence_back(r1, cut_frame, rotation_point=None, *, dry_run=False,
                          saw_on=False, saw_off=False, skip_initial_move=False,
                          use_original_rotation=False, on_arrived=None):
    """From-behind cut sequence (long stock 750/1000 with W_OBJ_CUT_BACK).

    Two-stage L-shaped approach: (Z+130, Y-150) -> (Z+0, Y-150), then feed
    +Y into cut_frame, retract -30mm in X.
    """
    if dry_run:
        print(f"  [CUT-BACK] frame: X={cut_frame.point.x:.1f} Y={cut_frame.point.y:.1f} Z={cut_frame.point.z:.1f} | saw_on={saw_on} | saw_off={saw_off}")
        return

    rotation_frame = _resolve_rotation_frame(cut_frame, rotation_point, use_original_rotation)

    if not skip_initial_move:
        r1.send(rrc.MoveToFrame(rotation_frame, SPEED_WITH_MEMBER, rrc.Zone.Z50, rrc.Motion.JOINT))

    # L-shaped approach behind the blade
    cut_approach_1 = cut_frame.copy()
    cut_approach_1.point.z += 130
    cut_approach_1.point.y -= 150
    r1.send(rrc.MoveToFrame(cut_approach_1, SPEED_WITH_MEMBER, rrc.Zone.Z10, rrc.Motion.LINEAR))

    cut_approach_2 = cut_frame.copy()
    cut_approach_2.point.y -= 150
    r1.send(rrc.MoveToFrame(cut_approach_2, SPEED_APPROACH, rrc.Zone.Z10, rrc.Motion.LINEAR))

    if saw_on:
        r1.send_and_wait(rrc.CustomInstruction("r_HSLU_SawOn", [], []))

    # Feed +Y into the blade
    r1.send(rrc.MoveToFrame(cut_frame, SPEED_CUT, rrc.Zone.FINE, rrc.Motion.LINEAR))

    if on_arrived is not None:
        on_arrived()

    # Retract sideways in -X to clear the blade
    cut_retract = cut_frame.copy()
    cut_retract.point.x -= 30
    r1.send(rrc.MoveToFrame(cut_retract, SPEED_CUT, rrc.Zone.FINE, rrc.Motion.LINEAR))

    if saw_off:
        r1.send_and_wait(rrc.CustomInstruction("r_HSLU_SawOff", [], []))

    r1.send(rrc.MoveToFrame(rotation_frame, SPEED_WITH_MEMBER, rrc.Zone.Z50, rrc.Motion.LINEAR))


def b_cut_station(r1, data, i, *, layer_idx=0, dry_run=False, saw_enabled=True, sim_beams=False):
    """Cut a beam at 2 positions (A and B).

    Both cuts are always executed (both ends of the beam). Compensates the X
    position for the track movement from the pick station to the cut station.

    Args:
        r1: AbbClient instance (or None for dry_run)
        data: Loaded fab_data dict
        i: Element index within the layer
        layer_idx: Layer index (0 or 1)
        dry_run: If True, prints planned moves without robot connection
        saw_enabled: If True, activates the saw motor via RAPID instruction.
                     If False, robot moves through cut path without cutting
                     (for position testing).
        sim_beams: If True, fires SimBeam swap hooks on cut_a/cut_b arrivals
                   (virtual controller only).
    """
    element = get_element(data, i, layer_idx=layer_idx)
    cut_a_frame = element["cut_position_a"]
    cut_b_frame = element["cut_position_b"]

    # Get beam size and track position from pick station. validate.py
    # already enforces a valid beam_size; missing/unknown values surface
    # as a "Unknown category" ValueError from storage.get_extax().
    beam_size = element.get("beam_size", "").strip('"').strip("'")
    pick_extax = storage.get_extax(beam_size)

    # When the track moves from pick position to cut position, the beam's
    # world X coordinate shifts by the same amount. We need to compensate
    # the robot's X position so the beam stays in the same place relative
    # to the saw work object.
    # Example: pick at extax=1000, cut at extax=500 -> delta=500mm
    track_delta = pick_extax - EXTAX_CUT

    # Pick the right wobj + cut sequence based on stock length:
    # 400/550 use the from-above setup, 750/1000 use the from-behind setup.
    cut_wobj, cut_sequence = _cut_strategy(beam_size)

    if dry_run:
        print(f"[CUT] i={i} layer={layer_idx} | saw={saw_enabled} | wobj={cut_wobj}")
        print(f"  beam_size: {beam_size} | pick_extax: {pick_extax} | delta: {track_delta}")
        cut_sequence(None, cut_a_frame, dry_run=True, saw_on=saw_enabled, saw_off=False)
        cut_sequence(None, cut_b_frame, dry_run=True, saw_on=False, saw_off=saw_enabled)
        return

    # === TRANSITION: Switch to cut wobj and read current position ===
    r1.send(rrc.SetWorkObject(cut_wobj))

    # Read current frame in cut wobj coordinates (robot is at pick exit position)
    current_frame = r1.send_and_wait(rrc.GetFrame())
    print(f"Current position in cut wobj: {current_frame.point}")

    # Adjust X for track movement: when track moves from pick_extax to EXTAX_CUT,
    # the robot needs to compensate in X direction
    rotation_point = Point(
        current_frame.point.x - track_delta,
        current_frame.point.y,
        current_frame.point.z
    )
    print(f"Adjusted rotation point (delta={track_delta}): {rotation_point}")

    # === CUT A: Coordinated move with orientation change ===
    # Create start frame: adjusted position + cut_a orientation
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

    # Run cut A sequence (saw ON at start, stays on)
    cut_sequence(r1, cut_a_frame, rotation_point,
                 saw_on=saw_enabled, saw_off=False, skip_initial_move=True,
                 on_arrived=on_arrived_a)

    # === CUT B: Use original rotation frame (saw stays on, OFF at end) ===
    cut_sequence(r1, cut_b_frame,
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
