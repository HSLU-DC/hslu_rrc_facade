# a_pick_station.py
"""Pick station: Grasp beam from dynamic wood storage with CSS (Compliant Servo)."""
import sys
import os
_parent = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _parent not in sys.path:
    sys.path.insert(0, _parent)

import compas_rrc as rrc
from compas.geometry import Frame

import _skills.custom_motion as cm
from _skills.fabdata import load_data, get_element
from _skills.gripper import gripper_open, gripper_close
from _skills.SimBeam import sim_beam_activate
from _skills.WoodStorage.wood_storage import WoodStorage

from globals import (
    ROBOT_NAME, TOOL_GRIPPER,
    SPEED_NO_MEMBER, SPEED_WITH_MEMBER, SPEED_APPROACH, SPEED_PRECISE,
)
from joint_positions import jp_pick

# Time for coordinated moves (seconds)
COORD_MOVE_TIME = 1


def a_pick_station(r1, data, i, *, layer_idx=0, dry_run=False, css_enabled=True, sim_beams=False):
    """Pick beam from storage.

    Args:
        r1: ABB robot client
        data: Loaded fab_data
        i: Element index
        layer_idx: Layer index (0 or 1)
        dry_run: If True, only print what would happen
        css_enabled: If True, use CSS for compliant gripping
    """
    storage = WoodStorage()

    element = get_element(data, i, layer_idx=layer_idx)
    beam_size = element.get("beam_size", "small").strip('"').strip("'")

    pick_frame, compartment_id, wobj, extax = storage.get_pick_frame(beam_size)

    # Approach frames
    pre_approach = Frame([300, 380, 200], [-1, 0, 0], [0, 1, 0])

    approach_high = pick_frame.copy()
    approach_high.point.z = 200

    approach_low = pick_frame.copy()
    approach_low.point.z += 50

    # Retract frames (offset to avoid scraping edge)
    retract_offset = 10
    retract_start = Frame(
        [pick_frame.point.x + retract_offset, pick_frame.point.y, pick_frame.point.z + retract_offset],
        pick_frame.xaxis, pick_frame.yaxis
    )
    retract_low = Frame(
        [approach_low.point.x + retract_offset, approach_low.point.y, approach_low.point.z + retract_offset],
        approach_low.xaxis, approach_low.yaxis
    )
    retract_high = Frame(
        [approach_high.point.x + retract_offset, approach_high.point.y, approach_high.point.z + retract_offset],
        approach_high.xaxis, approach_high.yaxis
    )

    if dry_run:
        print(f"[PICK] layer={layer_idx} i={i}")
        print(f"  beam_size: {beam_size}")
        print(f"  compartment: {compartment_id}")
        print(f"  wobj: {wobj}, extax: {extax}")
        print(f"  pick_frame: X={pick_frame.point.x:.0f} Y={pick_frame.point.y:.0f} Z={pick_frame.point.z:.0f}")
        return

    # Set work object
    r1.send(rrc.SetWorkObject(wobj))

    # Open gripper
    gripper_open(r1, dry_run=dry_run, wait=False)

    # === APPROACH ===

    # Coordinated move to pre-approach with track
    r1.send_and_wait(cm.MoveToRobtarget(
        frame=pre_approach,
        ext_axes=[extax],
        time=COORD_MOVE_TIME,
        zone=rrc.Zone.Z50,
        motion_type=rrc.Motion.JOINT
    ))
    print(f"At pre-approach (extax={extax}).")

    # Down to approach_high (Z=200)
    r1.send(rrc.MoveToFrame(approach_high, SPEED_NO_MEMBER, rrc.Zone.Z10, rrc.Motion.LINEAR))

    # Down to approach_low (50mm above pick)
    r1.send(rrc.MoveToFrame(approach_low, SPEED_APPROACH, rrc.Zone.Z5, rrc.Motion.LINEAR))

    # Down to pick_frame
    r1.send(rrc.MoveToFrame(pick_frame, SPEED_PRECISE, rrc.Zone.FINE, rrc.Motion.LINEAR))

    # === CSS + GRIP ===

    if css_enabled:
        r1.send_and_wait(rrc.CustomInstruction('r_RRC_CI_CSS', ['Define', 'CSS_YZ'], [50, 50, 100]))
        r1.send_and_wait(rrc.CustomInstruction('r_RRC_CI_CSS', ['On', 'AllowMove'], []))

        pick_press = pick_frame.copy()
        pick_press.point.z -= 5  # Smaller press for 25mm beams
        r1.send(rrc.MoveToFrame(pick_press, SPEED_PRECISE, rrc.Zone.FINE, rrc.Motion.LINEAR))

    r1.send(rrc.WaitTime(0.5))

    # Close gripper
    gripper_close(r1, dry_run=dry_run, wait=True)

    # Activate beam geometry in simulation (beam appears at TCP)
    if sim_beams:
        sim_beam_activate(r1, layer_idx, i, dry_run=dry_run)

    # Define and activate gripper load
    r1.send(rrc.CustomInstruction('r_RRC_CI_GripLoad', ['Define'], [0.3, 0, 0, 0.01, 1, 0, 0, 0, 0, 0, 0]))
    r1.send(rrc.CustomInstruction('r_RRC_CI_GripLoad', ['On'], []))

    r1.send(rrc.WaitTime(0.5))

    # Update storage inventory
    storage.take_beam(compartment_id)

    # === RETRACT (offset path) ===

    r1.send(rrc.MoveToFrame(retract_start, SPEED_PRECISE, rrc.Zone.Z5, rrc.Motion.LINEAR))
    r1.send(rrc.MoveToFrame(retract_low, SPEED_PRECISE, rrc.Zone.Z5, rrc.Motion.LINEAR))
    r1.send(rrc.MoveToFrame(retract_high, SPEED_APPROACH, rrc.Zone.Z10, rrc.Motion.LINEAR))

    if css_enabled:
        r1.send_and_wait(rrc.CustomInstruction('r_RRC_CI_CSS', ['Off'], []))

    exit_frame = pre_approach.copy()
    exit_frame.point.y -= 150
    r1.send_and_wait(rrc.MoveToFrame(exit_frame, SPEED_WITH_MEMBER, rrc.Zone.Z50, rrc.Motion.LINEAR))

    print(f"Left pick station. Took from {compartment_id}.")


if __name__ == "__main__":
    DATA = load_data()

    storage = WoodStorage()
    storage.print_status()

    ros = rrc.RosClient()
    ros.run()

    r1 = rrc.AbbClient(ros, ROBOT_NAME)
    print("Connected.")

    r1.send(rrc.SetTool(TOOL_GRIPPER))

    a_pick_station(r1, DATA, i=0, layer_idx=0, dry_run=False)

    storage.print_status()

    print("Finished")
    ros.close()
    ros.terminate()
