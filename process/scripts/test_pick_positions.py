# test_pick_positions.py
"""Singularity test for all pick positions across all compartments.

Visits every possible pick frame in wood_storage.json without grabbing
anything. Useful before production to verify no compartment / stack height
combination hits a singularity or is unreachable.

Per compartment, per beam slot 1..capacity:
    1. Coordinated track + joint move to pre_approach (between compartments
       the robot returns to jp_pick first to reset state)
    2. Linear approach: pre_approach -> approach_high -> approach_low -> pick_frame
    3. Brief pause for visual inspection
    4. Linear retract: pick_frame -> approach_low -> approach_high -> pre_approach

Every slot iteration both starts and ends at pre_approach, so the full
descent/ascent path is exercised — not just the pick frame itself.

What it does NOT do (so it is safe to run on full compartments):
    - Open/close gripper
    - Activate CSS (Cartesian Soft Servo)
    - Press 5mm into the beam
    - Define GripLoad
    - Decrement storage counts in wood_storage.json

Stack iteration runs from slot 1 (bottom of stack) to slot capacity (top),
testing the full Z range each compartment can hold regardless of the
current count.
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import compas_rrc as rrc
from compas.geometry import Frame, Point, Vector

import _skills.custom_motion as cm
from _skills.WoodStorage.wood_storage import WoodStorage

from globals import (
    ROBOT_NAME, TOOL_GRIPPER,
    SPEED_NO_MEMBER, SPEED_APPROACH, SPEED_PRECISE,
)
from joint_positions import jp_pick

# Same pre_approach pose as a_pick_station.py
PRE_APPROACH = Frame([300, 380, 200], [-1, 0, 0], [0, 1, 0])

# Time for coordinated (track + robot) moves
COORD_MOVE_TIME = 1

# Pause at each pick position so the user can visually inspect
PAUSE_AT_PICK = 0.3  # seconds


def parse_frame(frame_data):
    """Parse base_frame dict from wood_storage.json into a COMPAS Frame."""
    return Frame(
        Point(*frame_data["point"]),
        Vector(*frame_data["xaxis"]),
        Vector(*frame_data["yaxis"]),
    )


def test_compartment(r1, cid, c):
    """Visit every beam slot (1..capacity) in one compartment."""
    capacity = c["capacity"]
    wobj = c["wobj"]
    extax = c["extax"]
    stack_offset = c["stack_offset_z"]
    base_frame = parse_frame(c["base_frame"])

    print(f"\n{'='*50}")
    print(f"  Compartment {cid} | wobj={wobj} | extax={extax} | capacity={capacity}")
    print(f"{'='*50}")

    r1.send(rrc.SetWorkObject(wobj))

    # Coordinated move: track + robot to pre_approach
    r1.send_and_wait(cm.MoveToRobtarget(
        frame=PRE_APPROACH,
        ext_axes=[extax],
        time=COORD_MOVE_TIME,
        zone=rrc.Zone.Z50,
        motion_type=rrc.Motion.JOINT,
    ))

    # approach_high uses base_frame X/Y with fixed Z=200 (constant within compartment)
    approach_high = Frame(
        [base_frame.point.x, base_frame.point.y, 200],
        base_frame.xaxis,
        base_frame.yaxis,
    )

    # Visit each beam slot. Every iteration starts and ends at pre_approach
    # so the full descent/ascent path is exercised for each slot.
    for n in range(1, capacity + 1):
        z_pick = base_frame.point.z + (n - 1) * stack_offset
        pick_frame = Frame(
            [base_frame.point.x, base_frame.point.y, z_pick],
            base_frame.xaxis,
            base_frame.yaxis,
        )
        approach_low = pick_frame.copy()
        approach_low.point.z += 50

        print(f"  beam {n:>2}/{capacity}  z_pick={z_pick:7.1f}")

        # Descend: pre_approach -> approach_high -> approach_low -> pick
        r1.send(rrc.MoveToFrame(approach_high, SPEED_NO_MEMBER, rrc.Zone.Z10, rrc.Motion.LINEAR))
        r1.send(rrc.MoveToFrame(approach_low, SPEED_APPROACH, rrc.Zone.Z5, rrc.Motion.LINEAR))
        r1.send_and_wait(rrc.MoveToFrame(pick_frame, SPEED_PRECISE, rrc.Zone.FINE, rrc.Motion.LINEAR))

        if PAUSE_AT_PICK > 0:
            r1.send_and_wait(rrc.WaitTime(PAUSE_AT_PICK))

        # Ascend: pick -> approach_low -> approach_high -> pre_approach
        r1.send(rrc.MoveToFrame(approach_low, SPEED_APPROACH, rrc.Zone.Z5, rrc.Motion.LINEAR))
        r1.send(rrc.MoveToFrame(approach_high, SPEED_NO_MEMBER, rrc.Zone.Z10, rrc.Motion.LINEAR))
        r1.send_and_wait(rrc.MoveToFrame(PRE_APPROACH, SPEED_NO_MEMBER, rrc.Zone.Z10, rrc.Motion.LINEAR))

    print(f"  [OK] {capacity} positions visited in {cid}")


def main():
    storage = WoodStorage()

    print("=" * 50)
    print("PICK POSITION SINGULARITY TEST")
    print("=" * 50)

    compartments = list(storage.data["compartments"].items())
    total = sum(c["capacity"] for _, c in compartments)
    print(f"\nCompartments: {len(compartments)}")
    for cid, c in compartments:
        print(f"  {cid}: {c['capacity']} positions (extax={c['extax']})")
    print(f"\nTotal positions to visit: {total}")

    response = input("\nWeiter? (j/n): ").strip().lower()
    if response != 'j':
        print("Abgebrochen.")
        return

    # Connect
    ros = rrc.RosClient()
    ros.run()
    r1 = rrc.AbbClient(ros, ROBOT_NAME)
    print("Connected.")
    r1.send(rrc.SetTool(TOOL_GRIPPER))

    # Start: home position
    r1.send_and_wait(cm.MoveToJoints(jp_pick.robax, jp_pick.extax, COORD_MOVE_TIME, rrc.Zone.Z50))

    # Visit each compartment, returning to jp_pick between them
    for cid, c in compartments:
        test_compartment(r1, cid, c)
        r1.send_and_wait(cm.MoveToJoints(jp_pick.robax, jp_pick.extax, COORD_MOVE_TIME, rrc.Zone.Z50))

    print(f"\n{'='*50}")
    print(f"  DONE - {total} positions visited successfully")
    print(f"{'='*50}")

    ros.close()
    ros.terminate()


if __name__ == "__main__":
    main()
