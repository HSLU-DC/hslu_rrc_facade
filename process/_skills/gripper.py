# gripper.py
from __future__ import annotations

import compas_rrc as rrc


def gripper_close(r1, *, dry_run: bool = False, wait: bool = True) -> None:
    """Close the gripper via RAPID custom instruction."""
    if dry_run or r1 is None:
        print("[GRIPPER] close")
        return

    cmd = rrc.CustomInstruction("r_HSLU_GripperClose", [], [])
    if wait:
        r1.send_and_wait(cmd)
    else:
        r1.send(cmd)


def gripper_open(r1, *, dry_run: bool = False, wait: bool = True) -> None:
    """Open the gripper via RAPID custom instruction."""
    if dry_run or r1 is None:
        print("[GRIPPER] open")
        return

    cmd = rrc.CustomInstruction("r_HSLU_GripperOpen", [], [])
    if wait:
        r1.send_and_wait(cmd)
    else:
        r1.send(cmd)
