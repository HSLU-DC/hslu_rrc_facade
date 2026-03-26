# soft_servo.py
"""Soft Servo control functions.

Makes individual robot axes compliant for pressing/assembly operations
without affecting Cartesian positioning accuracy.
"""
from __future__ import annotations

import compas_rrc as rrc


def soft_act(r1, axis: int = 6, softness: float = 50, *, dry_run: bool = False) -> None:
    """Activate soft servo on a specific axis.

    Makes the specified axis compliant - it will yield to external forces
    without triggering SafeMove errors. Other axes remain stiff.

    Args:
        r1: AbbClient instance
        axis: Robot axis number (1-6), default: 6 (wrist rotation)
        softness: Softness percentage (0-100%), default: 50
                  0% = stiff, 100% = very soft
        dry_run: If True, only prints what would happen
    """
    if dry_run or r1 is None:
        print(f"[SOFT] Activate axis {axis} at {softness}% softness")
        return

    cmd = rrc.CustomInstruction("r_RRC_CI_SoftAct", [str(axis)], [softness])
    r1.send(cmd)


def soft_deact(r1, *, dry_run: bool = False, wait: bool = True) -> None:
    """Deactivate soft servo on all axes.

    Returns all axes to normal stiff operation.

    Args:
        r1: AbbClient instance
        dry_run: If True, only prints what would happen
        wait: If True, waits for confirmation (default: True)
    """
    if dry_run or r1 is None:
        print("[SOFT] Deactivate")
        return

    cmd = rrc.CustomInstruction("r_RRC_CI_SoftDeact", [], [])
    if wait:
        r1.send_and_wait(cmd)
    else:
        r1.send(cmd)
