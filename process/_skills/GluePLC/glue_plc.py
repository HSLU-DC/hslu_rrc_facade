# glue_plc.py
"""Glue system PLC control with safety handshake.

Controls the glue system via PLC signals:
- Waits for safety confirmation before enabling
- Checks that glue system is actually running before proceeding
"""
from __future__ import annotations

import compas_rrc as rrc


def glue_on(r1, *, dry_run: bool = False, wait: bool = True) -> None:
    """Turn glue system on with PLC safety handshake.

    Waits for:
    1. di_PLC1_GlueReady (safety cell closed and locked)
    2. di_PLC1_GlueOn (glue system confirmed running)

    Args:
        r1: AbbClient instance
        dry_run: If True, only prints what would happen
        wait: If True, waits for confirmation (default: True)
    """
    if dry_run or r1 is None:
        print("[GLUE-PLC] GlueOn (wait for safety, then enable)")
        return

    cmd = rrc.CustomInstruction("r_HSLU_GlueOn", [], [])
    if wait:
        r1.send_and_wait(cmd)
    else:
        r1.send(cmd)


def glue_off(r1, *, dry_run: bool = False, wait: bool = True) -> None:
    """Turn glue system off.

    Waits for confirmation that glue system has stopped.

    Args:
        r1: AbbClient instance
        dry_run: If True, only prints what would happen
        wait: If True, waits for confirmation (default: True)
    """
    if dry_run or r1 is None:
        print("[GLUE-PLC] GlueOff")
        return

    cmd = rrc.CustomInstruction("r_HSLU_GlueOff", [], [])
    if wait:
        r1.send_and_wait(cmd)
    else:
        r1.send(cmd)
