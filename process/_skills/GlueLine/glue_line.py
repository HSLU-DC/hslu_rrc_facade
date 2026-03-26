# glue_line.py
"""All-in-one glue line execution in RAPID.

Executes a complete glue line (move + pulse + move) entirely in RAPID
to avoid Python-ROS-RAPID latency issues.
"""
from __future__ import annotations

import compas_rrc as rrc


def glue_line(r1, frame, x_offset: float = -30, speed: float = 50,
              pulse_on_ms: float = 20, pulse_off_ms: float = 20,
              accel_dist: float = 0, *, dry_run: bool = False) -> None:
    """Execute a glue line with pulsing - all in RAPID.

    Moves to start frame, optionally moves accel_dist mm first (acceleration phase
    without pulsing), then pulses while moving the remaining distance to x_offset.
    All synchronized in RAPID - no Python latency issues.

    Requires background task T_GLUE running on the controller.

    Args:
        r1: AbbClient instance
        frame: Start frame (position + orientation)
        x_offset: Offset in X direction for end point in mm (negative = -X direction)
        speed: TCP speed in mm/s
        pulse_on_ms: Valve open time in ms
        pulse_off_ms: Valve closed time in ms
        accel_dist: Acceleration distance in mm (move without pulsing first, default: 0)
        dry_run: If True, only prints what would happen
    """
    if dry_run or r1 is None:
        print(f"[GLUE-LINE] start={frame.point} x_offset={x_offset}mm @ {speed}mm/s, pulse={pulse_on_ms}/{pulse_off_ms}ms, accel={accel_dist}mm")
        return

    # Same layout as MoveToFrame: V1-V7 pos+orient, V8-V13 extax, V14 speed, V15 pulse_off, V16 accel_dist
    pos = list(frame.point)
    rot = list(frame.quaternion)
    ext_axes = [9E+09] * 6  # Use current external axis positions

    float_values = pos + rot + ext_axes + [speed, pulse_off_ms, accel_dist]

    # St1: X offset, St2: pulse ON time
    string_values = [str(x_offset), str(pulse_on_ms)]

    cmd = rrc.CustomInstruction("r_RRC_CI_GlueLine", string_values, float_values)
    r1.send_and_wait(cmd)
