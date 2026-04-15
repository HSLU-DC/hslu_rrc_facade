# sim_beam.py
"""SimBeam: Beam geometry visualization for RobotStudio simulation.

Controls a SmartComponent via RAPID custom instructions that set
digital/group outputs. Only active on virtual controller (NOT RobOS).

RAPID module: HSLU_SimBeam.mod
Signals: go_SC_ElementID, go_SC_LayerID, do_SC_Activate,
         do_SC_SwapCutA, do_SC_SwapCutB, do_SC_Release, di_SC_Ready
"""
from __future__ import annotations

import compas_rrc as rrc


def sim_beam_activate(r1, layer: int, element: int,
                      *, dry_run: bool = False) -> None:
    """Show raw beam and attach to gripper.

    Sets ElementID and LayerID group outputs, then pulses Activate.
    SmartComponent loads the corresponding STL and attaches it to the flange.

    Args:
        r1: AbbClient instance
        layer: Layer index (0 or 1)
        element: Element index
        dry_run: If True, only prints what would happen
    """
    if dry_run or r1 is None:
        print(f"[SIM-BEAM] Activate L{layer}_E{element}")
        return

    cmd = rrc.CustomInstruction("r_HSLU_SimBeamActivate", [], [layer, element])
    r1.send_and_wait(cmd)


def sim_swap_cut_a(r1, *, dry_run: bool = False) -> None:
    """Swap beam geometry to after-cut-A state.

    Replaces the current raw beam mesh with the cut-A mesh
    (one end mitered, one end square).

    Args:
        r1: AbbClient instance
        dry_run: If True, only prints what would happen
    """
    if dry_run or r1 is None:
        print("[SIM-BEAM] SwapCutA")
        return

    r1.send_and_wait(rrc.CustomInstruction("r_HSLU_SimSwapCutA", [], []))


def sim_swap_cut_b(r1, *, dry_run: bool = False) -> None:
    """Swap beam geometry to after-cut-B state (finished beam).

    Replaces the current mesh with the finished beam
    (both ends mitered).

    Args:
        r1: AbbClient instance
        dry_run: If True, only prints what would happen
    """
    if dry_run or r1 is None:
        print("[SIM-BEAM] SwapCutB")
        return

    r1.send_and_wait(rrc.CustomInstruction("r_HSLU_SimSwapCutB", [], []))


def sim_beam_release(r1, *, dry_run: bool = False) -> None:
    """Detach beam from gripper. Beam stays at current world position.

    The placed beam remains visible in the station, accumulating
    the facade structure over time.

    Args:
        r1: AbbClient instance
        dry_run: If True, only prints what would happen
    """
    if dry_run or r1 is None:
        print("[SIM-BEAM] Release")
        return

    r1.send_and_wait(rrc.CustomInstruction("r_HSLU_SimBeamRelease", [], []))


def sim_beam_reset(r1, *, dry_run: bool = False) -> None:
    """Delete all placed beams and clear SmartComponent state.

    Call at the start of a production run so the facade starts empty.

    Args:
        r1: AbbClient instance
        dry_run: If True, only prints what would happen
    """
    if dry_run or r1 is None:
        print("[SIM-BEAM] Reset")
        return

    r1.send_and_wait(rrc.CustomInstruction("r_HSLU_SimBeamReset", [], []))
