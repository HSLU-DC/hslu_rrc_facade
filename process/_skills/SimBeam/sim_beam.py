# sim_beam.py
"""SimBeam: Beam geometry visualization for RobotStudio simulation.

Controls a SmartComponent via RAPID custom instructions that set
digital/group outputs. Only active on virtual controller (NOT RobOS).

RAPID module: HSLU_SimBeam.mod
Signals: go_SC_ElementID, go_SC_LayerID, do_SC_Activate,
         do_SC_SwapCutA, do_SC_SwapCutB, do_SC_Release, do_SC_Reset
"""
from __future__ import annotations

from pathlib import Path

import compas_rrc as rrc


# _skills/SimBeam/sim_beam.py -> process/data/geometry
_DEFAULT_GEOMETRY_FOLDER = (
    Path(__file__).resolve().parents[2] / "data" / "geometry"
)


def _default_geometry_folder() -> str:
    """Return the repo-relative geometry folder with forward slashes.

    Mirrors the convention used by fabdata.py (<process>/data/<...>) so
    students don't need to configure a path in RobotStudio by hand.
    """
    return str(_DEFAULT_GEOMETRY_FOLDER).replace("\\", "/")


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


def sim_beam_reset(
    r1,
    *,
    geometry_folder: str | None = None,
    tool_name: str | None = None,
    dry_run: bool = False,
) -> None:
    """Delete all placed beams, clear SC state, push folder/tool to RAPID.

    Also forwards the STL folder path and the tool name to the
    SmartComponent (via RAPID PERS variables ``sim_geometry_folder``
    and ``sim_tool_name``) so students don't have to set them
    manually on the SmartComponent's property panel in RobotStudio.

    Call at the start of a production run so the facade starts empty
    and the SC picks up the current paths.

    Args:
        r1: AbbClient instance
        geometry_folder: Absolute path to STL folder. Defaults to
            ``<process>/data/geometry`` (derived relative to this file).
        tool_name: RAPID tool data name. Defaults to
            ``globals.TOOL_GRIPPER``.
        dry_run: If True, only prints what would happen.
    """
    if geometry_folder is None:
        geometry_folder = _default_geometry_folder()
    else:
        geometry_folder = geometry_folder.replace("\\", "/")

    if tool_name is None:
        # Local import keeps this helper usable from GH / stand-alone scripts
        # where `globals` is not necessarily on the import path.
        from globals import TOOL_GRIPPER
        tool_name = TOOL_GRIPPER

    if dry_run or r1 is None:
        print(f"[SIM-BEAM] Reset folder='{geometry_folder}' tool='{tool_name}'")
        return

    cmd = rrc.CustomInstruction(
        "r_HSLU_SimBeamReset",
        [geometry_folder, tool_name],  # .S1, .S2
        [],
    )
    r1.send_and_wait(cmd)
