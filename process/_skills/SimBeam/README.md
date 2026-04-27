# SimBeam Skill

Beam geometry visualization for RobotStudio simulation via SmartComponent.

## Overview

Controls the `BeamSimulator` SmartComponent in RobotStudio to load, swap and
release beam geometry during the production simulation. Only active on the
virtual controller (`NOT RobOS()`).

## Python API

```python
from _skills.SimBeam import (
    sim_beam_activate,
    sim_swap_cut_a,
    sim_swap_cut_b,
    sim_beam_release,
    sim_beam_reset,
)

# Start of production: clear previously placed beams
sim_beam_reset(r1, dry_run=False)

# Before pick: show raw beam, attach to gripper
sim_beam_activate(r1, layer=0, element=3, dry_run=False)

# After cut A: swap to one-end-mitered geometry
sim_swap_cut_a(r1, dry_run=False)

# After cut B: swap to finished beam geometry
sim_swap_cut_b(r1, dry_run=False)

# After place: detach beam (stays visible at place position)
sim_beam_release(r1, dry_run=False)
```

## RAPID Custom Instructions

| Instruction | Parameters | Description |
|---|---|---|
| `r_HSLU_SimBeamActivate` | V1=Layer, V2=Element | Set IDs + pulse Activate |
| `r_HSLU_SimSwapCutA` | (none) | Pulse SwapCutA |
| `r_HSLU_SimSwapCutB` | (none) | Pulse SwapCutB |
| `r_HSLU_SimBeamRelease` | (none) | Pulse Release |
| `r_HSLU_SimBeamReset` | (none) | Pulse Reset (clear placed beams) |

## EIO Signals (Virtual Controller)

| Signal | Type | Description |
|---|---|---|
| `go_SC_ElementID` | GO (8 bit) | Element index (0-255) |
| `go_SC_LayerID` | GO (2 bit) | Layer index (0-3) |
| `do_SC_Activate` | DO | Load raw beam + attach |
| `do_SC_SwapCutA` | DO | Swap to cut-A geometry |
| `do_SC_SwapCutB` | DO | Swap to finished geometry |
| `do_SC_Release` | DO | Detach (beam stays) |
| `do_SC_Reset` | DO | Delete all placed beams |

The SmartComponent signal names match the RAPID names 1:1, so I/O connections
in RobotStudio can be wired with identical names on both sides.

## Geometry Files

Expected in `process/data/geometry/`:
```
L{layer}_E{element}_raw.stl    # Raw stock beam
L{layer}_E{element}_cutA.stl   # After first miter cut
L{layer}_E{element}_cutB.stl   # After second cut (finished)
```

All STLs are in grip-center-relative coordinates (origin = centerline midpoint),
so the SmartComponent can attach them at the gripper TCP without further
transforms. Produced by the `ExportFacade` Grasshopper component
(`design/gh_python/ExportFacade.py`) alongside the fab_data JSON.
