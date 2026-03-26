# HSLU RRC Facade - Development Guide

## Project Overview
Robotic facade element production for 9 student groups. ABB Gofa CRB 15000 + Güdel Track.
Pipeline: Grasshopper (Design) → JSON Export → Python + compas_rrc → OmniCore Controller.

Based on the Swissbau26 project (`C:\Users\jurij\Documents\GitHub\hslu_rrc_Swissbau26`), simplified for student use.

## Architecture

```
grasshopper/
  export_fab_data.py     # GHPython: Rhino Planes → COMPAS Frames → JSON

process/
  production.py          # Main loop: Pick → Cut → Glue → Place
  validate.py            # Pre-flight validation (run before any robot motion)
  globals.py             # Speeds, tools, workobjects, frame dimensions
  joint_positions.py     # Taught joint targets per station

  _skills/               # Low-level robot capabilities (DO NOT MODIFY for students)
    custom_motion.py     # Güdel track coordinated motion (MoveToJoints, MoveToRobtarget)
    fabdata.py           # JSON data loading (compas.json_load), v3 layer structure
    gripper.py           # Open/close via RAPID custom instructions
    GlueLine/            # All-in-one glue line execution in RAPID (avoids Python latency)
    GluePLC/             # PLC safety handshake for glue system
    SoftAct/             # Compliant servo for soft gripping/pressing
    WoodStorage/         # Inventory management (small/large categories, round-robin pick)

  stations/              # Station implementations (DO NOT MODIFY for students)
    a_pick_station.py    # CSS grip from dynamic storage
    b_cut_station.py     # Dual 1D miter cuts (no Schifterschnitte)
    d_glue_station.py    # POS/NEG auto-dispatch, predefined glue path
    e_place_station.py   # Dynamic track offset, approach computed from place_position

  data/
    fab_data.json        # Student export (from GH)
    wood_storage.json    # Inventory state (persisted between runs)
```

## Key Differences from Swissbau26
- No label station (c_lable_station removed)
- No Layer 2 / diagonal code (all diagonal logic removed from every station)
- Only 2 beam categories: small, large (no medium)
- 25x25mm beams (not 40x40mm) → stack_offset_z = 25, grip load = 0.3kg
- Place station computes approach frames from place_position only (students don't provide pre-app, app, rot frames)
- Glue: student provides plane, robot drives predefined path pattern
- JSON has 6 fields per element (not 13)

## Data Format (v3 Facade)
```json
{
  "layers": [{
    "id": 0,
    "elements": [{
      "id": 0,
      "beam_size": "small|large",
      "place_position": Frame,
      "cut_position_a": Frame,
      "cut_position_b": Frame,
      "glue_position_a": Frame,
      "glue_position_b": Frame | null
    }]
  }],
  "metadata": { "version": "3.0", "project": "facade" }
}
```

## compas_rrc Connection Pattern
```python
import compas_rrc as rrc
ros = rrc.RosClient()
ros.run()
r1 = rrc.AbbClient(ros, "/rob1")
r1.send(rrc.SetTool('t_HSLU_GripperZimmer'))
# ... robot commands ...
ros.close()
ros.terminate()
```

## Custom RAPID Instructions
| Instruction | Purpose |
|---|---|
| `r_Gudel_HSLU_MoveToJoints` | Coordinated 6-axis + track motion |
| `r_Gudel_HSLU_MoveTo` | Cartesian coordinated motion |
| `r_HSLU_GripperOpen/Close` | Pneumatic gripper |
| `r_HSLU_SawOn/Off` | Cutting station |
| `r_HSLU_GlueOn/Off` | Glue system PLC handshake |
| `r_RRC_CI_CSS` | Compliant servo (Define/On/Off) |
| `r_RRC_CI_GripLoad` | Workpiece load definition |
| `r_RRC_CI_GlueLine` | All-in-one glue line in RAPID |
| `r_RRC_CI_SoftAct/Deact` | Soft servo axis |

## Workobjects
- `ob_HSLU_Pick_small` / `ob_HSLU_Pick_large` — Pick station (defined in wood_storage.json)
- `ob_HSLU_Cut` — Cut station
- `ob_HSLU_Glue` — Glue station
- `ob_HSLU_Place` — Place station (OFFSET_TRACK = 593mm)

## Development Commands
```bash
# Docker
cd docker && docker compose up -d        # Start ROS + ABB driver
cd docker && docker compose down          # Stop

# Test connection
cd process && python scripts/test_connection.py

# Validate data
cd process && python validate.py

# Dry run (no robot motion)
# Set dry_run=True in production.py main() call

# Production
cd process && python production.py
```

## Validation
Two-stage validation prevents crashes:
1. **GH Export** (`export_fab_data.py`): warns about missing fields, bounds violations
2. **Python** (`validate.py`): hard stop before robot motion — checks format, bounds, cut angles, beam lengths, orientations

## Student Constraints
- Frame: 600mm (Y) × 2500mm (X)
- Max 2 layers
- Only 1D miter cuts (Gehrungsschnitte)
- beam_size: "small" or "large" only
- 1-2 glue planes per element
- Students must NOT modify `_skills/` or `stations/`

## Known TODOs
- Joint positions need teaching at the machine (search for `TODO: verify`)
- Wood storage base_frame Z values need verification for 25mm cross-section
- MAX_BEAM_LENGTH values in validate.py need verification with actual stock
- Place station dynamic offset may need recalibration for facade frame
- GH template (.gh file) needs to be created in Grasshopper
