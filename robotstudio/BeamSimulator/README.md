# BeamSimulator SmartComponent

RobotStudio 2025 SmartComponent for dynamic beam geometry visualization during facade production simulation.

## Overview

Loads STL geometry files on-demand and manages the beam lifecycle:
1. **Activate** - Load raw beam STL, attach to gripper flange
2. **SwapCutA** - Replace with after-cut-A geometry
3. **SwapCutB** - Replace with finished beam geometry
4. **Release** - Detach from gripper (beam stays at place position)
5. **Reset** - Delete all placed beams

Placed beams accumulate, visually building up the facade.

## Build

Requires RobotStudio 2025 SDK (ABB.Robotics.RobotStudio assemblies).

```
dotnet build BeamSimulator.csproj
```

If the SDK path differs, adjust the HintPath values in the .csproj file.

## Installation in RobotStudio

1. Build the project to get `BeamSimulator.dll`
2. Copy `BeamSimulator.dll` and `BeamSimulator.xml` to a folder
3. In RobotStudio: Home > Import Library > navigate to the XML file
4. Drag the SmartComponent into the station
5. Set `GeometryFolder` to the path containing STL files
6. Wire signals via Station Logic

## Signal Wiring (Station Logic)

| Controller Signal | Direction | SmartComponent Signal |
|---|---|---|
| `go_SC_ElementID` (GO) | --> | `ElementID` |
| `go_SC_LayerID` (GO) | --> | `LayerID` |
| `do_SC_Activate` (DO) | --> | `Activate` |
| `do_SC_SwapCutA` (DO) | --> | `SwapCutA` |
| `do_SC_SwapCutB` (DO) | --> | `SwapCutB` |
| `do_SC_Release` (DO) | --> | `Release` |
| `do_SC_Reset` (DO) | --> | `Reset` |
| `di_SC_Ready` (DI) | <-- | `Ready` |

## Properties

| Property | Default | Description |
|---|---|---|
| `GeometryFolder` | (empty) | Path to STL + metadata folder |
| `MetadataFile` | `sim_metadata.json` | Metadata filename |
| `MechanismName` | (empty) | Robot mechanism (empty = first found) |
| `ToolName` | `t_HSLU_GripperZimmer` | Tool for TCP offset |
