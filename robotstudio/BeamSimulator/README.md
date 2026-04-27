# BeamSimulator SmartComponent

RobotStudio 2025 SmartComponent for dynamic beam geometry visualization
during facade production simulation.

## Overview

Loads STL geometry files on-demand and manages the beam lifecycle:

1. **Activate** – Load raw beam STL, attach to gripper flange
2. **SwapCutA** – Replace with after-cut-A geometry
3. **SwapCutB** – Replace with finished beam geometry
4. **Release** – Detach from gripper (beam stays at place position)
5. **Reset** – Delete all placed beams

Placed beams accumulate, visually building up the facade. Each loaded beam
is coloured in a light wood tone (RGB 180/130/80) so elements stand out
against the station background.

## Build

Requires RobotStudio 2025 SDK (`ABB.Robotics.RobotStudio*` assemblies under
`C:\Program Files (x86)\ABB\RobotStudio 2025\Bin\`) and the .NET SDK 8 for
the SDK-style `.csproj`.

```
dotnet build -c Release BeamSimulator.csproj
```

A post-build MSBuild target invokes `LibraryCompiler.exe` and produces
`BeamSimulator.rslib` next to the project. If the SDK is installed elsewhere,
adjust `HintPath` / `Exec Command` in `BeamSimulator.csproj`.

## Installation in RobotStudio

1. Close RobotStudio (otherwise the existing DLL is locked).
2. Copy `BeamSimulator.rslib` to
   `C:\Program Files (x86)\ABB\RobotStudio 2025\ABB Library\Components\`
   (requires admin rights – run `Copy-Item` from an elevated shell).
3. Restart RobotStudio and open the station.
4. If an old BeamSimulator instance exists, delete it first — property /
   signal layout may have changed between builds.
5. Home → Import Library → `BeamSimulator.rslib`.
6. Drag the SmartComponent into the station.
7. Set the `GeometryFolder` property to the folder holding the STLs (e.g.
   `…\process\data\geometry`).

The `GeometryFolder` is cached per station under
`%APPDATA%\HSLU_BeamSimulator\station_paths.txt`, so subsequent opens of the
same `.rsstn` auto-populate the property.

## Signal Wiring (Station Logic)

SmartComponent signal names are identical to the RAPID side – wire 1:1:

| Controller Signal | Direction | SmartComponent Signal |
|---|---|---|
| `go_SC_ElementID` (GO, 8 bit) | → | `go_SC_ElementID` |
| `go_SC_LayerID` (GO, 2 bit) | → | `go_SC_LayerID` |
| `do_SC_Activate` (DO) | → | `do_SC_Activate` |
| `do_SC_SwapCutA` (DO) | → | `do_SC_SwapCutA` |
| `do_SC_SwapCutB` (DO) | → | `do_SC_SwapCutB` |
| `do_SC_Release` (DO) | → | `do_SC_Release` |
| `do_SC_Reset` (DO) | → | `do_SC_Reset` |

## Properties

| Property | Default | Description |
|---|---|---|
| `GeometryFolder` | (empty) | Folder containing the `L{layer}_E{element}_{raw\|cutA\|cutB}.stl` files. Auto-restored from `%APPDATA%\HSLU_BeamSimulator\station_paths.txt` on station open. |
| `ToolName` | `t_HSLU_GripperZimmer` | Name of the `tooldata` whose `Frame.Matrix` is used as TCP offset when attaching a beam to the flange. STLs are exported in grip-center coordinates; this offset moves the beam from TCP space to flange space. |

## STL File Convention

Filenames are derived from the element/layer group signals:

```
L{layer}_E{element}_raw.stl    # Raw stock beam (box)
L{layer}_E{element}_cutA.stl   # After first miter cut
L{layer}_E{element}_cutB.stl   # Finished beam (both cuts)
```

All meshes are expressed in grip-center-relative coordinates (origin =
centerline midpoint). The Grasshopper component
`design/gh_python/ExportFacade.py` produces them alongside the fab_data
JSON.
