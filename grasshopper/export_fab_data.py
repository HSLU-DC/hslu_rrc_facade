# Grasshopper Component: Export Fab Data for Facade
# ghenv.Component.Message = 'Export Facade JSON'
#
# ==============================================================================
# INPUTS
# ==============================================================================
#
#   update (bool):        Trigger export
#   file_path (str):      Path to output JSON file
#   fab_data (DataTree):  Tree with {layer;element} structure
#
# ==============================================================================
# FAB_DATA INDEX MAPPING (Facade)
# ==============================================================================
#
#   0 = place_position    << Frame (in ob_HSLU_Place coordinates)
#   1 = cut_position_a    << Frame (in ob_HSLU_Cut coordinates)
#   2 = cut_position_b    << Frame (in ob_HSLU_Cut coordinates)
#   3 = glue_position_a   << Frame (in ob_HSLU_Glue coordinates)
#   4 = glue_position_b   << Frame (in ob_HSLU_Glue coordinates, can be null)
#   5 = beam_size         << String ("small" or "large")
#
# ==============================================================================
# OUTPUT
# ==============================================================================
#   output (str): Status message

import os
from datetime import datetime

import compas
from compas.geometry import Point, Frame, Vector

from Grasshopper import DataTree
from Grasshopper.Kernel.Data import GH_Path
import Rhino


# ==============================================================================
# Index mapping: index -> position name (Frames only)
# ==============================================================================
INDEX_MAP = {
    0: "place_position",
    1: "cut_position_a",
    2: "cut_position_b",
    3: "glue_position_a",
    4: "glue_position_b",
}

# Index for string values
BEAM_SIZE_INDEX = 5


# ==============================================================================
# Helper functions
# ==============================================================================

def to_compas_frame(plane):
    """Convert Rhino Plane to COMPAS Frame."""
    if plane is None:
        return None
    if not isinstance(plane, Rhino.Geometry.Plane):
        return None
    return Frame(
        Point(plane.Origin.X, plane.Origin.Y, plane.Origin.Z),
        Vector(plane.XAxis.X, plane.XAxis.Y, plane.XAxis.Z),
        Vector(plane.YAxis.X, plane.YAxis.Y, plane.YAxis.Z)
    )


def get_branch(tree, path):
    """Get branch from DataTree as list."""
    if tree is None:
        return []
    try:
        branch = tree.Branch(path)
        return list(branch) if branch else []
    except:
        return []


def analyze_tree(tree):
    """Analyze tree structure and return (n_layers, elements_per_layer)."""
    if tree is None:
        return 0, {}

    paths = tree.Paths
    if not paths:
        return 0, {}

    elements_per_layer = {}
    for p in paths:
        if p.Length >= 2:
            layer = p[0]
            element = p[1]
            if layer not in elements_per_layer:
                elements_per_layer[layer] = 0
            elements_per_layer[layer] = max(elements_per_layer[layer], element + 1)

    n_layers = max(elements_per_layer.keys()) + 1 if elements_per_layer else 0
    return n_layers, elements_per_layer


def validate_element_basic(element, layer_idx, elem_idx):
    """Basic validation during export. Returns list of warnings."""
    warnings = []
    prefix = "L{} E{}".format(layer_idx, elem_idx)

    # Check required frames
    for field in ["place_position", "cut_position_a", "cut_position_b", "glue_position_a"]:
        if field not in element:
            warnings.append("{}: '{}' fehlt!".format(prefix, field))

    # Check beam_size
    beam_size = element.get("beam_size", "")
    if beam_size not in ("small", "large"):
        warnings.append("{}: beam_size '{}' ungueltig (nur 'small' oder 'large')".format(prefix, beam_size))

    # Check place bounds (600 x 2500)
    place = element.get("place_position")
    if place is not None:
        x, y = place.point.x, place.point.y
        if x < -10 or x > 2510:
            warnings.append("{}: place X={:.0f} ausserhalb Rahmen (0-2500)".format(prefix, x))
        if y < -10 or y > 610:
            warnings.append("{}: place Y={:.0f} ausserhalb Rahmen (0-600)".format(prefix, y))

    return warnings


# ==============================================================================
# Main Export Logic
# ==============================================================================

print("=" * 50)
print("Export Facade Fab Data")
print("=" * 50)

if update:
    # Create output directory if needed
    last_slash = max(file_path.rfind('/'), file_path.rfind('\\'))
    data_dir = file_path[:last_slash] if last_slash != -1 else ""

    if data_dir and not os.path.exists(data_dir):
        os.makedirs(data_dir)

    # Analyze tree
    n_layers, elements_per_layer = analyze_tree(fab_data)

    if n_layers > 2:
        print("[WARNUNG] {} Layer erkannt - maximal 2 erlaubt!".format(n_layers))

    print("Erkannt: {} Layer".format(n_layers))
    for layer, count in sorted(elements_per_layer.items()):
        print("  Layer {}: {} Elemente".format(layer, count))

    # Build layers
    layers = []
    total_elements = 0
    all_warnings = []

    for layer_idx in range(n_layers):
        n_elements = elements_per_layer.get(layer_idx, 0)
        print("\nVerarbeite Layer {} ({} Elemente)...".format(layer_idx, n_elements))

        elements = []

        for elem_idx in range(n_elements):
            branch = get_branch(fab_data, GH_Path(layer_idx, elem_idx))

            element = {"id": elem_idx}

            # Extract beam_size (string)
            if BEAM_SIZE_INDEX < len(branch) and branch[BEAM_SIZE_INDEX] is not None:
                element["beam_size"] = str(branch[BEAM_SIZE_INDEX])

            # Extract position frames
            for idx, pos_name in INDEX_MAP.items():
                if idx < len(branch):
                    frame = to_compas_frame(branch[idx])
                    if frame is not None:
                        element[pos_name] = frame

            # Validate
            warnings = validate_element_basic(element, layer_idx, elem_idx)
            all_warnings.extend(warnings)

            elements.append(element)

        if elements:
            print("  Keys: {}".format(list(elements[0].keys())))

        layers.append({
            "id": layer_idx,
            "elements": elements
        })
        total_elements += len(elements)

    # Build export structure
    export_data = {
        "layers": layers,
        "metadata": {
            "created": datetime.now().isoformat(),
            "layer_count": len(layers),
            "total_element_count": total_elements,
            "version": "3.0",
            "project": "facade",
            "frame_size": [600, 2500],
            "beam_section": 25,
        }
    }

    # Show warnings
    if all_warnings:
        print("\n" + "!" * 50)
        print("{} WARNUNGEN:".format(len(all_warnings)))
        print("!" * 50)
        for w in all_warnings:
            print("  - " + w)
        print("!" * 50)

    # Export JSON
    compas.json_dump(export_data, file_path, pretty=True)

    # Summary
    print("\n" + "=" * 50)
    print("EXPORT ABGESCHLOSSEN")
    print("=" * 50)
    print("Datei: {}".format(file_path))
    print("Layer: {}".format(len(layers)))
    for layer in layers:
        print("  Layer {}: {} Elemente".format(layer["id"], len(layer["elements"])))
    print("Total: {} Elemente".format(total_elements))

    if all_warnings:
        output = "WARNUNG: {} Elemente, {} Warnungen".format(total_elements, len(all_warnings))
    else:
        output = "OK: {} Layer, {} Elemente".format(len(layers), total_elements)

else:
    output = "Set update=True"
    print("Warte auf update=True")

print("\nDone")
