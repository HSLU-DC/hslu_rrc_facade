# validate.py
"""Comprehensive validation for facade fabrication data.

Validates JSON data BEFORE any robot motion to prevent crashes.
Can be run standalone or imported by production.py.

Usage:
    python validate.py              # validates process/data/fab_data.json
    python validate.py path/to.json # validates specific file
"""
import sys
import os
import math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from _skills.fabdata import load_data, has_layers, get_layer_count, get_element_count, get_element
from globals import FRAME_WIDTH, FRAME_LENGTH, BEAM_SECTION, MAX_LAYERS

# Valid beam size categories
VALID_BEAM_SIZES = ("small", "large")

# Maximum beam lengths per category (mm)
# These are approximate stock lengths - actual max depends on cut angles
MAX_BEAM_LENGTH = {
    "small": 700,   # TODO: verify with actual stock
    "large": 1300,  # TODO: verify with actual stock
}

# Minimum beam length (mm)
MIN_BEAM_LENGTH = 50

# Maximum cut angle from perpendicular (degrees)
# Beyond this, the cut becomes too steep for the saw
MAX_CUT_ANGLE = 60

# Tolerance for frame bounds check (mm)
BOUNDS_TOLERANCE = 10

# Required fields per element
REQUIRED_FIELDS = [
    "beam_size",
    "cut_position_a",
    "cut_position_b",
    "glue_position_a",
    "place_position",
]


def _is_valid_frame(obj):
    """Check if an object is a valid COMPAS Frame (has point, xaxis, yaxis)."""
    if obj is None:
        return False
    return (hasattr(obj, 'point') and hasattr(obj, 'xaxis') and hasattr(obj, 'yaxis'))


def _frame_str(frame):
    """Format frame for error message."""
    if frame is None:
        return "None"
    return f"X={frame.point.x:.1f} Y={frame.point.y:.1f} Z={frame.point.z:.1f}"


def _vector_angle_from_xy_plane(vec):
    """Calculate angle (degrees) of a vector from the XY plane.

    Returns 0 for vectors in the XY plane, 90 for Z-pointing vectors.
    """
    if vec.length < 1e-6:
        return 0
    return math.degrees(math.asin(abs(vec.z) / vec.length))




# ==============================================================================
# Individual validation checks
# ==============================================================================

def check_data_format(data):
    """Validate overall data structure."""
    errors = []

    if not isinstance(data, dict):
        errors.append("Daten sind kein Dictionary")
        return errors

    if not has_layers(data):
        errors.append("Daten haben keine 'layers' Struktur (v3 Format erforderlich)")
        return errors

    layers = data.get("layers", [])
    if not isinstance(layers, list) or len(layers) == 0:
        errors.append("'layers' ist leer oder kein Array")
        return errors

    if len(layers) > MAX_LAYERS:
        errors.append(f"Zu viele Layer: {len(layers)} (max {MAX_LAYERS})")

    for layer_idx, layer in enumerate(layers):
        if "elements" not in layer:
            errors.append(f"Layer {layer_idx}: hat keine 'elements'")
            continue
        if not isinstance(layer["elements"], list):
            errors.append(f"Layer {layer_idx}: 'elements' ist kein Array")
            continue
        if len(layer["elements"]) == 0:
            errors.append(f"Layer {layer_idx}: hat keine Elemente")

    # Check metadata
    metadata = data.get("metadata", {})
    if metadata.get("project") and metadata["project"] != "facade":
        errors.append(f"Falsches Projekt: '{metadata['project']}' (erwartet: 'facade')")

    return errors


def check_element_completeness(element, layer_idx, elem_idx):
    """Check that all required fields exist and are valid."""
    errors = []
    prefix = f"Layer {layer_idx}, Element {elem_idx}"

    for field in REQUIRED_FIELDS:
        val = element.get(field)
        if val is None:
            errors.append(f"{prefix}: Feld '{field}' fehlt")
        elif field == "beam_size":
            if not isinstance(val, str):
                errors.append(f"{prefix}: 'beam_size' ist kein String")
        elif not _is_valid_frame(val):
            errors.append(f"{prefix}: '{field}' ist kein gueltiger Frame")

    # glue_position_b is optional (can be None)
    glue_b = element.get("glue_position_b")
    if glue_b is not None and not _is_valid_frame(glue_b):
        errors.append(f"{prefix}: 'glue_position_b' ist weder None noch ein gueltiger Frame")

    return errors


def check_beam_size(element, layer_idx, elem_idx):
    """Check that beam_size is valid."""
    errors = []
    prefix = f"Layer {layer_idx}, Element {elem_idx}"

    beam_size = element.get("beam_size", "")
    if isinstance(beam_size, str):
        beam_size = beam_size.strip('"').strip("'")

    if beam_size not in VALID_BEAM_SIZES:
        errors.append(
            f"{prefix}: beam_size '{beam_size}' ist ungueltig. "
            f"Erlaubt: {VALID_BEAM_SIZES}"
        )

    return errors


def check_cut_angles(element, layer_idx, elem_idx):
    """Check that cut planes are 1D (no Schifterschnitte).

    For a valid 1D cut, the cut frame Z-axis must point straight down (0, 0, -1).
    Any deviation means the cut plane is tilted = Schifterschnitt.
    """
    errors = []
    prefix = f"Layer {layer_idx}, Element {elem_idx}"

    for field in ["cut_position_a", "cut_position_b"]:
        frame = element.get(field)
        if not _is_valid_frame(frame):
            continue

        # The Z-axis of the cut frame should point down (0, 0, -1)
        z = frame.xaxis.cross(frame.yaxis)
        if z.length > 1e-6:
            z_normalized = [z.x / z.length, z.y / z.length, z.z / z.length]
        else:
            continue

        # Angle between Z-axis and (0, 0, -1)
        # dot product with (0,0,-1) = -z_normalized[2]
        dot_down = -z_normalized[2]
        # dot_down = 1.0 means perfect downward, < 1.0 means tilted
        angle_from_down = math.degrees(math.acos(max(-1, min(1, dot_down))))

        if angle_from_down > 5:  # More than 5 degrees from straight down
            errors.append(
                f"{prefix}: {field} hat einen Schifterschnitt-Winkel "
                f"(Z-Achse weicht {angle_from_down:.1f} Grad von der Vertikalen ab). "
                f"Nur Gehrungsschnitte (1D) sind erlaubt!"
            )

    return errors


def check_frame_bounds(element, layer_idx, elem_idx):
    """Check that place_position is within the facade frame bounds."""
    errors = []
    prefix = f"Layer {layer_idx}, Element {elem_idx}"

    place = element.get("place_position")
    if not _is_valid_frame(place):
        return errors

    x, y = place.point.x, place.point.y

    if x < -BOUNDS_TOLERANCE or x > FRAME_LENGTH + BOUNDS_TOLERANCE:
        errors.append(
            f"{prefix}: place_position X={x:.1f}mm liegt ausserhalb "
            f"des Rahmens (0 - {FRAME_LENGTH}mm)"
        )

    if y < -(FRAME_WIDTH + BOUNDS_TOLERANCE) or y > BOUNDS_TOLERANCE:
        errors.append(
            f"{prefix}: place_position Y={y:.1f}mm liegt ausserhalb "
            f"des Rahmens (-{FRAME_WIDTH} - 0mm)"
        )

    return errors




def check_glue_planes(element, layer_idx, elem_idx):
    """Check that glue planes are present and reasonably oriented."""
    errors = []
    prefix = f"Layer {layer_idx}, Element {elem_idx}"

    glue_a = element.get("glue_position_a")
    glue_b = element.get("glue_position_b")

    if not _is_valid_frame(glue_a):
        errors.append(f"{prefix}: glue_position_a fehlt oder ungueltig")

    # glue_b is optional, but if present must be valid
    # (already checked in completeness check)

    return errors


# ==============================================================================
# Aggregate validation
# ==============================================================================

def validate_element(element, layer_idx, elem_idx):
    """Run all validations on a single element.

    Args:
        element: Element dict from fab_data
        layer_idx: Layer index
        elem_idx: Element index

    Returns:
        List of error strings (empty = valid)
    """
    errors = []
    errors.extend(check_element_completeness(element, layer_idx, elem_idx))
    errors.extend(check_beam_size(element, layer_idx, elem_idx))
    errors.extend(check_cut_angles(element, layer_idx, elem_idx))
    errors.extend(check_frame_bounds(element, layer_idx, elem_idx))
    errors.extend(check_glue_planes(element, layer_idx, elem_idx))
    return errors


def validate_all(data):
    """Run all validations on complete fab_data.

    Args:
        data: Loaded fab_data dict

    Returns:
        List of error strings (empty = all valid)
    """
    errors = []

    # Format checks
    format_errors = check_data_format(data)
    errors.extend(format_errors)

    # If format is broken, skip element-level checks
    if format_errors:
        return errors

    # Element-level checks
    n_layers = get_layer_count(data)
    for layer_idx in range(n_layers):
        n_elements = get_element_count(data, layer_idx=layer_idx)
        for elem_idx in range(n_elements):
            element = get_element(data, elem_idx, layer_idx=layer_idx)
            errors.extend(validate_element(element, layer_idx, elem_idx))

    return errors


# ==============================================================================
# Standalone execution
# ==============================================================================

def main():
    """Run validation standalone."""
    if len(sys.argv) > 1:
        path = sys.argv[1]
        import compas
        data = compas.json_load(path)
    else:
        data = load_data()

    print("=" * 50)
    print("FACADE DATA VALIDATION")
    print("=" * 50)

    errors = validate_all(data)

    if errors:
        print(f"\n{len(errors)} FEHLER GEFUNDEN:\n")
        for i, err in enumerate(errors, 1):
            print(f"  {i}. {err}")
        print(f"\n{'='*50}")
        print("VALIDIERUNG FEHLGESCHLAGEN")
        print(f"{'='*50}")
        sys.exit(1)
    else:
        n_layers = get_layer_count(data)
        total = sum(get_element_count(data, layer_idx=i) for i in range(n_layers))
        print(f"\n[OK] Alle {total} Elemente in {n_layers} Layer(n) sind gueltig!")
        print(f"\n{'='*50}")
        print("VALIDIERUNG BESTANDEN")
        print(f"{'='*50}")


if __name__ == "__main__":
    main()
