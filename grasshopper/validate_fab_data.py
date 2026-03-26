# Grasshopper Component: Validate Facade Data
# ghenv.Component.Message = 'Validate Facade'
#
# ==============================================================================
# INPUTS
# ==============================================================================
#
#   fab_data (DataTree):    Same tree as export component ({layer;element})
#   frame_length (float):   Frame length in mm (default: 2500)
#   frame_width (float):    Frame width in mm (default: 600)
#
# ==============================================================================
# OUTPUTS
# ==============================================================================
#
#   valid (DataTree):       True/False per element
#   messages (DataTree):    Validation messages per element
#   colors (DataTree):      Display colors per element (green=OK, red=error, orange=warning)
#
# ==============================================================================
# FAB_DATA INDEX MAPPING (same as export_fab_data.py)
# ==============================================================================
#
#   0 = place_position    << Plane
#   1 = cut_position_a    << Plane
#   2 = cut_position_b    << Plane
#   3 = glue_position_a   << Plane
#   4 = glue_position_b   << Plane (can be null)
#   5 = beam_size         << String ("small" or "large")

import math
import System.Drawing as sd

from Grasshopper import DataTree
from Grasshopper.Kernel.Data import GH_Path
import Rhino


# ==============================================================================
# Configuration
# ==============================================================================

# Default frame dimensions (can be overridden by inputs)
DEFAULT_FRAME_LENGTH = 2500  # mm (X)
DEFAULT_FRAME_WIDTH = 600    # mm (Y)

# Beam stock lengths per category
MAX_BEAM_LENGTH = {
    "small": 700,
    "large": 1300,
}
MIN_BEAM_LENGTH = 50

# Max layers
MAX_LAYERS = 2

# Bounds tolerance
BOUNDS_TOL = 10  # mm

# Valid beam sizes
VALID_BEAM_SIZES = ("small", "large")

# Colors
COLOR_OK = sd.Color.FromArgb(0, 180, 0)        # Green
COLOR_WARN = sd.Color.FromArgb(255, 165, 0)     # Orange
COLOR_ERROR = sd.Color.FromArgb(220, 0, 0)      # Red


# ==============================================================================
# Helpers
# ==============================================================================

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


def plane_point(plane):
    """Extract point from Rhino Plane."""
    if plane is None or not isinstance(plane, Rhino.Geometry.Plane):
        return None
    return plane.Origin


def distance_3d(p1, p2):
    """3D distance between two Rhino Point3d."""
    dx = p1.X - p2.X
    dy = p1.Y - p2.Y
    dz = p1.Z - p2.Z
    return math.sqrt(dx*dx + dy*dy + dz*dz)


def cut_plane_z_angle(plane):
    """Calculate angle of cut plane normal from XY plane (degrees).

    For a 1D miter cut, the Z component of the normal should be ~0.
    Returns angle in degrees (0 = perfect, 90 = vertical).
    """
    if plane is None or not isinstance(plane, Rhino.Geometry.Plane):
        return 0

    # Compute Z-axis as X cross Y
    zx = plane.XAxis.Y * plane.YAxis.Z - plane.XAxis.Z * plane.YAxis.Y
    zy = plane.XAxis.Z * plane.YAxis.X - plane.XAxis.X * plane.YAxis.Z
    zz = plane.XAxis.X * plane.YAxis.Y - plane.XAxis.Y * plane.YAxis.X

    length = math.sqrt(zx*zx + zy*zy + zz*zz)
    if length < 1e-6:
        return 0

    return math.degrees(math.asin(abs(zz) / length))


# ==============================================================================
# Validation per element
# ==============================================================================

def validate_element(branch, layer_idx, elem_idx, f_length, f_width):
    """Validate a single element.

    Returns:
        (is_valid, messages, color)
    """
    msgs = []
    has_error = False
    has_warning = False

    prefix = "L{} E{}".format(layer_idx, elem_idx)

    # --- Check: place_position exists and is within bounds ---
    place_plane = branch[0] if len(branch) > 0 else None

    if place_plane is None or not isinstance(place_plane, Rhino.Geometry.Plane):
        msgs.append("{}: place_position fehlt!".format(prefix))
        has_error = True
    else:
        pt = place_plane.Origin
        if pt.X < -BOUNDS_TOL or pt.X > f_length + BOUNDS_TOL:
            msgs.append("{}: place X={:.0f} ausserhalb (0-{})".format(prefix, pt.X, f_length))
            has_error = True
        if pt.Y < -BOUNDS_TOL or pt.Y > f_width + BOUNDS_TOL:
            msgs.append("{}: place Y={:.0f} ausserhalb (0-{})".format(prefix, pt.Y, f_width))
            has_error = True

    # --- Check: cut positions exist ---
    cut_a = branch[1] if len(branch) > 1 else None
    cut_b = branch[2] if len(branch) > 2 else None

    if cut_a is None or not isinstance(cut_a, Rhino.Geometry.Plane):
        msgs.append("{}: cut_position_a fehlt!".format(prefix))
        has_error = True

    if cut_b is None or not isinstance(cut_b, Rhino.Geometry.Plane):
        msgs.append("{}: cut_position_b fehlt!".format(prefix))
        has_error = True

    # --- Check: cut angles (1D only, no Schifterschnitte) ---
    for label, cut_plane in [("cut_a", cut_a), ("cut_b", cut_b)]:
        if cut_plane is not None and isinstance(cut_plane, Rhino.Geometry.Plane):
            angle = cut_plane_z_angle(cut_plane)
            if angle > 5:
                msgs.append("{}: {} hat Schifterschnitt ({:.1f} Grad Z-Anteil)".format(prefix, label, angle))
                has_error = True
            elif angle > 2:
                msgs.append("{}: {} leicht schief ({:.1f} Grad Z-Anteil)".format(prefix, label, angle))
                has_warning = True

    # --- Check: glue_position_a exists ---
    glue_a = branch[3] if len(branch) > 3 else None
    if glue_a is None or not isinstance(glue_a, Rhino.Geometry.Plane):
        msgs.append("{}: glue_position_a fehlt!".format(prefix))
        has_error = True

    # glue_b is optional (index 4)

    # --- Check: beam_size ---
    beam_size = str(branch[5]).strip('"').strip("'") if len(branch) > 5 and branch[5] is not None else ""
    if beam_size not in VALID_BEAM_SIZES:
        msgs.append("{}: beam_size '{}' ungueltig (nur 'small'/'large')".format(prefix, beam_size))
        has_error = True

    # --- Check: element length vs beam_size ---
    if cut_a is not None and cut_b is not None:
        if isinstance(cut_a, Rhino.Geometry.Plane) and isinstance(cut_b, Rhino.Geometry.Plane):
            dist = distance_3d(cut_a.Origin, cut_b.Origin)

            if beam_size in MAX_BEAM_LENGTH:
                max_len = MAX_BEAM_LENGTH[beam_size]
                if dist > max_len:
                    msgs.append("{}: Laenge {:.0f}mm > max {:.0f}mm fuer '{}'".format(prefix, dist, max_len, beam_size))
                    has_error = True
                elif dist > max_len * 0.9:
                    msgs.append("{}: Laenge {:.0f}mm nahe am Limit ({:.0f}mm)".format(prefix, dist, max_len))
                    has_warning = True

            if dist < MIN_BEAM_LENGTH:
                msgs.append("{}: Laenge nur {:.0f}mm (min {:.0f}mm)".format(prefix, dist, MIN_BEAM_LENGTH))
                has_error = True

    # --- Result ---
    if has_error:
        return False, msgs, COLOR_ERROR
    elif has_warning:
        return True, msgs, COLOR_WARN
    else:
        msgs.append("{}: OK".format(prefix))
        return True, msgs, COLOR_OK


# ==============================================================================
# Main Validation
# ==============================================================================

# Use input values or defaults
f_length = frame_length if 'frame_length' in dir() and frame_length else DEFAULT_FRAME_LENGTH
f_width = frame_width if 'frame_width' in dir() and frame_width else DEFAULT_FRAME_WIDTH

# Output trees
valid = DataTree[object]()
messages = DataTree[object]()
colors = DataTree[object]()

print("=" * 50)
print("Facade Validation")
print("Frame: {}mm x {}mm".format(f_length, f_width))
print("=" * 50)

if fab_data is not None:
    n_layers, elements_per_layer = analyze_tree(fab_data)

    # Layer count check
    if n_layers > MAX_LAYERS:
        print("[FEHLER] {} Layer erkannt - max {} erlaubt!".format(n_layers, MAX_LAYERS))

    total_ok = 0
    total_warn = 0
    total_err = 0

    for layer_idx in range(n_layers):
        n_elements = elements_per_layer.get(layer_idx, 0)

        for elem_idx in range(n_elements):
            branch = get_branch(fab_data, GH_Path(layer_idx, elem_idx))
            path = GH_Path(layer_idx, elem_idx)

            is_valid, msgs, color = validate_element(branch, layer_idx, elem_idx, f_length, f_width)

            valid.Add(is_valid, path)
            colors.Add(color, path)
            for msg in msgs:
                messages.Add(msg, path)

            if not is_valid:
                total_err += 1
            elif color == COLOR_WARN:
                total_warn += 1
            else:
                total_ok += 1

    # Summary
    total = total_ok + total_warn + total_err
    print("\n{} Elemente geprueft:".format(total))
    print("  {} OK".format(total_ok))
    if total_warn > 0:
        print("  {} Warnungen".format(total_warn))
    if total_err > 0:
        print("  {} FEHLER".format(total_err))

    if total_err > 0:
        print("\n!!! NICHT EXPORTIEREN - Fehler zuerst beheben !!!")
    elif total_warn > 0:
        print("\nExport moeglich, aber Warnungen pruefen.")
    else:
        print("\nAlles OK - bereit zum Export.")

else:
    print("Kein fab_data input verbunden.")
