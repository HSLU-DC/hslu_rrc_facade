# Grasshopper Component: Export Simulation Geometry
# ghenv.Component.Message = 'Export STL v1'
#
# ==============================================================================
# INPUTS
# ==============================================================================
#
#   update (bool):        Trigger export
#   folder_path (str):    Output folder (e.g. process/data/geometry)
#   fab_data (DataTree):  Same tree as fab_data export: {Layer;Element}
#
# ==============================================================================
# FAB_DATA INDEX MAPPING (Facade project)
# ==============================================================================
#
#   0  = Brep              << Finished beam geometry (= cutB)
#   1  = Centerline        << Line (for raw beam + grip center)
#   2  = Cut Plane A       << Plane (Z outward, for cutA split)
#   3  = Cut Plane B       << Plane (Z outward)
#   4  = Glue Plane A      (skip)
#   5  = Glue Plane B      (skip, optional)
#
# Adjust indices below if your GH template uses a different tree layout.
#
# ==============================================================================
# OUTPUT
# ==============================================================================
#   output (str): Export log
#
# ==============================================================================
# FILES PRODUCED
# ==============================================================================
#   L{layer}_E{element}_raw.stl    Raw stock beam (box)
#   L{layer}_E{element}_cutA.stl   After first miter cut
#   L{layer}_E{element}_cutB.stl   Finished beam (both cuts)
#
# All STLs in grip-center-relative coordinates (origin = centerline midpoint).
# The SmartComponent derives filenames from the element and layer signals
# via the L{layer}_E{element}_{stage}.stl convention, so no metadata file
# is needed.

import os
import struct
import System

import Rhino
import Rhino.Geometry as rg

from Grasshopper import DataTree
from Grasshopper.Kernel.Data import GH_Path


# ==============================================================================
# Configuration — adjust if your tree layout differs
# ==============================================================================

BREP_INDEX = 0           # Finished Brep
CENTERLINE_INDEX = 1     # Centerline (Line)
CUT_PLANE_A_INDEX = 2    # Cut Plane A
CUT_PLANE_B_INDEX = 3    # Cut Plane B

BEAM_SECTION = 25.0      # mm cross-section (must match globals.py)
STOCK_LENGTHS = {         # mm stock lengths per category (must match validate.py)
    "small": 700.0,
    "large": 1300.0,
}
SIZE_THRESHOLD = 700.0    # Centerline <= threshold -> "small", else "large"

MESH_MIN_EDGE = 0.5
MESH_MAX_EDGE = 50.0
SPLIT_TOLERANCE = 0.01
SPLIT_PLANE_SIZE = 5000.0  # Half-size of cutting plane surface (mm)


# ==============================================================================
# Geometry dereferencing (Rhino 8 CPython: tree items can be Guids)
# ==============================================================================

def deref(item):
    """Dereference a DataTree item to its Rhino geometry.

    In Rhino 8 CPython, geometry in DataTrees comes as System.Guid
    references. GH-created geometry lives in ghdoc, Rhino-referenced
    geometry lives in the Rhino document. This tries both.
    """
    # GH_Goo wrapper (e.g. GH_Brep, GH_Line, GH_Plane)
    if hasattr(item, 'Value'):
        return deref(item.Value)

    # Guid reference -> try GH doc first, then Rhino doc
    if isinstance(item, System.Guid):
        import scriptcontext as sc
        old_doc = sc.doc

        # Strategy 1: GH document (geometry created by GH components)
        try:
            sc.doc = ghdoc
            rhino_obj = sc.doc.Objects.FindId(item)
            if rhino_obj is not None:
                sc.doc = old_doc
                return rhino_obj.Geometry
        except:
            pass

        # Strategy 2: Active Rhino document (referenced geometry)
        try:
            sc.doc = Rhino.RhinoDoc.ActiveDoc
            rhino_obj = sc.doc.Objects.FindId(item)
            if rhino_obj is not None:
                sc.doc = old_doc
                return rhino_obj.Geometry
        except:
            pass

        sc.doc = old_doc
        return None

    return item


def to_brep(item):
    """Extract Brep from a tree item."""
    geom = deref(item)
    if isinstance(geom, rg.Brep):
        return geom
    if isinstance(geom, rg.Extrusion):
        return geom.ToBrep()
    return None


def to_line(item):
    """Extract Line from a tree item (handles LineCurve, NurbsCurve, etc.)."""
    geom = deref(item)
    if isinstance(geom, rg.Line):
        return geom
    if isinstance(geom, rg.LineCurve):
        return geom.Line
    if isinstance(geom, rg.Curve):
        return rg.Line(geom.PointAtStart, geom.PointAtEnd)
    return None


def to_plane(item):
    """Extract Plane from a tree item."""
    geom = deref(item)
    if isinstance(geom, rg.Plane):
        return geom
    # PlaneSurface referenced from Rhino
    if isinstance(geom, rg.PlaneSurface):
        ok, frame = geom.TryGetPlane()
        if ok:
            return frame
    # Surface -> try to get plane
    if isinstance(geom, rg.Surface):
        ok, frame = geom.TryGetPlane()
        if ok:
            return frame
    return None


# ==============================================================================
# Tree helpers (same pattern as export_fab_data_v3.py)
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
    """Analyze tree structure, return (n_layers, elements_per_layer dict)."""
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


# ==============================================================================
# Geometry helpers
# ==============================================================================

def get_beam_size(centerline):
    """Determine beam size category from centerline length."""
    length = centerline.Length
    return "small" if length <= SIZE_THRESHOLD else "large"


def get_stock_length(size):
    """Get stock length for beam size category."""
    return STOCK_LENGTHS.get(size, STOCK_LENGTHS["small"])


def make_grip_frame(centerline):
    """Compute grip frame: origin at centerline midpoint.

    Convention (matches gripper TCP):
      X = along beam (centerline direction)
      Z = -world Z (gripper approaches from above)
      Y = Z x X (derived, right-handed)

    Returns (grip_center, grip_plane, transform_to_local).
    """
    grip_center = centerline.PointAt(0.5)

    grip_x = centerline.Direction
    grip_x.Unitize()
    grip_z = -rg.Vector3d.ZAxis  # Gripper points down
    grip_y = rg.Vector3d.CrossProduct(grip_z, grip_x)
    grip_y.Unitize()

    grip_plane = rg.Plane(grip_center, grip_x, grip_y)
    to_local = rg.Transform.PlaneToPlane(grip_plane, rg.Plane.WorldXY)

    return grip_center, grip_plane, to_local


def make_raw_brep(centerline, grip_center, grip_plane, size):
    """Generate raw stock beam as Box Brep."""
    stock_len = get_stock_length(size)
    half_len = stock_len / 2.0
    half_sec = BEAM_SECTION / 2.0

    box = rg.Box(
        grip_plane,
        rg.Interval(-half_len, half_len),
        rg.Interval(-half_sec, half_sec),
        rg.Interval(-half_sec, half_sec)
    )
    return box.ToBrep()


def split_brep_with_plane(brep, cut_plane, grip_center):
    """Split brep with a cut plane, keep fragment containing grip_center.

    Returns the split Brep, or the original if split fails.
    """
    # Create large planar surface as cutter
    srf = rg.PlaneSurface(
        cut_plane,
        rg.Interval(-SPLIT_PLANE_SIZE, SPLIT_PLANE_SIZE),
        rg.Interval(-SPLIT_PLANE_SIZE, SPLIT_PLANE_SIZE)
    )
    cutter = srf.ToBrep()

    # Try Split
    fragments = brep.Split(cutter, SPLIT_TOLERANCE)
    if fragments and len(fragments) > 0:
        # Keep fragment closest to grip center
        best = None
        best_dist = float('inf')
        for frag in fragments:
            bb = frag.GetBoundingBox(False)
            dist = grip_center.DistanceTo(bb.Center)
            if dist < best_dist:
                best_dist = dist
                best = frag
        if best:
            # Cap the open cut face to make it a closed solid
            capped = best.CapPlanarHoles(SPLIT_TOLERANCE)
            if capped is not None:
                best = capped
            return best, "Split OK ({} fragments)".format(len(fragments))

    # Fallback: Trim
    trimmed = brep.Trim(cut_plane, SPLIT_TOLERANCE)
    if trimmed and len(trimmed) > 0:
        best = None
        best_dist = float('inf')
        for frag in trimmed:
            dist = grip_center.DistanceTo(frag.GetBoundingBox(False).Center)
            if dist < best_dist:
                best_dist = dist
                best = frag
        if best:
            capped = best.CapPlanarHoles(SPLIT_TOLERANCE)
            if capped is not None:
                best = capped
            return best, "Trim fallback OK ({} fragments)".format(len(trimmed))

    # Last resort: return original
    return brep, "WARNING: Split+Trim failed, using original"


def brep_to_mesh(brep):
    """Convert Brep to triangulated Mesh."""
    mp = rg.MeshingParameters.Default
    mp.MinimumEdgeLength = MESH_MIN_EDGE
    mp.MaximumEdgeLength = MESH_MAX_EDGE

    meshes = rg.Mesh.CreateFromBrep(brep, mp)
    if not meshes:
        return None

    combined = rg.Mesh()
    for m in meshes:
        combined.Append(m)
    combined.Faces.ConvertQuadsToTriangles()
    combined.Normals.ComputeNormals()
    combined.Compact()
    return combined


# ==============================================================================
# Binary STL writer
# ==============================================================================

def write_binary_stl(mesh, file_path):
    """Write mesh as binary STL file."""
    with open(file_path, 'wb') as f:
        # 80-byte header
        header = b'HSLU Facade SimBeam' + b'\x00' * (80 - 19)
        f.write(header)

        # Triangle count
        n_faces = mesh.Faces.Count
        f.write(struct.pack('<I', n_faces))

        # Triangles
        for i in range(n_faces):
            face = mesh.Faces[i]

            # Normal
            n = mesh.FaceNormals[i]
            f.write(struct.pack('<fff', n.X, n.Y, n.Z))

            # Vertex A
            v = mesh.Vertices[face.A]
            f.write(struct.pack('<fff', v.X, v.Y, v.Z))

            # Vertex B
            v = mesh.Vertices[face.B]
            f.write(struct.pack('<fff', v.X, v.Y, v.Z))

            # Vertex C
            v = mesh.Vertices[face.C]
            f.write(struct.pack('<fff', v.X, v.Y, v.Z))

            # Attribute byte count
            f.write(struct.pack('<H', 0))


def export_brep_as_stl(brep, to_local, file_path):
    """Mesh brep, transform to local coords, write STL.

    Returns (success, triangle_count, message).
    """
    if brep is None:
        return False, 0, "Null brep"

    mesh = brep_to_mesh(brep)
    if mesh is None:
        return False, 0, "Meshing failed"

    mesh.Transform(to_local)
    write_binary_stl(mesh, file_path)
    return True, mesh.Faces.Count, "OK"


# ==============================================================================
# Main export logic
# ==============================================================================

print("=" * 50)
print("Export Simulation Geometry")
print("=" * 50)

if update:
    # Create output folder
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

    # Analyze tree
    n_layers, elements_per_layer = analyze_tree(fab_data)

    print("Detected {} layers".format(n_layers))
    for layer, count in sorted(elements_per_layer.items()):
        print("  Layer {}: {} elements".format(layer, count))

    if n_layers == 0:
        output = "ERROR: No layers detected in DataTree"
        print(output)
    else:
        total_elements = 0
        total_files = 0
        errors = []

        for layer_idx in range(n_layers):
            n_elements = elements_per_layer.get(layer_idx, 0)
            print("\nLayer {} ({} elements)".format(layer_idx, n_elements))

            for elem_idx in range(n_elements):
                branch = get_branch(fab_data, GH_Path(layer_idx, elem_idx))
                prefix = "L{}_E{}".format(layer_idx, elem_idx)

                if len(branch) < 4:
                    msg = "{}: Branch too short ({} items, need >= 4)".format(prefix, len(branch))
                    errors.append(msg)
                    print("  " + msg)
                    continue

                # Extract data from tree (dereference Guids -> geometry)
                finished_brep = to_brep(branch[BREP_INDEX])
                centerline = to_line(branch[CENTERLINE_INDEX])
                cut_plane_a = to_plane(branch[CUT_PLANE_A_INDEX])

                if finished_brep is None or centerline is None or cut_plane_a is None:
                    msg = "{}: Missing Brep={} Centerline={} CutPlaneA={}".format(
                        prefix,
                        type(branch[BREP_INDEX]).__name__ if finished_brep is None else "OK",
                        type(branch[CENTERLINE_INDEX]).__name__ if centerline is None else "OK",
                        type(branch[CUT_PLANE_A_INDEX]).__name__ if cut_plane_a is None else "OK")
                    errors.append(msg)
                    print("  " + msg)
                    continue

                # Beam size from centerline length
                size = get_beam_size(centerline)

                # Grip frame
                grip_center, grip_plane, to_local = make_grip_frame(centerline)

                # --- RAW beam ---
                raw_brep = make_raw_brep(centerline, grip_center, grip_plane, size)
                raw_path = os.path.join(folder_path, "{}_raw.stl".format(prefix))
                ok, n_tri, msg = export_brep_as_stl(raw_brep, to_local, raw_path)
                print("  {} raw: {} ({} tri)".format(prefix, msg, n_tri))
                if ok:
                    total_files += 1
                else:
                    errors.append("{} raw: {}".format(prefix, msg))

                # --- CutA beam (raw split by Cut Plane A) ---
                if raw_brep is not None:
                    cut_a_brep, split_msg = split_brep_with_plane(raw_brep, cut_plane_a, grip_center)
                else:
                    cut_a_brep = None
                    split_msg = "No raw brep"

                cut_a_path = os.path.join(folder_path, "{}_cutA.stl".format(prefix))
                ok, n_tri, msg = export_brep_as_stl(cut_a_brep, to_local, cut_a_path)
                print("  {} cutA: {} - {} ({} tri)".format(prefix, split_msg, msg, n_tri))
                if ok:
                    total_files += 1
                else:
                    errors.append("{} cutA: {}".format(prefix, msg))

                # --- CutB beam (finished = student's Brep) ---
                cut_b_path = os.path.join(folder_path, "{}_cutB.stl".format(prefix))
                ok, n_tri, msg = export_brep_as_stl(finished_brep, to_local, cut_b_path)
                print("  {} cutB: {} ({} tri)".format(prefix, msg, n_tri))
                if ok:
                    total_files += 1
                else:
                    errors.append("{} cutB: {}".format(prefix, msg))

                total_elements += 1

        # Summary
        print("\n" + "=" * 50)
        print("EXPORT COMPLETE")
        print("=" * 50)
        print("Folder: {}".format(folder_path))
        print("Elements: {}".format(total_elements))
        print("STL files: {}".format(total_files))
        if errors:
            print("\nErrors ({}):\n  {}".format(len(errors), "\n  ".join(errors)))

        output = "OK: {} elements, {} STL files".format(total_elements, total_files)
        if errors:
            output += " ({} errors)".format(len(errors))

else:
    output = "Set update=True"
    print("Waiting for update=True")

print("\nDone")
