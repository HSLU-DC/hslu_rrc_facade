# production.py
"""Main production orchestrator for the Facade FS26 robot cell.

This script controls the full fabrication workflow for timber facade element
assembly. Each beam goes through 4 stations in sequence:

    1. PICK   — Retrieve beam from wood storage (WoodStorage inventory system)
    2. CUT    — Cut beam to length at circular saw station (2 cuts per beam)
    3. GLUE   — Apply adhesive with Robatech glue system (PLC-controlled)
    4. PLACE  — Place beam onto facade frame (track-compensated approach)

The fabrication data (positions, sizes, orientations) is exported from
Grasshopper as JSON (see design/simulation/ExportFacade.py) and organized
in layers. Each layer contains multiple elements (beams).

Usage:
    1. Start Docker (cd docker && docker-compose -f REAL-docker-compose.yml up -d)
    2. Configure RUN CONFIG below
    3. Run: python production.py

Configuration:
    - Enable/disable individual stations with DO_PICK, DO_CUT, etc.
    - Set LAYER to select which layer from fab_data (0 or 1, MAX_LAYERS=2)
    - Set N_RUNS for how many beams to process per run
    - Set START_I to skip already-placed beams
    - Hardware flags (CSS_ENABLED, SAW_ENABLED, GLUE_VALVE_ENABLED)
      allow testing motion paths without activating tools
    - SIM_BEAMS drives the BeamSimulator SmartComponent in RobotStudio
      (virtual controller only — has no effect on the real cell)

Validation:
    Two-stage validation runs before any robot motion:
    1. validate.py checks fab_data structure, bounds, cut angles
    2. Per-element validation checks each beam in the planned range
    Production aborts if any check fails.
"""

# ==============================
# Imports
# ==============================
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import compas_rrc as rrc

from _skills.fabdata import load_data, has_layers, get_layer_count, get_element_count, get_element
from _skills.SimBeam import sim_beam_reset
from _skills.WoodStorage.wood_storage import WoodStorage, VALID_CATEGORIES
from globals import ROBOT_NAME, TOOL_GRIPPER, MAX_LAYERS
import _skills.custom_motion as cm

from stations import a_pick_station, b_cut_station, d_glue_station, e_place_station

# Try to import validation (non-fatal if missing during development)
try:
    from validate import validate_all, validate_element
    HAS_VALIDATION = True
except ImportError:
    HAS_VALIDATION = False
    print("[WARN] validate.py not found - running WITHOUT validation!")


# ==============================
# RUN CONFIG
# ==============================
# Station toggles — set to False to skip a station (robot still moves between stations)
DO_PICK  = True
DO_CUT   = True
DO_GLUE  = True
DO_PLACE = True

# Hardware toggles — False = dry-run motion without tool activation
CSS_ENABLED = True            # Cartesian Soft Servo for gentle gripping at pick
SAW_ENABLED = True            # Circular saw on/off during cut moves
GLUE_VALVE_ENABLED = True     # Glue valve pulsing during glue moves
SIM_BEAMS = True              # BeamSimulator SmartComponent (virtual controller only)

# Production range
LAYER    = 0      # Which layer from fab_data (0 or 1)
N_RUNS   = 10     # How many beams to process per run
START_I  = 0      # Start index within the layer (skip already placed beams)


def check_wood_storage(data, layer_idx, start_i, n_runs):
    """Pre-flight check: verify wood storage has enough beams for the planned run.

    Interactive prompt: asks the operator whether the storage was refilled.
    If yes, resets all compartment counts to full capacity. Then checks
    whether the required beam sizes (small/large) are available.

    Args:
        data: Loaded fab_data dict (from fabdata.load_data())
        layer_idx: Which layer to check elements from
        start_i: Starting element index
        n_runs: Number of elements planned for this run

    Returns:
        True if enough beams are available, False otherwise (aborts production)
    """
    storage = WoodStorage()

    # Ask operator first
    print("\n" + "=" * 50)
    print("HOLZLAGER CHECK")
    print("=" * 50)

    response = input("\nWurde das Holzlager KOMPLETT aufgefuellt? (j/n): ").strip().lower()
    if response == 'j':
        storage.refill_all()
        print("[OK] Lager auf VOLL gesetzt.")
    else:
        print(f"[INFO] Eingabe war '{response}' - Lager NICHT zurueckgesetzt.")

    # Show status (after potential refill)
    storage.print_status()

    # Count required beams per category
    required = {cat: 0 for cat in VALID_CATEGORIES}
    for k in range(n_runs):
        i = start_i + k
        element = get_element(data, i, layer_idx=layer_idx)
        beam_size = element.get("beam_size", "").strip('"').strip("'")
        if beam_size not in required:
            print(f"\n[FEHLER] Unbekannte beam_size: '{beam_size}' bei Element {i}")
            return False
        required[beam_size] += 1

    # Compare with available inventory
    status = storage.get_status()
    missing = {}
    for cat, count in required.items():
        available = status[cat]["available"]
        if count > available:
            missing[cat] = count - available

    if missing:
        print("\n[FEHLER] NICHT GENUG HOLZ!")
        for cat, count in missing.items():
            print(f"  {cat}: {count} Stueck fehlen")
        print("\nBitte Holzlager auffuellen und neu starten.")
        return False

    print(f"\n[OK] Genug Holz vorhanden fuer {n_runs} Teile")
    return True


def main(*, dry_run=False):
    """Run the production loop.

    Workflow:
        1. Load fabrication data from JSON, run validation
        2. Check wood storage inventory (interactive, skipped in dry_run)
        3. Connect to robot via ROS bridge, optionally reset SimBeam state
        4. Loop over elements: PICK -> CUT -> GLUE -> PLACE
        5. Move to safe end position, disconnect

    Args:
        dry_run: If True, simulates the entire workflow without robot
                 connection. All stations print their planned moves
                 instead of executing them. Useful for verifying
                 fab_data before running on the real robot.
    """

    # ==============================
    # 1. Load & Validate Data
    # ==============================
    print("\n" + "=" * 50)
    print(f"FACADE PRODUCTION - Layer {LAYER}")
    print("=" * 50)

    DATA = load_data()

    # Validate data before anything else
    if HAS_VALIDATION:
        print("\nValidiere Daten...")
        errors = validate_all(DATA)
        if errors:
            print("\n[FEHLER] Validierung fehlgeschlagen!")
            print("=" * 50)
            for err in errors:
                print(f"  - {err}")
            print("=" * 50)
            print("\nBitte Daten in Grasshopper korrigieren und neu exportieren.")
            return
        print("[OK] Alle Validierungen bestanden.\n")

    # Layer validation
    if has_layers(DATA):
        n_layers = get_layer_count(DATA)
        if LAYER >= n_layers:
            print(f"[FEHLER] Layer {LAYER} existiert nicht! Daten haben {n_layers} Layer (0-{n_layers-1}).")
            return
        if LAYER >= MAX_LAYERS:
            print(f"[FEHLER] Maximal {MAX_LAYERS} Layer erlaubt (0-{MAX_LAYERS-1}).")
            return
        print(f"Daten: {n_layers} Layer")
    else:
        print("Daten im v2-Format (kein Layer)")

    n_total = get_element_count(DATA, layer_idx=LAYER)
    n_to_run = min(N_RUNS, max(0, n_total - START_I))

    if n_to_run == 0:
        print(f"[FEHLER] Keine Elemente zum Produzieren! (Total: {n_total}, Start: {START_I})")
        return

    print(f"Elemente in Layer: {n_total}, produziere {n_to_run} (ab Index {START_I})")

    # Per-element validation
    if HAS_VALIDATION:
        for k in range(n_to_run):
            i = START_I + k
            element = get_element(DATA, i, layer_idx=LAYER)
            elem_errors = validate_element(element, LAYER, i)
            if elem_errors:
                print(f"\n[FEHLER] Element {i} (Layer {LAYER}) ist ungueltig:")
                for err in elem_errors:
                    print(f"  - {err}")
                return

    # ==============================
    # 2. Check Wood Storage
    # ==============================
    if not dry_run:
        if not check_wood_storage(DATA, LAYER, START_I, n_to_run):
            return

    # ==============================
    # 3. Connect Robot
    # ==============================
    if dry_run:
        r1 = None
        ros = None
        print("\n*** DRY RUN MODE - keine Roboterbewegungen ***\n")
    else:
        ros = rrc.RosClient()
        ros.run()
        r1 = rrc.AbbClient(ros, ROBOT_NAME)
        print("Connected.")
        r1.send(rrc.SetTool(TOOL_GRIPPER))

        # Clear any beams from previous run so the facade starts empty
        if SIM_BEAMS:
            sim_beam_reset(r1)

    # ==============================
    # 4. Production Loop
    # ==============================
    for k in range(n_to_run):
        i = START_I + k

        print(f"\n{'='*50}")
        print(f"  LAYER {LAYER} | ELEMENT {i} ({k+1}/{n_to_run})")
        print(f"{'='*50}")

        if DO_PICK:
            print("\n--- PICK ---")
            a_pick_station.a_pick_station(r1, DATA, i, layer_idx=LAYER, dry_run=dry_run, css_enabled=CSS_ENABLED, sim_beams=SIM_BEAMS)

        if DO_CUT:
            print("\n--- CUT ---")
            b_cut_station.b_cut_station(r1, DATA, i, layer_idx=LAYER, dry_run=dry_run, saw_enabled=SAW_ENABLED, sim_beams=SIM_BEAMS)

        if DO_GLUE:
            print("\n--- GLUE ---")
            d_glue_station.d_glue_station(r1, DATA, i, layer_idx=LAYER, dry_run=dry_run, glue_valve_enabled=GLUE_VALVE_ENABLED)

        if DO_PLACE:
            print("\n--- PLACE ---")
            e_place_station.e_place_station(r1, DATA, i, layer_idx=LAYER, dry_run=dry_run, sim_beams=SIM_BEAMS)

    # ==============================
    # 5. Cleanup
    # ==============================
    if not dry_run and r1 is not None:
        # Move to safe position after all elements are placed
        r1.send_and_wait(cm.MoveToJoints([-40, 20, 0, 0, 70, -40], [500], 1, rrc.Zone.Z50))

    if not dry_run and ros is not None:
        ros.close()
        ros.terminate()

    print(f"\n{'='*50}")
    print(f"  FERTIG - {n_to_run} Elemente produziert")
    print(f"{'='*50}")


if __name__ == "__main__":
    main(dry_run=False)  # Set True to simulate without robot connection
