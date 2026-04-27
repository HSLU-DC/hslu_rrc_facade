# production.py
"""Main production orchestrator for the Facade FS26 robot cell.

This script controls the full fabrication workflow for timber facade element
assembly. Each beam goes through 4 stations in sequence:

    1. PICK   — Retrieve beam from wood storage (WoodStorage inventory system)
    2. CUT    — Cut beam to length at circular saw station (2 cuts per beam)
    3. GLUE   — Apply adhesive with Robatech glue system (PLC-controlled)
    4. PLACE  — Place beam onto facade frame (track-compensated approach)

The fabrication data (positions, sizes, orientations) is exported from
Grasshopper as JSON (see design/gh_python/ExportFacade.py) and organized
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
from joint_positions import jp_park
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


def _compute_demand(data, layer_idx, start_i, n_runs):
    """Count required beams per category for the planned run."""
    demand = {cat: 0 for cat in VALID_CATEGORIES}
    for k in range(n_runs):
        i = start_i + k
        element = get_element(data, i, layer_idx=layer_idx)
        beam_size = element.get("beam_size", "").strip('"').strip("'")
        if beam_size not in demand:
            return None, beam_size, i
        demand[beam_size] += 1
    return demand, None, None


def _prompt_int(prompt, default, lo, hi):
    """Prompt for an integer in [lo, hi]. Empty input returns default."""
    while True:
        response = input(prompt).strip()
        if not response:
            return default
        try:
            value = int(response)
        except ValueError:
            print("    Bitte Zahl eingeben.")
            continue
        if value < lo or value > hi:
            print(f"    Ungueltig ({lo}..{hi})")
            continue
        return value


def check_wood_storage(data, layer_idx, start_i, n_runs):
    """Pre-flight: show design demand, take per-category counts from operator.

    Workflow:
        1. Compute demand per category from fab_data
        2. Show demand vs lager capacity (warns if multiple refills will be needed)
        3. Per category, prompt for actual stock count (default = recommended)
        4. Write counts back to wood_storage.json

    Categories with zero demand are auto-set to 0 (no prompt).

    Mid-production refills (when a lager runs empty) are handled separately
    by refill_lager() in the production loop.

    Returns:
        True on success, False on bad data (unknown beam_size in design)
    """
    storage = WoodStorage()

    demand, bad_size, bad_idx = _compute_demand(data, layer_idx, start_i, n_runs)
    if demand is None:
        print(f"\n[FEHLER] Unbekannte beam_size: '{bad_size}' bei Element {bad_idx}")
        return False

    print("\n" + "=" * 50)
    print("HOLZLAGER KONFIGURATION")
    print("=" * 50)

    print("\nHolzbedarf laut Design:")
    for cat in VALID_CATEGORIES:
        cap = storage.get_capacity(cat)
        warn = ""
        if demand[cat] > cap:
            n_refills = (demand[cat] - 1) // cap
            warn = f"  -> mind. {n_refills} Auffuellungen waehrend Produktion!"
        print(f"  {cat} mm: {demand[cat]:>3} Stueck (Lager-Kapazitaet: {cap}){warn}")

    print("\nWieviele Stueck zum Start? (Enter = empfohlener Wert)")

    for cat in VALID_CATEGORIES:
        cap = storage.get_capacity(cat)
        if demand[cat] == 0:
            storage.set_count(cat, 0)
            continue
        recommended = min(demand[cat], cap)
        count = _prompt_int(f"  {cat} mm [{recommended}]: ", recommended, 0, cap)
        storage.set_count(cat, count)

    print()
    storage.print_status()
    return True


def refill_lager(r1, storage, category, remaining_demand, *, dry_run=False):
    """Mid-production refill prompt for an empty lager category.

    Parks the robot at jp_park (track retracted, robot folded), then asks
    the operator how many beams were nachgefuellt. Updates storage counts
    via storage.set_count(category, n).

    Args:
        r1: AbbClient instance (or None for dry_run)
        storage: WoodStorage instance to update
        category: Empty category that triggered the refill
        remaining_demand: How many beams of this category are still needed
                          for the rest of the production run
        dry_run: If True, skips robot motion (the prompt still runs)
    """
    capacity = storage.get_capacity(category)
    recommended = min(remaining_demand, capacity)

    if not dry_run and r1 is not None:
        r1.send_and_wait(cm.MoveToJoints(jp_park.robax, jp_park.extax, 2, rrc.Zone.Z50))

    print("\n" + "=" * 50)
    print(f"  LAGER LEER: {category} mm")
    print("=" * 50)
    print(f"\n  Noch benoetigt:    {remaining_demand}")
    print(f"  Lager-Kapazitaet:  {capacity}")
    print(f"  Empfohlen jetzt:   {recommended}")

    count = _prompt_int(
        f"\nWieviele Stueck nachgefuellt? [{recommended}]: ",
        recommended, 1, capacity,
    )
    storage.set_count(category, count)
    print(f"  [OK] Lager '{category}' auf {count} gesetzt.\n")


def _remaining_demand(data, layer_idx, start_k, n_to_run, start_i, category):
    """Count how many beams of `category` are still needed in the rest of the run."""
    n = 0
    for kk in range(start_k, n_to_run):
        beam = get_element(data, start_i + kk, layer_idx=layer_idx) \
                  .get("beam_size", "").strip('"').strip("'")
        if beam == category:
            n += 1
    return n


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
    # Storage instance for the lager-empty fail-safe (separate from the one
    # the pick station maintains internally; we reload from disk before each
    # check so take_beam() decrements from inside a_pick_station are visible).
    storage_check = None if dry_run else WoodStorage()

    for k in range(n_to_run):
        i = START_I + k

        print(f"\n{'='*50}")
        print(f"  LAYER {LAYER} | ELEMENT {i} ({k+1}/{n_to_run})")
        print(f"{'='*50}")

        if DO_PICK:
            element = get_element(DATA, i, layer_idx=LAYER)
            beam_size = element.get("beam_size", "").strip('"').strip("'")

            # Fail-safe: if the lager for this category is empty, park robot
            # and prompt operator to refill before attempting the pick.
            if not dry_run:
                storage_check.reload()
                if not storage_check.has_beams(beam_size):
                    remaining = _remaining_demand(DATA, LAYER, k, n_to_run, START_I, beam_size)
                    refill_lager(r1, storage_check, beam_size, remaining)

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
