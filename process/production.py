# production.py
"""Main production script for facade element fabrication.

Pipeline: Pick -> Cut -> Glue -> Place
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import compas_rrc as rrc

from _skills.fabdata import load_data, has_layers, get_layer_count, get_element_count, get_element
from _skills.WoodStorage.wood_storage import WoodStorage
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
DO_PICK  = True
DO_CUT   = True
DO_GLUE  = True
DO_PLACE = True

CSS_ENABLED = True            # Soft Servo (CSS) for compliant gripping
SAW_ENABLED = True            # Saw on/off control
GLUE_VALVE_ENABLED = True     # Glue valve on/off control

LAYER    = 0      # Layer index (0 or 1)
N_RUNS   = 1      # Number of elements to produce per run
START_I  = 0      # Start element index


def check_wood_storage(data, layer_idx, start_i, n_runs):
    """Check wood storage before production start.

    Returns:
        True if enough wood, False if not
    """
    storage = WoodStorage()

    print("\n" + "=" * 50)
    print("HOLZLAGER CHECK")
    print("=" * 50)

    response = input("\nWurde das Holzlager KOMPLETT aufgefuellt? (j/n): ").strip().lower()
    if response == 'j':
        storage.refill_all()
        print("[OK] Lager auf VOLL gesetzt.")
    else:
        print(f"[INFO] Eingabe war '{response}' - Lager NICHT zurueckgesetzt.")

    storage.print_status()

    # Count required beams per category
    required = {"small": 0, "large": 0}
    for k in range(n_runs):
        i = start_i + k
        element = get_element(data, i, layer_idx=layer_idx)
        beam_size = element.get("beam_size", "small").strip('"').strip("'")
        if beam_size not in required:
            print(f"\n[FEHLER] Unbekannte beam_size: '{beam_size}' bei Element {i}")
            return False
        required[beam_size] += 1

    # Compare with available stock
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
    """Main production loop."""

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
            a_pick_station.a_pick_station(r1, DATA, i, layer_idx=LAYER, dry_run=dry_run, css_enabled=CSS_ENABLED)

        if DO_CUT:
            print("\n--- CUT ---")
            b_cut_station.b_cut_station(r1, DATA, i, layer_idx=LAYER, dry_run=dry_run, saw_enabled=SAW_ENABLED)

        if DO_GLUE:
            print("\n--- GLUE ---")
            d_glue_station.d_glue_station(r1, DATA, i, layer_idx=LAYER, dry_run=dry_run, glue_valve_enabled=GLUE_VALVE_ENABLED)

        if DO_PLACE:
            print("\n--- PLACE ---")
            e_place_station.e_place_station(r1, DATA, i, layer_idx=LAYER, dry_run=dry_run)

    # ==============================
    # 5. Cleanup
    # ==============================
    if not dry_run and r1 is not None:
        # Move to safe position
        r1.send_and_wait(cm.MoveToJoints([-40, 20, 0, 0, 70, -40], [500], 1, rrc.Zone.Z50))

    if not dry_run and ros is not None:
        ros.close()
        ros.terminate()

    print(f"\n{'='*50}")
    print(f"  FERTIG - {n_to_run} Elemente produziert")
    print(f"{'='*50}")


if __name__ == "__main__":
    main(dry_run=False)
