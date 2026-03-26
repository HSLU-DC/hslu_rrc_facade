# wood_storage.py
"""Wood storage inventory management system.

Manages multiple compartments for different beam sizes (small, large).
Each compartment holds stacked beams and tracks inventory count.

Round-robin logic ensures even distribution across compartments of the same category.
"""
from __future__ import annotations

import json
import os
from datetime import datetime
from typing import Optional

from compas.geometry import Frame, Point, Vector

VALID_CATEGORIES = ("small", "large")


class WoodStorage:
    """Manages wood beam inventory across multiple storage compartments.

    Attributes:
        path: Path to the JSON inventory file
        data: Loaded inventory data
    """

    def __init__(self, json_path: str = None):
        """Initialize WoodStorage.

        Args:
            json_path: Path to inventory JSON file. If None, uses default location.
        """
        if json_path is None:
            this_dir = os.path.dirname(os.path.abspath(__file__))
            json_path = os.path.join(this_dir, "..", "..", "data", "wood_storage.json")

        self.path = os.path.abspath(json_path)
        self.data = self._load()

    def _load(self) -> dict:
        """Load inventory data from JSON file."""
        if not os.path.exists(self.path):
            raise FileNotFoundError(f"Inventory file not found: {self.path}")

        with open(self.path, "r", encoding="utf-8") as f:
            return json.load(f)

    def _save(self) -> None:
        """Save inventory data to JSON file."""
        self.data["last_updated"] = datetime.now().isoformat()
        with open(self.path, "w", encoding="utf-8") as f:
            json.dump(self.data, f, indent=2, ensure_ascii=False)

    def _parse_frame(self, frame_data: dict) -> Frame:
        """Parse frame data from JSON into COMPAS Frame."""
        point = Point(*frame_data["point"])
        xaxis = Vector(*frame_data["xaxis"])
        yaxis = Vector(*frame_data["yaxis"])
        return Frame(point, xaxis, yaxis)

    def get_pick_frame(self, category: str) -> tuple[Frame, str, str, float]:
        """Get pick frame for the topmost beam of a category.

        Uses round-robin logic: always picks from the fullest compartment
        to ensure even distribution across all compartments of the category.

        Args:
            category: Beam category ("small" or "large")

        Returns:
            Tuple of (pick_frame, compartment_id, wobj, extax)

        Raises:
            ValueError: If all compartments of the category are empty
        """
        if category not in VALID_CATEGORIES:
            raise ValueError(
                f"Ungueltige Kategorie: '{category}'. "
                f"Erlaubt: {VALID_CATEGORIES}"
            )

        matching = [
            (cid, c) for cid, c in self.data["compartments"].items()
            if c["category"] == category and c["count"] > 0
        ]

        if not matching:
            available = self.get_status()
            raise ValueError(
                f"Alle {category} Faecher sind leer!\n"
                f"Verfuegbar: {available}"
            )

        # Sort by count (highest first) - always take from fullest compartment
        matching.sort(key=lambda x: (-x[1]["count"], x[0]))
        compartment_id, compartment = matching[0]

        # Calculate pick frame based on current stack height
        base_frame = self._parse_frame(compartment["base_frame"])
        count = compartment["count"]
        stack_offset = compartment["stack_offset_z"]

        # Z offset: beam 1 is at base, beam N is at base + (N-1)*offset
        z_offset = (count - 1) * stack_offset

        pick_frame = Frame(
            Point(base_frame.point.x, base_frame.point.y, base_frame.point.z + z_offset),
            base_frame.xaxis,
            base_frame.yaxis
        )

        wobj = compartment.get("wobj", "wobj0")
        extax = compartment.get("extax", 0)

        return pick_frame, compartment_id, wobj, extax

    def take_beam(self, compartment_id: str) -> None:
        """Decrement beam count after successful pickup.

        Call this AFTER the robot has successfully gripped the beam.

        Args:
            compartment_id: ID of the compartment to decrement
        """
        if compartment_id not in self.data["compartments"]:
            raise KeyError(f"Unknown compartment: {compartment_id}")

        compartment = self.data["compartments"][compartment_id]

        if compartment["count"] <= 0:
            raise ValueError(f"Compartment {compartment_id} is already empty!")

        compartment["count"] -= 1
        self._save()

        print(f"[STORAGE] Took beam from {compartment_id}, remaining: {compartment['count']}")

    def refill(self, compartment_id: str, count: int = None) -> None:
        """Refill a compartment after manual restocking.

        Args:
            compartment_id: ID of the compartment to refill
            count: Number of beams (default: fill to capacity)
        """
        if compartment_id not in self.data["compartments"]:
            raise KeyError(f"Unknown compartment: {compartment_id}")

        compartment = self.data["compartments"][compartment_id]

        if count is None:
            count = compartment["capacity"]

        if count > compartment["capacity"]:
            raise ValueError(
                f"Count {count} exceeds capacity {compartment['capacity']} "
                f"for {compartment_id}"
            )

        compartment["count"] = count
        self._save()

        print(f"[STORAGE] Refilled {compartment_id} to {count} beams")

    def refill_all(self) -> None:
        """Refill all compartments to full capacity."""
        for cid, compartment in self.data["compartments"].items():
            compartment["count"] = compartment["capacity"]

        self._save()
        print("[STORAGE] All compartments refilled to capacity")

    def get_status(self) -> dict:
        """Get overview of all compartments.

        Returns:
            Dict with category totals and per-compartment details
        """
        status = {
            "small": {"total": 0, "available": 0, "compartments": {}},
            "large": {"total": 0, "available": 0, "compartments": {}},
        }

        for cid, c in self.data["compartments"].items():
            cat = c["category"]
            if cat not in status:
                continue
            status[cat]["total"] += c["capacity"]
            status[cat]["available"] += c["count"]
            status[cat]["compartments"][cid] = {
                "count": c["count"],
                "capacity": c["capacity"]
            }

        return status

    def print_status(self) -> None:
        """Print formatted status overview."""
        status = self.get_status()

        print("\n" + "=" * 50)
        print("HOLZLAGER STATUS")
        print("=" * 50)

        for category in ["small", "large"]:
            cat_status = status[category]
            print(f"\n{category.upper()}:")
            print(f"  Available: {cat_status['available']} / {cat_status['total']}")
            for cid, cs in cat_status["compartments"].items():
                bar = "#" * cs["count"] + "-" * (cs["capacity"] - cs["count"])
                print(f"  {cid}: [{bar}] {cs['count']}/{cs['capacity']}")

        print("\n" + "=" * 50)


def load_storage(json_path: str = None) -> WoodStorage:
    """Load wood storage inventory.

    Args:
        json_path: Optional path to JSON file

    Returns:
        WoodStorage instance
    """
    return WoodStorage(json_path)
