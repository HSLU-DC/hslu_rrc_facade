# fabdata.py
"""Fabrication data loading and access helpers.

Supports:
- v2: Element-oriented (data["elements"][i])
- v3: Layer + Element-oriented (data["layers"][layer]["elements"][i])
"""
import compas
from pathlib import Path

# _skills/fabdata.py  ->  data/fab_data.json
PATH = Path(__file__).resolve().parents[1] / "data" / "fab_data.json"


def load_data(path=PATH):
    """Load fabrication data from JSON file."""
    return compas.json_load(str(path))


# ==============================================================================
# Version detection
# ==============================================================================

def has_layers(data):
    """Check if data uses v3 layer structure.

    Args:
        data: Loaded fab_data dict

    Returns:
        True if layer-oriented (v3), False otherwise
    """
    return "layers" in data


def is_element_oriented(data):
    """Check if data uses element-oriented format (v2 or v3).

    Args:
        data: Loaded fab_data dict

    Returns:
        True if element-oriented, False if column-oriented
    """
    return "elements" in data or "layers" in data


# ==============================================================================
# Layer functions (v3)
# ==============================================================================

def get_layer_count(data):
    """Get number of layers.

    Args:
        data: Loaded fab_data dict

    Returns:
        Number of layers (1 if no layer structure)
    """
    if has_layers(data):
        return len(data["layers"])
    return 1


def get_layer(data, layer_idx):
    """Get layer by index.

    Args:
        data: Loaded fab_data dict
        layer_idx: Layer index (0, 1, 2, ...)

    Returns:
        Layer dict with "id" and "elements" keys
    """
    if has_layers(data):
        return data["layers"][layer_idx]
    # Fallback for v2: wrap in layer-like structure
    return {"id": 0, "elements": data["elements"]}


def get_layer_elements(data, layer_idx):
    """Get elements list for a specific layer.

    Args:
        data: Loaded fab_data dict
        layer_idx: Layer index

    Returns:
        List of element dicts
    """
    return get_layer(data, layer_idx)["elements"]


# ==============================================================================
# Element functions (work with both v2 and v3)
# ==============================================================================

def get_element(data, i, layer_idx=0):
    """Get element by index.

    Args:
        data: Loaded fab_data dict
        i: Element index
        layer_idx: Layer index (default 0, ignored for v2)

    Returns:
        Element dict with all positions for this element
    """
    if has_layers(data):
        return data["layers"][layer_idx]["elements"][i]
    return data["elements"][i]


def get_element_count(data, layer_idx=0):
    """Get number of elements.

    Args:
        data: Loaded fab_data dict
        layer_idx: Layer index (default 0, ignored for v2)

    Returns:
        Number of elements
    """
    if has_layers(data):
        return len(data["layers"][layer_idx]["elements"])
    return len(data["elements"])


def get_total_element_count(data):
    """Get total number of elements across all layers.

    Args:
        data: Loaded fab_data dict

    Returns:
        Total number of elements
    """
    if has_layers(data):
        return sum(len(layer["elements"]) for layer in data["layers"])
    return len(data["elements"])
