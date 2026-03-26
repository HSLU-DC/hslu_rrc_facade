# HSLU RRC Facade - Parametrische Fassadenelemente

Robotische Produktion von parametrischen Fassadenelementen mit dem ABB Gofa CRB 15000.

## Übersicht

Ihr designt in Grasshopper ein parametrisches Fassadenelement aus 25x25mm Holzlatten,
das robotisch auf einen Grundrahmen (600 x 2500mm, 40x40mm) platziert wird.

**Pipeline:** Grasshopper (Design) → JSON Export → Python (Roboter-Steuerung)

**Stationen:** Pick → Cut → Glue → Place

| Station | Was passiert |
|---------|-------------|
| **Pick** | Roboter holt Holz aus dem Lager |
| **Cut** | Roboter fährt zur Säge, schneidet beide Enden (Gehrungsschnitte) |
| **Glue** | Roboter fährt zur Leimstation, Leim wird aufgetragen |
| **Place** | Roboter platziert das Holz auf dem Rahmen |

## Was ihr liefern müsst

Aus Grasshopper müsst ihr folgende Daten exportieren:

| # | Daten | Typ | Beschreibung |
|---|-------|-----|-------------|
| 0 | `place_position` | Plane | Wo das Holz auf dem Rahmen platziert wird (in `ob_HSLU_Place` Koordinaten) |
| 1 | `cut_position_a` | Plane | Schnittposition Ende A (in `ob_HSLU_Cut` Koordinaten) |
| 2 | `cut_position_b` | Plane | Schnittposition Ende B (in `ob_HSLU_Cut` Koordinaten) |
| 3 | `glue_position_a` | Plane | Leimposition 1 (in `ob_HSLU_Glue` Koordinaten) |
| 4 | `glue_position_b` | Plane | Leimposition 2 (optional, kann leer sein) |
| 5 | `beam_size` | String | `"small"` oder `"large"` (Holzlänge-Kategorie) |

Diese Daten werden als **DataTree** mit `{Layer;Element}` Pfad-Struktur organisiert.

## Constraints / Einschränkungen

**Unbedingt beachten:**

- **Rahmen:** 600mm (Y) x 2500mm (X) - alle Elemente müssen innerhalb liegen
- **Max. 2 Layer** (Layer 0 und Layer 1)
- **Holzquerschnitt:** 25 x 25mm
- **Nur Gehrungsschnitte** (1D) - keine Schifterschnitte (2D)!
- **beam_size:** nur `"small"` oder `"large"` - bestimmt welches Holz aus dem Lager geholt wird
- **1-2 Leimpunkte** pro Element (mindestens `glue_position_a`)
- **Platzierungsreihenfolge** = Reihenfolge im DataTree (ihr bestimmt die Sequenz)

## Workflow

### 1. Design in Grasshopper

1. Öffne das GH-Template (`grasshopper/hslu_rrc_facade.gh`)
2. Verbinde eure Geometrie mit den Inputs
3. Prüfe die Validierung (grün = OK, rot = Fehler)
4. Exportiere die Daten (Button `update = True`)

### 2. JSON prüfen

Die exportierte Datei liegt unter `process/data/fab_data.json`.

Ihr könnt die Daten vorab prüfen:

```bash
cd process
python validate.py
```

### 3. Produktion starten

```bash
# 1. Docker starten (einmalig)
cd docker
docker compose up -d

# 2. Produktion starten
cd ../process
python production.py
```

### Konfiguration in `production.py`

```python
LAYER    = 0      # Welcher Layer (0 oder 1)
N_RUNS   = 1      # Wie viele Elemente produzieren
START_I  = 0      # Ab welchem Element-Index starten

DO_PICK  = True   # Stationen ein/aus (zum Testen)
DO_CUT   = True
DO_GLUE  = True
DO_PLACE = True
```

## Datenformat (JSON)

```json
{
  "layers": [
    {
      "id": 0,
      "elements": [
        {
          "id": 0,
          "beam_size": "small",
          "cut_position_a": { COMPAS Frame },
          "cut_position_b": { COMPAS Frame },
          "glue_position_a": { COMPAS Frame },
          "glue_position_b": { COMPAS Frame oder null },
          "place_position": { COMPAS Frame }
        }
      ]
    }
  ],
  "metadata": {
    "version": "3.0",
    "project": "facade",
    "frame_size": [600, 2500],
    "beam_section": 25
  }
}
```

## Projektstruktur

```
hslu_rrc_facade/
├── docker/
│   └── docker-compose.yml       # ROS + ABB Driver
├── grasshopper/
│   ├── hslu_rrc_facade.gh       # GH Template
│   └── export_fab_data.py       # GH Python Export-Script
├── process/
│   ├── production.py            # Hauptskript
│   ├── validate.py              # Daten-Validierung
│   ├── globals.py               # Konfiguration
│   ├── data/
│   │   ├── fab_data.json        # Euer Export (aus GH)
│   │   └── wood_storage.json    # Holzlager-Inventar
│   ├── _skills/                 # Robot-Skills (nicht verändern!)
│   └── stations/                # Station-Code (nicht verändern!)
└── design/                      # Eure Design-Dateien
```

**Wichtig:** Dateien in `_skills/` und `stations/` bitte NICHT verändern!

## Troubleshooting

| Problem | Lösung |
|---------|--------|
| `validate.py` zeigt Fehler | Daten in GH korrigieren und neu exportieren |
| "Nicht genug Holz" | Holzlager physisch auffüllen, Script fragt danach |
| Docker-Fehler | `docker compose down && docker compose up -d` |
| Roboter antwortet nicht | Prüfe ob Controller eingeschaltet und im AUTO-Modus |
| "beam_size ungültig" | Nur `"small"` oder `"large"` verwenden |

## Kontakt

Bei Problemen: Jurij kontaktieren.
