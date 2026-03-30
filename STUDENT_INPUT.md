# Fassadenelemente - Input-Spezifikation

## Koordinatensystem

Alle Geometrie wird im **Weltkoordinatensystem** von Rhino erstellt. Dieses entspricht dem Roboter-Workobject `ob_HSLU_Place`.

Der **Ursprung (0, 0, 0) liegt oben links** am Grundrahmen.

- **X-Achse:** entlang der langen Seite des Rahmens (0 bis 2500mm)
- **Y-Achse:** entlang der kurzen Seite des Rahmens (0 bis -600mm)
- **Z-Achse:** nach oben, weg vom Rahmen

<img src="docs/images/00_koordinatensystem.png" width="600">

---

## Datenstruktur

Die Daten werden als **Grasshopper DataTree** mit der Pfad-Struktur `{Layer;Element}` organisiert.

- **Layer 0:** Erste Lage direkt auf dem Grundrahmen
- **Layer 1:** Zweite Lage auf Layer 0 (optional)
- Maximal **2 Layer** erlaubt

Die **Reihenfolge der Elemente im Tree bestimmt die Reihenfolge**, in der der Roboter die Elemente platziert. Überlegt euch die Sequenz sorgfältig - ein Element kann nur dort platziert werden, wo es aufliegt.

Jeder Branch enthält **5 bis 6 Einträge** (Glue Plane B ist optional):

| Index | Name | Typ | Beschreibung |
|-------|------|-----|-------------|
| 0 | Brep | Brep | Fertige Balkengeometrie |
| 1 | Centerline | Line | Mittelachse des Balkens |
| 2 | Cut Plane A | Plane | Schnittebene Ende A |
| 3 | Cut Plane B | Plane | Schnittebene Ende B |
| 4 | Glue Plane A | Plane | Leimebene 1 |
| 5 | Glue Plane B | Plane | Leimebene 2 (optional) |

<img src="docs/images/01_datatree.png" width="600">

---

## 0 - Brep (Balkengeometrie)

Die vollständige 3D-Geometrie des **fertigen Holzstabs** mit 25x25mm Querschnitt. Die Gehrungsschnitte an beiden Enden müssen bereits ausgeführt sein - das Brep zeigt den Balken so, wie er am Ende auf dem Rahmen liegt.

Das Brep wird **nicht an den Roboter übertragen**. Es dient zur Visualisierung, Kollisionsprüfung und als Grundlage für die automatische Berechnung der Roboterpositionen.

**Anforderungen:**
- Querschnitt: 25 x 25mm
- Geschlossenes Brep (Solid)
- Gehrungsschnitte an beiden Enden bereits modelliert

<img src="docs/images/02_brep.png" width="600">

---

## 1 - Centerline (Mittelachse)

Die Mittelachse des **fertigen** (zugeschnittenen) Balkens als Linie. Die Centerline verläuft exakt durch die Mitte des Balkenquerschnitts.

**Anforderungen:**
- **Startpunkt:** Mitte der Stirnfläche an Ende A
- **Endpunkt:** Mitte der Stirnfläche an Ende B
- Einfache Linie (Line), keine Kurve

Die Centerline bestimmt automatisch:
- Die **Platzierungsposition** (Mittelpunkt der Centerline)
- Die **Platzierungsrichtung** (Richtung der Centerline)
- Die **beam_size** (Länge der Centerline bestimmt ob "small" oder "large" Holz aus dem Lager geholt wird)

<img src="docs/images/03_centerline.png" width="600">

---

## 2 - Cut Plane A (Schnittebene Ende A)

Die Schnittebene am **ersten Ende** des Balkens (Startpunkt der Centerline). Diese Plane definiert, wie die Säge das Holz schneidet.

**Position:**
- Mitte der Stirnfläche an Ende A (= Startpunkt der Centerline)

**Orientierung:**
- **Z-Achse:** zeigt **nach aussen**, weg vom Balken (= Normale der Schnittfläche)
- **Y-Achse:** zeigt **nach oben** (Welt-Z Richtung)
- **X-Achse:** ergibt sich automatisch

**Wichtig:**
- Nur **Gehrungsschnitte** (1D) sind erlaubt. Das bedeutet: die Schnittebene darf nur um die vertikale Achse gedreht sein. Eine Neigung der Schnittebene nach oben oder unten (Schifterschnitt / 2D-Schnitt) ist **nicht möglich** und wird von der Validierung abgelehnt.

<img src="docs/images/04_cut_plane_orientierung.png" width="600">

---

## 3 - Cut Plane B (Schnittebene Ende B)

Die Schnittebene am **zweiten Ende** des Balkens (Endpunkt der Centerline). Identische Konvention wie Cut Plane A.

**Position:**
- Mitte der Stirnfläche an Ende B (= Endpunkt der Centerline)

**Orientierung:**
- **Z-Achse:** zeigt **nach aussen**, weg vom Balken
- **Y-Achse:** zeigt **nach oben** (Welt-Z Richtung)
- **X-Achse:** ergibt sich automatisch


---

## 4 - Glue Plane A (Leimebene 1)

Die erste Leimebene definiert, wo der Roboter Leim aufträgt. Der Leim wird in der Leimstation auf die **Unterseite** des Balkens aufgetragen, bevor das Element platziert wird.

**Position:**
- Am **Rand/Ende** des Balkens
- In der **Mitte** der Leimfläche
- Die Leimfläche ist 25 x 25mm (= Querschnitt des Balkens)
- Die Plane sitzt 12.5mm von allen Rändern der Leimfläche entfernt

**Orientierung:**
- **Z-Achse:** zeigt **nach unten** (Richtung Unterlage / Rahmen)
- **X-Achse:** zeigt **Richtung Mitte** des Elements (entlang der Centerline, zur Balkenmitte hin)
- **Y-Achse:** ergibt sich automatisch

**Warum am Rand?** Der Leim muss dort aufgetragen werden, wo der Balken auf einem anderen Element oder dem Grundrahmen aufliegt. Das ist typischerweise an den Enden des Balkens, wo er ein darunterliegendes Element kreuzt.

<img src="docs/images/05_glue_plane_orientierung.png" width="600">

---

## 5 - Glue Plane B (Leimebene 2, optional)

Die zweite Leimebene am **anderen Ende** des Balkens. Identische Konvention wie Glue Plane A.

Glue Plane B ist **optional** und kann weggelassen werden, wenn der Balken nur an einem Ende verleimt werden muss.

**Wann brauche ich zwei Leimebenen?**
- Wenn der Balken an **beiden Enden** auf einem anderen Element aufliegt (typisch bei Layer 1)
- Wenn der Balken an **beiden Enden** auf dem Grundrahmen aufliegt

**Wann reicht eine Leimebene?**
- Wenn der Balken nur an **einem Ende** fixiert werden muss
- Entscheidung liegt bei euch - überlegt was strukturell sinnvoll ist


---

## Zusammenfassung der Plane-Konventionen

| Plane | Z-Achse | Y-Achse | X-Achse |
|-------|---------|---------|---------|
| Cut Plane A | nach aussen (weg vom Balken) | nach oben (Welt-Z) | ergibt sich |
| Cut Plane B | nach aussen (weg vom Balken) | nach oben (Welt-Z) | ergibt sich |
| Glue Plane A | nach unten | ergibt sich | Richtung Balkenmitte |
| Glue Plane B | nach unten | ergibt sich | Richtung Balkenmitte |

<img src="docs/images/06_uebersicht_alle_planes.png" width="600">

---

## Constraints / Einschränkungen

| Constraint | Wert |
|-----------|------|
| Rahmengrösse | X: 0 - 2500mm, Y: 0 - -600mm |
| Balkenquerschnitt | 25 x 25mm |
| Maximale Anzahl Layer | 2 |
| Schnitttyp | Nur Gehrungsschnitte (1D) |
| Leimebenen pro Element | 1 - 2 |
| Platzierungsreihenfolge | = Reihenfolge im DataTree |
