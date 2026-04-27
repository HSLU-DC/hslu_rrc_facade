# Grasshopper Python Component: Holzbedarf aus fab_data
# ghenv.Component.Message = 'Holzbedarf v1'
#
# Zaehlt Elemente pro beam_size Kategorie und summiert die benoetigten
# Stock-Laufmeter (= was bestellt werden muss).
#
# ==============================================================================
# INPUTS
# ==============================================================================
#   fab_data (DataTree):  Tree mit {layer;element} Struktur,
#                         beam_size an Index 11
#                         Type Hint: GenericObject, Access: tree
#
# ==============================================================================
# OUTPUTS
# ==============================================================================
#   summary (str):  Mehrzeiliger Bedarfs-Report -- auf ein Panel verbinden

VALID_SIZES = ("400", "550", "750", "1000")
BEAM_SIZE_INDEX = 11

# Stock-Laenge pro Kategorie in Meter
STOCK_LENGTHS_M = {"400": 0.40, "550": 0.55, "750": 0.75, "1000": 1.00}

counts = {s: 0 for s in VALID_SIZES}
unknown = 0
total = 0

if fab_data is not None:
    for path in fab_data.Paths:
        branch = list(fab_data.Branch(path))
        if BEAM_SIZE_INDEX < len(branch) and branch[BEAM_SIZE_INDEX] is not None:
            size = str(branch[BEAM_SIZE_INDEX]).strip().strip('"').strip("'")
            if size in counts:
                counts[size] += 1
            else:
                unknown += 1
            total += 1

lm_total = sum(counts[s] * STOCK_LENGTHS_M[s] for s in VALID_SIZES)

lines = ["Holzbedarf:"]
for s in VALID_SIZES:
    lm = counts[s] * STOCK_LENGTHS_M[s]
    lines.append("  {} mm: {:>3} Stueck = {:>5.2f} lm".format(s, counts[s], lm))
if unknown:
    lines.append("  ?:       {:>3} Stueck (unbekannte Groesse!)".format(unknown))
lines.append("  " + "-" * 28)
lines.append("  Total:   {:>3} Stueck = {:>5.2f} lm".format(total, lm_total))

summary = "\n".join(lines)
