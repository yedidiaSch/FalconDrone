#!/usr/bin/env python3
"""
Write the order files for falcon_fc.kicad_pcb:

build/jlcpcb/
  falcon_fc_gerbers.zip   Gerbers (4 copper layers, mask, paste, silk, outline) + Excellon drill
  falcon_fc_bom.csv       Comment, Designator, Footprint, LCSC Part # (grouped by value + footprint)
  falcon_fc_cpl.csv       Designator, Mid X, Mid Y, Layer, Rotation (both sides)
build/pcbway/
  falcon_fc_pcbway_bom.csv  PCBWay assembly BOM: manufacturer part numbers (PCBWay buys
                            from DigiKey, Mouser, LCSC...), so the GPS and the exact
                            AP7361C are included; upload with the same gerbers zip and CPL

LCSC Part # is filled in for the parts found in LCSC (looked up 2026-10-21, see
ASSEMBLY.md); the passives are left to JLCPCB's BOM matcher, which picks basic
parts by value and footprint. Check stock on upload: it changes daily.

Usage:  python3 export_fab.py   (after route_pcb.py; checks DRC first)
"""

import csv
import json
import os
import shutil
import subprocess
import sys
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gen_pcb  # noqa: E402  (netlist reader)

PCB = os.path.join(HERE, gen_pcb.PROJECT + ".kicad_pcb")
OUT = os.path.join(HERE, "build", "jlcpcb")
LAYERS = ["F.Cu", "In1.Cu", "In2.Cu", "B.Cu", "F.Paste", "B.Paste", "F.Silkscreen",
          "B.Silkscreen", "F.Mask", "B.Mask", "Edge.Cuts"]
NOT_ASSEMBLED = ("H",)     # mounting holes
# BOM comments JLCPCB can match, where the schematic value is a role name
FP_COMMENTS = {
    "PinHeader_1x05_P2.54mm_Vertical": "Pin header 1x05 2.54mm vertical",
    "nRF24L01_Breakout": "Female header 2x04 2.54mm vertical (nRF24L01 module socket)",
}

# LCSC numbers by schematic value (see ASSEMBLY.md for the substitutions)
LCSC = {
    "STM32F446RET6": "C69336",
    "ICM-42688-P": "C1850418",
    "MS5611-01BA03": "C15639",        # listed as MS561101BA03-50
    "LP5907MFX-3.3": "C80670",
    "SP0504BAHT": "C10675",
    "AO3401A": "C15127",
    "SMAJ5.0A": "C499731",
    "AP7361C-33E": "C150719",         # AP7361-33E-13: the C version isn't listed
}
FP_LCSC = {
    "JST_GH_SM03B-GHS-TB_1x03-1MP_P1.25mm_Horizontal": "C514175",
    "JST_GH_SM06B-GHS-TB_1x06-1MP_P1.25mm_Horizontal": "C133065",
    "SW_SPST_PTS810": "C116501",
}

# PCBWay BOM: schematic value -> (manufacturer, MPN, description). Passives
# have no MPN on purpose: PCBWay sources them to the description.
MPN = {
    "STM32F446RET6": ("STMicroelectronics", "STM32F446RET6", "MCU ARM Cortex-M4 180 MHz 512 KB flash"),
    "AP7361C-33E": ("Diodes Incorporated", "AP7361C-33E-13", "LDO 3.3 V 1 A"),
    "ICM-42688-P": ("TDK InvenSense", "ICM-42688-P", "6-axis IMU"),
    "LP5907MFX-3.3": ("Texas Instruments", "LP5907MFX-3.3/NOPB", "LDO 3.3 V 250 mA low noise"),
    "MS5611-01BA03": ("TE Connectivity", "MS561101BA03-50", "Barometric pressure sensor"),
    "SAM-M10Q": ("u-blox", "SAM-M10Q-00B", "GNSS module with patch antenna"),
    "SP0504BAHT": ("Littelfuse", "SP0504BAHTG", "4-channel TVS/ESD array"),
    "AO3401A": ("Alpha & Omega Semiconductor", "AO3401A", "P-MOSFET -30 V -4 A"),
    "SMAJ5.0A": ("Littelfuse", "SMAJ5.0A", "TVS diode 5 V 400 W unidirectional"),
    "600R@100MHz": ("Sunlord", "GZ1608D601TF", "Ferrite bead 600R @ 100 MHz"),
    "green": ("", "", "LED green"),
}
FP_MPN = {
    "JST_GH_SM03B-GHS-TB_1x03-1MP_P1.25mm_Horizontal":
        ("JST", "SM03B-GHS-TB(LF)(SN)", "Connector JST-GH 3-pin right angle SMD"),
    "JST_GH_SM06B-GHS-TB_1x06-1MP_P1.25mm_Horizontal":
        ("JST", "SM06B-GHS-TB(LF)(SN)", "Connector JST-GH 6-pin right angle SMD"),
    "SW_SPST_PTS810": ("C&K", "PTS810SJM250SMTRLFS", "Tactile switch SMD"),
    "PinHeader_1x05_P2.54mm_Vertical": ("", "", "Pin header 1x05 2.54 mm vertical (ESC)"),
    "nRF24L01_Breakout": ("", "", "Female header 2x04 2.54 mm vertical (nRF24L01 module socket)"),
}
THT = ("PinHeader_", "nRF24L01_Breakout")


def pcbway_line(value, fp):
    """(manufacturer, MPN, description) for one BOM line."""
    if fp in FP_MPN:
        return FP_MPN[fp]
    if value in MPN:
        return MPN[value]
    size = fp.split("_")[1]                      # e.g. 0603
    if fp.startswith("R_"):
        return "", "", f"Resistor {value}Ω 1% {size}"
    if fp.startswith("C_"):
        return "", "", f"Capacitor {value}F MLCC X5R/X7R 16 V or higher {size}"
    raise SystemExit(f"{value} ({fp}): no PCBWay part; add it to MPN")


def comment(value, fp):
    if fp.startswith("JST_GH_"):
        return fp.split("_")[2]           # e.g. SM06B-GHS-TB
    return FP_COMMENTS.get(fp, value)


def kicad_cli(*args):
    subprocess.run(["kicad-cli", *args], check=True, capture_output=True)


def check_drc():
    report = os.path.join(OUT, "drc.json")
    kicad_cli("pcb", "drc", "--format", "json", "-o", report, PCB)
    drc = json.load(open(report))
    # J2's silk was trimmed where the nRF module overhangs the edge, so it no
    # longer matches the library copy; that one is expected.
    errors = [v for v in drc["violations"] if v["severity"] == "error"]
    warnings = [v for v in drc["violations"] if v["severity"] != "error"
                and v["type"] != "lib_footprint_mismatch"]
    if errors or drc["unconnected_items"]:
        raise SystemExit(f"DRC: {len(errors)} errors, {len(drc['unconnected_items'])} unconnected "
                         f"(see {report})")
    for v in warnings:
        print("DRC warning:", v["description"])


def gerbers():
    tmp = os.path.join(OUT, "gerbers")
    shutil.rmtree(tmp, ignore_errors=True)
    os.makedirs(tmp)
    kicad_cli("pcb", "export", "gerbers", "-o", tmp + "/", "--layers", ",".join(LAYERS),
              "--subtract-soldermask", PCB)
    kicad_cli("pcb", "export", "drill", "-o", tmp + "/", "--format", "excellon",
              "--excellon-units", "mm", "--excellon-separate-th", PCB)
    path = os.path.join(OUT, gen_pcb.PROJECT + "_gerbers.zip")
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        for name in sorted(os.listdir(tmp)):
            z.write(os.path.join(tmp, name), name)
    shutil.rmtree(tmp)
    return path


def bom():
    comps, _ = gen_pcb.load_netlist()
    groups = {}
    for ref, (value, fpid) in comps.items():
        fp = fpid.split(":")[1]
        groups.setdefault((comment(value, fp), fp), []).append(ref)
    key = lambda r: (r.rstrip("0123456789"), int(r[len(r.rstrip("0123456789")):] or 0))
    path = os.path.join(OUT, gen_pcb.PROJECT + "_bom.csv")
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Comment", "Designator", "Footprint", "LCSC Part #"])
        for (value, fp), refs in sorted(groups.items(), key=lambda g: key(sorted(g[1], key=key)[0])):
            lcsc = FP_LCSC.get(fp) or LCSC.get(value, "")
            w.writerow([value, ",".join(sorted(refs, key=key)), fp, lcsc])
    return path


def pcbway_bom():
    comps, _ = gen_pcb.load_netlist()
    groups = {}
    for ref, (value, fpid) in comps.items():
        fp = fpid.split(":")[1]
        groups.setdefault((pcbway_line(value, fp), fp), []).append(ref)
    key = lambda r: (r.rstrip("0123456789"), int(r[len(r.rstrip("0123456789")):] or 0))
    out = os.path.join(HERE, "build", "pcbway")
    os.makedirs(out, exist_ok=True)
    path = os.path.join(out, gen_pcb.PROJECT + "_pcbway_bom.csv")
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Item", "Designator", "Qty", "Manufacturer", "Manufacturer Part Number",
                    "Description", "Package", "Type", "LCSC (reference)"])
        lines = sorted(groups.items(), key=lambda g: key(sorted(g[1], key=key)[0]))
        for i, (((mfr, mpn, desc), fp), refs) in enumerate(lines, 1):
            value = comps[refs[0]][0]
            lcsc = FP_LCSC.get(fp) or LCSC.get(value, "")
            if value == "AP7361C-33E":
                lcsc = ""                         # LCSC's is the non-C part; PCBWay can get the C
            w.writerow([i, ",".join(sorted(refs, key=key)), len(refs), mfr, mpn, desc, fp,
                        "THT" if fp.startswith(THT) else "SMD", lcsc])
    return path


def cpl():
    raw = os.path.join(OUT, "pos.csv")
    kicad_cli("pcb", "export", "pos", "-o", raw, "--format", "csv", "--units", "mm",
              "--side", "both", PCB)
    path = os.path.join(OUT, gen_pcb.PROJECT + "_cpl.csv")
    with open(raw) as src, open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Designator", "Mid X", "Mid Y", "Layer", "Rotation"])
        for row in csv.DictReader(src):
            if row["Ref"].startswith(NOT_ASSEMBLED):
                continue
            w.writerow([row["Ref"], row["PosX"] + "mm", row["PosY"] + "mm",
                        "Top" if row["Side"] == "top" else "Bottom", row["Rot"]])
    os.remove(raw)
    return path


def main():
    os.makedirs(OUT, exist_ok=True)
    check_drc()
    for path in (gerbers(), bom(), cpl(), pcbway_bom()):
        print("wrote", os.path.relpath(path, HERE))


if __name__ == "__main__":
    main()
