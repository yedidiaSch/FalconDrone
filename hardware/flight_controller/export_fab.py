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

LCSC Part # is filled in for every part (looked up 2026-10-25, see ASSEMBLY.md);
J2/J4 are through-hole, so turn on JLCPCB's THT assembly. Check stock on upload:
it changes daily. The JLCPCB BOM and CPL leave out JLC_DNP (fitted later by
hand); the PCBWay BOM still lists every part.

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
# Left off the JLCPCB order to save cost (~$60 for two boards); their support
# parts (U6, FB1, decoupling) are still fitted, so these can go on later.
JLC_DNP = {"U5", "U7"}     # MS5611 barometer, SAM-M10Q GPS
# BOM comments JLCPCB can match, where the schematic value is a role name
FP_COMMENTS = {
    "PinHeader_1x05_P2.54mm_Vertical": "PZ254V-11-05P",
    "nRF24L01_Breakout": "A2541HWV-2x4P",
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
    "AP7361C-33E": "C500795",         # AP7361C-33E-13
    "SAM-M10Q": "C5443880",           # SAM-M10Q-00B
    "600R@100MHz": "C1002",           # Sunlord GZ1608D601TF
    "green": "C51933293",             # GL0603UG01
    # passives: the basic parts JLCPCB's matcher picks for these values
    "10u": "C15850",                  # 0805 25 V X5R
    "4.7u": "C1779",                  # 0805 25 V X5R
    "2.2u": "C23630",                 # 0603 16 V X5R
    "1u": "C15849",                   # 0603 50 V X5R
    "100n": "C14663",                 # 0603 50 V X7R
    "10n": "C57112",                  # 0603 50 V X7R
    "10k": "C25804",
    "4.7k": "C23162",
    "1k": "C21190",
}
FP_LCSC = {
    "JST_GH_SM03B-GHS-TB_1x03-1MP_P1.25mm_Horizontal": "C514175",
    "JST_GH_SM06B-GHS-TB_1x06-1MP_P1.25mm_Horizontal": "C133065",
    "SW_SPST_PTS810": "C221895",
    "nRF24L01_Breakout": "C239344",              # CJT A2541HWV-2x4P, as on the controller
    "PinHeader_1x05_P2.54mm_Vertical": "C492404",  # PZ254V-11-05P
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
    "SW_SPST_PTS810": ("C&K", "PTS810SJG250SMTRLFS", "Tactile switch SMD"),
    "PinHeader_1x05_P2.54mm_Vertical": ("XFCN", "PZ254V-11-05P", "Pin header 1x05 2.54 mm vertical (ESC)"),
    "nRF24L01_Breakout": ("CJT", "A2541HWV-2x4P", "Female header 2x04 2.54 mm vertical (nRF24L01 module socket)"),
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
        if ref in JLC_DNP:
            continue
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
            w.writerow([i, ",".join(sorted(refs, key=key)), len(refs), mfr, mpn, desc, fp,
                        "THT" if fp.startswith(THT) else "SMD", lcsc])
    return path


# JLCPCB model rotation minus KiCad's, per package. Read off JLCPCB's placement
# viewer on 2026-10-25 (pink pin-1 dot vs the silk pin-1 triangle); anything
# not listed (passives, LGA-14 IMU, GPS, JST-GH, buttons) needed no correction.
ROT_CORRECTION = [
    ("SOT-23", 180),             # SOT-23, SOT-23-5
    ("SOT-223", 180),
    ("LQFP-", 270),
    ("LGA-8_3x5mm", 270),        # MS5611: JLCPCB's footprint lies along X
]
BOTTOM_CORRECTION = [
    ("JST_GH_", 180),            # J5/J6 sat right before the bottom mirroring
]


def jlc_rotation(package, angle, bottom):
    """kicad-jlcpcb-tools convention: bottom angles mirrored (180 - a), then
    the per-package correction."""
    table = ROT_CORRECTION + (BOTTOM_CORRECTION if bottom else [])
    angle = (180 - angle) if bottom else angle
    angle += next((c for pat, c in table if package.startswith(pat)), 0)
    return angle % 360


def tht_placement():
    """ref -> (x, y, rotation) for the through-hole headers. KiCad's origin is
    pin 1 and the row runs along Y; JLCPCB places their model by its centre,
    with the row along X at 0 deg."""
    import pcbnew
    board = pcbnew.LoadBoard(PCB)
    out = {}
    for fp in board.GetFootprints():
        if not str(fp.GetFPID().GetLibItemName()).startswith(THT):
            continue
        pads = list(fp.Pads())
        box = pads[0].GetBoundingBox()
        for pad in pads[1:]:
            box.Merge(pad.GetBoundingBox())
        c = box.GetCenter()
        rot = (fp.GetOrientation().AsDegrees() + 90) % 360
        out[fp.GetReference()] = (pcbnew.ToMM(c.x), -pcbnew.ToMM(c.y), rot)
    return out


def cpl(out=OUT):
    tht = tht_placement()
    raw = os.path.join(out, "pos.csv")
    kicad_cli("pcb", "export", "pos", "-o", raw, "--format", "csv", "--units", "mm",
              "--side", "both", PCB)
    path = os.path.join(out, gen_pcb.PROJECT + "_cpl.csv")
    with open(raw) as src, open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Designator", "Mid X", "Mid Y", "Layer", "Rotation"])
        for row in csv.DictReader(src):
            if row["Ref"].startswith(NOT_ASSEMBLED) or row["Ref"] in JLC_DNP:
                continue
            x, y = row["PosX"], row["PosY"]
            rot = f'{jlc_rotation(row["Package"], float(row["Rot"]), row["Side"] == "bottom"):.6f}'
            if row["Ref"] in tht:
                x, y, rot = (f"{v:.6f}" for v in tht[row["Ref"]])
            w.writerow([row["Ref"], x + "mm", y + "mm",
                        "Top" if row["Side"] == "top" else "Bottom", rot])
    os.remove(raw)
    return path


def main():
    os.makedirs(OUT, exist_ok=True)
    check_drc()
    for path in (gerbers(), bom(), cpl(), pcbway_bom()):
        print("wrote", os.path.relpath(path, HERE))


if __name__ == "__main__":
    main()
