#!/usr/bin/env python3
"""
Write the JLCPCB order files for falcon_fc.kicad_pcb into build/jlcpcb/:

  falcon_fc_gerbers.zip   Gerbers (4 copper layers, mask, paste, silk, outline) + Excellon drill
  falcon_fc_bom.csv       Comment, Designator, Footprint, LCSC Part # (grouped by value + footprint)
  falcon_fc_cpl.csv       Designator, Mid X, Mid Y, Layer, Rotation (both sides)

LCSC Part # is left empty: JLCPCB's BOM tool matches each line by value and
footprint on upload, where the part choice can be checked against stock.

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
            w.writerow([value, ",".join(sorted(refs, key=key)), fp, ""])
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
    for path in (gerbers(), bom(), cpl()):
        print("wrote", os.path.relpath(path, HERE))


if __name__ == "__main__":
    main()
