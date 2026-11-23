#!/usr/bin/env python3
"""
JLCPCB fabrication + assembly outputs for the routed nRF24 adapter board.

Writes build/jlcpcb/:
  nrf24_adapter_gerbers.zip   gerbers + Excellon drills (upload as the PCB)
  nrf24_adapter_bom.csv       Comment, Designator, Footprint, LCSC (assembly BOM)
  nrf24_adapter_cpl.csv       Designator, Val, Package, Mid X, Mid Y, Rotation, Layer
and build/hand_solder.csv: the through-hole parts left for hand assembly.

JLCPCB assembles the 0805 resistors and capacitors only (top side, one
reflow pass, all basic parts). The 2x4 socket and the 1x8 header are
hand-soldered afterwards.

Part numbers were picked from the JLCPCB library on 2026-10-26; `--stock`
re-checks them against JLCPCB's live stock.

CPL follows the kicad-jlcpcb-tools conventions: part centre = centre of the
pads, Y up. Two-terminal parts only, so nothing can be placed backwards.

Usage:  python3 gen_fab.py [--stock]
"""

import csv
import json
import os
import re
import subprocess
import sys
import urllib.request
import zipfile

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT = "nrf24_adapter"
OUT = os.path.join(HERE, "build", "jlcpcb")

# 0805: all JLCPCB "basic" parts (no setup fee)
RES = {"100k": "C149504"}
CAP = {"100n": "C49678", "1u": "C28323", "10u": "C15850", "22u": "C45783"}   # 25-50 V X5R/X7R

HAND_SMD = set()

# ref -> (LCSC, manufacturer part number)
PARTS = {}

ROTATIONS = []          # 0805 R/C only: KiCad and JLCPCB agree


def part_for(fp):
    ref, value = fp.GetReference(), fp.GetValue()
    if ref in PARTS:
        return PARTS[ref]
    table = RES if ref.startswith("R") else CAP if ref.startswith("C") else {}
    if value in table:
        return table[value], value
    raise SystemExit(f"{ref} ({value}): no part number; add it to PARTS")


def smd(fp):
    return bool(fp.GetAttributes() & pcbnew.FP_SMD)


def centre(fp):
    pads = list(fp.Pads())
    box = pads[0].GetBoundingBox()
    for pad in pads[1:]:
        box.Merge(pad.GetBoundingBox())
    return box.GetCenter()


def rotation(fp):
    angle = fp.GetOrientation().AsDegrees()
    name = str(fp.GetFPID().GetLibItemName())
    hits = [(m.end() - m.start(), corr) for pat, corr in ROTATIONS
            if (m := re.search(pat, name))]
    if hits:
        angle += max(hits)[1]
    return round(angle % 360, 3)


def natural(ref):
    m = re.match(r"([A-Za-z]+)(\d+)", ref)
    return (m.group(1), int(m.group(2))) if m else (ref, 0)


def write_bom(fps):
    groups = {}
    for fp in fps:
        lcsc, mpn = part_for(fp)
        groups.setdefault((lcsc, mpn, str(fp.GetFPID().GetLibItemName())), []).append(fp.GetReference())
    for refs in groups.values():
        refs.sort(key=natural)
    path = os.path.join(OUT, PROJECT + "_bom.csv")
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Comment", "Designator", "Footprint", "LCSC"])
        for (code, mpn, pkg), refs in sorted(groups.items(), key=lambda kv: natural(kv[1][0])):
            w.writerow([mpn, ",".join(refs), pkg, code])
    return path, groups


def write_cpl(fps):
    path = os.path.join(OUT, PROJECT + "_cpl.csv")
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Designator", "Val", "Package", "Mid X", "Mid Y", "Rotation", "Layer"])
        for fp in sorted(fps, key=lambda fp: natural(fp.GetReference())):
            c = centre(fp)
            w.writerow([fp.GetReference(), fp.GetValue(), str(fp.GetFPID().GetLibItemName()),
                        f"{pcbnew.ToMM(c.x):.4f}", f"{-pcbnew.ToMM(c.y):.4f}", rotation(fp), "top"])
    return path


def write_hand_list(fps):
    groups = {}
    for fp in fps:
        key = (fp.GetValue(), str(fp.GetFPID().GetLibItemName()))
        groups.setdefault(key, []).append(fp.GetReference())
    path = os.path.join(HERE, "build", "hand_solder.csv")
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Qty", "Value", "Footprint", "References"])
        for (value, pkg), refs in sorted(groups.items(), key=lambda kv: natural(min(kv[1], key=natural))):
            w.writerow([len(refs), value, pkg, " ".join(sorted(refs, key=natural))])
    return path


def write_gerbers(pcb):
    gdir = os.path.join(HERE, "build", "gerbers")
    os.makedirs(gdir, exist_ok=True)
    for f in os.listdir(gdir):
        os.remove(os.path.join(gdir, f))
    layers = "F.Cu,B.Cu,F.Paste,B.Paste,F.Silkscreen,B.Silkscreen,F.Mask,B.Mask,Edge.Cuts"
    subprocess.run(["kicad-cli", "pcb", "export", "gerbers", "-o", gdir, "-l", layers,
                    "--subtract-soldermask", "--no-x2", pcb], check=True, capture_output=True)
    subprocess.run(["kicad-cli", "pcb", "export", "drill", "-o", gdir + "/", "--format", "excellon",
                    "--excellon-separate-th", "--excellon-units", "mm", pcb],
                   check=True, capture_output=True)
    path = os.path.join(OUT, PROJECT + "_gerbers.zip")
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        for f in sorted(os.listdir(gdir)):
            z.write(os.path.join(gdir, f), f)
    return path, sorted(os.listdir(gdir))


def check_stock(groups, boards=5):
    """Ask JLCPCB's parts search for each LCSC number's live stock."""
    url = "https://jlcpcb.com/api/overseas-pcb-order/v1/shoppingCart/smtGood/selectSmtComponentList"
    for (code, value, _), refs in sorted(groups.items()):
        req = urllib.request.Request(url, headers={"Content-Type": "application/json",
                                                   "User-Agent": "Mozilla/5.0"},
                                     data=json.dumps({"keyword": code, "currentPage": 1,
                                                      "pageSize": 5}).encode())
        data = json.load(urllib.request.urlopen(req, timeout=30))["data"]["componentPageInfo"]["list"]
        hit = next((c for c in data or [] if c["componentCode"] == code), None)
        need = len(refs) * boards
        if hit is None:
            print(f"  {code:<9} {value:<30} NOT FOUND")
            continue
        stock, kind = hit["stockCount"], hit["componentLibraryType"]
        flag = "" if stock >= need else f"  <-- short for {boards} boards (need {need})"
        print(f"  {code:<9} {value:<30} {kind:<6} stock {stock:<9}{flag}")


def main():
    pcb = os.path.join(HERE, PROJECT + ".kicad_pcb")
    board = pcbnew.LoadBoard(pcb)
    if not len(board.GetTracks()):
        raise SystemExit(f"{pcb} is not routed; run route_pcb.py first")
    os.makedirs(OUT, exist_ok=True)
    parts = [fp for fp in board.GetFootprints() if fp.Pads() and not fp.GetReference().startswith("H")]
    fab = [fp for fp in parts if smd(fp) and fp.GetReference() not in HAND_SMD]
    hand = [fp for fp in parts if fp not in fab]
    zpath, files = write_gerbers(pcb)
    bom, groups = write_bom(fab)
    cpl = write_cpl(fab)
    hand_path = write_hand_list(hand)
    print(f"wrote {zpath} ({len(files)} files)\nwrote {bom}: {len(groups)} lines\n"
          f"wrote {cpl}: {len(fab)} SMD parts\nwrote {hand_path}: {len(hand)} hand-soldered parts")
    if "--stock" in sys.argv:
        check_stock(groups)


if __name__ == "__main__":
    main()
