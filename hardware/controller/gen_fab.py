#!/usr/bin/env python3
"""
JLCPCB fabrication + assembly outputs for the routed falcon_rc board.

Writes build/jlcpcb/ (like the flight controller's export_fab.py):
  falcon_rc_gerbers.zip   gerbers + Excellon drills (upload as the PCB)
  falcon_rc_bom.csv       Comment, Designator, Footprint, LCSC (assembly BOM)
  falcon_rc_cpl.csv       Designator, Val, Package, Mid X, Mid Y, Rotation, Layer
and build/pcbway/falcon_rc_pcbway_bom.csv: the same parts by manufacturer part
number, for ordering from PCBWay with the same gerbers zip and CPL.

Every part is assembled: SMD on the back by reflow, through-hole parts on
the front (sticks, toggles, tact switches, LEDs, sockets) by JLCPCB's THT
service. The one exception is J2 (debug UART pins, DNP in the schematic).

Part numbers were picked from the JLCPCB library on 2026-10-21; `--stock`
re-checks them against JLCPCB's live stock. Parts with no LCSC number
(SW9) are bought by JLCPCB through Global Sourcing: on the BOM page search
the MPN under "Global Sourcing" (or send the part in as consigned stock).

CPL follows the kicad-jlcpcb-tools conventions: part centre = centre of the
pads, Y up, bottom-side angle mirrored (180 - a), plus the per-package
rotation corrections of JLCKicadTools' cpl_rotations_db. Check the rotations
in JLCPCB's placement preview before paying; their engineers review it too.

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
PROJECT = "falcon_rc"
OUT = os.path.join(HERE, "build", "jlcpcb")

# 0603 resistors / 0603 + 0805 MLCCs: all JLCPCB "basic" parts (no setup fee)
RES = {"22": "C23345", "100": "C22775", "1k": "C21190", "1.2k": "C22765", "1.5k": "C22843",
       "4.7k": "C23162", "5.1k": "C23186", "10k": "C25804", "68k": "C23231", "100k": "C25803",
       "150k": "C22807", "220k": "C22961"}
CAP = {"10n": "C57112", "100n": "C14663", "1u": "C15849",              # 50 V X7R / X5R
       "4.7u": "C1779", "10u": "C15850", "22u": "C45783"}             # 0805, 25 V X5R

# ref -> (LCSC, manufacturer part number)
PARTS = {
    "J1": ("C165948", "HRO TYPE-C-31-M-12"),
    "U1": ("C7519", "ST USBLC6-2SC6"),
    "U2": ("C15220", "TI BQ24073RGTR"),
    "U3": ("C351410", "PUOLOP DW01A"),
    "Q1": ("C5148694", "HXY 8205A (TSSOP-8, FS8205A pinout)"),
    "U4": ("C51118", "Diodes AP2112K-3.3TRG1"),
    "SW1": ("C221841", "C&K PCM12SMTR"),
    "FB1": ("C1002", "Sunlord GZ1608D601TF"),
    "U5": ("C2913198", "Espressif ESP32-S3-WROOM-1-N8"),
    "SW2": ("C221895", "C&K PTS810SJG250SMTRLFS"),
    "SW3": ("C221895", "C&K PTS810SJG250SMTRLFS"),
    "Q2": ("C20526", "MMBT3904"),
    "BZ1": ("C95163", "CUI CPT-9019S-SMT-TR"),
    "BT1": ("C5249310", "Keystone 1042"),
    "J3": ("C239344", "CJT A2541HWV-2x4P (nRF24 socket)"),
    "J4": ("C2718488", "BOOMELE 2.54-1*4P female (OLED socket)"),
    "JS1": ("C146170", "Alps RKJXV1224005"),
    "JS2": ("C146170", "Alps RKJXV1224005"),
    "SW4": ("C110153", "HRO K2-1102DP-C4SW-04"),
    "SW5": ("C110153", "HRO K2-1102DP-C4SW-04"),
    "SW6": ("C110153", "HRO K2-1102DP-C4SW-04"),
    "SW7": ("C110153", "HRO K2-1102DP-C4SW-04"),
    "SW8": ("C221549", "C&K 7101SYCQE"),
    "SW9": ("", "C&K 7103SYCQE (Global Sourcing)"),
    "D1": ("C85161", "Everlight 204-10SYGD/S530-E3 green"),
    "D2": ("C84774", "Everlight 204-10SDRD/S530-A3-L red"),
    "D4": ("C85160", "Everlight 204-10UYD/S530-A3-L yellow"),
}

# JLCKicadTools cpl_rotations_db.csv, the patterns this board uses
ROTATIONS = [
    (r"^SOT-23", -90),
    (r"^TSSOP-", 270),
    (r"^(.*?_|V)?QFN-(16|20|24|28|40)(-|_|$)", 270),
    (r"^USB_C_Receptacle_HRO_TYPE-C-31-M-12", 180),
]


def part_for(fp):
    ref, value = fp.GetReference(), fp.GetValue()
    if ref in PARTS:
        return PARTS[ref]
    table = RES if ref.startswith("R") else CAP if ref.startswith("C") else {}
    if value in table:
        return table[value], value
    raise SystemExit(f"{ref} ({value}): no part number; add it to PARTS")


def assembled(fp):
    return not fp.IsDNP() and not fp.IsExcludedFromBOM()


def centre(fp):
    pads = list(fp.Pads())
    if not pads:
        return fp.GetPosition()
    box = pads[0].GetBoundingBox()
    for pad in pads[1:]:
        box.Merge(pad.GetBoundingBox())
    return box.GetCenter()


def rotation(fp):
    angle = fp.GetOrientation().AsDegrees()
    if fp.IsFlipped():
        angle = 180 - angle
    name = str(fp.GetFPID().GetLibItemName())
    hits = [(m.end() - m.start(), corr) for pat, corr in ROTATIONS
            if (m := re.search(pat, name))]
    if hits:
        angle += max(hits)[1]
    return round(angle % 360, 3)


def write_bom(fps):
    groups = {}
    for fp in fps:
        lcsc, mpn = part_for(fp)
        key = (lcsc or mpn, mpn, str(fp.GetFPID().GetLibItemName()))
        groups.setdefault(key, []).append(fp.GetReference())
    for refs in groups.values():
        refs.sort(key=natural)
    path = os.path.join(OUT, PROJECT + "_bom.csv")
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Comment", "Designator", "Footprint", "LCSC"])
        for (code, mpn, pkg), refs in sorted(groups.items(), key=lambda kv: natural(kv[1][0])):
            w.writerow([mpn, ",".join(refs), pkg, code if code != mpn else ""])
    return path, groups


def write_pcbway_bom(fps):
    """PCBWay's BOM layout: manufacturer + MPN per line (PARTS keeps them as one
    "Maker MPN (note)" string), passives by description, LCSC as a cross-reference."""
    groups = {}
    for fp in fps:
        lcsc, mpn = part_for(fp)
        pkg = str(fp.GetFPID().GetLibItemName())
        groups.setdefault((lcsc, mpn, pkg), []).append(fp)
    path = os.path.join(HERE, "build", "pcbway", PROJECT + "_pcbway_bom.csv")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Item", "Designator", "Qty", "Manufacturer", "Manufacturer Part Number",
                    "Description", "Package", "Type", "Notes", "LCSC (reference)"])
        lines = sorted(groups.items(), key=lambda kv: natural(min(fp.GetReference() for fp in kv[1])))
        for i, ((lcsc, mpn, pkg), group) in enumerate(lines, 1):
            refs = sorted((fp.GetReference() for fp in group), key=natural)
            ref = refs[0]
            if ref in PARTS:
                words = mpn.split(" ", 2) if " " in mpn else ["", mpn]
                maker, number = (words[0], words[1]) if len(words) > 1 else ("", words[0])
                note, desc = (words[2] if len(words) > 2 else ""), group[0].GetValue()
                note = "" if note == "(Global Sourcing)" else note     # JLCPCB-only remark
            else:
                size = pkg.split("_")[1]
                maker = number = note = ""
                desc = (f"Resistor {mpn}Ω 1% {size}" if ref.startswith("R")
                        else f"Capacitor {mpn}F MLCC X5R/X7R 16 V or higher {size}")
            tht = not group[0].GetAttributes() & pcbnew.FP_SMD
            w.writerow([i, ",".join(refs), len(refs), maker, number, desc, pkg,
                        "THT" if tht else "SMD", note, lcsc])
    return path


def write_cpl(fps):
    path = os.path.join(OUT, PROJECT + "_cpl.csv")
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Designator", "Val", "Package", "Mid X", "Mid Y", "Rotation", "Layer"])
        for fp in sorted(fps, key=lambda fp: natural(fp.GetReference())):
            c = centre(fp)
            w.writerow([fp.GetReference(), fp.GetValue(), str(fp.GetFPID().GetLibItemName()),
                        f"{pcbnew.ToMM(c.x):.4f}", f"{-pcbnew.ToMM(c.y):.4f}",
                        rotation(fp), "bottom" if fp.IsFlipped() else "top"])
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
    for (code, value, _), refs in sorted(groups.items()):     # value = MPN
        if not code.startswith("C") or not code[1:].isdigit():
            print(f"  {code:<34} global sourcing ({','.join(refs)})")
            continue
        req = urllib.request.Request(url, headers={"Content-Type": "application/json",
                                                   "User-Agent": "Mozilla/5.0"},
                                     data=json.dumps({"keyword": code, "currentPage": 1,
                                                      "pageSize": 5}).encode())
        data = json.load(urllib.request.urlopen(req, timeout=30))["data"]["componentPageInfo"]["list"]
        hit = next((c for c in data if c["componentCode"] == code), None)
        need = len(refs) * boards
        if hit is None:
            print(f"  {code:<9} {value:<16} NOT FOUND")
            continue
        stock, kind = hit["stockCount"], hit["componentLibraryType"]
        flag = "" if stock >= need else f"  <-- short for {boards} boards (need {need})"
        print(f"  {code:<9} {value:<16} {kind:<6} stock {stock:<9}{flag}")


def natural(ref):
    m = re.match(r"([A-Za-z]+)(\d+)", ref)
    return (m.group(1), int(m.group(2))) if m else (ref, 0)


def main():
    pcb = os.path.join(HERE, PROJECT + ".kicad_pcb")
    board = pcbnew.LoadBoard(pcb)
    if not len(board.GetTracks()):
        raise SystemExit(f"{pcb} is not routed; run route_pcb.py first")
    os.makedirs(OUT, exist_ok=True)
    fps = [fp for fp in board.GetFootprints() if assembled(fp)]
    zpath, files = write_gerbers(pcb)
    bom, groups = write_bom(fps)
    cpl = write_cpl(fps)
    pcbway = write_pcbway_bom(fps)
    smd = sum(1 for fp in fps if fp.GetAttributes() & pcbnew.FP_SMD)
    print(f"wrote {zpath} ({len(files)} files)\nwrote {bom}: {len(groups)} lines\n"
          f"wrote {cpl}: {len(fps)} parts ({smd} SMD, {len(fps) - smd} THT)\nwrote {pcbway}")
    skipped = [fp.GetReference() for fp in board.GetFootprints() if not assembled(fp)]
    print(f"not assembled: {', '.join(sorted(skipped, key=natural))}")
    if "--stock" in sys.argv:
        check_stock(groups)


if __name__ == "__main__":
    main()
