#!/usr/bin/env python3
"""
Create falcon_rc_carrier.kicad_pcb: board outline, 2-layer stackup, JLCPCB
rules, GND pours, mounting holes and the part placement.

Board: 150 x 100 mm, held in both hands. Coordinates: x 50..200,
y 50..150, north (y = 50) is the edge facing away from the pilot.
  Top    : ESP32 DevKit north-centre on two 1x15 sockets, USB-C at the north
           edge, antenna end over a copper keepout. OLED socket below it,
           the module covering the centre. KY-023 sticks lower left / right
           (80 mm apart), flat on four M3 screws each, wired to a 5-hole row
           south of the module. nRF24 north-east with its antenna past the
           north edge. ARM top-left, MODE top-right.
  Bottom : the RC power board (hardware/rc_power) on four 11 mm M3
           standoffs, plugged into the 1x4 socket J10. It covers x 50..152,
           y 98..142 with its USB-C on the west edge (it is mirrored: power
           board x maps to 202 - x here, y to y + 48).
  SMD    : only the 0805 resistors and capacitors, all on top (JLCPCB).
Layers : F.Cu signals + GND pour | B.Cu signals + GND pour.

KY-023 hole pattern 20.32 x 26.67 mm (measured against the DevKit's pin
pitch: 8 and 10.5 pitches), stick assumed at its centre.

Usage:  python3 gen_pcb.py [--force]
"""

import json
import math
import os
import subprocess
import sys

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import schgen as sch  # noqa: E402  (reuses the s-expression parser)

PROJECT = "falcon_rc_carrier"
FP_DIR = "/usr/share/kicad/footprints"
X0, Y0, W, H = 50.0, 50.0, 150.0, 100.0
CORNER_R = 6.0
HOLE_INSET = 4.0               # corner hole centre from each edge

ESP_X, ESP_Y = 125.0, 58.7     # DevKit centre line, first pin row (3V3 / VIN)
STICK_L, STICK_R, STICK_Y = 80.0, 170.0, 112.0
STICK_DX, STICK_DY = 20.32 / 2, 26.67 / 2
ANTENNA = (110.5, 95.0, 139.5, 101.0)   # DevKit PCB antenna, no copper below

mm = pcbnew.FromMM

# ref: (x, y, rotation_deg, side). Coordinates in mm, KiCad frame (y down),
# as seen from the top. Rotation is counter-clockwise as shown in KiCad;
# a 1xN header at 90 runs east from pin 1.
PLACEMENT = {
    "J1": (ESP_X - 12.7, ESP_Y, 0, "top"),
    "J2": (ESP_X + 12.7, ESP_Y, 0, "top"),
    "C2": (143.0, 60.5, 90, "top"),          # at VIN
    "R1": (144.0, 87.0, 90, "top"),          # battery sense, next to VP (J2.14)
    "R2": (144.0, 91.5, 90, "top"),
    "C1": (147.5, 91.5, 90, "top"),
    # OLED: 1x4 socket, GND VCC SCL SDA west to east, module hanging south
    "J7": (ESP_X - 3.81, 105.0, 90, "top"),
    "C5": (133.5, 105.0, 90, "top"),
    # nRF24: pins south, module and antenna north past the edge
    "J6": (152.0, 68.0, 90, "top"),
    "C3": (156.0, 73.0, 0, "top"),
    "C4": (156.0, 76.0, 0, "top"),
    # sticks: 5-hole rows south of the modules, GND +V X Y SW west to east
    "J8": (STICK_L - 5.08, 134.0, 90, "top"),
    "J9": (STICK_R - 5.08, 134.0, 90, "top"),
    "H5": (STICK_L - STICK_DX, STICK_Y - STICK_DY, 0, "top"),
    "H6": (STICK_L + STICK_DX, STICK_Y - STICK_DY, 0, "top"),
    "H7": (STICK_L - STICK_DX, STICK_Y + STICK_DY, 0, "top"),
    "H8": (STICK_L + STICK_DX, STICK_Y + STICK_DY, 0, "top"),
    "H9": (STICK_R - STICK_DX, STICK_Y - STICK_DY, 0, "top"),
    "H10": (STICK_R + STICK_DX, STICK_Y - STICK_DY, 0, "top"),
    "H11": (STICK_R - STICK_DX, STICK_Y + STICK_DY, 0, "top"),
    "H12": (STICK_R + STICK_DX, STICK_Y + STICK_DY, 0, "top"),
    # switches
    "SW2": (62.0, 57.0, 90, "top"),          # ARM
    "SW3": (180.0, 57.0, 90, "top"),         # MODE
    # power board: socket on the back, pin 1 (+5V) east; standoffs = its
    # holes (53.3 / 148.7, 53.5 / 90.5) mirrored
    "J10": (122.0, 138.0, 270, "bottom"),
    "H13": (148.7, 101.5, 0, "top"),
    "H14": (53.3, 101.5, 0, "top"),
    "H15": (148.7, 138.5, 0, "top"),
    "H16": (53.3, 138.5, 0, "top"),
}

# JLCPCB 2-layer standard capabilities (no extra-cost options), with margin.
RULES = {
    "min_clearance": 0.127,
    "min_track_width": 0.127,
    "min_via_diameter": 0.45,
    "min_through_hole_diameter": 0.3,
    "min_via_annular_width": 0.075,
    "min_hole_to_hole": 0.25,
    "min_copper_edge_clearance": 0.3,
    "min_silk_clearance": 0.0,
    "min_text_height": 0.8,
    "min_text_thickness": 0.15,
}
NETCLASSES = [
    # name, track, clearance, via dia, via drill, net patterns
    ("Default", 0.25, 0.2, 0.6, 0.3, []),
    # battery / 5 V: ESP32 + nRF24 peaks ~0.5 A at 5 V, ~0.8 A from the cell
    ("Power", 0.6, 0.2, 0.8, 0.4, ["+5V", "+3V3", "GND"]),
    ("Analog", 0.25, 0.2, 0.6, 0.3, ["STK_*", "VBAT_SENSE"]),
]

def fp_dir(lib):
    return f"{FP_DIR}/{lib}.pretty"


def load_netlist():
    path = os.path.join(HERE, "build", PROJECT + ".net")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    subprocess.run(["kicad-cli", "sch", "export", "netlist", "-o", path,
                    os.path.join(HERE, PROJECT + ".kicad_sch")], check=True, capture_output=True)
    t = sch.parse_sexp(open(path).read())
    comps = {}
    for c in sch.find(sch.find1(t, "components"), "comp"):
        dnp = any(str(sch.find1(prop, "name")[1]) == "dnp" for prop in sch.find(c, "property"))
        comps[str(sch.find1(c, "ref")[1])] = (str(sch.find1(c, "value")[1]),
                                              str(sch.find1(c, "footprint")[1]), dnp)
    nets = {}
    for n in sch.find(sch.find1(t, "nets"), "net"):
        name = str(sch.find1(n, "name")[1])           # local labels come as "/NAME"
        nets[name] = [(str(sch.find1(x, "ref")[1]), str(sch.find1(x, "pin")[1]))
                      for x in sch.find(n, "node")]
    return comps, nets


def outline_points(inset=0.0, arc_steps=8):
    """Rounded rectangle as a polygon (for zones)."""
    r = CORNER_R - inset
    x0, y0, x1, y1 = X0 + inset, Y0 + inset, X0 + W - inset, Y0 + H - inset
    pts = []
    for cx, cy, a0 in ((x1 - r, y0 + r, 270), (x1 - r, y1 - r, 0), (x0 + r, y1 - r, 90), (x0 + r, y0 + r, 180)):
        for k in range(arc_steps + 1):
            a = math.radians(a0 + 90 * k / arc_steps)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


def add_outline(board):
    x0, y0, x1, y1 = X0, Y0, X0 + W, Y0 + H
    r = CORNER_R

    def seg(a, b):
        s = pcbnew.PCB_SHAPE(board, pcbnew.SHAPE_T_SEGMENT)
        s.SetStart(pcbnew.VECTOR2I(mm(a[0]), mm(a[1])))
        s.SetEnd(pcbnew.VECTOR2I(mm(b[0]), mm(b[1])))
        s.SetLayer(pcbnew.Edge_Cuts)
        s.SetWidth(mm(0.05))
        board.Add(s)

    def arc(cx, cy, start_deg):
        s = pcbnew.PCB_SHAPE(board, pcbnew.SHAPE_T_ARC)
        s.SetCenter(pcbnew.VECTOR2I(mm(cx), mm(cy)))
        a = math.radians(start_deg)
        s.SetStart(pcbnew.VECTOR2I(mm(cx + r * math.cos(a)), mm(cy + r * math.sin(a))))
        s.SetArcAngleAndEnd(pcbnew.EDA_ANGLE(90, pcbnew.DEGREES_T), True)
        s.SetLayer(pcbnew.Edge_Cuts)
        s.SetWidth(mm(0.05))
        board.Add(s)

    seg((x0 + r, y0), (x1 - r, y0))
    seg((x1, y0 + r), (x1, y1 - r))
    seg((x1 - r, y1), (x0 + r, y1))
    seg((x0, y1 - r), (x0, y0 + r))
    arc(x1 - r, y0 + r, 270)
    arc(x1 - r, y1 - r, 0)
    arc(x0 + r, y1 - r, 90)
    arc(x0 + r, y0 + r, 180)


def add_zone(board, layer, net, priority=0):
    z = pcbnew.ZONE(board)
    z.SetLayer(layer)
    z.SetNet(net)
    z.SetAssignedPriority(priority)
    z.SetMinThickness(mm(0.2))
    z.SetLocalClearance(mm(0.25))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
    z.SetThermalReliefGap(mm(0.3))
    z.SetThermalReliefSpokeWidth(mm(0.4))
    ol = z.Outline()
    ol.NewOutline()
    for x, y in outline_points(0.3):
        ol.Append(mm(x), mm(y))
    board.Add(z)
    return z


def add_text(board, text, x, y, bottom=False, angle=0, size=1.0):
    txt = pcbnew.PCB_TEXT(board)
    txt.SetText(text)
    txt.SetPosition(pcbnew.VECTOR2I(mm(x), mm(y)))
    txt.SetLayer(pcbnew.B_SilkS if bottom else pcbnew.F_SilkS)
    txt.SetMirrored(bottom)
    txt.SetTextAngleDegrees(angle)
    txt.SetTextSize(pcbnew.VECTOR2I(mm(size), mm(size)))
    txt.SetTextThickness(mm(0.15 if size < 1.2 else 0.25))
    board.Add(txt)


def add_line(board, a, b, layer=pcbnew.F_SilkS, width=0.15):
    s = pcbnew.PCB_SHAPE(board, pcbnew.SHAPE_T_SEGMENT)
    s.SetStart(pcbnew.VECTOR2I(mm(a[0]), mm(a[1])))
    s.SetEnd(pcbnew.VECTOR2I(mm(b[0]), mm(b[1])))
    s.SetLayer(layer)
    s.SetWidth(mm(width))
    board.Add(s)



def add_rect(board, x0, y0, x1, y1, layer=pcbnew.F_SilkS):
    for a, b in (((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))):
        add_line(board, a, b, layer)


def add_silkscreen(board):
    """Module outlines and labels for the person assembling the board."""
    for fp in board.Footprints():
        if fp.GetReference().startswith(("J", "SW", "H")):
            fp.Reference().SetVisible(False)       # the labels below name them
    # nRF24: the footprint outlines the whole module, which overhangs the north edge
    j6 = board.FindFootprintByReference("J6")
    for item in list(j6.GraphicalItems()):
        if item.GetLayer() in (pcbnew.F_SilkS, pcbnew.B_SilkS) and \
                pcbnew.ToMM(item.GetBoundingBox().GetTop()) < Y0 + 0.3:
            j6.Remove(item)
    for ref in ("C3", "C4"):                    # stacked pair: labels to the east
        fp = board.FindFootprintByReference(ref)
        fp.Reference().SetPosition(fp.GetPosition() + pcbnew.VECTOR2I(mm(3.2), 0))
    # module outlines
    add_rect(board, ESP_X - 14.7, Y0 + 0.6, ESP_X + 14.7, ANTENNA[3])
    add_text(board, "ESP32 DevKit  (USB-C up)", ESP_X, 77.0, size=0.9)
    add_text(board, "antenna: keep clear", ESP_X, 98.0, size=0.8)
    add_rect(board, ESP_X - 17.7, 102.5, ESP_X + 17.7, 136.0)
    add_text(board, "OLED", ESP_X, 120.0, size=1.2)
    add_text(board, "GND VCC SCL SDA", ESP_X, 107.6, size=0.8)
    for cx, name in ((STICK_L, "LEFT: THR / YAW"), (STICK_R, "RIGHT: PITCH / ROLL")):
        add_rect(board, cx - 13.0, STICK_Y - 17.0, cx + 13.0, STICK_Y + 17.0)
        add_text(board, name, cx, STICK_Y - 18.6, size=0.9)
        add_text(board, "G  V  X  Y  SW", cx, 136.6, size=0.8)
    add_text(board, "nRF24", 160.0, 80.0, size=0.9)
    add_text(board, "ARM", 64.5, 60.0, size=1.0)
    add_text(board, "MODE", 182.5, 60.0, size=1.0)
    add_text(board, "FalconDrone RC carrier rev A", ESP_X, 146.0, size=1.0)
    # back: the power board's outline and socket
    add_rect(board, 50.6, 98.6, 151.4, 141.4, layer=pcbnew.B_SilkS)
    add_text(board, "RC power board  (USB-C this edge)", 101.0, 120.0, bottom=True, size=1.2)
    add_text(board, "5V G VB KP", 118.2, 135.6, bottom=True, size=0.8)


def add_keepout(board, rect):
    """No copper at all under the DevKit's PCB antenna (both layers)."""
    z = pcbnew.ZONE(board)
    z.SetIsRuleArea(True)
    z.SetDoNotAllowTracks(True)
    z.SetDoNotAllowVias(True)
    z.SetDoNotAllowPads(False)
    z.SetDoNotAllowCopperPour(True)
    z.SetDoNotAllowFootprints(False)
    ls = pcbnew.LSET()
    ls.AddLayer(pcbnew.F_Cu)
    ls.AddLayer(pcbnew.B_Cu)
    z.SetLayerSet(ls)
    ol = z.Outline()
    ol.NewOutline()
    x0, y0, x1, y1 = rect
    for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
        ol.Append(mm(x), mm(y))
    board.Add(z)


def place(fp, x, y, rot, side):
    if side == "bottom":
        fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_LEFT_RIGHT)
        rot += 180
    fp.SetPosition(pcbnew.VECTOR2I(mm(x), mm(y)))
    fp.SetOrientationDegrees(rot % 360)


def patch_project_rules():
    path = os.path.join(HERE, PROJECT + ".kicad_pro")
    pro = json.load(open(path)) if os.path.exists(path) else {
        "meta": {"filename": PROJECT + ".kicad_pro", "version": 3}}
    ds = pro.setdefault("board", {}).setdefault("design_settings", {})
    ds.setdefault("rules", {}).update(RULES)
    ns = pro.setdefault("net_settings", {})
    ns["classes"] = [{
        "name": name, "track_width": tw, "clearance": cl, "via_diameter": vd, "via_drill": vdr,
        "microvia_diameter": 0.3, "microvia_drill": 0.1, "diff_pair_width": 0.3,
        "diff_pair_gap": 0.15, "diff_pair_via_gap": 0.25, "wire_width": 6, "bus_width": 12,
        "line_style": 0, "pcb_color": "rgba(0, 0, 0, 0.000)",
        "schematic_color": "rgba(0, 0, 0, 0.000)", "priority": 2147483647 if name == "Default" else i,
    } for i, (name, tw, cl, vd, vdr, _) in enumerate(NETCLASSES)]
    ns["netclass_patterns"] = [{"netclass": name, "pattern": "/" + pat}
                               for name, *_, pats in NETCLASSES for pat in pats]
    ns.setdefault("meta", {"version": 4})
    with open(path, "w") as f:
        json.dump(pro, f, indent=2)
        f.write("\n")


def main():
    out = os.path.join(HERE, PROJECT + ".kicad_pcb")
    if os.path.exists(out) and "--force" not in sys.argv:
        old = pcbnew.LoadBoard(out)
        if len(old.GetTracks()):
            raise SystemExit(f"{out} already has tracks; edit it in KiCad (or pass --force)")

    comps, nets = load_netlist()
    board = pcbnew.CreateEmptyBoard()
    board.SetCopperLayerCount(2)
    board.GetDesignSettings().SetBoardThickness(mm(1.6))

    netinfo = {}
    for name in nets:
        ni = pcbnew.NETINFO_ITEM(board, name)
        board.Add(ni)
        netinfo[name] = ni
    pad_net = {(r, p): name for name, nodes in nets.items() for r, p in nodes}

    add_outline(board)

    holes = [(X0 + HOLE_INSET, Y0 + HOLE_INSET), (X0 + W - HOLE_INSET, Y0 + HOLE_INSET),
             (X0 + HOLE_INSET, Y0 + H - HOLE_INSET), (X0 + W - HOLE_INSET, Y0 + H - HOLE_INSET)]
    sockets = {"J1": "PinSocket_1x15_P2.54mm_Vertical", "J2": "PinSocket_1x15_P2.54mm_Vertical",
               "J7": "PinSocket_1x04_P2.54mm_Vertical", "J10": "PinSocket_1x04_P2.54mm_Vertical"}
    unplaced = []
    for ref, (value, fpid, dnp) in sorted(comps.items()):
        lib, name = fpid.split(":")
        fp = pcbnew.FootprintLoad(fp_dir(lib), name)
        fp.SetReference(ref)
        fp.SetValue(value)
        fp.SetFPID(pcbnew.LIB_ID(lib, name))
        fp.SetDNP(dnp)
        board.Add(fp)
        for pad in fp.Pads():
            net = pad_net.get((ref, pad.GetNumber()))
            if net:
                pad.SetNet(netinfo[net])
        if ref in sockets:
            # female sockets go in these holes: show one in the 3D view
            fp.Models()[0].m_Filename = ("${KICAD9_3DMODEL_DIR}/Connector_PinSocket_2.54mm.3dshapes/"
                                         + sockets[ref] + ".step")
        if ref in PLACEMENT:
            place(fp, *PLACEMENT[ref])
        elif ref.startswith("H") and ref[1:].isdigit() and int(ref[1:]) <= 4:
            place(fp, *holes[int(ref[1:]) - 1], 0, "top")
        else:
            unplaced.append(ref)

    add_keepout(board, ANTENNA)
    add_zone(board, pcbnew.F_Cu, netinfo["/GND"])
    add_zone(board, pcbnew.B_Cu, netinfo["/GND"])
    add_silkscreen(board)

    board.Save(out)
    # ZONE_FILLER segfaults on a board built in memory, but works once the
    # board has been loaded from disk.
    board = pcbnew.LoadBoard(out)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    board.Save(out)
    patch_project_rules()
    print(f"wrote {out}: {len(comps)} parts" + (f", UNPLACED: {unplaced}" if unplaced else ""))


if __name__ == "__main__":
    main()
