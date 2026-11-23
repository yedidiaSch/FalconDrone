#!/usr/bin/env python3
"""
3D-printable enclosures for the two FalconDrone boards, built in CadQuery from
the part positions in the .kicad_pcb files (so moving a part on the board and
rerunning this script moves its cut-out too).

Handheld controller (falcon_rc, 150 x 100 mm):
  rc_top      lid over the front side. Holes for the sticks, the 4 menu
              buttons (printed caps), LED light tubes, OLED window, toggle
              bushings in recessed cups (deep enough for the nut), and a slot
              for the nRF24's SMA out of the north wall.
  rc_bottom   tray under the back side, deep enough for the 18650 holder.
              USB-C and power-slide openings, paperclip tubes for RESET / BOOT,
              buzzer grille. M3 counterbores from below.
  rc_battery_door  flush door over the 18650: its west edge is bevelled and
              hooks under the floor, its east end is a tab in a rebate held by
              one M3 x 6 screw (self-tapping into the printed boss). To open:
              remove the screw, lift the tab with a fingernail, slide east.
  rc_buttons  4 button caps; they ride in the lid holes and press the
              5 mm tact switches.
  Assembly: 4 x M3 heat-set inserts (M3 x 5.7, d4.0 hole) in the lid bosses,
  4 x M3 x 25 socket-head screws from below. Bosses clamp the PCB at its holes.

Flight controller (falcon_fc, 45 x 45 mm on the 30.5 mm stack):
  fc_canopy   cover over the top side and the nRF24 module that overhangs the
              south edge. Openings: POWER_IN (north), AUX (west), ESC header
              (open from above, so the plug goes in vertically), SMA slot
              (south), LED window and CAL button pin-hole. FWD arrow on the roof.
  fc_tray     base under the board: SWD + TELEMETRY openings (north), RESET
              pin-hole. Sits on the frame.
  The board floats on its rubber grommets between the canopy tubes and the
  tray bosses (1 mm gap each side), so the soft mount still isolates the IMU;
  the enclosure itself is bolted rigidly by the stack screws (4 x M3 x 35 to
  the frame, or the length your stack needs).

Outputs in build/: one STL per part in print orientation (no supports needed:
lids and canopy print roof-down, trays floor-down, button caps top-down),
STEP files of each assembly with the board for fit checks in FreeCAD / KiCad,
and renders (<board>_top / _bottom / _exploded.png; dark blocks stand in for
parts with no KiCad 3D model). --check intersects every printed part with the board's 3D
model (kicad-cli STEP export) plus keep-out blocks for the parts that have no
3D model (sticks, toggles, OLED, nRF24, GPS, ...), and fails on any overlap.

Heights not in the KiCad models are from datasheets; check them against the
real parts before printing (see PART HEIGHTS below).

Usage:  xvfb-run -a ~/.venvs/cad/bin/python gen_enclosure.py [--check]
        (a venv with cadquery + pyvista, created with --system-site-packages
        so that KiCad's pcbnew module is importable; xvfb-run only for the
        off-screen renders on a machine without a display)
"""

import math
import os
import subprocess
import sys

import cadquery as cq
import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
HW = os.path.dirname(HERE)
BUILD = os.path.join(HERE, "build")
RC_PCB = os.path.join(HW, "controller", "falcon_rc.kicad_pcb")
FC_PCB = os.path.join(HW, "flight_controller", "falcon_fc.kicad_pcb")

PCB_T = 1.6            # board thickness; model z = 0 at the board's back face
CLR = 0.2              # printed-fit clearance (lap joint, holes for parts)

# ---------------- PART HEIGHTS (mm above / below the board) ----------------
# from the KiCad 3D models where they exist, else datasheets
LED_H = 8.3            # 3 mm LED body top (KiCad model)
TACT_H = 5.0           # 6 mm tact switch, H5 (KiCad model)
TOGGLE_BODY_H = 12.7   # C&K 7101 body top, C terminals (.500")
TOGGLE_BUSH_D = 6.35   # 1/4-40 bushing
NRF_H = 16.0           # nRF24 PA+LNA on an 8.5 mm socket, to the top of its shield
NRF_SMA_Z = 11.8       # SMA centre (edge-launch, module PCB mid-plane)
OLED_GLASS_H = 13.4    # 0.96" module on its 8.5 mm socket
BATT_DEPTH = 20.0      # 18650 + Keystone 1042 below the board (model: 16.4 holder)
GPS_H = 6.3            # SAM-M10Q

# ---------------- controller ----------------
RC_WALL = 2.0
RC_GAP = 0.5           # board edge to inner wall
RC_TOP_CLEAR = 17.0    # board top to lid underside (nRF24 + OLED fit under it)
RC_LID = 2.0
RC_BOT_CLEAR = 22.0    # board back to floor top
RC_FLOOR = 2.0
RC_LIP = 1.5           # lap joint height
RC_BOSS_D = 7.0
RC_INSERT_D = 4.0      # M3 x 5.7 heat-set insert
RC_INSERT_DEPTH = 7.0
RC_SCREW_LEN = 25.0
RC_STICK_HOLE_D = 26.0 # thumb stick swing (+/-25 deg) with the cap's skirt
RC_BTN_HOLE_D = 6.6
RC_OLED_WIN = (24.0, 13.0)   # active area 21.7 x 10.9
RC_OLED_WIN_OFS = (0.0, 11.5)  # window centre from the socket (east, south)
RC_DOOR_HALF_LEN = 40.0  # battery opening, each side of the holder centre (cell 65)
RC_DOOR_CLR = 0.25
RC_DOOR_TAB_T = 1.2    # screw tab thickness (sits in a rebate in the floor)

# ---------------- flight controller ----------------
FC_WALL = 1.4
FC_GAP = 1.0           # the board floats: keep clear of it all round
FC_TOP_CLEAR = 16.5    # board top to roof underside (nRF24 on its socket)
FC_ROOF = 1.4
FC_BOT_CLEAR = 5.5     # board back to floor top (JST-GH on the back: 4.3)
FC_FLOOR = 1.2
FC_LIP = 1.4
FC_GROMMET = 1.0       # grommet flange: gap between board and tubes / bosses
FC_BOSS_D = 7.0
FC_SCREW_D = 3.4


# ======================================================================
# board data
# ======================================================================

class Board:
    """Footprint positions and the outline of a .kicad_pcb, in model
    coordinates: centred on the board, y up (KiCad's y points down)."""

    def __init__(self, path):
        self.b = pcbnew.LoadBoard(path)
        bb = self.b.GetBoardEdgesBoundingBox()
        mm = pcbnew.ToMM
        self.x0, self.y0 = mm(bb.GetX()), mm(bb.GetY())
        self.w, self.h = mm(bb.GetWidth()), mm(bb.GetHeight())
        # the bounding box includes half the edge line width
        lw = self.w - round(self.w)
        self.x0 += lw / 2
        self.y0 += lw / 2
        self.w, self.h = round(self.w, 2), round(self.h, 2)
        self.cx, self.cy = self.x0 + self.w / 2, self.y0 + self.h / 2

    def xy(self, x, y):
        return (x - self.cx, self.cy - y)

    def fp(self, ref):
        f = self.b.FindFootprintByReference(ref)
        p = f.GetPosition()
        return self.xy(pcbnew.ToMM(p.x), pcbnew.ToMM(p.y))

    def bbox(self, ref):
        """Footprint body box (pads + courtyard), model coords:
        (xmin, xmax, ymin, ymax)."""
        f = self.b.FindFootprintByReference(ref)
        bb = f.GetBoundingBox(False)
        mm = pcbnew.ToMM
        x0, y0 = self.xy(mm(bb.GetX()), mm(bb.GetY()))
        x1, y1 = self.xy(mm(bb.GetRight()), mm(bb.GetBottom()))
        return (min(x0, x1), max(x0, x1), min(y0, y1), max(y0, y1))

    def centre(self, ref):
        x0, x1, y0, y1 = self.bbox(ref)
        return ((x0 + x1) / 2, (y0 + y1) / 2)


# ======================================================================
# geometry helpers
# ======================================================================

def box(x0, x1, y0, y1, z0, z1):
    return (cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False)
            .translate((x0, y0, z0)))


def cyl(x, y, d, z0, z1):
    return (cq.Workplane("XY").workplane(offset=z0).center(x, y)
            .circle(d / 2).extrude(z1 - z0))


def tube(x, y, d_out, d_in, z0, z1):
    return cyl(x, y, d_out, z0, z1).cut(cyl(x, y, d_in, z0 - 1, z1 + 1))


def cone(x, y, d0, d1, z0, z1):
    s = cq.Solid.makeCone(d0 / 2, d1 / 2, z1 - z0, cq.Vector(x, y, z0))
    return cq.Workplane("XY").add(s)


def sma_slot(x, w, y0, y1, seam):
    """U-slot through a wall (y0..y1) for an SMA jack, open at the seam so the
    part drops over the jack; the round end sits 0.5 mm above the SMA axis."""
    zc = PCB_T + NRF_SMA_Z + 0.5
    return box(x - w / 2, x + w / 2, y0, y1, seam - 0.01, zc).union(
        cq.Workplane("XZ").workplane(offset=-y1).center(x, zc).circle(w / 2).extrude(y1 - y0))


class Outline:
    """Rounded rectangle (the outer skin) and offsets of it inward."""

    def __init__(self, x0, x1, y0, y1, r):
        self.x0, self.x1, self.y0, self.y1, self.r = x0, x1, y0, y1, r

    def solid(self, off, z0, z1, fillet_bottom=0.0, fillet_top=0.0):
        w = self.x1 - self.x0 - 2 * off
        h = self.y1 - self.y0 - 2 * off
        r = max(self.r - off, 0.5)
        s = (cq.Workplane("XY").workplane(offset=z0)
             .center((self.x0 + self.x1) / 2, (self.y0 + self.y1) / 2)
             .rect(w, h).extrude(z1 - z0).edges("|Z").fillet(r))
        if fillet_bottom:
            s = s.faces("<Z").edges().fillet(fillet_bottom)
        if fillet_top:
            s = s.faces(">Z").edges().fillet(fillet_top)
        return s

    def band(self, off_out, off_in, z0, z1):
        return self.solid(off_out, z0, z1).cut(self.solid(off_in, z0 - 1, z1 + 1))


def shell(outline, wall, z_bot, z_top, floor, roof, seam, lip, fillet):
    """Closed box split at `seam` with a lap joint: the bottom part keeps the
    outer half of the wall up to seam + lip, the top part the inner half down
    to the seam. Returns (bottom, top, cavity)."""
    body = outline.solid(0, z_bot, z_top, fillet, fillet)
    cavity = outline.solid(wall, z_bot + floor, z_top - roof)
    body = body.cut(cavity)
    big = 1000
    below = box(-big, big, -big, big, z_bot - 1, seam)
    above = box(-big, big, -big, big, seam, z_top + 1)
    bottom = body.intersect(below).union(outline.band(0, wall / 2, seam, seam + lip))
    top = body.intersect(above).cut(outline.band(-1, wall / 2 - CLR, seam - 1, seam + lip + CLR))
    return bottom, top, cavity


# ======================================================================
# controller
# ======================================================================

def rc_parts():
    bd = Board(RC_PCB)
    o = RC_GAP + RC_WALL
    ol = Outline(-bd.w / 2 - o, bd.w / 2 + o, -bd.h / 2 - o, bd.h / 2 + o, 6.0 + o)
    seam = PCB_T
    z_bot = -RC_BOT_CLEAR - RC_FLOOR
    lid_under = PCB_T + RC_TOP_CLEAR
    z_top = lid_under + RC_LID
    bottom, top, _ = shell(ol, RC_WALL, z_bot, z_top, RC_FLOOR, RC_LID, seam, RC_LIP, 3.0)
    north, west = bd.h / 2 + o, -bd.w / 2 - o

    # --- mounting: bosses clamp the board at its 4 holes
    for ref in ("H1", "H2", "H3", "H4"):
        x, y = bd.fp(ref)
        top = top.union(cyl(x, y, RC_BOSS_D, PCB_T, lid_under + 0.5))
        top = top.cut(cyl(x, y, RC_INSERT_D, PCB_T - 1, PCB_T + RC_INSERT_DEPTH))
        bottom = bottom.union(cyl(x, y, RC_BOSS_D, z_bot + 0.5, 0))
        bottom = bottom.cut(cyl(x, y, 3.4, z_bot - 1, 1))
        # screw head seat so that an M3 x RC_SCREW_LEN ends inside the insert
        seat = PCB_T + RC_INSERT_DEPTH - 0.5 - RC_SCREW_LEN
        bottom = bottom.cut(cyl(x, y, 6.5, z_bot - 1, seat))

    # --- sticks
    for ref in ("JS1", "JS2"):
        x, y = bd.fp(ref)
        top = top.cut(cyl(x, y, RC_STICK_HOLE_D, lid_under - 1, z_top + 1))
        top = top.cut(cone(x, y, RC_STICK_HOLE_D, RC_STICK_HOLE_D + 3, z_top - 1.5, z_top + 0.01))

    # --- menu buttons: caps ride in these holes
    buttons = [bd.centre(r) for r in ("SW4", "SW5", "SW6", "SW7")]
    for x, y in buttons:
        top = top.cut(cyl(x, y, RC_BTN_HOLE_D, lid_under - 1, z_top + 1))

    # --- LEDs: light tubes down to just above each LED
    for ref in ("D1", "D2", "D4"):
        x, y = bd.fp(ref)
        x += 1.27                       # footprint origin is pad 1, body centre +1.27
        top = top.union(cyl(x, y, 5.4, PCB_T + LED_H + 0.6, lid_under + 0.5))
        top = top.cut(cyl(x, y, 3.4, PCB_T + LED_H, z_top + 1))

    # --- OLED window
    sx, sy = bd.fp("J4")
    wx, wy = sx + RC_OLED_WIN_OFS[0], sy - RC_OLED_WIN_OFS[1]
    win = (cq.Workplane("XY").workplane(offset=lid_under - 1).center(wx, wy)
           .rect(*RC_OLED_WIN).extrude(RC_LID + 2).edges("|Z").fillet(1.0))
    top = top.cut(win)

    # --- toggles: recessed cup so the bushing clears the panel for its nut
    cup_floor = PCB_T + TOGGLE_BODY_H + 0.3
    for ref in ("SW8", "SW9"):
        x, y = bd.fp(ref)
        top = top.union(cyl(x, y, 18.0, cup_floor, lid_under + 0.5))
        top = top.cut(cyl(x, y, 14.0, cup_floor + 2.0, z_top + 1))
        top = top.cut(cyl(x, y, TOGGLE_BUSH_D + 0.3, cup_floor - 1, z_top + 1))

    # --- nRF24 SMA out of the north wall: U-slot open at the seam
    x0, x1, _, _ = bd.bbox("J3")
    sma_x, sma_w = (x0 + x1) / 2, 7.5
    slot = sma_slot(sma_x, sma_w, north - RC_WALL - 1, north + 1, seam)
    top = top.cut(slot)
    bottom = bottom.cut(slot)

    # --- USB-C (back side, mouth north): room for the plug's overmould
    ux, _ = bd.centre("J1")
    usb = box(ux - 6.25, ux + 6.25, north - RC_WALL - 1, north + 1, -4.8, seam + RC_LIP + 0.5)
    bottom = bottom.cut(usb)
    top = top.cut(usb)

    # --- power slide (back side, lever west): finger-nail slot
    _, py = bd.centre("SW1")
    pwr = box(west - 1, west + RC_WALL + 1, py - 5, py + 5, -3.5, seam + RC_LIP + 0.5)
    bottom = bottom.cut(pwr)
    top = top.cut(pwr)

    # --- RESET / BOOT (back side, facing down): paperclip guide tubes
    for ref in ("SW2", "SW3"):
        x, y = bd.centre(ref)
        bottom = bottom.union(cyl(x, y, 5.0, z_bot + 0.5, -3.3))
        bottom = bottom.cut(cyl(x, y, 2.4, z_bot - 1, 0))

    # --- buzzer grille
    bx, by = bd.centre("BZ1")
    for i in range(-2, 3):
        for j in range(-2, 3):
            if i * i + j * j <= 5:
                bottom = bottom.cut(cyl(bx + 2.6 * i, by + 2.6 * j, 1.6, z_bot - 1, z_bot + RC_FLOOR + 1))

    bottom, door = rc_battery_door(bd, bottom, z_bot)

    # --- button caps (separate prints), in their assembled position
    caps = None
    for x, y in buttons:
        z_btn = PCB_T + TACT_H + 0.2
        cap = (cyl(x, y, RC_BTN_HOLE_D - 0.5, lid_under, z_top + 1.0)
               .union(cone(x, y, 9.0, RC_BTN_HOLE_D - 0.5, lid_under - 1.5, lid_under))
               .union(cyl(x, y, 9.0, lid_under - 2.5, lid_under - 1.5))
               .union(cyl(x, y, 4.0, z_btn, lid_under - 2.5)))
        caps = cap if caps is None else caps.union(cap)

    keepouts = rc_keepouts(bd, north, z_top)
    parts = {"rc_top": top, "rc_bottom": bottom, "rc_buttons": caps, "rc_battery_door": door}
    return bd, parts, keepouts, (z_bot, z_top)


def rc_battery_door(bd, bottom, z_bot):
    """Cut the battery opening in the floor and build its door. Seen from the
    side the opening's west edge is a 45 deg undercut and the door's west edge
    matches it, so the door can only leave inward-and-east; the door is
    shorter than the opening by that bevel so it drops in shifted east, then
    slides west to hook. The east tab and its screw stop it sliding back."""
    hx0, hx1, hy0, hy1 = bd.bbox("BT1")
    hx = (hx0 + hx1) / 2
    ox0, ox1 = hx - RC_DOOR_HALF_LEN, hx + RC_DOOR_HALF_LEN
    oy0, oy1 = hy0 + 0.5, hy1 - 0.5
    zb, zf = z_bot, z_bot + RC_FLOOR
    bev, c = RC_FLOOR, RC_DOOR_CLR

    def prism(pts, y0, y1):
        return (cq.Workplane("XZ").workplane(offset=-y1)
                .polyline(pts).close().extrude(y1 - y0))

    bottom = bottom.cut(prism([(ox0 + 1, zb - 1), (ox1, zb - 1), (ox1, zf + 1),
                               (ox0 - bev - 1, zf + 1)], oy0, oy1))
    xe = ox1 - bev - c
    door = prism([(ox0 + c, zb), (xe, zb), (xe, zf), (ox0 - bev + c, zf)], oy0 + c, oy1 - c)

    # east tab in a rebate; the rebate runs bev + 1 past the tab: insertion
    # slack, then a fingernail gap once the door is home
    ty = (oy0 + oy1) / 2
    tab_end = ox1 + 8.0
    door = door.union(box(xe - 4, tab_end, ty - 4, ty + 4, zb, zb + RC_DOOR_TAB_T))
    bottom = bottom.cut(box(ox1 - 1, tab_end + bev + 1, ty - 4 - c, ty + 4 + c,
                            zb - 1, zb + RC_DOOR_TAB_T + c))
    sx = ox1 + 4.5
    door = door.cut(cyl(sx, ty, 3.4, zb - 1, zf))
    bottom = bottom.union(cyl(sx, ty, 6.0, zf - 0.5, zf + 4.0))
    bottom = bottom.cut(cyl(sx, ty, 2.5, zb - 1, zf + 3.5))
    return bottom, door


def rc_keepouts(bd, north, z_top):
    """Parts with no KiCad 3D model, as blocks, for --check (and the renders)."""
    k = {}
    for ref in ("JS1", "JS2"):
        x, y = bd.fp(ref)
        k[ref] = box(x - 8.5, x + 8.5, y - 8.5, y + 8.5, PCB_T, PCB_T + 11.0).union(
            cyl(x, y, 4.0, PCB_T + 11.0, z_top + 4.0))
        # thumb cap: d20, sits 3 mm above the lid
        k[ref + " cap"] = cyl(x, y, 20.0, z_top + 3.0, z_top + 8.0)
    for ref in ("SW8", "SW9"):
        x, y = bd.fp(ref)
        bush_top = PCB_T + TOGGLE_BODY_H + 8.9
        k[ref] = box(x - 6.5, x + 6.5, y - 4.0, y + 4.0, PCB_T, PCB_T + TOGGLE_BODY_H).union(
            cyl(x, y, TOGGLE_BUSH_D, PCB_T + TOGGLE_BODY_H, bush_top)).union(
            cyl(x, y, 2.4, bush_top, bush_top + 11.0))
    x0, x1, y0, y1 = bd.bbox("J3")
    k["J3 nRF24"] = box(x0, x1, y0, y1, PCB_T, PCB_T + NRF_H).union(
        cq.Workplane("XZ").workplane(offset=-(north + 10)).center((x0 + x1) / 2, PCB_T + NRF_SMA_Z)
        .circle(6.35 / 2).extrude(north + 10 - y1))
    sx, sy = bd.fp("J4")
    k["J4 OLED"] = box(sx - 13.6, sx + 13.6, sy - 25.8, sy + 2.0, PCB_T + 8.5, PCB_T + OLED_GLASS_H)
    ux, _ = bd.centre("J1")
    k["J1 USB-C"] = box(ux - 4.5, ux + 4.5, bd.h / 2 - 7.3, bd.h / 2 + 0.5, -3.3, 0)
    bx, by = bd.centre("BZ1")
    k["BZ1"] = box(bx - 4.5, bx + 4.5, by - 4.5, by + 4.5, -4.0, 0)
    x0, x1, y0, y1 = bd.bbox("BT1")
    k["18650 cell"] = (cq.Workplane("YZ").workplane(offset=(x0 + x1) / 2 - 32.5)
                       .center((y0 + y1) / 2, -BATT_DEPTH + 9.3).circle(9.3).extrude(65))
    return k


# ======================================================================
# flight controller
# ======================================================================

def fc_parts():
    bd = Board(FC_PCB)
    o = FC_GAP + FC_WALL
    # the nRF24 module overhangs the south edge: extend the box over it
    _, _, nrf_y0, _ = bd.bbox("J2")
    south_in = min(-bd.h / 2, nrf_y0) - FC_GAP
    ol = Outline(-bd.w / 2 - o, bd.w / 2 + o, south_in - FC_WALL, bd.h / 2 + o, 2.0 + o)
    seam = PCB_T
    z_bot = -FC_BOT_CLEAR - FC_FLOOR
    roof_under = PCB_T + FC_TOP_CLEAR
    z_top = roof_under + FC_ROOF
    tray, canopy, _ = shell(ol, FC_WALL, z_bot, z_top, FC_FLOOR, FC_ROOF, seam, FC_LIP, 1.5)
    north, south = bd.h / 2 + o, south_in - FC_WALL
    west, east = -bd.w / 2 - o, bd.w / 2 + o

    # --- stack: tubes / bosses at the grommet holes, 1 mm clear of the board
    for ref in ("H1", "H2", "H3", "H4"):
        x, y = bd.fp(ref)
        canopy = canopy.union(cyl(x, y, FC_BOSS_D, PCB_T + FC_GROMMET, roof_under + 0.5))
        canopy = canopy.cut(cyl(x, y, FC_SCREW_D, PCB_T, z_top + 1))
        tray = tray.union(cyl(x, y, FC_BOSS_D, z_bot + 0.5, -FC_GROMMET))
        tray = tray.cut(cyl(x, y, FC_SCREW_D, z_bot - 1, 0))

    # --- POWER_IN (top, mouth north)
    x0, x1, _, _ = bd.bbox("J1")
    cut = box(x0 - 0.5, x1 + 0.5, north - FC_WALL - 1, north + 1, seam - 0.01, PCB_T + 6.0)
    canopy, tray = canopy.cut(cut), tray.cut(cut)
    # --- SWD + TELEMETRY (back, mouths north)
    x0 = min(bd.bbox("J5")[0], bd.bbox("J6")[0])
    x1 = max(bd.bbox("J5")[1], bd.bbox("J6")[1])
    cut = box(x0 - 0.5, x1 + 0.5, north - FC_WALL - 1, north + 1, -5.2, seam + FC_LIP + 0.5)
    canopy, tray = canopy.cut(cut), tray.cut(cut)
    # --- AUX (top, mouth west)
    _, _, y0, y1 = bd.bbox("J3")
    cut = box(west - 1, west + FC_WALL + 1, y0 - 0.5, y1 + 0.5, seam - 0.01, PCB_T + 6.0)
    canopy, tray = canopy.cut(cut), tray.cut(cut)
    # --- ESC header (top, east edge, pins up): open from above for the plug
    x0, _, y0, y1 = bd.bbox("J4")
    cut = box(x0 - 0.5, east + 1, y0 - 0.5, y1 + 0.5, seam - 0.01, z_top + 1)
    canopy, tray = canopy.cut(cut), tray.cut(cut)
    # --- nRF24 SMA (south): U-slot open at the seam
    x0, x1, _, _ = bd.bbox("J2")
    sx, sw = (x0 + x1) / 2, 7.5
    slot = sma_slot(sx, sw, south - 1, south + FC_WALL + 1, seam)
    canopy, tray = canopy.cut(slot), tray.cut(slot)
    # --- status LED: light tube; CAL: pin-hole with a guide tube
    x, y = bd.fp("D1")
    canopy = canopy.union(cyl(x, y, 3.6, PCB_T + 1.5, roof_under + 0.5)).cut(cyl(x, y, 2.0, PCB_T, z_top + 1))
    x, y = bd.centre("SW2")
    canopy = canopy.union(cyl(x, y, 4.4, PCB_T + 3.0, roof_under + 0.5)).cut(cyl(x, y, 2.4, PCB_T, z_top + 1))
    # --- RESET (back): pin-hole
    x, y = bd.centre("SW1")
    tray = tray.union(cyl(x, y, 4.4, z_bot + 0.5, -3.3)).cut(cyl(x, y, 2.4, z_bot - 1, 0))
    # --- FWD arrow (north) engraved in the roof
    arrow = (cq.Workplane("XY").workplane(offset=z_top - 0.6)
             .polyline([(0, 18.0), (-5, 11.0), (-1.6, 11.0), (-1.6, 5.0),
                        (1.6, 5.0), (1.6, 11.0), (5, 11.0)]).close().extrude(1.0))
    canopy = canopy.cut(arrow)

    keepouts = {}
    x0, x1, y0, y1 = bd.bbox("J2")
    keepouts["J2 nRF24"] = box(x0, x1, y0, y1, PCB_T, PCB_T + NRF_H)
    x, y = bd.fp("U7")
    keepouts["U7 GPS"] = box(x - 7.75, x + 7.75, y - 7.75, y + 7.75, PCB_T, PCB_T + GPS_H)
    x0, x1, y0, y1 = bd.bbox("J4")
    keepouts["J4 ESC plug"] = box(x0 + 0.5, x1 - 0.5, y0 + 0.5, y1 - 0.5, PCB_T, PCB_T + 25)
    return bd, {"fc_canopy": canopy, "fc_tray": tray}, keepouts, (z_bot, z_top)


# ======================================================================
# output
# ======================================================================

def board_step(pcb, bd):
    os.makedirs(BUILD, exist_ok=True)
    out = os.path.join(BUILD, os.path.basename(pcb).replace(".kicad_pcb", "_board.step"))
    if not os.path.exists(out) or os.path.getmtime(out) < os.path.getmtime(pcb):
        subprocess.run(["kicad-cli", "pcb", "export", "step", "--subst-models", "-f",
                        "-o", out, pcb], check=True, capture_output=True)
    shape = cq.importers.importStep(out)
    return shape.translate((-bd.cx, bd.cy, 0))


def check(name, parts, board, keepouts):
    """Every printed part against every board solid and keep-out: no overlap."""
    bad = 0
    obstacles = [(f"board solid {i}", s) for i, s in enumerate(board.solids().vals())]
    obstacles += [(k, v.val()) for k, v in keepouts.items()]
    for pname, part in parts.items():
        ps = part.val()
        pb = ps.BoundingBox()
        for oname, os_ in obstacles:
            ob = os_.BoundingBox()
            if (ob.xmin > pb.xmax or ob.xmax < pb.xmin or ob.ymin > pb.ymax or
                    ob.ymax < pb.ymin or ob.zmin > pb.zmax or ob.zmax < pb.zmin):
                continue
            v = ps.intersect(os_).Volume()
            if v > 0.05:
                b = os_.BoundingBox()
                print(f"  OVERLAP {pname} x {oname}: {v:.2f} mm3 "
                      f"(at x {b.xmin:.1f}..{b.xmax:.1f} y {b.ymin:.1f}..{b.ymax:.1f} "
                      f"z {b.zmin:.1f}..{b.zmax:.1f})")
                bad += 1
    print(f"{name}: {'OK, no overlaps' if not bad else f'{bad} overlaps'}")
    return bad


def print_pose(name, part, z_bot, z_top):
    """Rotate into print orientation and drop onto z = 0."""
    if name in ("rc_top", "fc_canopy", "rc_buttons"):
        part = part.rotate((0, 0, 0), (1, 0, 0), 180)
        return part.translate((0, 0, z_top))
    return part.translate((0, 0, -z_bot))


SHELL = "#3d4450"
COLOURS = {"top": SHELL, "bottom": SHELL, "canopy": SHELL, "tray": SHELL,
           "door": "#4a5361", "buttons": "#f2a33a"}
STAND_IN = "#20242a"   # parts with no KiCad model (sticks, toggles, modules)


def mesh(shape):
    import pyvista as pv
    verts, tris = shape.val().tessellate(0.03, 0.15)
    pts = [(v.x, v.y, v.z) for v in verts]
    faces = [n for t in tris for n in (3, *t)]
    return pv.PolyData(pts, faces).compute_normals(split_vertices=True, feature_angle=35)


def render(pcb, bd, parts, keepouts, stem):
    """Assembled (top and bottom) and exploded views, VTK off-screen. Needs a
    display: run under xvfb-run on a headless box."""
    import numpy as np
    import pyvista as pv

    glb = os.path.join(BUILD, f"{stem}_board.glb")
    if not os.path.exists(glb) or os.path.getmtime(glb) < os.path.getmtime(pcb):
        subprocess.run(["kicad-cli", "pcb", "export", "glb", "--subst-models", "-f",
                        "-o", glb, pcb], check=True, capture_output=True)
    part_meshes = {n: mesh(p) for n, p in parts.items()}
    stand_ins = {n: mesh(k) for n, k in keepouts.items()}
    explode = ({"top": 45, "buttons": 45, "bottom": -25, "door": -55} if stem == "falcon_rc"
               else {"canopy": 32, "tray": -18})
    views = (("top", (-0.9, -1.5, 1.2), {}),
             ("bottom", (-0.9, -1.5, -1.1), {}),
             ("exploded", (-0.9, -1.5, 0.9), explode))
    for view, vec, dz in views:
        pl = pv.Plotter(off_screen=True, window_size=(1600, 1100))
        pl.set_background("#f4f5f7", top="#d9dde3")
        # board: glTF is metres, Y up, KiCad's y down -> scale, rotate +90 about X, centre
        before = set(pl.renderer.actors)
        pl.import_gltf(glb)
        m = np.array([[1000, 0, 0, -bd.cx], [0, 0, -1000, bd.cy], [0, 1000, 0, 0], [0, 0, 0, 1]], float)
        for name, actor in pl.renderer.actors.items():
            if name not in before:
                own = actor.GetUserMatrix()
                own = np.eye(4) if own is None else pv.array_from_vtkmatrix(own)
                actor.SetUserMatrix(pv.vtkmatrix_from_array(m @ own))
        for n, s_ in stand_ins.items():
            if view == "exploded" and n.endswith(" cap"):
                continue                # stick caps would float inside the lifted lid
            pl.add_mesh(s_, color=STAND_IN, smooth_shading=True, specular=0.3)
        for n, msh in part_meshes.items():
            kind = n.split("_")[-1]
            pl.add_mesh(msh.translate((0, 0, dz.get(kind, 0)), inplace=False), color=COLOURS[kind],
                        smooth_shading=True, specular=0.25, specular_power=20)
        pl.enable_anti_aliasing("ssaa")
        pl.view_vector(vec, viewup=(0, 0, 1))
        pl.reset_camera()
        pl.camera.zoom(1.25)
        out = os.path.join(BUILD, f"{stem}_{view}.png")
        pl.screenshot(out)
        pl.close()
        print(f"  {os.path.relpath(out, HERE)}")


def main():
    do_check = "--check" in sys.argv
    os.makedirs(BUILD, exist_ok=True)
    fails = 0
    for name, pcb, build in (("controller", RC_PCB, rc_parts),
                             ("flight controller", FC_PCB, fc_parts)):
        bd, parts, keepouts, (z_bot, z_top) = build()
        board = board_step(pcb, bd)
        asm = cq.Assembly()
        for pname, part in parts.items():
            pose = print_pose(pname, part, z_bot, z_top)
            cq.exporters.export(pose, os.path.join(BUILD, f"{pname}.stl"),
                                tolerance=0.02, angularTolerance=0.1)
            vol = part.val().Volume() / 1000
            bb = pose.val().BoundingBox()
            print(f"{pname:11s} {bb.xlen:6.1f} x {bb.ylen:6.1f} x {bb.zlen:5.1f} mm  "
                  f"{vol:5.1f} cm3  ~{vol * 1.24:4.0f} g PLA solid")
            asm.add(part, name=pname)
        asm.add(board, name="board")
        stem = "falcon_rc" if pcb == RC_PCB else "falcon_fc"
        asm.export(os.path.join(BUILD, f"{stem}_enclosure.step"))
        render(pcb, bd, parts, keepouts, stem)
        if do_check:
            fails += check(name, parts, board, keepouts)
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
