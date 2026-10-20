#!/usr/bin/env python3
"""
Route falcon_fc.kicad_pcb (as placed by gen_pcb.py) with Freerouting.

  1. Fan out: every SMD pad on GND or +3V3 gets a short stub and a via down to
     its inner plane (In1 GND, In2 +3V3). Freerouting does not drop vias into
     planes on its own; these are locked so the autorouter leaves them alone.
  2. Lay the short local links and the IMU bus with the maze router (PREROUTE),
     then route the crowded groups with Freerouting (STAGES) one at a time,
     locking each, so their short direct paths are not taken by other nets.
  3. Route everything else, drop the router's dangling leftovers, join what is
     still open with the maze router, put the outer GND pours back and refill.

The DSN is exported without the outer GND pours, inner layers marked as power
layers so Freerouting keeps signals off the planes. Only the maze router may put
a track on In2, inside gen_pcb.IN2_WINDOW. The pours are refilled around it all.

Needs Java 25+ and the Freerouting 2.x jar:
  FREEROUTING_JAR=/path/freerouting.jar JAVA=/path/java python3 route_pcb.py [--force]

--force strips existing tracks first (otherwise a routed board is left alone).
"""

import math
import os
import random
import re
import subprocess
import sys

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gen_pcb  # noqa: E402  (zones, outline constants)

mm = pcbnew.FromMM
PLANE_NETS = {"/GND", "/+3V3"}     # schematic local labels: "/" prefix
VIA_D, VIA_DRILL = 0.6, 0.3
STUB_W = 0.25
CLEAR = 0.2                    # fanout copper-to-copper, a bit above the 0.15 rule
EDGE_KEEPOUT = 0.8             # via centre to board edge
PASSES = 100
# Freerouting is deterministic but sensitive to where the fanout vias land; the
# seed nudges the fanout's preferences. 0 is the plain heuristic; other values
# were searched for one that routes completely (see FANOUT_SEED in the log).
FANOUT_SEED = int(os.environ.get("FANOUT_SEED", "0"))
# Parts whose power pins fan out inward, under the package: keeps the lanes
# between the pins free for the signals escaping outward. (Under U1 on F.Cu is
# the GPS, where u-blox wants GND vias anyway.)
FANOUT_INWARD = {"U1"}
# No fanout vias here: the B.Cu corridor between the IMU and the MCU's SPI3 pins.
# The GPS ground pads above it are in the top GND pour anyway.
NO_VIA = [(90.6, 99.3, 93.45, 105.2)]
# Laid first with the maze router, in this order, and locked: short local links
# that must stay short (Freerouting tends to wander), and the IMU bus, whose two
# pin rings order it so that some lines must change layer (the maze router can
# use the In2 window for that, Freerouting can't).
# (The IMU lines go in an order that was found to route all five; the maze
# router is greedy, and most orders box the last line in.)
PREROUTE = ["/" + n for n in ("BOOT0", "VCAP1", "NRST", "+3V3A", "+3V3_SENS",
                               "SPI3_SCK", "IMU_INT", "IMU_CS", "SPI3_MISO", "SPI3_MOSI")]

# Routed in this order, each group alone and then locked, before everything else.
# 1. The AUX connector: short top-side runs to its ESD array, then to the MCU's
#    south pins. Routed first so the IMU bus's top-side hops don't wall it off.
# 2. The rest of the sensor cluster west of the MCU: SWO/SWCLK (their MCU pins
#    sit among the IMU pins) and the baro I2C (next to IMU_INT).
STAGES = [
    ("aux", ["/UART1_TX", "/UART1_RX", "/I2C3_SCL", "/I2C3_SDA"]),
    ("sensors", ["/SWO", "/SWCLK", "/I2C1_SCL", "/I2C1_SDA"]),
]
MIN_TRACK = 0.13               # Freerouting necks down to 0.12 at pads; JLCPCB min is 0.127


def to_mm(v):
    return pcbnew.ToMM(v.x), pcbnew.ToMM(v.y)


def rect_dist(px, py, r):
    """Distance from a point to an axis-aligned rect (x0, y0, x1, y1); 0 inside."""
    dx = max(r[0] - px, 0, px - r[2])
    dy = max(r[1] - py, 0, py - r[3])
    return math.hypot(dx, dy)


def seg_rect_dist(a, b, r, steps=12):
    return min(rect_dist(a[0] + (b[0] - a[0]) * t / steps, a[1] + (b[1] - a[1]) * t / steps, r)
               for t in range(steps + 1))


def seg_seg_dist(a, b, c, d, steps=12):
    return min(point_seg_dist(a[0] + (b[0] - a[0]) * t / steps, a[1] + (b[1] - a[1]) * t / steps, c, d)
               for t in range(steps + 1))


def point_seg_dist(px, py, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    L = dx * dx + dy * dy
    u = 0 if L == 0 else max(0, min(1, ((px - a[0]) * dx + (py - a[1]) * dy) / L))
    return math.hypot(px - a[0] - u * dx, py - a[1] - u * dy)


def pad_rect(pad):
    bb = pad.GetBoundingBox()
    return (pcbnew.ToMM(bb.GetLeft()), pcbnew.ToMM(bb.GetTop()),
            pcbnew.ToMM(bb.GetRight()), pcbnew.ToMM(bb.GetBottom()))


def fanout(board):
    rnd = random.Random(FANOUT_SEED)
    turn_cost = 0.6 if FANOUT_SEED == 0 else rnd.uniform(0.2, 1.2)
    d0 = 0.5 if FANOUT_SEED == 0 else rnd.uniform(0.5, 0.8)
    pads = [p for fp in board.Footprints() for p in fp.Pads()]
    info = []
    for p in pads:
        layers = {l for l in (pcbnew.F_Cu, pcbnew.B_Cu) if p.IsOnLayer(l)}
        th = p.GetAttribute() in (pcbnew.PAD_ATTRIB_PTH, pcbnew.PAD_ATTRIB_NPTH)
        info.append((p, pad_rect(p), layers, th))
    h = gen_pcb.SIZE / 2 - EDGE_KEEPOUT
    no_tracks = []             # (layer, rect) of track keepouts, e.g. under the GPS
    for z in board.Zones():
        if z.GetIsRuleArea() and z.GetDoNotAllowTracks():
            bb = z.GetBoundingBox()
            no_tracks.append((z.GetLayer(), (pcbnew.ToMM(bb.GetLeft()), pcbnew.ToMM(bb.GetTop()),
                                             pcbnew.ToMM(bb.GetRight()), pcbnew.ToMM(bb.GetBottom()))))
    vias = []                  # (x, y, net)
    tracks = []                # (a, b, half width, layer, net) of tracks already routed
    for tr in board.GetTracks():
        if tr.GetClass() == "PCB_VIA":
            vias.append((*to_mm(tr.GetPosition()), tr.GetNetname()))
        else:
            tracks.append((to_mm(tr.GetStart()), to_mm(tr.GetEnd()), pcbnew.ToMM(tr.GetWidth()) / 2,
                           tr.GetLayer(), tr.GetNetname()))
    nets = board.GetNetsByName()
    placed, failed, fanned = 0, [], []

    for p, rect, layers, th in info:
        net = p.GetNetname()
        if net not in PLANE_NETS or th:
            continue
        layer = pcbnew.F_Cu if pcbnew.F_Cu in layers else pcbnew.B_Cu
        px, py = to_mm(p.GetPosition())
        fx, fy = to_mm(p.GetParentFootprint().GetPosition())
        out = math.atan2(py - fy, px - fx) if (px, py) != (fx, fy) else 0.0
        if p.GetParentFootprint().GetReference() in FANOUT_INWARD:
            out += math.pi
        best = None
        for k in range(8):
            ang = k * math.pi / 4
            # prefer leaving the package outward
            diff = abs((ang - out + math.pi) % (2 * math.pi) - math.pi)
            for i in range(25):
                d = d0 + 0.1 * i
                score = d + diff * turn_cost
                if best and score >= best[0]:
                    break
                vx, vy = px + d * math.cos(ang), py + d * math.sin(ang)
                if abs(vx - gen_pcb.CX) > h or abs(vy - gen_pcb.CY) > h:
                    break
                if any(x0 - VIA_D / 2 < vx < x1 + VIA_D / 2 and y0 - VIA_D / 2 < vy < y1 + VIA_D / 2
                       for x0, y0, x1, y1 in NO_VIA):
                    continue
                if any(math.hypot(vx - x, vy - y) < (VIA_D + (0.25 if n == net else CLEAR))
                       for x, y, n in vias):
                    continue
                if any(n != net and point_seg_dist(x, y, (px, py), (vx, vy)) < VIA_D / 2 + STUB_W / 2 + CLEAR
                       for x, y, n in vias):
                    continue
                if any(l == layer and seg_rect_dist((px, py), (vx, vy), r) < STUB_W / 2
                       for l, r in no_tracks):
                    continue
                if any(n != net and (point_seg_dist(vx, vy, a, b2) < VIA_D / 2 + w + CLEAR or
                                     (l == layer and seg_seg_dist((px, py), (vx, vy), a, b2)
                                      < STUB_W / 2 + w + CLEAR))
                       for a, b2, w, l, n in tracks):
                    continue
                ok = True
                for q, r, ql, qth in info:
                    if q is p:
                        continue
                    same = q.GetNetname() == net and not qth
                    need = VIA_D / 2 + (0.1 if same else CLEAR)
                    if qth and not q.GetNetname():
                        need = VIA_D / 2 + 0.5      # mounting holes
                    if rect_dist(vx, vy, r) < need:
                        ok = False
                        break
                    if not same and layer in ql and \
                            seg_rect_dist((px, py), (vx, vy), r) < STUB_W / 2 + CLEAR:
                        ok = False
                        break
                if ok:
                    best = (score, vx, vy)
                    break
        if not best:
            failed.append(p)
            continue
        _, vx, vy = best
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(p.GetPosition())
        t.SetEnd(pcbnew.VECTOR2I(mm(vx), mm(vy)))
        t.SetWidth(mm(STUB_W))
        t.SetLayer(layer)
        t.SetNet(nets[net])
        t.SetLocked(True)
        board.Add(t)
        v = pcbnew.PCB_VIA(board)
        v.SetPosition(pcbnew.VECTOR2I(mm(vx), mm(vy)))
        v.SetWidth(mm(VIA_D))
        v.SetDrill(mm(VIA_DRILL))
        v.SetNet(nets[net])
        v.SetLocked(True)
        board.Add(v)
        vias.append((vx, vy, net))
        tracks.append(((px, py), (vx, vy), STUB_W / 2, layer, net))
        fanned.append(p)
        placed += 1

    # No room (dense LGA, cap right off the pin): bridge straight to a nearby pad
    # on the same net and layer, preferring one that has its own via.
    linked = 0
    for p in failed:
        fp, pos = p.GetParentFootprint(), p.GetPosition()
        a = to_mm(pos)
        layer = pcbnew.F_Cu if p.IsOnLayer(pcbnew.F_Cu) else pcbnew.B_Cu
        mates = []
        for q, r, ql, qth in info:
            d = pcbnew.ToMM((q.GetPosition() - pos).EuclideanNorm())
            if q is p or q.GetNetname() != p.GetNetname() or layer not in ql or d > 2.5:
                continue
            if q.GetParentFootprint() == fp and d > 0.55:
                continue           # within a part only the next pin: never under its body
            b2 = to_mm(q.GetPosition())
            if any(o.GetNetname() != p.GetNetname() and seg_rect_dist(a, b2, orr) < STUB_W / 2 + CLEAR
                   for o, orr, ol, _ in info if layer in ol) or \
               any(n != p.GetNetname() and point_seg_dist(x, y, a, b2) < VIA_D / 2 + STUB_W / 2 + CLEAR
                   for x, y, n in vias) or \
               any(n != p.GetNetname() and l == layer and seg_seg_dist(a, b2, c, e) < STUB_W / 2 + w + CLEAR
                   for c, e, w, l, n in tracks):
                continue
            mates.append((q not in fanned, d, q))
        mates = [q for *_, q in sorted(mates, key=lambda m: m[:2])]
        if not mates:
            print(f"  fanout: no room for {fp.GetReference()}.{p.GetNumber()} ({p.GetNetname()})")
            continue
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(pos)
        t.SetEnd(mates[0].GetPosition())
        t.SetWidth(mm(STUB_W))
        t.SetLayer(layer)
        t.SetNet(p.GetNet())
        t.SetLocked(True)
        board.Add(t)
        tracks.append((a, to_mm(mates[0].GetPosition()), STUB_W / 2, layer, p.GetNetname()))
        linked += 1
    print(f"fanout (seed {FANOUT_SEED}): {placed} vias, {linked} pads bridged to a neighbour, "
          f"{len(failed) - linked} left to the router")


def export_dsn(board, path, only=None):
    """Write the DSN; with `only`, every other net loses its pins (pads stay as obstacles)."""
    if not pcbnew.ExportSpecctraDSN(board, path):
        raise SystemExit("DSN export failed")
    t = open(path).read()
    # Both inner layers are planes to Freerouting. (Given the In2 window, it makes
    # things worse: it reads the In2 keepouts as via keepouts too. Only finish()
    # uses the window.)
    for name in ("GND", "3V3"):
        t = t.replace(f"(layer {name}\n      (type signal)", f"(layer {name}\n      (type power)")
    t = re.sub(r'\(wire_keepout "" \(polygon 3V3 [^)]*\)\s*\)', "", t)
    # Freerouting rounds to within a micron of the rule; give it 10 um of margin.
    t = t.replace("(clearance 150)", "(clearance 160)")
    if only:
        def strip(m):
            name = m.group(1).strip('"')
            return m.group(0) if name in only else f"(net {m.group(1)}\n    )"
        t = re.sub(r'\(net ("[^"]*"|\S+)\s*\(pins[^)]*\)\s*\)', strip, t)
    open(path, "w").write(t)


def freeroute(board, build, stage, only=None):
    dsn, ses = (os.path.join(build, f"falcon_fc_{stage}.{ext}") for ext in ("dsn", "ses"))
    export_dsn(board, dsn, only)
    with open(os.path.join(build, f"freerouting_{stage}.log"), "w") as log:
        env = dict(os.environ, FREEROUTING__ROUTER__COPPER_TO_EDGE_CLEARANCE_UM="350")
        subprocess.run([os.environ.get("JAVA", "java"), "-Djava.awt.headless=true", "-jar",
                        os.environ["FREEROUTING_JAR"], "-de", dsn, "-do", ses, "-mp", str(PASSES)],
                       check=True, stdout=log, stderr=subprocess.STDOUT, env=env)
    if not pcbnew.ImportSpecctraSES(board, ses):
        raise SystemExit(f"SES import failed ({stage})")
    m = re.findall(r"final score: [\d.]+ \((\d+) unrouted", open(log.name).read())
    unrouted = int(m[-1]) if m else 999
    print(f"{stage}: {len(board.GetTracks())} tracks and vias, {unrouted} unrouted")
    return unrouted


def drop_dangling(board):
    """Remove router leftovers: vias reaching one layer only, tracks ending in nothing."""
    removed = 0
    while True:
        items = list(board.GetTracks())
        pads = [p for fp in board.Footprints() for p in fp.Pads()]
        ends = {}
        for t in items:
            if t.GetClass() != "PCB_VIA":
                for pt in (t.GetStart(), t.GetEnd()):
                    ends.setdefault((pt.x, pt.y), set()).add(t.GetLayer())
        gone = []
        for t in items:
            if t.GetClass() != "PCB_VIA" and t.GetStart() == t.GetEnd():
                gone.append(t)
                continue
            if t.GetClass() == "PCB_VIA":
                pos = t.GetPosition()
                if t.GetNetname() not in PLANE_NETS and len(ends.get((pos.x, pos.y), ())) < 2:
                    gone.append(t)
                continue
            for pt in (t.GetStart(), t.GetEnd()):
                n_tracks = sum(1 for o in items if o is not t and o.GetClass() != "PCB_VIA"
                               and o.GetLayer() == t.GetLayer() and pt in (o.GetStart(), o.GetEnd()))
                on_via = any(o.GetClass() == "PCB_VIA" and o.GetPosition() == pt for o in items)
                on_pad = any(p.GetNetname() == t.GetNetname() and p.IsOnLayer(t.GetLayer())
                             and p.HitTest(pt) for p in pads)
                if not (n_tracks or on_via or on_pad):
                    gone.append(t)
                    break
        if not gone:
            break
        for t in gone:
            board.Delete(t)
        removed += len(gone)
    print(f"cleanup: removed {removed} dangling tracks/vias")


def islands(board, net):
    """Connected groups of a net's pads, tracks and vias (by geometric contact)."""
    items = [p for fp in board.Footprints() for p in fp.Pads() if p.GetNetname() == net]
    items += [t for t in board.GetTracks() if t.GetNetname() == net]
    parent = list(range(len(items)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    def points(it):
        if it.GetClass() == "PCB_TRACK":
            return [(it.GetStart(), {it.GetLayer()}), (it.GetEnd(), {it.GetLayer()})]
        layers = {l for l in (pcbnew.F_Cu, pcbnew.B_Cu) if it.IsOnLayer(l)}
        return [(it.GetPosition(), layers)]

    for i, a in enumerate(items):
        for j in range(i + 1, len(items)):
            b = items[j]
            if find(i) == find(j):
                continue
            touch = any(b.HitTest(pt) and any(b.IsOnLayer(l) for l in ls) for pt, ls in points(a)) or \
                any(a.HitTest(pt) and any(a.IsOnLayer(l) for l in ls) for pt, ls in points(b))
            if touch:
                parent[find(i)] = find(j)
    groups = {}
    for i, it in enumerate(items):
        groups.setdefault(find(i), []).append(it)
    return list(groups.values())


def on_board(x, y, keep):
    """Inside the rounded board outline by at least `keep` mm."""
    h, r = gen_pcb.SIZE / 2 - keep, gen_pcb.CORNER_R - keep
    dx, dy = abs(x - gen_pcb.CX), abs(y - gen_pcb.CY)
    if dx > h or dy > h:
        return False
    cx, cy = h - r, h - r
    return not (dx > cx and dy > cy and math.hypot(dx - cx, dy - cy) > r)


def maze_route(board, net, src, dst, margin=4.0, step=0.05, width=0.2):
    """Grid A* between two islands of a net on F.Cu, B.Cu and (inside the window,
    see gen_pcb.IN2_WINDOW) In2, with vias. Returns True on success."""
    import heapq
    layers = (pcbnew.F_Cu, pcbnew.In2_Cu, pcbnew.B_Cu)
    pts = lambda grp: [(to_mm(p), {l for l in layers if it.IsOnLayer(l)})
                       for it in grp for p in ([it.GetStart(), it.GetEnd()] if it.GetClass() == "PCB_TRACK"
                                               else [it.GetPosition()])]
    S, D = pts(src), pts(dst)
    xs = [p[0] for p, _ in S + D]
    ys = [p[1] for p, _ in S + D]
    x0, y0 = min(xs) - margin, min(ys) - margin
    nx, ny = int((max(xs) + margin - x0) / step) + 1, int((max(ys) + margin - y0) / step) + 1
    clear = 0.15 + step         # rule + one grid step: diagonal moves cut cell corners
    # blocked[l][i] for track centre lines; vblock[i] for via centres
    blocked = [bytearray(nx * ny) for _ in layers]
    vblock = bytearray(nx * ny)


    def mark(arr, cx0, cy0, cx1, cy1, inside):
        for gy in range(max(0, int((cy0 - y0) / step)), min(ny, int((cy1 - y0) / step) + 2)):
            for gx in range(max(0, int((cx0 - x0) / step)), min(nx, int((cx1 - x0) / step) + 2)):
                if inside(x0 + gx * step, y0 + gy * step):
                    arr[gy * nx + gx] = 1

    tr_r, via_r = width / 2 + clear, VIA_D / 2 + clear
    for fp in board.Footprints():
        for p in fp.Pads():
            if p.GetNetname() == net and p.GetNetname():
                continue
            r = pad_rect(p)
            for k, l in enumerate(layers):
                if p.IsOnLayer(l):
                    mark(blocked[k], r[0] - tr_r, r[1] - tr_r, r[2] + tr_r, r[3] + tr_r,
                         lambda x, y, r=r: rect_dist(x, y, r) < tr_r)
            mark(vblock, r[0] - via_r, r[1] - via_r, r[2] + via_r, r[3] + via_r,
                 lambda x, y, r=r: rect_dist(x, y, r) < via_r)
    for t in board.GetTracks():
        if t.GetNetname() == net:
            continue
        if t.GetClass() == "PCB_VIA":
            (cx, cy), rr = to_mm(t.GetPosition()), pcbnew.ToMM(t.GetWidth(pcbnew.F_Cu)) / 2
            for k in range(len(layers)):
                mark(blocked[k], cx - rr - tr_r, cy - rr - tr_r, cx + rr + tr_r, cy + rr + tr_r,
                     lambda x, y: math.hypot(x - cx, y - cy) < rr + tr_r)
            mark(vblock, cx - rr - via_r, cy - rr - via_r, cx + rr + via_r, cy + rr + via_r,
                 lambda x, y: math.hypot(x - cx, y - cy) < max(rr + via_r, VIA_D + 0.25))
            continue
        if t.GetLayer() not in layers:
            continue
        a, b, w = to_mm(t.GetStart()), to_mm(t.GetEnd()), pcbnew.ToMM(t.GetWidth()) / 2
        k = layers.index(t.GetLayer())
        bx0, by0 = min(a[0], b[0]) - w, min(a[1], b[1]) - w
        bx1, by1 = max(a[0], b[0]) + w, max(a[1], b[1]) + w
        mark(blocked[k], bx0 - tr_r, by0 - tr_r, bx1 + tr_r, by1 + tr_r,
             lambda x, y: point_seg_dist(x, y, a, b) < w + tr_r)
        mark(vblock, bx0 - via_r, by0 - via_r, bx1 + via_r, by1 + via_r,
             lambda x, y: point_seg_dist(x, y, a, b) < w + via_r)
    for z in board.Zones():        # rule areas: no tracks there (In2 is open only in its window)
        if z.GetIsRuleArea() and z.GetDoNotAllowTracks():
            bb = z.GetBoundingBox()
            k = layers.index(z.GetLayer()) if z.GetLayer() in layers else None
            if k is not None:
                r = (pcbnew.ToMM(bb.GetLeft()), pcbnew.ToMM(bb.GetTop()),
                     pcbnew.ToMM(bb.GetRight()), pcbnew.ToMM(bb.GetBottom()))
                mark(blocked[k], r[0] - tr_r, r[1] - tr_r, r[2] + tr_r, r[3] + tr_r,
                     lambda x, y, r=r: rect_dist(x, y, r) < width / 2)
    cell = lambda x, y: (min(nx - 1, max(0, round((x - x0) / step))), min(ny - 1, max(0, round((y - y0) / step))))
    goal = {}
    for (x, y), ls in D:
        gx, gy = cell(x, y)
        for l in ls:
            goal[(layers.index(l), gx, gy)] = True
    heap, came, cost = [], {}, {}
    gxs = [g[1] for g in goal]
    gys = [g[2] for g in goal]
    hfun = lambda gx, gy: min(math.hypot(gx - a, gy - b) for a, b in zip(gxs, gys))
    for (x, y), ls in S:
        gx, gy = cell(x, y)
        for l in ls:
            s = (layers.index(l), gx, gy)
            cost[s] = 0
            heapq.heappush(heap, (hfun(gx, gy), 0, s))
    moves = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1), (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]
    end = None
    near_start = {(k, gx, gy) for k, gx, gy in cost}
    while heap:
        _, c, node = heapq.heappop(heap)
        if c > cost.get(node, 1e18):
            continue
        if node in goal:
            end = node
            break
        k, gx, gy = node
        nbrs = []
        for dx, dy, mc in moves:
            ex, ey = gx + dx, gy + dy
            if 0 <= ex < nx and 0 <= ey < ny:
                if not on_board(x0 + ex * step, y0 + ey * step, 0.3 + width / 2 + 0.05):
                    continue
                nbrs.append(((k, ex, ey), mc + (0.4 if (dx, dy) != came_dir(came, node) else 0)))
        if not vblock[gy * nx + gx]:
            nbrs += [((j, gx, gy), 40) for j in range(len(layers)) if j != k]
        for nb, mc in nbrs:
            nk, ex, ey = nb
            # own-net copper at the ends is fine; elsewhere respect the masks
            if blocked[nk][ey * nx + ex] and nb not in goal and nb not in near_start:
                continue
            nc = c + mc
            if nc < cost.get(nb, 1e18):
                cost[nb] = nc
                came[nb] = node
                heapq.heappush(heap, (nc + hfun(ex, ey), nc, nb))
    if end is None:
        return False
    path = [end]
    while path[-1] in came:
        path.append(came[path[-1]])
    path.reverse()
    nets = board.GetNetsByName()
    xy = lambda n: pcbnew.VECTOR2I(mm(x0 + n[1] * step), mm(y0 + n[2] * step))
    run = [path[0]]
    for prev, node in zip(path, path[1:]):
        if node[0] != prev[0]:                     # layer change: via
            v = pcbnew.PCB_VIA(board)
            v.SetPosition(xy(node))
            v.SetWidth(mm(VIA_D))
            v.SetDrill(mm(VIA_DRILL))
            v.SetNet(nets[net])
            board.Add(v)
            emit_run(board, run, xy, nets[net], layers, width)
            run = [node]
        else:
            run.append(node)
    emit_run(board, run, xy, nets[net], layers, width)
    return True


def came_dir(came, node):
    p = came.get(node)
    if p is None or p[0] != node[0]:
        return None
    return (node[1] - p[1], node[2] - p[2])


def emit_run(board, run, xy, netinfo, layers, width):
    """Tracks along a run of grid cells on one layer, merged into straight pieces."""
    if len(run) < 2:
        return
    corners = [run[0]]
    for a, b, c in zip(run, run[1:], run[2:]):
        if (b[1] - a[1], b[2] - a[2]) != (c[1] - b[1], c[2] - b[2]):
            corners.append(b)
    corners.append(run[-1])
    for a, b in zip(corners, corners[1:]):
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(xy(a))
        t.SetEnd(xy(b))
        t.SetWidth(mm(width))
        t.SetLayer(layers[a[0]])
        t.SetNet(netinfo)
        board.Add(t)


def finish(board):
    """Join what the autorouter left open (signal nets; plane nets go through vias/pours)."""
    names = {p.GetNetname() for fp in board.Footprints() for p in fp.Pads()}
    join_nets(board, sorted(n for n in names if n and n not in PLANE_NETS
                            and not n.startswith("unconnected-")))


def join_nets(board, names):
    """Connect each net's islands with maze_route, in the order given."""
    for net in names:
        groups = islands(board, net)
        while len(groups) > 1:
            groups.sort(key=len)
            src, dst = groups[0], [it for g in groups[1:] for it in g]
            if not (maze_route(board, net, src, dst) or maze_route(board, net, src, dst, margin=10.0)):
                print(f"maze: could not join {net}")
                break
            print(f"maze: joined {net}")
            groups = islands(board, net)


def main():
    pcb = os.path.join(HERE, gen_pcb.PROJECT + ".kicad_pcb")
    build = os.path.join(HERE, "build")
    os.makedirs(build, exist_ok=True)
    if not os.environ.get("FREEROUTING_JAR"):
        raise SystemExit("set FREEROUTING_JAR to the Freerouting 2.x jar")

    board = pcbnew.LoadBoard(pcb)
    if len(board.GetTracks()):
        if "--force" not in sys.argv:
            raise SystemExit(f"{pcb} already has tracks (pass --force to reroute from scratch)")
        for t in list(board.GetTracks()):
            board.Delete(t)

    for z in [z for z in board.Zones() if not z.GetIsRuleArea()
              and z.GetLayer() in (pcbnew.F_Cu, pcbnew.B_Cu)]:
        board.Delete(z)

    fanout(board)
    join_nets(board, PREROUTE)
    for t in board.GetTracks():
        t.SetLocked(True)
    for stage, nets in STAGES:
        freeroute(board, build, stage, set(nets))
        for t in board.GetTracks():
            t.SetLocked(True)
    freeroute(board, build, "all")
    drop_dangling(board)
    finish(board)
    for tr in board.GetTracks():
        if tr.GetClass() != "PCB_VIA" and tr.GetWidth() < mm(MIN_TRACK):
            tr.SetWidth(mm(MIN_TRACK))

    # Restore the outer pours, then refill everything from disk (see gen_pcb.py).
    nets = board.GetNetsByName()
    gen_pcb.add_zone(board, pcbnew.F_Cu, nets["/GND"])
    gen_pcb.add_zone(board, pcbnew.B_Cu, nets["/GND"])
    board.Save(pcb)
    board = pcbnew.LoadBoard(pcb)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    board.Save(pcb)


if __name__ == "__main__":
    main()
