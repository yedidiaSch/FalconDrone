#!/usr/bin/env python3
"""
Route falcon_rc.kicad_pcb (as placed by gen_pcb.py) with Freerouting.

  1. Fan out: every SMD pad on GND gets a short stub and a via. Every SMD part
     sits on B.Cu, so F.Cu (through-hole controls only) is close to a solid GND
     plane, and the vias tie each ground pad straight into it. Locked, so the
     autorouter leaves them alone.
  2. Route every net except GND with Freerouting (GND pins are stripped from
     the DSN; GND is the pours plus the fanout vias).
  3. Drop the router's dangling leftovers, join anything it left open with a
     small grid maze router, put the GND pours back, stitch the two pours
     together with a grid of GND vias (plus a via in any pour patch the grid
     missed), route a GND track out of any patch still walled in by tracks on
     both layers, and refill.

Same approach as hardware/flight_controller/route_pcb.py, cut down to two
layers and no staged pre-routing.

Needs Java 25+ and the Freerouting 2.x jar:
  FREEROUTING_JAR=/path/freerouting.jar JAVA=/path/java python3 route_pcb.py [--force]

--force strips existing tracks first (otherwise a routed board is left alone).
"""

import fnmatch
import heapq
import math
import os
import re
import subprocess
import sys

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gen_pcb  # noqa: E402  (outline, zones)

mm = pcbnew.FromMM
GND = "/GND"
VIA_D, VIA_DRILL = 0.6, 0.3
STUB_W = 0.3
CLEAR = 0.2                    # fanout copper-to-copper, a bit above the 0.15 rule
EDGE_KEEP = 0.8                # via centre to board edge
PASSES = 100
MIN_TRACK = 0.13               # Freerouting necks down to 0.12 at pads; JLCPCB min is 0.127
LAYERS = (pcbnew.F_Cu, pcbnew.B_Cu)


def to_mm(v):
    return pcbnew.ToMM(v.x), pcbnew.ToMM(v.y)


def rect_dist(px, py, r):
    """Distance from a point to an axis-aligned rect (x0, y0, x1, y1); 0 inside."""
    dx = max(r[0] - px, 0, px - r[2])
    dy = max(r[1] - py, 0, py - r[3])
    return math.hypot(dx, dy)


def point_seg_dist(px, py, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    L = dx * dx + dy * dy
    u = 0 if L == 0 else max(0, min(1, ((px - a[0]) * dx + (py - a[1]) * dy) / L))
    return math.hypot(px - a[0] - u * dx, py - a[1] - u * dy)


def seg_rect_dist(a, b, r, steps=12):
    return min(rect_dist(a[0] + (b[0] - a[0]) * t / steps, a[1] + (b[1] - a[1]) * t / steps, r)
               for t in range(steps + 1))


def pad_rect(pad):
    bb = pad.GetBoundingBox()
    return (pcbnew.ToMM(bb.GetLeft()), pcbnew.ToMM(bb.GetTop()),
            pcbnew.ToMM(bb.GetRight()), pcbnew.ToMM(bb.GetBottom()))


def on_board(x, y, keep):
    """Inside the rounded board outline by at least `keep` mm."""
    x0, y0 = gen_pcb.X0 + keep, gen_pcb.Y0 + keep
    x1, y1 = gen_pcb.X0 + gen_pcb.W - keep, gen_pcb.Y0 + gen_pcb.H - keep
    if not (x0 <= x <= x1 and y0 <= y <= y1):
        return False
    r = gen_pcb.CORNER_R - keep
    cx = min(max(x, x0 + r), x1 - r)
    cy = min(max(y, y0 + r), y1 - r)
    return math.hypot(x - cx, y - cy) <= r


def keepouts(board):
    """Rule areas that forbid vias (the ESP32 antenna keepout)."""
    out = []
    for z in list(board.Zones()) + [z for fp in board.Footprints() for z in fp.Zones()]:
        if z.GetIsRuleArea() and z.GetDoNotAllowVias():
            out.append(pad_rect(z))
    return out


def thermal_vias(board, ref, pitch=0.75):
    """2 x 2 vias in a part's exposed pad: the BQ24073's heat path into the
    F.Cu GND plane (and its only GND path the fanout cannot reach)."""
    fp = board.FindFootprintByReference(ref)
    ep = max(fp.Pads(), key=lambda p: p.GetSize(pcbnew.B_Cu).x * p.GetSize(pcbnew.B_Cu).y)
    cx, cy = to_mm(ep.GetPosition())
    for dx in (-pitch / 2, pitch / 2):
        for dy in (-pitch / 2, pitch / 2):
            v = pcbnew.PCB_VIA(board)
            v.SetPosition(pcbnew.VECTOR2I(mm(cx + dx), mm(cy + dy)))
            v.SetWidth(mm(0.55))
            v.SetDrill(mm(0.3))
            v.SetNet(ep.GetNet())
            v.SetLocked(True)
            board.Add(v)
    print(f"thermal vias: 4 in {ref}.{ep.GetNumber()} ({ep.GetNetname()})")


def fanout(board):
    pads = [p for fp in board.Footprints() for p in fp.Pads()]
    info = [(p, pad_rect(p), {l for l in LAYERS if p.IsOnLayer(l)},
             p.GetAttribute() in (pcbnew.PAD_ATTRIB_PTH, pcbnew.PAD_ATTRIB_NPTH)) for p in pads]
    no_via = keepouts(board)
    nets = board.GetNetsByName()
    vias = []
    placed, failed = 0, []
    for p, rect, layers, th in info:
        if p.GetNetname() != GND or th:
            continue
        px, py = to_mm(p.GetPosition())
        fx, fy = to_mm(p.GetParentFootprint().GetPosition())
        out = math.atan2(py - fy, px - fx) if (px, py) != (fx, fy) else 0.0
        best = None
        for k in range(8):
            ang = k * math.pi / 4
            diff = abs((ang - out + math.pi) % (2 * math.pi) - math.pi)
            for i in range(25):
                d = 0.5 + 0.1 * i
                score = d + diff * 0.6
                if best and score >= best[0]:
                    break
                vx, vy = px + d * math.cos(ang), py + d * math.sin(ang)
                if not on_board(vx, vy, EDGE_KEEP):
                    break
                if any(rect_dist(vx, vy, r) < VIA_D / 2 + 0.1 for r in no_via):
                    continue
                if any(math.hypot(vx - x, vy - y) < VIA_D + 0.25 for x, y in vias):
                    continue
                ok = True
                for q, r, ql, qth in info:
                    if q is p:
                        continue
                    same = q.GetNetname() == GND and not qth
                    need = VIA_D / 2 + (0.1 if same else CLEAR)
                    if qth and not q.GetNetname():
                        need = VIA_D / 2 + 0.5          # mounting / locating holes
                    if rect_dist(vx, vy, r) < need or (
                            not same and pcbnew.B_Cu in ql and
                            seg_rect_dist((px, py), (vx, vy), r) < STUB_W / 2 + CLEAR):
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
        t.SetLayer(pcbnew.B_Cu)
        t.SetNet(nets[GND])
        t.SetLocked(True)
        board.Add(t)
        v = pcbnew.PCB_VIA(board)
        v.SetPosition(pcbnew.VECTOR2I(mm(vx), mm(vy)))
        v.SetWidth(mm(VIA_D))
        v.SetDrill(mm(VIA_DRILL))
        v.SetNet(nets[GND])
        v.SetLocked(True)
        board.Add(v)
        vias.append((vx, vy))
        placed += 1
    for p in failed:
        print(f"  fanout: no room at {p.GetParentFootprint().GetReference()}.{p.GetNumber()}"
              " (left to the bottom pour)")
    print(f"fanout: {placed} GND vias")


def export_dsn(board, path):
    if not pcbnew.ExportSpecctraDSN(board, path):
        raise SystemExit("DSN export failed")
    t = open(path).read()
    # Freerouting rounds to within a micron of the rule; give it 10 um of margin.
    t = t.replace("(clearance 150)", "(clearance 160)")
    # GND is the pours: strip its pins so the router leaves it alone.
    t = re.sub(r'\(net ("?/GND"?)\s*\(pins[^)]*\)\s*\)', r"(net \1\n    )", t)
    open(path, "w").write(t)


def freeroute(board, build):
    dsn, ses = (os.path.join(build, f"{gen_pcb.PROJECT}.{ext}") for ext in ("dsn", "ses"))
    export_dsn(board, dsn)
    logp = os.path.join(build, "freerouting.log")
    with open(logp, "w") as log:
        env = dict(os.environ, FREEROUTING__ROUTER__COPPER_TO_EDGE_CLEARANCE_UM="350")
        subprocess.run([os.environ.get("JAVA", "java"), "-Djava.awt.headless=true", "-jar",
                        os.environ["FREEROUTING_JAR"], "-de", dsn, "-do", ses, "-mp", str(PASSES)],
                       check=True, stdout=log, stderr=subprocess.STDOUT, env=env)
    if not pcbnew.ImportSpecctraSES(board, ses):
        raise SystemExit("SES import failed")
    m = re.findall(r"final score: [\d.]+ \((\d+) unrouted", open(logp).read())
    print(f"freerouting: {len(board.GetTracks())} tracks and vias, "
          f"{m[-1] if m else '?'} unrouted")


def drop_dangling(board):
    """Remove router leftovers: vias reaching one layer only, tracks ending in nothing."""
    removed = 0
    pads = [p for fp in board.Footprints() for p in fp.Pads()]
    while True:
        items = list(board.GetTracks())
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
                if t.GetNetname() != GND and len(ends.get((pos.x, pos.y), ())) < 2:
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
        return [(it.GetPosition(), {l for l in LAYERS if it.IsOnLayer(l)})]

    for i, a in enumerate(items):
        for j in range(i + 1, len(items)):
            b = items[j]
            if find(i) == find(j):
                continue
            if any(b.HitTest(pt) and any(b.IsOnLayer(l) for l in ls) for pt, ls in points(a)) or \
               any(a.HitTest(pt) and any(a.IsOnLayer(l) for l in ls) for pt, ls in points(b)):
                parent[find(i)] = find(j)
    groups = {}
    for i, it in enumerate(items):
        groups.setdefault(find(i), []).append(it)
    return list(groups.values())


def maze_route(board, net, src, dst, margin=4.0, step=0.05, width=0.2):
    """Grid A* between two islands of a net on F.Cu / B.Cu, with vias."""
    pts = lambda grp: [(to_mm(p), {l for l in LAYERS if it.IsOnLayer(l)})
                       for it in grp for p in ([it.GetStart(), it.GetEnd()] if it.GetClass() == "PCB_TRACK"
                                               else [it.GetPosition()])]
    S, D = pts(src), pts(dst)
    xs = [p[0] for p, _ in S + D]
    ys = [p[1] for p, _ in S + D]
    x0, y0 = min(xs) - margin, min(ys) - margin
    nx, ny = int((max(xs) + margin - x0) / step) + 1, int((max(ys) + margin - y0) / step) + 1
    clear = 0.15 + step
    blocked = [bytearray(nx * ny) for _ in LAYERS]
    vblock = bytearray(nx * ny)

    def mark(arr, cx0, cy0, cx1, cy1, inside):
        for gy in range(max(0, int((cy0 - y0) / step)), min(ny, int((cy1 - y0) / step) + 2)):
            for gx in range(max(0, int((cx0 - x0) / step)), min(nx, int((cx1 - x0) / step) + 2)):
                if inside(x0 + gx * step, y0 + gy * step):
                    arr[gy * nx + gx] = 1

    tr_r, via_r = width / 2 + clear, VIA_D / 2 + clear
    for fp in board.Footprints():
        for p in fp.Pads():
            if p.GetNetname() == net and net:
                continue
            r = pad_rect(p)
            for k, l in enumerate(LAYERS):
                if p.IsOnLayer(l):
                    mark(blocked[k], r[0] - tr_r, r[1] - tr_r, r[2] + tr_r, r[3] + tr_r,
                         lambda x, y, r=r: rect_dist(x, y, r) < tr_r)
            mark(vblock, r[0] - via_r, r[1] - via_r, r[2] + via_r, r[3] + via_r,
                 lambda x, y, r=r: rect_dist(x, y, r) < via_r)
    for r in keepouts(board):
        for arr, rr in ((blocked[0], tr_r), (blocked[1], tr_r), (vblock, via_r)):
            mark(arr, r[0] - rr, r[1] - rr, r[2] + rr, r[3] + rr,
                 lambda x, y, r=r, rr=rr: rect_dist(x, y, r) < rr)
    for t in board.GetTracks():
        if t.GetNetname() == net:
            continue
        if t.GetClass() == "PCB_VIA":
            (cx, cy), rr = to_mm(t.GetPosition()), pcbnew.ToMM(t.GetWidth(pcbnew.F_Cu)) / 2
            for k in range(len(LAYERS)):
                mark(blocked[k], cx - rr - tr_r, cy - rr - tr_r, cx + rr + tr_r, cy + rr + tr_r,
                     lambda x, y: math.hypot(x - cx, y - cy) < rr + tr_r)
            mark(vblock, cx - rr - via_r, cy - rr - via_r, cx + rr + via_r, cy + rr + via_r,
                 lambda x, y: math.hypot(x - cx, y - cy) < max(rr + via_r, VIA_D + 0.25))
            continue
        a, b, w = to_mm(t.GetStart()), to_mm(t.GetEnd()), pcbnew.ToMM(t.GetWidth()) / 2
        k = LAYERS.index(t.GetLayer())
        bx0, by0 = min(a[0], b[0]) - w, min(a[1], b[1]) - w
        bx1, by1 = max(a[0], b[0]) + w, max(a[1], b[1]) + w
        mark(blocked[k], bx0 - tr_r, by0 - tr_r, bx1 + tr_r, by1 + tr_r,
             lambda x, y: point_seg_dist(x, y, a, b) < w + tr_r)
        mark(vblock, bx0 - via_r, by0 - via_r, bx1 + via_r, by1 + via_r,
             lambda x, y: point_seg_dist(x, y, a, b) < w + via_r)
    cell = lambda x, y: (min(nx - 1, max(0, round((x - x0) / step))), min(ny - 1, max(0, round((y - y0) / step))))
    goal = {}
    for (x, y), ls in D:
        gx, gy = cell(x, y)
        for l in ls:
            goal[(LAYERS.index(l), gx, gy)] = True
    gxs = [g[1] for g in goal]
    gys = [g[2] for g in goal]
    hfun = lambda gx, gy: min(math.hypot(gx - a, gy - b) for a, b in zip(gxs, gys))
    heap, came, cost = [], {}, {}
    for (x, y), ls in S:
        gx, gy = cell(x, y)
        for l in ls:
            s = (LAYERS.index(l), gx, gy)
            cost[s] = 0
            heapq.heappush(heap, (hfun(gx, gy), 0, s))
    near_start = set(cost)
    moves = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1),
             (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]
    end = None
    while heap:
        _, c, node = heapq.heappop(heap)
        if c > cost.get(node, 1e18):
            continue
        if node in goal:
            end = node
            break
        k, gx, gy = node
        p = came.get(node)
        cdir = None if p is None or p[0] != k else (gx - p[1], gy - p[2])
        nbrs = []
        for dx, dy, mc in moves:
            ex, ey = gx + dx, gy + dy
            if 0 <= ex < nx and 0 <= ey < ny and on_board(x0 + ex * step, y0 + ey * step, 0.35 + width / 2):
                nbrs.append(((k, ex, ey), mc + (0.4 if (dx, dy) != cdir else 0)))
        if not vblock[gy * nx + gx]:
            nbrs.append(((1 - k, gx, gy), 40))
        for nb, mc in nbrs:
            nk, ex, ey = nb
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
    netinfo = board.GetNetsByName()[net]
    xy = lambda n: pcbnew.VECTOR2I(mm(x0 + n[1] * step), mm(y0 + n[2] * step))
    run = [path[0]]
    for prev, node in zip(path, path[1:]):
        if node[0] != prev[0]:
            v = pcbnew.PCB_VIA(board)
            v.SetPosition(xy(node))
            v.SetWidth(mm(VIA_D))
            v.SetDrill(mm(VIA_DRILL))
            v.SetNet(netinfo)
            board.Add(v)
            emit_run(board, run, xy, netinfo, width)
            run = [node]
        else:
            run.append(node)
    emit_run(board, run, xy, netinfo, width)
    return True


def emit_run(board, run, xy, netinfo, width):
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
        t.SetLayer(LAYERS[a[0]])
        t.SetNet(netinfo)
        board.Add(t)


def class_width(net):
    """Track width of the net's class, from the same table gen_pcb writes."""
    for name, width, *_, patterns in gen_pcb.NETCLASSES[1:]:
        if any(fnmatch.fnmatch(net, "/" + pat) for pat in patterns):
            return width
    return gen_pcb.NETCLASSES[0][1]


def finish(board):
    """Join what the autorouter left open (every net but GND, which is the pours),
    at the net's class width."""
    names = sorted({p.GetNetname() for fp in board.Footprints() for p in fp.Pads()}
                   - {"", GND})
    for net in names:
        if net.startswith("unconnected-"):
            continue
        groups = islands(board, net)
        while len(groups) > 1:
            groups.sort(key=len)
            src, dst = groups[0], [it for g in groups[1:] for it in g]
            w = class_width(net)
            if not (maze_route(board, net, src, dst, width=w) or
                    maze_route(board, net, src, dst, margin=12.0, width=w)):
                print(f"maze: could not join {net}")
                break
            print(f"maze: joined {net}")
            groups = islands(board, net)


def gnd_clusters(board):
    """GND split into electrically connected groups, pours included: pad
    outlines and track ends touching each filled fragment, vias and
    through-hole pads joining the layers. Returns the groups' pads, tracks and
    vias, largest group last."""
    frags = []
    for z in board.Zones():
        if z.GetIsRuleArea() or z.GetNetname() != GND:
            continue
        for l in LAYERS:
            if z.IsOnLayer(l):
                polys = z.GetFilledPolysList(l)
                frags += [(l, polys.Outline(i)) for i in range(polys.OutlineCount())]
    items = [p for fp in board.Footprints() for p in fp.Pads() if p.GetNetname() == GND]
    items += [t for t in board.GetTracks() if t.GetNetname() == GND]
    n = len(items)
    parent = list(range(n + len(frags)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    acc = mm(0.1)
    for i, it in enumerate(items):
        for k, (l, o) in enumerate(frags):
            if not it.IsOnLayer(l):
                continue
            if it.GetClass() == "PCB_TRACK":
                pts = [it.GetStart(), it.GetEnd()]
            elif it.GetClass() == "PCB_VIA":
                pts = [it.GetPosition()]
            else:
                poly = it.GetEffectivePolygon(l)
                pts = [poly.Outline(j).CPoint(q) for j in range(poly.OutlineCount())
                       for q in range(poly.Outline(j).PointCount())]
            if any(o.PointInside(p, acc) for p in pts) or (
                    it.GetClass() == "PAD" and o.BBox().Intersects(it.GetBoundingBox()) and
                    any(it.HitTest(o.CPoint(q), acc) for q in range(o.PointCount()))):
                parent[find(i)] = find(n + k)
    index = {it.m_Uuid.AsString(): i for i, it in enumerate(items)}
    for grp in islands(board, GND):                   # direct pad / track / via contact
        idx = [index[it.m_Uuid.AsString()] for it in grp]
        for j in idx[1:]:
            parent[find(j)] = find(idx[0])
    groups = {}
    for i, it in enumerate(items):
        groups.setdefault(find(i), []).append(it)
    return sorted(groups.values(), key=len)


def join_gnd_islands(board):
    """A patch of pour walled in by tracks on both layers keeps its own vias
    but never meets the main pour: route a GND track out of it."""
    for _ in range(6):
        pcbnew.ZONE_FILLER(board).Fill(board.Zones())
        groups = gnd_clusters(board)
        if len(groups) == 1:
            return
        src = groups[0]
        cx = sum(to_mm(it.GetPosition())[0] for it in src) / len(src)
        cy = sum(to_mm(it.GetPosition())[1] for it in src) / len(src)
        dst = [it for it in groups[-1] if it.GetClass() != "PAD" and
               math.hypot(to_mm(it.GetPosition())[0] - cx, to_mm(it.GetPosition())[1] - cy) < 15]
        w = class_width(GND)
        if not (maze_route(board, GND, src, dst, width=w) or
                maze_route(board, GND, src, dst, margin=12.0, width=w)):
            names = sorted(f"{it.GetParentFootprint().GetReference()}.{it.GetNumber()}"
                           for it in src if it.GetClass() == "PAD")
            print(f"maze: could not join GND group {names[:8]}")
            return
        print(f"maze: joined a GND island ({len(src)} items)")


STITCH_AVOID = ("U5", "J1", "BT1", "SW1")   # module / connector / holder bodies sit flat here


def stitch(board, pitch=6.0, via_d=0.6, drill=0.3):
    """GND stitching vias on a grid, wherever both pours have copper with room
    for the via. Islands are kept for the test (fill with area-based removal),
    so a patch of pour boxed in by tracks gets a via and survives the final
    fill instead of being dropped."""
    zones = [z for z in board.Zones() if not z.GetIsRuleArea() and z.GetNetname() == GND]
    for z in zones:
        z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_AREA)
        z.SetMinIslandArea(int(mm(1) * mm(1) * 4))           # keep islands >= 4 mm2
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    by_layer = {z.GetLayer(): z for z in zones}
    no_via = keepouts(board)
    r = via_d / 2 + 0.1
    ring = [(r * math.cos(k * math.pi / 4), r * math.sin(k * math.pi / 4)) for k in range(8)] + [(0, 0)]
    gndnet = board.GetNetsByName()[GND]
    avoid = []
    for ref in STITCH_AVOID:
        fp = board.FindFootprintByReference(ref)
        bb = fp.GetCourtyard(pcbnew.B_Cu if fp.IsFlipped() else pcbnew.F_Cu).BBox()
        avoid.append((pcbnew.ToMM(bb.GetLeft()), pcbnew.ToMM(bb.GetTop()),
                      pcbnew.ToMM(bb.GetRight()), pcbnew.ToMM(bb.GetBottom())))
    holes = [(*to_mm(p.GetPosition()), pcbnew.ToMM(max(p.GetDrillSize().x, p.GetDrillSize().y)) / 2)
             for fp in board.Footprints() for p in fp.Pads() if p.GetDrillSize().x > 0]
    holes += [(*to_mm(t.GetPosition()), pcbnew.ToMM(t.GetDrill()) / 2)
              for t in board.GetTracks() if t.GetClass() == "PCB_VIA"]
    def fits(x, y, keepaway):
        return on_board(x, y, 1.5) and not any(rect_dist(x, y, k) < 0.5 for k in keepaway) and \
            all(math.hypot(x - hx, y - hy) > hr + drill / 2 + 0.35 for hx, hy, hr in holes) and all(
                by_layer[l].HitTestFilledArea(l, pcbnew.VECTOR2I(mm(x + dx), mm(y + dy)), 0)
                for l in LAYERS for dx, dy in ring)

    def add_via(x, y):
        v = pcbnew.PCB_VIA(board)
        v.SetPosition(pcbnew.VECTOR2I(mm(x), mm(y)))
        v.SetWidth(mm(via_d))
        v.SetDrill(mm(drill))
        v.SetNet(gndnet)
        v.SetLocked(True)
        board.Add(v)
        holes.append((x, y, drill / 2))

    added = 0
    y = gen_pcb.Y0 + 2.5
    while y < gen_pcb.Y0 + gen_pcb.H - 2:
        x = gen_pcb.X0 + 2.5
        while x < gen_pcb.X0 + gen_pcb.W - 2:
            if fits(x, y, no_via + avoid):
                add_via(x, y)
                added += 1
            x += pitch
        y += pitch

    # Island rescue: a pour fragment boxed in by tracks that the grid missed
    # would hang on its pads alone (or split the net), so give every fragment
    # of 4 mm2 or more a via of its own, searched on a fine grid. The avoid
    # courtyards are allowed here; the keepouts still apply.
    rescued = 0
    for _ in range(3):
        pcbnew.ZONE_FILLER(board).Fill(board.Zones())
        vias = [t.GetPosition() for t in board.GetTracks() if t.GetClass() == "PCB_VIA"
                and t.GetNetname() == GND]
        todo = []
        for l in LAYERS:
            polys = by_layer[l].GetFilledPolysList(l)
            for i in range(polys.OutlineCount()):
                o = polys.Outline(i)
                if o.Area() >= mm(1) * mm(1) * 4 and not any(o.PointInside(v) for v in vias):
                    todo.append(o)
        if not todo:
            break
        for o in todo:
            bb = o.BBox()
            x0, y0 = pcbnew.ToMM(bb.GetLeft()), pcbnew.ToMM(bb.GetTop())
            x1, y1 = pcbnew.ToMM(bb.GetRight()), pcbnew.ToMM(bb.GetBottom())
            spots = [(x0 + 0.25 * i, y0 + 0.25 * j) for i in range(int((x1 - x0) / 0.25) + 1)
                     for j in range(int((y1 - y0) / 0.25) + 1)]
            cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
            for x, y in sorted(spots, key=lambda p: math.hypot(p[0] - cx, p[1] - cy)):
                if o.PointInside(pcbnew.VECTOR2I(mm(x), mm(y))) and fits(x, y, no_via):
                    add_via(x, y)
                    rescued += 1
                    break
    for z in zones:
        z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
    print(f"stitching: {added} GND vias + {rescued} island rescue")


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
    for z in [z for z in board.Zones() if not z.GetIsRuleArea()]:
        board.Delete(z)

    thermal_vias(board, "U2")
    fanout(board)
    for t in board.GetTracks():
        t.SetLocked(True)
    freeroute(board, build)
    drop_dangling(board)
    finish(board)
    for tr in board.GetTracks():
        if tr.GetClass() != "PCB_VIA" and tr.GetWidth() < mm(MIN_TRACK):
            tr.SetWidth(mm(MIN_TRACK))

    # Restore the pours, then refill from disk (see gen_pcb.py).
    nets = board.GetNetsByName()
    gen_pcb.add_zone(board, pcbnew.F_Cu, nets[GND])
    gen_pcb.add_zone(board, pcbnew.B_Cu, nets[GND])
    board.Save(pcb)
    board = pcbnew.LoadBoard(pcb)
    stitch(board)
    join_gnd_islands(board)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    board.Save(pcb)
    print(f"wrote {pcb}")


if __name__ == "__main__":
    main()
