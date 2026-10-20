"""
Shared machinery for the script-generated KiCad 9 schematics in hardware/.

A board script calls init(project, out_dir), describes its circuit with
part() / R() / C() / header() / note(), then calls write_sch(title) and
write_project_lib(symbols).

Every pin is connected with a net label placed on the pin end (no wires),
so connectivity is exact and the file stays diff-friendly. Symbols are
copied from the stock KiCad library, flattened if derived. Parts missing
from it are built with custom_symbol() and written to the project library.
"""

import os
import uuid as uuidlib

LIB_DIR = "/usr/share/kicad/symbols"

OUT_DIR = None
PROJECT = None
ROOT_UUID = None

_uuid_counter = 0


def init(project, out_dir):
    global PROJECT, OUT_DIR, ROOT_UUID
    PROJECT, OUT_DIR = project, out_dir
    ROOT_UUID = str(uuidlib.uuid5(uuidlib.NAMESPACE_URL, f"{project}/root"))


def new_uuid():
    """Deterministic UUIDs so regenerating gives a clean diff."""
    global _uuid_counter
    _uuid_counter += 1
    return str(uuidlib.uuid5(uuidlib.NAMESPACE_URL, f"{PROJECT}/{_uuid_counter}"))


# ---------------------------------------------------------------- s-expressions

class Q(str):
    """A quoted string atom."""


def parse_sexp(text):
    tokens, i, n = [], 0, len(text)
    while i < n:
        c = text[i]
        if c in "()":
            tokens.append(c)
            i += 1
        elif c.isspace():
            i += 1
        elif c == '"':
            j, buf = i + 1, []
            while text[j] != '"':
                if text[j] == "\\":
                    j += 1
                buf.append(text[j])
                j += 1
            tokens.append(Q("".join(buf)))
            i = j + 1
        else:
            j = i
            while j < n and not text[j].isspace() and text[j] not in '()"':
                j += 1
            tokens.append(text[i:j])
            i = j
    stack = [[]]
    for t in tokens:
        if t == "(" and not isinstance(t, Q):
            stack.append([])
        elif t == ")" and not isinstance(t, Q):
            done = stack.pop()
            stack[-1].append(done)
        else:
            stack[-1].append(t)
    return stack[0][0]


def fmt_atom(a):
    if isinstance(a, Q):
        return '"' + a.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n") + '"'
    if isinstance(a, float):
        s = f"{a:.4f}".rstrip("0").rstrip(".")
        return "0" if s == "-0" else s
    return str(a)


def dump(node, indent=0):
    if not isinstance(node, list):
        return fmt_atom(node)
    if all(not isinstance(x, list) for x in node):
        return "(" + " ".join(fmt_atom(x) for x in node) + ")"
    pad = "\t" * (indent + 1)
    out = "("
    for k, x in enumerate(node):
        if isinstance(x, list):
            out += "\n" + pad + dump(x, indent + 1)
        else:
            out += ("" if k == 0 else " ") + fmt_atom(x)
    return out + "\n" + "\t" * indent + ")"


def find(node, key):
    return [x for x in node if isinstance(x, list) and x and x[0] == key]


def find1(node, key):
    r = find(node, key)
    return r[0] if r else None


# ---------------------------------------------------------------- library

_lib_cache = {}


def load_lib(lib):
    if lib not in _lib_cache:
        lib_dir = OUT_DIR if lib == PROJECT else LIB_DIR
        with open(os.path.join(lib_dir, lib + ".kicad_sym")) as f:
            root = parse_sexp(f.read())
        _lib_cache[lib] = {s[1]: s for s in find(root, "symbol")}
    return _lib_cache[lib]


def flat_symbol(lib, name):
    """Return the symbol with any 'extends' chain resolved (as KiCad embeds it)."""
    syms = load_lib(lib)
    sym = syms[name]
    ext = find1(sym, "extends")
    if not ext:
        return [x for x in sym]
    parent = flat_symbol(lib, ext[1])
    pname = parent[1]
    child_props = {p[1]: p for p in find(sym, "property")}
    out = ["symbol", Q(name)]
    for x in parent[2:]:
        if isinstance(x, list) and x[0] == "property":
            out.append(child_props.pop(x[1], x))
        elif isinstance(x, list) and x[0] == "symbol":
            sub = list(x)
            sub[1] = Q(name + sub[1][len(pname):])
            out.append(sub)
        else:
            out.append(x)
    for p in child_props.values():
        out.append(p)
    return out


def symbol_pins(sym):
    pins = []
    for sub in find(sym, "symbol"):
        for p in find(sub, "pin"):
            at = find1(p, "at")
            pins.append({
                "type": p[1],
                "x": float(at[1]), "y": float(at[2]), "rot": int(float(at[3])),
                "name": str(find1(p, "name")[1]),
                "number": str(find1(p, "number")[1]),
            })
    return pins


# ---------------------------------------------------------------- project library

def _font(hide=False, justify=None):
    eff = ["effects", ["font", ["size", 1.27, 1.27]]]
    if justify:
        eff.append(["justify", justify])
    if hide:
        eff.append(["hide", "yes"])
    return eff


def custom_symbol(name, ref, footprint, datasheet, description, keywords, fp_filter,
                  pins, half_w=10.16, half_h=10.16):
    """
    A box symbol. pins: (number, name, electrical type, x, y, rotation) in
    symbol units (mm, y up), pin ends on the 2.54 grid outside the box.
    """
    props = [
        ("Reference", ref, -half_w, half_h + 1.27, False, "left"),
        ("Value", name, 3.81, half_h + 1.27, False, "left"),
        ("Footprint", footprint, 0, -half_h - 6.35, True, None),
        ("Datasheet", datasheet, 0, 0, True, None),
        ("Description", description, 0, 0, True, None),
        ("ki_keywords", keywords, 0, 0, True, None),
        ("ki_fp_filters", fp_filter, 0, 0, True, None),
    ]
    sym = ["symbol", Q(name), ["exclude_from_sim", "no"], ["in_bom", "yes"], ["on_board", "yes"]]
    for key, val, x, y, hide, just in props:
        sym.append(["property", Q(key), Q(val), ["at", x, y, 0], _font(hide, just)])
    sym.append(["symbol", Q(name + "_0_1"),
                ["rectangle", ["start", -half_w, half_h], ["end", half_w, -half_h],
                 ["stroke", ["width", 0.254], ["type", "default"]],
                 ["fill", ["type", "background"]]]])
    unit = ["symbol", Q(name + "_1_1")]
    for num, pname, etype, x, y, rot in pins:
        unit.append(["pin", etype, "line", ["at", x, y, rot], ["length", 2.54],
                     ["name", Q(pname), _font()], ["number", Q(num), _font()]])
    sym.append(unit)
    sym.append(["embedded_fonts", "no"])
    return sym


def write_project_lib(symbols, descr):
    lib = ["kicad_symbol_lib", ["version", 20241209], ["generator", Q("gen_schematic")],
           ["generator_version", Q("9.0")]] + list(symbols)
    with open(os.path.join(OUT_DIR, PROJECT + ".kicad_sym"), "w") as f:
        f.write(dump(lib) + "\n")
    with open(os.path.join(OUT_DIR, "sym-lib-table"), "w") as f:
        f.write('(sym_lib_table\n\t(version 7)\n'
                f'\t(lib (name "{PROJECT}") (type "KiCad") '
                f'(uri "${{KIPRJMOD}}/{PROJECT}.kicad_sym") (options "") (descr "{descr}"))\n)\n')
    _lib_cache.pop(PROJECT, None)


# ---------------------------------------------------------------- schematic model

lib_symbols = {}      # lib_id -> flattened symbol
items = []            # placed schematic items
parts = []            # for the summary printout

GRID = 1.27


def snap(v):
    return round(round(v / GRID) * GRID, 4)


def label(net, x, y, pin_rot):
    angle = (pin_rot + 180) % 360
    justify = "left" if angle in (0, 90) else "right"
    items.append(["label", Q(net), ["at", x, y, angle],
                  ["fields_autoplaced", "yes"],
                  ["effects", ["font", ["size", 1.27, 1.27]], ["justify", justify, "bottom"]],
                  ["uuid", Q(new_uuid())]])


def no_connect(x, y):
    items.append(["no_connect", ["at", x, y], ["uuid", Q(new_uuid())]])


def note(text, x, y, size=2.54):
    items.append(["text", Q(text), ["exclude_from_sim", "no"], ["at", snap(x), snap(y), 0],
                  ["effects", ["font", ["size", size, size], ["thickness", 0.4], ["bold", "yes"]],
                   ["justify", "left", "bottom"]],
                  ["uuid", Q(new_uuid())]])


def part(ref, lib, name, value, at, nets, footprint=None, nc_ok=False, in_bom=True, dnp=False):
    """
    Place a symbol and label its pins.
    nets maps pin name or number -> net name. Unmapped pins get a no-connect
    flag if nc_ok, otherwise it is an error (catches typos in pin names).
    """
    lib_id = f"{lib}:{name}"
    sym = flat_symbol(lib, name)
    if lib_id not in lib_symbols:
        embedded = list(sym)
        embedded[1] = Q(lib_id)
        lib_symbols[lib_id] = embedded
    X, Y = snap(at[0]), snap(at[1])
    pins = symbol_pins(sym)

    known = {p["name"] for p in pins} | {p["number"] for p in pins}
    bad = [k for k in nets if k not in known]
    if bad:
        raise SystemExit(f"{ref} ({name}): unknown pins {bad}; has {sorted(known)}")

    node = ["symbol", ["lib_id", Q(lib_id)], ["at", X, Y, 0], ["unit", 1],
            ["exclude_from_sim", "no"], ["in_bom", "yes" if in_bom else "no"], ["on_board", "yes"],
            ["dnp", "yes" if dnp else "no"], ["fields_autoplaced", "yes"], ["uuid", Q(new_uuid())]]
    overrides = {"Reference": ref, "Value": value}
    if footprint:
        overrides["Footprint"] = footprint
    for p in find(sym, "property"):
        key = str(p[1])
        if key.startswith("ki_"):
            continue
        val = overrides.get(key, str(p[2]))
        pat = find1(p, "at")
        prop = ["property", Q(key), Q(val),
                ["at", round(X + float(pat[1]), 4), round(Y - float(pat[2]), 4), 0]]
        eff = find1(p, "effects")
        if eff:
            prop.append(eff)
        node.append(prop)
    for p in pins:
        node.append(["pin", Q(p["number"]), ["uuid", Q(new_uuid())]])
    node.append(["instances", ["project", Q(PROJECT),
                               ["path", Q("/" + ROOT_UUID), ["reference", Q(ref)], ["unit", 1]]]])
    items.append(node)

    done = set()
    for p in pins:
        px, py = round(X + p["x"], 4), round(Y - p["y"], 4)
        if (px, py) in done:          # stacked pins share one connection point
            continue
        done.add((px, py))
        net = nets.get(p["number"], nets.get(p["name"]))
        if net:
            label(net, px, py, p["rot"])
        elif p["type"] == "no_connect" or nc_ok:
            no_connect(px, py)
        else:
            raise SystemExit(f"{ref}: pin {p['number']} ({p['name']}) left unconnected")
    parts.append((ref, value))


# Shorthands for the common two-pin parts -------------------------------------

R_FP = "Resistor_SMD:R_0603_1608Metric"
C_FP = "Capacitor_SMD:C_0603_1608Metric"
C_FP_BULK = "Capacitor_SMD:C_0805_2012Metric"


def R(ref, value, at, top, bottom):
    part(ref, "Device", "R", value, at, {"1": top, "2": bottom}, R_FP)


def C(ref, value, at, top, bottom="GND", bulk=False):
    part(ref, "Device", "C", value, at, {"1": top, "2": bottom}, C_FP_BULK if bulk else C_FP)


def header(ref, value, at, pin_nets, jst_gh=False, footprint=None, dnp=False):
    """1xN connector: 2.54 mm pin header, a keyed JST-GH for off-board cables, or any footprint."""
    n = len(pin_nets)
    fp = footprint or (
        f"Connector_JST:JST_GH_SM{n:02d}B-GHS-TB_1x{n:02d}-1MP_P1.25mm_Horizontal" if jst_gh
        else f"Connector_PinHeader_2.54mm:PinHeader_1x{n:02d}_P2.54mm_Vertical")
    part(ref, "Connector_Generic", f"Conn_01x{n:02d}", value, at,
         {str(i + 1): net for i, net in enumerate(pin_nets)}, fp, dnp=dnp)


# ---------------------------------------------------------------- output

def write_sch(title, paper="A3"):
    sch = ["kicad_sch", ["version", 20250114], ["generator", Q("eeschema")],
           ["generator_version", Q("9.0")], ["uuid", Q(ROOT_UUID)], ["paper", Q(paper)],
           ["title_block", ["title", Q(title)],
            ["rev", Q("A")], ["company", Q("FalconDrone")],
            ["comment", 1, Q("Generated by gen_schematic.py - edit the script, not this file")]],
           ["lib_symbols"] + list(lib_symbols.values())]
    sch += items
    sch += [["sheet_instances", ["path", Q("/"), ["page", Q("1")]]],
            ["embedded_fonts", "no"]]
    path = os.path.join(OUT_DIR, PROJECT + ".kicad_sch")
    with open(path, "w") as f:
        f.write(dump(sch) + "\n")
    print(f"wrote {path}: {len([p for p in parts if not p[0].startswith('#')])} parts")
