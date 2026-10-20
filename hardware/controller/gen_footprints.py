#!/usr/bin/env python3
"""
Write the project footprint library falcon_rc.pretty (parts missing from the
stock KiCad library) and the fp-lib-table that points at it.

Joystick_Alps_RKJXV122400x
  Alps RKJXV1224005 (PS2-type thumb stick, 2 x 10k pots + push switch).
  From the Alps drawing KJXV14005 "Hole dimensions for attachment" (+/-0.1),
  top view, origin = stick centre:
    VR2 (top edge)    3 x d1.0 at y -8.73, x +2.5 (1) / 0 (2) / -2.5 (3)
    VR1 (right edge)  3 x d1.0 at x +8.73, y +2.5 (1) / 0 (2) / -2.5 (3)
    switch            4 x d1.2 at x +/-3.25, y +5.75 (a, c) / +10.25 (b, d);
                      a-c and b-d are tied inside, the contact closes ab
    frame tabs        4 x d1.5 at (+/-6.325, +/-5), soldered
    bosses            2 x d1.6 NPTH at (+/-4.3, 0)
  The a/c and b/d rows are dimensioned relative to each other (4.5); their
  absolute y was measured off the drawing (5.75 / 10.25 = 8 -/+ 2.25, the
  8 mm being the dimensioned centre of the d3.5 relief).
  Pads: 1-3 VR1, 4-6 VR2 (4 = its pin 1), 7 a, 8 c, 9 b, 10 d, MP frame.
  Alps recommends a 1.2 mm board; on 1.6 mm the soldered frame tabs still hold.

Toggle_SPDT_P4.70mm
  Sub-miniature PCB-pin toggle, C&K 7101 (ON-ON) / 7103 (ON-OFF-ON) with
  "C" PC thru-hole terminals, and the MTS-102/103 clones: 3 pins in a row at
  4.70 mm, centre pin = common. C&K's PC mounting layout asks for d1.85 holes
  (its .050 x .030 pins are 1.48 mm across the diagonal).

Usage:  python3 gen_footprints.py
"""

import os

HERE = os.path.dirname(os.path.abspath(__file__))
LIB = "falcon_rc"


def pad_th(num, x, y, drill, size, shape="circle", npth=False):
    if npth:
        return (f'\t(pad "" np_thru_hole circle (at {x} {y}) (size {drill} {drill}) '
                f'(drill {drill}) (layers "*.Cu" "*.Mask"))\n')
    return (f'\t(pad "{num}" thru_hole {shape} (at {x} {y}) (size {size} {size}) '
            f'(drill {drill}) (layers "*.Cu" "*.Mask"))\n')


def rect(layer, x0, y0, x1, y1, width):
    return (f'\t(fp_rect (start {x0} {y0}) (end {x1} {y1}) '
            f'(stroke (width {width}) (type solid)) (fill no) (layer "{layer}"))\n')


def circle(layer, x, y, r, width):
    return (f'\t(fp_circle (center {x} {y}) (end {x + r} {y}) '
            f'(stroke (width {width}) (type solid)) (fill no) (layer "{layer}"))\n')


def text(kind, value, x, y, layer, hide=False):
    h = " hide" if hide else ""
    return (f'\t(fp_text {kind} "{value}" (at {x} {y}) (layer "{layer}"){h}\n'
            '\t\t(effects (font (size 1 1) (thickness 0.15)))\n\t)\n')


def footprint(name, descr, body, ref_y, val_y):
    return (f'(footprint "{name}"\n\t(version 20240108)\n\t(generator "gen_footprints")\n'
            f'\t(layer "F.Cu")\n\t(descr "{descr}")\n\t(attr through_hole)\n'
            + text("reference", "REF**", 0, ref_y, "F.SilkS")
            + text("value", name, 0, val_y, "F.Fab")
            + text("user", "${REFERENCE}", 0, 0, "F.Fab")
            + body + ")\n")


def joystick():
    b = ""
    for i, y in enumerate((2.5, 0, -2.5)):            # VR1, right edge
        b += pad_th(str(1 + i), 8.73, y, 1.0, 1.7, "rect" if i == 0 else "circle")
    for i, x in enumerate((2.5, 0, -2.5)):            # VR2, top edge
        b += pad_th(str(4 + i), x, -8.73, 1.0, 1.7, "rect" if i == 0 else "circle")
    for num, x, y in (("7", -3.25, 5.75), ("8", 3.25, 5.75), ("9", -3.25, 10.25), ("10", 3.25, 10.25)):
        b += pad_th(num, x, y, 1.2, 1.9)
    for sx in (-1, 1):
        for sy in (-1, 1):
            b += pad_th("MP", sx * 6.325, sy * 5.0, 1.5, 2.3)
        b += pad_th("", sx * 4.3, 0, 1.6, 1.6, npth=True)
    b += rect("F.Fab", -6.575, -6.575, 6.575, 6.575, 0.1)      # 13.15 frame
    b += circle("F.SilkS", 0, 0, 12.5, 0.12)                    # cap + travel clearance
    b += rect("F.CrtYd", -8.6, -10.2, 10.2, 12.1, 0.05)
    return footprint("Joystick_Alps_RKJXV122400x",
                     "Alps RKJXV1224005 thumb joystick, 2 pots + push switch, drawing KJXV14005",
                     b, -12.0, 13.2)


def toggle():
    b = ""
    for i, x in enumerate((-4.7, 0, 4.7)):
        b += pad_th(str(1 + i), x, 0, 1.85, 2.8, "rect" if i == 0 else "circle")
    b += rect("F.SilkS", -6.6, -4.2, 6.6, 4.2, 0.12)
    b += circle("F.SilkS", 0, 0, 3.0, 0.12)                     # bushing
    b += rect("F.Fab", -6.4, -4.0, 6.4, 4.0, 0.1)
    b += rect("F.CrtYd", -6.85, -4.45, 6.85, 4.45, 0.05)
    return footprint("Toggle_SPDT_P4.70mm",
                     "Sub-miniature PCB toggle C&K 7101/7103 (C terminals) or MTS-102/103, 3 pins 4.7 mm pitch",
                     b, -5.4, 5.4)


def main():
    out = os.path.join(HERE, LIB + ".pretty")
    os.makedirs(out, exist_ok=True)
    for name, fn in (("Joystick_Alps_RKJXV122400x", joystick), ("Toggle_SPDT_P4.70mm", toggle)):
        with open(os.path.join(out, name + ".kicad_mod"), "w") as f:
            f.write(fn())
    with open(os.path.join(HERE, "fp-lib-table"), "w") as f:
        f.write('(fp_lib_table\n\t(version 7)\n'
                f'\t(lib (name "{LIB}") (type "KiCad") (uri "${{KIPRJMOD}}/{LIB}.pretty") '
                '(options "") (descr "FalconDrone controller footprints"))\n)\n')
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
