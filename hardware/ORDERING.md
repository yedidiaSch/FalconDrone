# Ordering the boards: what to pick on PCBWay / JLCPCB

Two boards:

- **FC** = flight controller, `flight_controller/`
- **RC** = handheld controller, `controller/`

Anything not listed here: leave the site's default.

## Board (PCB) options

| Field | FC | RC | Why |
|---|---|---|---|
| Panel / delivery format | Single pieces | Single pieces | no panel needed |
| Size | 45 x 45 mm | 150 x 100 mm | read from the gerbers, just check it |
| Quantity | 5 | 5 | the minimum, and it's cheapest |
| Layers | **4** | **2** | |
| Material | FR-4 | FR-4 | |
| FR4-TG | TG 130-140 (default) | TG 130-140 (default) | TG 150+ is fine too, not needed |
| Thickness | 1.6 mm | 1.6 mm | |
| Min track/spacing | **5/5 mil** | **5/5 mil** | narrowest tracks/gaps are 0.15 mm = 5.9 mil; 6/6 mil would be too coarse |
| Min hole size | 0.3 mm | 0.3 mm | smallest via drill |
| Solder mask | Green (any colour works) | any | green is cheapest and fastest; matte black makes inspection harder |
| Silkscreen | White | White | |
| Edge connector | No | No | |
| Surface finish | **Immersion gold (ENIG)** | HASL lead free (ENIG also fine) | the IMU and baro are LGA parts with pads underneath; ENIG is flat, HASL is bumpy |
| Via process | Tenting vias | Tenting vias | |
| Finished copper (outer) | 1 oz | 1 oz | |
| Inner copper | 1 oz (JLCPCB: 0.5 oz is also fine) | — (2-layer) | |
| Impedance control | No | No | |
| Remove product / order No. | your choice | your choice | "No" is free, but the number gets printed somewhere on the board |
| Castellated holes, blind vias, gold fingers | No | No | |

On JLCPCB the same fields are called "PCB Qty", "Different Design: 1",
"Specify Layer Sequence: No", "Mark on PCB / Remove Order Number".

## Files to upload

| | FC | RC |
|---|---|---|
| Gerber (the board) | `flight_controller/build/jlcpcb/falcon_fc_gerbers.zip` | `controller/build/jlcpcb/falcon_rc_gerbers.zip` |
| BOM, JLCPCB | `flight_controller/build/jlcpcb/falcon_fc_bom.csv` | `controller/build/jlcpcb/falcon_rc_bom.csv` |
| BOM, PCBWay | `flight_controller/build/pcbway/falcon_fc_pcbway_bom.csv` | `controller/build/pcbway/falcon_rc_pcbway_bom.csv` |
| Placement (CPL / centroid) | `flight_controller/build/jlcpcb/falcon_fc_cpl.csv` | `controller/build/jlcpcb/falcon_rc_cpl.csv` |

The `build/` folders are not in git. Make them first:

```
cd hardware/flight_controller && python3 export_fab.py
cd hardware/controller && python3 gen_fab.py
```

The gerber zip is the same file for both sites. On PCBWay the zip and the
placement file from `build/jlcpcb/` are fine as they are.

## Assembly (PCBA) options

| Field | FC | RC |
|---|---|---|
| Assemble | Yes | Yes |
| Assembly side | **Both sides** (parts on top and bottom) | Bottom side for SMD, plus through-hole parts |
| Through-hole (THT) assembly | PCBWay: yes. JLCPCB: no, solder J2 and J4 yourself | Yes (sticks, switches, LEDs, sockets) |
| Tooling holes | Added by the factory | Added by the factory |
| Confirm parts placement | **Yes**: check the rotation preview before paying | **Yes** |
| Quantity to assemble | 2 (or all 5) | 2 (or all 5) |

## FC on JLCPCB only

- Every part has its LCSC number in the BOM, the GPS (U7) and the two
  through-hole headers (J2, J4) included. Turn on THT assembly.
- **Details:** see `flight_controller/ASSEMBLY.md`.

## RC on JLCPCB only

- Every part has its LCSC number. The toggles are E-Switch 100SP1T1B4M2QE (SW8,
  ARM) and Dailywell 1MS3T1B1M2QES-5 (SW9, MODE): the C&K ones are out of
  stock, and these have the same 4.7 mm straight PC pins.
