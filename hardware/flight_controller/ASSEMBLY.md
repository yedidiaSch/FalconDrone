# FalconDrone FC: who solders what

Two ways to order. Both use the same `build/jlcpcb/falcon_fc_gerbers.zip` and
`falcon_fc_cpl.csv`; only the BOM differs.

- **PCBWay** (`build/pcbway/falcon_fc_pcbway_bom.csv`): parts by manufacturer
  part number, bought from DigiKey/Mouser/LCSC. They can fit everything,
  including the GPS and the through-hole parts; they confirm sourcing in the
  quote. Usually dearer and slower than JLCPCB.
- **JLCPCB** (`build/jlcpcb/falcon_fc_bom.csv`): cheaper, but three parts are
  not theirs to fit; see below.

The rest of this page is about JLCPCB.

LCSC numbers were looked up on 2026-10-21. Stock changes daily, so check each
line in JLCPCB's BOM matcher when you upload `build/jlcpcb/falcon_fc_bom.csv`
(written by `export_fab.py`).

## JLCPCB SMT assembly (53 of the 56 parts)

| Ref | Part | LCSC | Notes |
|---|---|---|---|
| U1 | STM32F446RET6 (LQFP-64) | C69336 | stock has swung between 0 and ~2k, check |
| U2 | AP7361-33E-13 (SOT-223) | C150719 | the AP7361**C**-33E isn't listed; this is the earlier version of the same LDO. Check its pinout is IN-GND-OUT before ordering |
| U3 | ICM-42688-P (LGA-14) | C1850418 | extended part |
| U4, U6 | LP5907MFX-3.3/NOPB (SOT-23-5) | C80670 | TI original; C5370990 is a clone |
| U5 | MS5611 (TE MS561101BA03-50) | C15639 | |
| U8, U9, U10 | SP0504BAHTG (SOT-23-5) | C10675 | |
| Q1 | AO3401A (SOT-23) | C15127 | basic part |
| D2 | SMAJ5.0A (SMA) | C499731 | many makers; any 5 V / 400 W SMA TVS will do |
| J1, J6 | JST SM03B-GHS-TB | C514175 | |
| J3, J5 | JST SM06B-GHS-TB | C133065 | low stock at times |
| SW1, SW2 | C&K PTS810 SJM | C116501 | |
| R, C, FB1, D1 | 0603/0805 passives, green LED, 600R ferrite | matcher picks | all common basic parts |

## Not by JLCPCB SMT

| Ref | Part | How |
|---|---|---|
| U7 | u-blox SAM-M10Q-00B | Not in JLCPCB's library. Either order it from Mouser/DigiKey and send it to JLCPCB as a consigned part, or reflow it yourself (its pads are underneath, so a hot-air station is needed, not an iron). The SAM-M8Q (C5447387) is in their library and fits the same pads, but it is the older, higher-power GPS generation. |
| J2 | 2x4 female header, 2.54 mm (nRF24 socket) | through-hole: solder by hand (or pay for JLCPCB THT assembly) |
| J4 | 1x5 pin header, 2.54 mm (ESC) | through-hole: solder by hand, or solder the ESC wires straight in |
