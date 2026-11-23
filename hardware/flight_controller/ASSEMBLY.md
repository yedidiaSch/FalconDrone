# FalconDrone FC: who solders what

Two ways to order. Both use the same `build/jlcpcb/falcon_fc_gerbers.zip` and
`falcon_fc_cpl.csv`; only the BOM differs.

- **PCBWay** (`build/pcbway/falcon_fc_pcbway_bom.csv`): parts by manufacturer
  part number, bought from DigiKey/Mouser/LCSC. They can fit everything,
  including the GPS and the through-hole parts; they confirm sourcing in the
  quote. Usually dearer and slower than JLCPCB.
- **JLCPCB** (`build/jlcpcb/falcon_fc_bom.csv`): cheaper; fits 54 of the 56 parts
  with SMT plus THT assembly (not U5 and U7, see below).

The rest of this page is about JLCPCB.

LCSC numbers were looked up on 2026-10-25. Stock changes daily, so check each
line in JLCPCB's BOM matcher when you upload `build/jlcpcb/falcon_fc_bom.csv`
(written by `export_fab.py`).

## JLCPCB assembly (54 parts)

| Ref | Part | LCSC | Notes |
|---|---|---|---|
| U1 | STM32F446RET6 (LQFP-64) | C69336 | stock has swung between 0 and ~2k, check |
| U2 | AP7361C-33E-13 (SOT-223) | C500795 | not the -33ER (SOT-223R), whose pin order differs |
| U3 | ICM-42688-P (LGA-14) | C1850418 | extended part |
| U4, U6 | LP5907MFX-3.3/NOPB (SOT-23-5) | C80670 | TI original; C5370990 is a clone |
| U5 | MS5611 (TE MS561101BA03-50) | C15639 | not fitted by JLCPCB (`JLC_DNP`) |
| U7 | u-blox SAM-M10Q-00B | C5443880 | not fitted by JLCPCB (`JLC_DNP`) |
| U8, U9, U10 | SP0504BAHTG (SOT-23-5) | C10675 | |
| Q1 | AO3401A (SOT-23) | C15127 | basic part |
| D2 | SMAJ5.0A (SMA) | C499731 | many makers; any 5 V / 400 W SMA TVS will do |
| J1, J6 | JST SM03B-GHS-TB | C514175 | |
| J3, J5 | JST SM06B-GHS-TB | C133065 | low stock at times |
| SW1, SW2 | C&K PTS810 SJG | C221895 | |
| FB1 | Sunlord GZ1608D601TF | C1002 | |
| D1 | GL0603UG01 green LED | C51933293 | |
| R, C | 0603/0805 passives | in the BOM | basic parts |
| J2 | CJT A2541HWV-2x4P, 2x4 female 2.54 mm (nRF24 socket) | C239344 | through-hole: needs THT assembly |
| J4 | XFCN PZ254V-11-05P, 1x5 male 2.54 mm (ESC) | C492404 | through-hole: needs THT assembly |

## Left off the JLCPCB order

U5 (barometer) and U7 (GPS) are left out of the JLCPCB BOM and CPL to save
about $60 on two boards (`JLC_DNP` in `export_fab.py`). Everything around them
(U6 GPS LDO, FB1, decoupling, I2C pull-ups) is still fitted, so the board flies
without them and they can go on later. Both are LGA parts with no leads: they
need hot air or a hotplate plus paste, not an iron. Instead of U7, a GPS module
can plug into the AUX port (J3, Pixhawk GPS pinout). To fit them at the fab
again, empty `JLC_DNP` and rerun `export_fab.py`.
