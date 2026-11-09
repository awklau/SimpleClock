# SimpleClock hardware

Board revision: **00 prototype**

| Path | Content |
| --- | --- |
| `simple_clock.pdf` | Schematic (four sheets) |
| `altium/` | Altium Designer project: schematics, PCB, project libraries, BOM and output job |
| `production/` | Files to order the board: Gerbers and drill files (zip), bill of materials, pick-and-place |

The Altium project is self-contained: `PROJECT_LIB/simple_clock.SCHLIB` and
`main.PcbLib` hold every symbol and footprint the design uses. The files can be
viewed in the free Altium 365 viewer and imported into KiCad.

## Main parts

| Function | Part |
| --- | --- |
| GNSS timing receiver | Septentrio mosaic-G5 T |
| Oscillator | SiTime SIT5801AB-KV033JV-10 (10 MHz, I²C-tunable OCXO) |
| Microcontroller | STM32C071FBP6 |
| Temperature sensor | TI TMP117AIDRVR, beside the OCXO |
| Clock and PPS buffers | TI LMK1C1108APWR (×2) |
| Heater switch | AO3400A MOSFET |
| Regulator | TI TLV76133KVU |
| USB | USB-C receptacle (217179-0001), USBLC6-2P6 protection |
| Antenna input | SMA connector |

The full list with manufacturer part numbers is `production/bom.csv`.

## Ordering

4-layer board designed for JLCPCB's default 4-layer stackup **JLC04161H-7628**.
Send `production/gerber.zip` (Gerbers and drill files) to the PCB maker; for
assembly also send `production/bom.csv` and `production/pick-and-place.csv`
(millimetres, top side). A few BOM lines have no manufacturer part number; the
value, description and footprint are enough to choose a part.

The 10 MHz clock traces and the GNSS antenna trace are 50 Ω lines sized for that
stackup. With a different stackup, recalculate those trace widths for 50 Ω in
the Altium project before ordering; nothing else depends on the stackup.

## Assembly notes

- **Heater:** a 5 V, about 1 W thin-film heater on J18, mounted on a 54 g
  aluminium block that is attached to the back of the PCB with thermal tape. The
  firmware drives it at full power during warm-up and limits the temperature
  target to 0–60 °C (raw trip 70 °C).
- **Temperature sensor:** the TMP117 sits beside the OCXO; the thermal loops
  regulate this reading.
- **J17 (BOOT):** fit a jumper to start the STM32 bootloader for programming;
  remove it for normal operation. See [first flashing](../docs/FIRST_FLASHING.md).
- **Antenna:** an active GNSS antenna. Galileo HAS (PPP) needs one that covers
  the Galileo E6 band.
- **Programming and first start:** [first flashing](../docs/FIRST_FLASHING.md),
  then the [user manual](../docs/USER_MANUAL.md).

## Licence

Copyright © 2026 Albert Wai Kit Lau.

This source describes Open Hardware and is licensed under the CERN-OHL-S v2.
You may redistribute and modify it under the terms of the
[CERN-OHL-S v2](LICENSE) (https://ohwr.org/cern_ohl_s_v2.txt).

This source is distributed WITHOUT ANY EXPRESS OR IMPLIED WARRANTY, INCLUDING
OF MERCHANTABILITY, SATISFACTORY QUALITY AND FITNESS FOR A PARTICULAR PURPOSE.
Please see the CERN-OHL-S v2 for applicable conditions.

Source location: https://github.com/awklau/SimpleClock

As required by the licence, anyone who makes products from this design must
keep this notice and make their own modified source available under the same
licence.
