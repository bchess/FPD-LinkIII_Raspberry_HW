# fpdlink2raspi: KiCad PCB (rebuilt from the Gerbers)

The Altium source (`.PcbDoc`) was never published, so this KiCad 10 project was
**rebuilt** from the fabrication outputs in this repo:

| Input | What it gives |
|-------|---------------|
| `GerberX2/*` (Gerber X2, Altium 20.1) | copper, pads, vias, pours, mask, paste, drills, outline, mechanical layers |
| `Pick Place/*.txt` | reference, side, position and rotation of each part |
| `BOM/*.xlsx` | value, MPN, manufacturer, supplier, Altium LibRef |
| `fpdlink2raspi_SCH_0v4.PDF` | its bookmarks hold Altium's full netlist (net names and pin numbers) |

Open `fpdlink2raspi.kicad_pro` in KiCad 10. `render_top.png` is a KiCad 3D render.

## What you get

* A 4-layer board, 39 x 56 mm, with the stackup from the fab drawing (0.119 / 1.2 / 0.119 mm FR-4, 35 µm Cu, 1.6 mm total).
* 148 footprints: 136 placed parts, the 6 not-fitted parts, 4 mounting holes. Every pad is reconstructed
  exactly: shape, size, corner radius, mask and paste margins, and the segmented exposed-pad paste for U3/U4.
* 100 nets with the schematic's net names. Every pad carries its schematic pin number (U3 pins 1 to 49,
  J1 pins 1 to 15 plus S1/S2, and so on).
* Tracks and arcs, 171 vias (via-in-pad tenting as in Altium), 11 filled zones (inner planes and top pours),
  and the copper text labels ("PoC", "3V3", the zhaw/InES logo, ...).
* A project footprint library, `fpdlink2raspi.pretty`, with all 30 footprints (`fp-lib-table` is included).
* Courtyards (Altium M15/M16) and assembly drawings (M13/M14) inside each footprint. The other Altium
  mechanical layers are kept on named User layers (`User.1` to `User.8`).
* BOM data is in footprint fields. Not-fitted parts (C33, R4, R5, R6, R27, R29) are marked DNP, because the
  variant left them out of pick-and-place. Their identity comes from matching their copper against the netlist.

## How it was checked

* **Gerber round trip.** I exported Gerbers and drills from this KiCad board and compared them geometrically
  with the original Altium files.
  * All 4 copper layers, bottom solder mask and both paste layers match to better than 10 µm.
  * All 185 drill hits match in position and size.
  * The one remaining difference is on top solder mask. KiCad rounds the corners of rectangular pads' mask
    openings with a 50 µm radius; Altium leaves them sharp. That totals 0.16 mm² over the whole board.
* **Netlist.** The pin numbering was solved so that the copper connectivity agrees with the schematic netlist.
  * 21 pins carry a different name in the schematic. All of them are hierarchical-harness aliases of the same
    net: J1's CSI pins to U3's `D_*` nets, U1 GPIOs to U3 GPIOs, I2C through R33/R34, and `REF_CLK`/`TI_*`.
    Each copper net uses the user-given name.
  * There are no shorts between differently named nets and no nets split into islands.
* **KiCad DRC:** 0 clearance errors and 0 unconnected items.
  * There are 48 *courtyard overlap* errors. They are real overlaps in the original Altium layout (Altium
    doesn't check these by default).
  * There are 3 harmless "footprint differs from library" warnings.

## Things that are not in the Gerbers

* **Silkscreen.** The real board has a legend, but no overlay Gerber was published, so `F/B.SilkS` are
  empty. Reference designators are on the Fab layers.
* **Schematic.** There is no `.kicad_sch`. The nets and pin numbers are on the board, so a schematic drawn
  later can be linked with *Update PCB from Schematic*. Use the same references and pin numbers.
* **Altium design rules.** Only the basics are carried over: 0.125 mm clearance, 0.06 mm mask expansion
  (0.05 mm on fine-pitch pads), and 0.15 mm via drills. Altium also had a 1 mm pour-to-diff-pair clearance.
  It is visible in the existing fills, but it's not a KiCad rule.
* **Zones.** The zones are pre-filled with Altium's exact copper. Their outlines follow the filled shapes.
  If you *refill* in KiCad, it recomputes clearances with its own rules, so check the CSI diff pairs and the
  RF lines after a refill.
* **Footprint names.** They are descriptive (for example `VQFN-48-1EP_7x7mm_P0.5mm_EP4.1x4.1mm`). The
  footprints are the Altium ones reconstructed, not KiCad stock library parts, and have no 3D models.
* **BOM values** come from `BOM/*.xlsx`. That includes the 9 V PoC changes described in the top-level
  README, not the values printed in the schematic PDF.

## Regenerating

`conversion/` holds the converter. It runs in KiCad's bundled Python and needs `shapely`, `numpy` and
`pymupdf` installed into a `pylib` directory next to `conversion/`:

```bash
KP=../KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3
$KP -m pip install --target pylib "shapely==2.0.6" "numpy<2.1" pymupdf
$KP conversion/build_kicad.py .. out        # writes out/fpdlink2raspi.kicad_pcb etc.
```

`compare.py <fpdlink2raspi dir> <dir with KiCad-exported Gerbers> ''` repeats the Gerber round-trip check.
