# OneWire Temperature Controller DV

This tree is the DV workspace for the `onewire_master` plus
`onewire_master_controller` pair used in the FEB control-path Qsys system.

The immediate bring-up target is a six-line DS18B20 setup with one sensor per
DQ line. The unit harness keeps the real VHDL controller/master boundary intact:
software-visible Avalon-MM accesses enter through the controller CSR aperture,
the controller drives the master through the same internal Avalon-MM and
Avalon-ST links used in Qsys, and the DS18B20 behavior is modeled only at the
external 1-Wire DQ boundary.

Current status:

| item | status | note |
|---|---|---|
| user DV plan and buckets | active | `doc/DV_BASIC.md`, `doc/DV_EDGE.md`, `doc/DV_PROF.md`, `doc/DV_ERROR.md`, and `doc/DV_CROSS.md` are the scoreboard; implemented evidence is recorded under `REPORT/` |
| DS18B20 protocol model | initial | reset/presence, Skip ROM, Convert T, Read Scratchpad, Read ROM |
| named-case tb | passing | 12 implemented BASIC/ERROR cases pass in isolated mode and continuous no-reset frame |
| on-board correlation | pending | compare unit tb against System Console reads and SignalTap |
| serial-number feature | planned | add after temperature path is clean in tb and on board |

Protocol source:

- Analog Devices / Maxim DS18B20 data sheet. The model follows the documented
  1-Wire reset/presence behavior, `0xCC` Skip ROM, `0x44` Convert T, `0xBE`
  Read Scratchpad, `0x33` Read ROM, scratchpad byte order, and Maxim CRC-8.

The bucket files are intentionally kept as editable scoreboard documents. The
case catalog drives the harness, but functional coverage is reported as derived
bucket-level features in `doc/DV_COV.md`; this avoids pretending there is a
one-case/one-feature relationship. Current unit evidence proves the common CSR
header (`UID/META/SCRATCH`), nominal DS18B20 scratchpad-to-controller path, the
valid 0 C path, and the initial controller CSR negative-access slice. The
on-board path remains open until a rebuilt image with the common CSR header and
`STATUS[26] sample_valid` is flashed and checked through System Console.
