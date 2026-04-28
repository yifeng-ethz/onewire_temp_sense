# OneWire Temperature Sensor Controller IP

This repository packages the FEB OneWire temperature monitor IPs:

- `onewire_master`: link/physical layer for DS18B20-style 1-Wire transfers.
- `onewire_master_controller`: host-visible controller that periodically
  issues `Skip ROM`, `Convert T`, and `Read Scratchpad` per DQ line.
- `onewire_sense_vector_bridge`: FEB helper that bundles the six split sensor
  conduits into the shared `sense_dq` bus.

The active source tree follows the workspace RTL layout:

```text
doc/
model/
rtl/
script/
syn/
tb/
  doc/
  uvm/
  REPORT/
report/
misc/
trash_bin/
```

The current maintained RTL sources are under `rtl/`. Platform Designer package
files (`*_hw.tcl`) and SVD/package helpers live under `script/`. Unit DV plans
and scoreboards live under `tb/doc/`; the runnable Questa harness is under
`tb/uvm/`. The old monolithic sensor reader and archived legacy experiments live
under `trash_bin/legacy/`.

The FEB deployment assumes six independent DQ lines and one unique DS18B20
sensor per line. Serial-number probing is planned as a simplified one-device
`Read ROM` flow after the temperature readout path is closed in unit
simulation and on board.
