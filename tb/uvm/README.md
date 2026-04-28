# OneWire Controller Smoke Harness

This is a mixed-language Questa bring-up harness. It is intentionally minimal
until the project DV plan and bucket cases are supplied.

Run:

```bash
make -C onewire_temp_sense/tb/uvm smoke
```

The harness uses the workspace-standard Questa runtime and ETH license server
defined by the `verification-tools` skill. It compiles the real VHDL
controller/master pair, a small simulation `scfifo`, and the SystemVerilog
DS18B20 bus model.
