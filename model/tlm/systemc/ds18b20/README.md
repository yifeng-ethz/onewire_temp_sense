# DS18B20 SystemC Model

This directory contains the executable link/transaction reference for the
DS18B20-compatible one-sensor-per-wire FEB topology.

## Build Requirement

The standalone run uses an Accellera SystemC 3.0.x installation. The default
local path is:

```bash
~/.cache/codex/systemc-3.0.2
```

Override it with `SYSTEMC_HOME=/path/to/systemc/install` if needed. The local
Questa `sccom` path remains useful for mixed HDL/SystemC simulation, but the
standalone model is built as a normal executable so it can emit CSV evidence
without simulator elaboration.

## Run

```bash
make run
```

The run writes:

```text
model/artifacts/ds18b20_systemc_trace.csv
```

The CSV records reset/presence timing, LSB-first command bytes, scratchpad
reads, Read-ROM bytes, and an RC capacitance sweep.
