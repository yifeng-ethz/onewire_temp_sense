# DS18B20 Physical Timing Model

The 1-Wire bus is modeled as open-drain devices plus a pull-up resistor. For a
released bus after a low-driving interval, the voltage is:

```text
V(t) = VPU * (1 - exp(-t / (RPU * CBUS)))
```

The digital high threshold crossing time used by the RTL-facing model is:

```text
tVIH = -RPU * CBUS * ln(1 - VIH / VPU)
```

The first DS18B20 model gate uses the following default parameters:

| Parameter | Default | Source / reason |
|---|---:|---|
| `VPU` | 3.3 V | FEB local digital rail assumption |
| `RPU` | 4.7 kOhm | datasheet typical external pull-up topology |
| `RSTRONG` | 100 Ohm | strong-pull-up abstraction for parasite-power idle |
| `CIN/OUT` | 25 pF | DS18B20 datasheet maximum pin capacitance |
| `CBUS` | 125 pF | 25 pF device plus board/cable/FPGA-pin first-order budget |
| `VIL_MAX` | 0.8 V | DS18B20 datasheet input low threshold |
| `VIH_MIN` | 2.2 V | local-power lower bound from datasheet at 3.0 V to 5.5 V |

With the default weak pull-up:

```text
tVIH = -4700 * 125e-12 * ln(1 - 2.2 / 3.3) = 0.646 us
```

This is well inside the 15 us read-data-valid window. It is still modeled
explicitly because cable capacitance and weak/incorrect pull-up choices can
eat the read-1 margin before the RTL samples.

## Datasheet Timing Gates

| Symbol | Constraint used by model |
|---|---|
| `tRSTL` | reset low must be at least 480 us |
| `tRSTH` | reset high/recovery before next command at least 480 us |
| `tPDHIGH` | slave waits 15 us to 60 us after reset release |
| `tPDLOW` | slave presence pulse is 60 us to 240 us |
| `tSLOT` | read/write slots are 60 us to 120 us |
| `tREC` | recovery between slots is at least 1 us |
| `tLOW0` | write-0 low time is 60 us to 120 us |
| `tLOW1` | write-1 low time is 1 us to 15 us |
| `tRDV` | read data is valid within 15 us from read-slot falling edge |

The SV line model keeps only the first-order threshold-crossing behavior:
low-driving devices force the bus low immediately, while release is delayed
until the selected pull-up resistance charges `CBUS` above `VIH_MIN`.
