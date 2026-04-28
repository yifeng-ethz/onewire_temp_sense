# DS18B20 Model Report

Status: executable three-layer model checkpoint.

Evidence produced by this checkpoint:

- official DS18B20 Rev. 6 PDF is stored under `model/hardware/ds18b20/`
- analytical physical timing note defines the RC threshold model
- SystemVerilog RTL-simulation boundary now uses `onewire_rc_line_model.sv`
- SystemC executable TLM lives under `model/tlm/systemc/ds18b20/`
- SystemC trace: `model/artifacts/ds18b20_systemc_trace.csv`
- RTL regression: `tb/REPORT/onewire_regression_summary.json`
  (`2026-04-28T17:27:17`, 55/55 implemented BASIC+ERROR cases PASS)

The first physical layer is intentionally simple: any low-driving participant
forces the line low, and a released bus crosses digital high after the
datasheet-driven RC delay. This is enough to expose bad read-1 sampling margins
from excessive cable capacitance or weak pull-up selection while keeping the
simulation deterministic.

Measured default timing from the executable SystemC model:

| parameter set | value |
|---|---:|
| weak pull-up `R=4.7 kOhm`, `C=125 pF`, `VIH=2.2 V`, `VPU=3.3 V` | `0.645435 us` |
| strong pull-up `R=100 Ohm`, `C=125 pF`, `VIH=2.2 V`, `VPU=3.3 V` | `0.013733 us` |

The RTL regression used the same DS18B20 command ordering as the SystemC trace:
reset/presence, Skip-ROM, Convert-T, reset/presence, Skip-ROM, Read-Scratchpad,
and per-bit LSB-first byte transfer. Cases `B001` and `B002` prove the stale
reset-like temperature path is not accepted: line 0 valid 0 C reads
`0x00000000`, and lines 1..5 read distinct nonzero float32 words.

Next closure step: add RX byte-drain cases (`B045-B060`) that compare captured
scratchpad bytes and CRC directly against `ds18b20_systemc_trace.csv`, then
promote Read-ROM serial-number probing from planned to implemented.
