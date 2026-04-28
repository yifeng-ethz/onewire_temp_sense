# DS18B20 Model Plan

## Truth Source

The reference document is
[`../hardware/ds18b20/DS18B20_rev6_2019.pdf`](../hardware/ds18b20/DS18B20_rev6_2019.pdf),
downloaded from Analog Devices / Maxim Integrated. The SHA-256 checksum is kept
beside the PDF.

External SystemC/TLM references used for model architecture are recorded in
[`SYSTEMC_REFERENCE_SURVEY.md`](SYSTEMC_REFERENCE_SURVEY.md). They are
architecture references only; the DS18B20 model implementation is local and
datasheet-derived.

## Abstraction Layers

| Layer | File(s) | Purpose |
|---|---|---|
| physical | `model/analytical/ds18b20_physical_timing.md`, `tb/uvm/harness/models/ds18b20/onewire_rc_line_model.sv` | RC pull-up, voltage threshold crossing, reset/read/write sampling margins |
| link | `model/tlm/systemc/ds18b20/ds18b20_systemc_model.cpp`, `tb/uvm/harness/models/ds18b20/ds18b20_1wire_model.sv` | reset/presence, write-0/write-1 slots, read-0/read-1 slots, LSB-first byte transfer |
| transaction | `model/tlm/systemc/ds18b20/ds18b20_systemc_model.cpp`, `tb/uvm/harness/models/ds18b20/ds18b20_1wire_model.sv` | Skip-ROM, Convert-T, Read-Scratchpad, Read-ROM, CRC-8, ROM shadow |

## Closure Rules

1. Physical margins come from the analytical RC equations and the datasheet
   timing table, not from the current DUT.
2. The SystemC model is the executable transaction/link reference.
3. The SystemVerilog model used by RTL simulation must remain behaviorally
   aligned with the SystemC model for command decoding, byte order, CRC, and
   timing windows.
4. Any fixture-specific wire capacitance assumption must be parameterized and
   recorded in the model report.
