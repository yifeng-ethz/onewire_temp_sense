# OneWire Models

This tree is the DS18B20 reference-model stack for the OneWire IP.

| Tier | Directory | Role |
|---|---|---|
| hardware | `hardware/ds18b20/` | official DS18B20 datasheet copy and checksum |
| analytical | `analytical/` | physical timing equations and parameter assumptions |
| tlm | `tlm/systemc/ds18b20/` | SystemC executable link/transaction reference |
| REPORT | `REPORT/` | model evidence and closure summaries |
| doc | `doc/` | model plan, layer contract, and external reference survey |

The RTL simulation harness consumes the same model intent through
`../tb/uvm/harness/models/ds18b20/`: `onewire_rc_line_model.sv` owns the
physical RC threshold abstraction, while `ds18b20_1wire_model.sv` owns the
link/transaction behavior.
