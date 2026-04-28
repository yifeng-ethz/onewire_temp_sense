# SystemC Reference Survey

## Scope

This survey records external modeling references used to shape the DS18B20
model architecture. The local model remains authored from the Analog
Devices/Maxim DS18B20 datasheet and local RTL needs; no external source code is
copied into this IP.

## References

| Reference | What was inspected | Modeling pattern to reuse |
|---|---|---|
| Accellera SystemC 3.0.2 release and TLM examples: <https://www.accellera.org/downloads/standards/systemc>, <https://github.com/accellera-official/systemc> | Official language/TLM examples, especially loosely timed target models. | Use `sc_time` for observable timing, keep target-side behavior isolated from bus adapters, and report accumulated delays as first-class evidence. |
| Xilinx PCIe controller model: <https://github.com/Xilinx/pcie-model> | `PCIeController` TLM sockets and TLP-to-user-logic split. | Keep protocol-facing payload translation separate from user/device state. For DS18B20 this means separate slot codec, command FSM, and scoreboard-facing observations. |
| Xilinx libsystemctlm-soc: <https://github.com/Xilinx/libsystemctlm-soc> | QEMU/SystemC/RTL bridge repository structure and co-sim boundary model. | Use adapters at simulation boundaries instead of replacing internal generated RTL or firmware fabric. This aligns with the local `tb_int` rule. |
| VCML: <https://github.com/machineware-gmbh/vcml> | I2C/SPI peripheral models such as LM75, ADS1015, and MAX31855. | Model sensors as command-facing devices with typed register/state properties, explicit conversion/sample timing, and simple external commands to set physical quantities. |
| Simple_I2C: <https://github.com/mitya1337/Simple_I2C> | Small SystemC I2C master/slave bit-level implementation. | Open-drain serial protocols can be tested at pin/edge level, but device behavior still needs a higher transaction layer for maintainability. |

## DS18B20 Model Implications

The DS18B20 model uses four separable layers:

1. Physical line model: open-drain masters/slaves plus RC pull-up threshold
   crossing. This lives in SV for RTL simulation and is mirrored by timing
   parameters in the SystemC model.
2. Link model: reset/presence, read slots, write-0/write-1 slots, byte order,
   and slot timing margins.
3. Transaction model: ROM commands, function commands, scratchpad contents,
   conversion time, and CRC.
4. Adapter layer: RTL harness drivers and future `tb_int` boundary agents that
   connect to CSR/stream/conduit boundaries without replacing generated fabric.

The VCML sensor examples are the closest behavioral match: LM75 and ADS1015
keep sensor quantities as model properties, derive register contents from those
quantities, and perform sampling/conversion on a modeled schedule. The DS18B20
model follows that style with `TEMP_C_X16`, conversion completion, scratchpad
shadow state, and CRC generation.

The PCIe references are not sensor models, but they are useful for avoiding a
monolithic model. The TLP-side and BAR/user-side interfaces are explicitly
separated there; DS18B20 applies the same split between pin timing, 1-Wire slot
codec, and device command semantics.

## Non-Goals

1. The first model checkpoint is not a SPICE or SystemC-AMS implementation. It
   uses a first-order RC threshold model sufficient to catch read-slot margin
   and pull-up/wire-capacitance bugs.
2. The first transaction model assumes the FEB topology: six independent wires,
   one DS18B20 per wire. Multi-drop ROM search can be added later without
   weakening the single-drop `Read ROM` and serial-number probe tests.
3. External public models are reference material only. License boundaries stay
   clean by implementing the local model directly from the DS18B20 datasheet.
