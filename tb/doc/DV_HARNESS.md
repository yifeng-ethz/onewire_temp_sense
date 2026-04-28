# DV Harness: onewire_master_controller

**Harness root:** `onewire_temp_sense/tb/uvm`  
**Author:** Yifeng Wang (yifenwan@phys.ethz.ch)  
**Date:** 2026-04-28  
**Status:** Mixed-language named-case harness for the implemented 55-case BASIC/ERROR scoreboard slice.

---

## DUT Boundary

The harness instantiates the real `onewire_master_controller` and
`onewire_master` VHDL entities. It does not replace the controller/master
internal links with shortcuts.

Boundary agents:

| boundary | agent/model | role |
|---|---|---|
| controller CSR Avalon-MM | `tb_onewire_controller_smoke.sv` tasks | host reads/writes, named case sequencing, feature emission |
| controller-master Avalon-MM | real RTL | no replacement |
| controller-master Avalon-ST | real RTL | no replacement |
| six DQ conduits | `ds18b20_1wire_model.sv` | sensor bus behavior |
| scoreboard | smoke tb checks | expected common CSR header, float32 samples, status flags, and negative CSR no-op behavior |

## DS18B20 Model

The DS18B20 model is intentionally at the pin/protocol boundary. It reacts to:

- reset pulse and presence pulse
- byte writes, LSB first
- byte reads, LSB first
- `0xCC` Skip ROM
- `0x44` Convert T
- `0xBE` Read Scratchpad
- `0x33` Read ROM

The model computes the scratchpad CRC with the Maxim/Dallas CRC-8 polynomial.

## Current Case Flow

`uvm/tb_onewire_controller_smoke.sv`:

1. accepts `+OW_CASE=<case>` and `+OW_MODE=<mode>` plusargs
2. resets the real controller/master pair for isolated cases
3. runs the selected case or continuous frame in bucket order
4. emits `OW_CASE_PASS <case>` and `OW_FEATURE_PASS <bucket.feature>` markers
5. fails if any enabled line lacks `sample_valid`
6. fails if any enabled line reports the reset-like `1.0 C` signature
7. proves line 0 can report a valid 0 C sample as `0x00000000`

Implemented modes:

| mode | status | command |
|---|---|---|
| `isolated` | PASS for `B001-B033`, `B121-B125`, `X001-X012`, `X121-X125` | `make regress` |
| `all_buckets_frame` | PASS for the implemented BASIC then ERROR slice without inter-case reset | `make run CASE=ALL_BUCKETS_FRAME MODE=all_buckets_frame` |

The same scoreboard feature can be covered by multiple cases, and one case can
emit multiple feature markers. Functional coverage in `DV_COV.md` is therefore
bucket-feature based, not case-count based.
