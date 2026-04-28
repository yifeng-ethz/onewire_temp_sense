# DV Plan: onewire_master_controller (+ onewire_master)

**DUT scope (treated as one IP):**
- `rtl/vhdl/onewire_master/onewire_master.vhd` — link/physical layer for DS18B20-style 1-Wire transfers, six independent DQ lines, AVMM `ctrl` slave, AVST `tx` sink, AVST `rx` source, IRQ on completion.
- `rtl/vhdl/onewire_master/crc_checker_maxim_crc8.vhd` — Maxim CRC-8 LFSR helper (poly `x^8 + x^5 + x^4 + 1`) shared by the controller's planned CRC checker and Read-ROM probe.
- `rtl/vhdl/onewire_master_controller/onewire_master_controller.vhd` — host transaction layer that issues `Skip ROM` → `Convert T` (~850 ms wait) → `Skip ROM` → `Read Scratchpad` per DQ line, converts the raw S(5).F(11) result into IEEE-754 float32, and exposes a per-line CSR window with `processor_go`, `crc_err`, `init_err`, and `sample_valid`.
- `rtl/vhdl/onewire_sense_vector_bridge/onewire_sense_vector_bridge.vhd` — board-level conduit fan-out from the controller IP's six-bit `sense_dq` bus to the FEB's six split sensor conduits. Treated as a transparent bridge for unit DV; functional case `B128` proves the channel order.

**Packaging:**
- `script/onewire_master_hw.tcl`
- `script/onewire_master_controller_hw.tcl`
- `script/onewire_sense_vector_bridge_26p0p330_hw.tcl`

**Reference clock and timing budget:**
- Functional clock `csi_clock_clk` is sourced from the FEB system clock; nominal `REF_CLOCK_RATE = 156_250_000` Hz; the IP downconverts to a 1 MHz pseudo-tick used by every link-layer micro-state.
- Unit DV runs at a synthetic `REF_CLOCK_RATE = 1_000_000` Hz (1 µs per fast cycle, 1 µs per slow tick) so that the 1-Wire timing table compresses to a few thousand cycles per byte, keeping the 850 ms WAITING and 1 s ERROR holds simulatable in seconds rather than hours. The DS18B20 model is parameterized in real time (ns), so the DV harness asserts that the link-layer protocol timing is identical between fast and slow REF_CLOCK_RATE.

**Author:** Yifeng Wang (yifenwan@phys.ethz.ch)
**Date:** 2026-04-28
**Status:** Active implementation scoreboard for the FEB six-line DS18B20 controller path with planned single-device Read-ROM serial-number probe.

---

## 1. Verification Targets

The DV gate must prove every contract below before signoff. Each target maps to one or more bucket cases.

| target | RTL evidence | bucket cases | state |
|---|---|---|---|
| Master CSR identity and range checker | `proc_helper_range_checker`, `csr_w_0/1/2` mmap | `B001-B024`, `X001-X012` | catalog-ready |
| Master TX engine `IDLE → LOAD → INIT → SEND → EVAL → ACK` | `proc_tx_engine`, `tx_flow_t` | `B025-B044`, `E001-E020`, `X013-X040` | implemented first direct-master slice (`B025-B033` PASS); remaining byte-count/parasitic variants pending |
| Master RX engine `IDLE → EVAL_CMD → RECV → STORE → ACK` | `proc_rx_engine`, `rx_flow_t` | `B045-B060`, `E021-E040`, `X041-X055` | catalog-ready |
| Init pulse, presence sample, paracitic powering | `init_timing_t`, `csr.paracitic_pw`, `coe_sense_dq_in/out/oe` | `B061-B070`, `E041-E055`, `X056-X070` | catalog-ready |
| TX/RX FIFO scfifo control and bypass paths | `e_tx_fifo`, `e_rx_fifo`, addr 3 (rx fifo read), addr 4 (tx fifo write) | `B071-B080`, `E056-E075`, `X071-X085` | catalog-ready |
| Six-line round-robin via ticket lock | `proc_ticket_lock`, `proc_ticket_lock_comb`, priority rotation | `B081-B090`, `E076-E090`, `X086-X100`, `P081-P100` | catalog-ready |
| Controller core, tx, rx state spines | `core_state_t`, `core_tx_state_t`, `core_rx_state_t`, `tx_flow_t`, `rx_flow_t` | `B091-B100`, `E091-E110`, `X101-X115` | catalog-ready |
| Pipe IPC handshakes (`main2tx`, `main2rx`, `tx2flow`, `rx2flow`) | `proc_processor` per-line | `B101-B108`, `E111-E125`, `X111-X115` | catalog-ready |
| 850 ms WAITING and 1 s ERROR holds | `core_state` `WAITING` / `ERROR`, `slow_timer_cnt_unsigned > 850_000` / `> 1_000_000` | `B109-B113`, `P041-P060` | catalog-ready |
| `format_convertor` raw S.F → IEEE-754 float32 | `proc_format_convertor_comb`, `f32_sign/expo/frac` | `B001`, `B002`, `B114-B120` | partially implemented (`B001/002` PASS) |
| `sample_valid` clear-on-MM_CONFIG, set-on-ST_GET_DATA-tail | `rx_flow` MM_CONFIG, ST_GET_DATA, `STATUS[26]` | `B001`, `B124`, `E123-E125` | partially implemented (`B001/124` PASS) |
| Zero-temperature guard (`temp_frac=0` → `0x00000000`) | `proc_format_convertor_comb` | `B001`, `B120` | PASS in unit sim for `B001`; board pending |
| Controller CSR identity, sel_line, processor_go, error sticky | `proc_csr_hub` | `B003`, `B121-B125`, `X121-X125` | implemented for current CSR slice (`B003`, `B121-B125`, `X121-X125` PASS) |
| IRQ assertion and clear-on-CSR-touch | `ins_complete_irq` set in `pipe_handler`, cleared on read/write | `B089-B090`, `P061-P080`, `X116-X120` | catalog-ready |
| Concurrent six-line saturation, IRQ scaling | per-line `gen_processor` | `P001-P040`, `P081-P100` | catalog-ready |
| `crc_err` hookup (planned RTL fix) | `csr.crc_err`, `crc_checker_maxim_crc8` | `X116-X120` | reset fixed; CRC checker still gated on RTL fix |
| Read-ROM single-device serial probe (new feature, planned RTL) | new `core_state` `PROBE_*`, new CSR words `ROM_LO`/`ROM_HI`/`ROM_VALID` | `B126-B130`, `E126-E130`, `P121-P130`, `X126-X130` | catalog-ready (gated on RTL fix) |
| Six-line conduit ordering through `onewire_sense_vector_bridge` | `temp_mutrig{0,1}_*`, `temp_sipm{0,1}_*`, `temp_dab{0,1}_*` | `B082`, `X102` | catalog-ready |

A target is signed off only when every cited case passes in the harness with the specified primary checks.

## 2. Scope Boundary

**In scope:**

- One DS18B20-compatible device per DQ line, six DQ lines driven from one controller IP instance.
- Controller-master AVMM and AVST handshakes (no shortcut paths in the harness — the real controller drives the real master).
- Per-line ticket-lock arbitration through the master CSR/streaming bus.
- Read-Scratchpad temperature path including `format_convertor` corner cases.
- Read-ROM single-device serial-number probe (planned; see Section 5).
- IRQ behavior, CSR side-effects on IRQ, and `sample_valid` semantics.
- Recovery from a missing or briefly-absent presence pulse, including the 1 s `ERROR` hold.

**Out of scope (no claim in this gate):**

- Multi-drop search ROM (the FEB topology is one device per line; the master IP only needs Skip-ROM and Read-ROM).
- Strong pull-up driver electronics for parasitic powering (paracitic_pw is exercised at the protocol level only).
- altIOBUF tristate primitive — the harness drives `coe_sense_dq_in/out/oe` directly.
- Cross-IP integration with the SC hub, OneWire IRQ aggregator, or temperature monitor on FEB top.

## 3. Bucket Catalog

| bucket | id range | case count | file |
|---|---|---|---|
| BASIC | `B001-B130` | 130 | [DV_BASIC.md](DV_BASIC.md) |
| EDGE | `E001-E130` | 130 | [DV_EDGE.md](DV_EDGE.md) |
| PROF | `P001-P130` | 130 | [DV_PROF.md](DV_PROF.md) |
| ERROR | `X001-X130` | 130 | [DV_ERROR.md](DV_ERROR.md) |
| CROSS | `C001-C020` | 4 frame baselines | [DV_CROSS.md](DV_CROSS.md) |

Bucket case IDs are stable. New cases append within the reserved range; do not reorder.

## 4. Master CSR Map (frozen)

| addr | name | access | bits | meaning |
|:---:|---|:---:|---|---|
| 0 | `MASTER_CTRL_STATUS` | RW | `[0]` | `commit` (W) / `busy` (R) |
| 0 | | R | `[8]` | `out_of_range` sticky (range-checker rejected last descriptor) |
| 0 | | R | `[9]` | `init_fail` sticky (no presence pulse on last init) |
| 0 | | R | `[10]` | `tx_fifo_empty` error sticky (TX engine starved mid-transfer) |
| 0 | | R | `[11]` | `rx_fifo_full` error sticky (RX engine could not write last byte; only set when `DEBUG_LV>0`) |
| 1 | `MASTER_DESC` | RW | `[0]` | `direction` (1 = TX, 0 = RX) |
| 1 | | RW | `[1]` | `init` (1 = send init pulse before data) |
| 1 | | RW | `[2]` | `paracitic_pw` (1 = strong PULL_HIGH during DQ idle; only writable when generic `PARACITIC_POWERING = false`) |
| 1 | | RW | `[3]` | `usewire_id` (1 = override AVST channel with `wire_id`) |
| 1 | | RW | `[15:8]` | `tot_bytes` (range-checked against `MAX_BUFFER_DEPTH = 16`) |
| 1 | | RW | `[23:16]` | `wire_id` (range-checked against `N_DQ_LINES-1 = 5`) |
| 2 | `MASTER_FIFOFILL` | R | `[TX_FIFO_WIDTHU-1:0]` | `tx_fifo.usedw` |
| 2 | | R | `[RX_FIFO_WIDTHU-1+16:16]` | `rx_fifo.usedw` |
| 3 | `MASTER_RX_BYPASS` | R (two-phase) | `[AVST_DATA_WIDTH+AVST_CHANNEL_WIDTH-1:0]` | one popped RX FIFO entry (channel ‖ data); first read returns the entry, the immediately next read returns 0 with `waitrequest=1` |
| 4 | `MASTER_TX_BYPASS` | W (two-phase) | `[AVST_DATA_WIDTH+AVST_CHANNEL_WIDTH-1:0]` | one pushed TX FIFO entry; first write asserts `waitrequest=1`, the second write completes with `waitrequest=0` |

The master CSR is a **byte-addressed** Avalon-MM slave with a 4-bit address (16 word slots). Only addresses 0..4 are decoded; all other addresses fall through to `when others => null;` and complete with `readdata = 0` and `waitrequest = 0`.

## 5. Controller CSR Map (Section 5.1 frozen, Section 5.2 planned)

### 5.1 Frozen registers

The controller now follows the common Mu3e CSR-header template required by the
IP-packaging flow. Offsets are word indexes inside the IP's Avalon-MM CSR
slave; the FEB system integration adds the live base address.

| addr | name | access | bits | meaning |
|:---:|---|:---:|---|---|
| 0 | `UID` | R | `[31:0]` | common IP identifier, default ASCII `OWMC` (`0x4F574D43`) |
| 1 | `META` | RW/R | `[1:0]` | write selector: `0` VERSION, `1` VERSION_DATE, `2` VERSION_GIT, `3` INSTANCE_ID |
| 1 | | R | `[31:0]` | selected metadata payload; VERSION encodes `MAJOR[31:24]`, `MINOR[23:16]`, `PATCH[15:12]`, `BUILD[11:0]` |
| 2 | `SCRATCH` | RW | `[31:0]` | host scratch/readback word for CSR liveness |
| 3 | `CAPABILITY` | R | `[15:0]` | `n_dq_lines` (= compile-time `N_DQ_LINES`, fixed at 6 for FEB) |
| 3 | | R | `[31:16]` | `n_sensors` reserved (RAZ until Read-ROM probe lands) |
| 4 | `STATUS` | RW | `[15:0]` | `sel_line` index for per-line readback and `processor_go` writes |
| 4 | | RW | `[16]` | `processor_go` for the `sel_line`-addressed line |
| 4 | | R | `[24]` | `crc_err` for the addressed line (reset fixed to 0; checker hookup is still planned) |
| 4 | | R | `[25]` | `init_err` for the addressed line — reflects the most recent `tx_flow.READBACK` capture of the master `csr.init_fail` (master bit 9). Updates every temperature loop; cleared by reset. **Not** sticky-once-set on the controller side (unlike the master, which holds `init_fail` until reset). |
| 4 | | R | `[26]` | `sample_valid` for the addressed line — clears at `rx_flow.MM_CONFIG` (start of scratchpad read), sets at the tail of `rx_flow.ST_GET_DATA` (after the 9th byte) |
| 5..5+N_DQ_LINES-1 | `SENSORn_TEMP_F32` | R | `[31:0]` | IEEE-754 float32 temperature for sensor n; reads 0 if `sample_valid = 0` |

### 5.2 Planned for Read-ROM serial-number probe (RTL fix gated)

The Read-ROM feature is a single-device flow per line: send `init` then `0x33` (Read ROM), receive 8 bytes (family code, 6-byte serial, CRC), validate CRC against the Maxim CRC-8 polynomial, then latch the 64-bit ROM code into a per-line shadow.

The 4-bit CSR address aperture has only 16 words, so the planned probe map uses
a selected-line ROM window instead of a per-line ROM array. The earlier idea of
`SENSORn_ROM_LO/HI` arrays would require widening the CSR address bus and is not
the adopted contract for this IP.

| addr | name | access | bits | meaning |
|:---:|---|:---:|---|---|
| 4 | `STATUS` | RW | `[17]` | `probe_request` (W: rising-edge starts a Read-ROM on `sel_line`) |
| 4 | | R | `[27]` | `probe_busy` for the addressed line |
| 4 | | R | `[28]` | `probe_valid` (1 = ROM shadow holds a successful Read-ROM result) |
| 4 | | R | `[29]` | `probe_crc_err` (1 = last Read-ROM attempt failed the CRC-8 check) |
| 4 | | R | `[30]` | `probe_no_presence` (1 = no presence pulse on last Read-ROM init) |
| 3 | `CAPABILITY` | R | `[31:16]` | `n_sensors` updated to count of lines with `probe_valid = 1` after a global probe scan |
| 11 | `SENSOR_ROM_LO` | R | `[31:0]` | low 32 bits of the selected line's 64-bit ROM code (family code in `[7:0]`, low 24 bits of serial in `[31:8]`) |
| 12 | `SENSOR_ROM_HI` | R | `[31:0]` | high 32 bits of the selected line's ROM code (high 24 bits of serial in `[23:0]`, CRC in `[31:24]`) |
| 13 | `PROBE_VALID_MASK` | R | `[N_DQ_LINES-1:0]` | one bit per DQ line with a valid ROM shadow |

Read-ROM is illegal in the FEB when `processor_go = 1` for the same line — the
controller must reject `probe_request` while the temperature loop is live (case
`X126`).

## 6. Harness Modes (per CROSS plan)

| mode | use | per-case reset | notes |
|---|---|---|---|
| `isolated` | every BASIC/EDGE/ERROR case unless flagged | yes | fresh DUT reset and DS18B20-model rearm |
| `bucket_frame` | one continuous frame per bucket, ordered by case ID | no inter-case reset | exercises FSM survivability across transactions |
| `all_buckets_frame` | the four buckets concatenated | no inter-case reset | full crossing coverage, soak target |
| `prof_long_soak` | PROF only | no | 24 h equivalent simulated (with REF_CLOCK_RATE shrink and slow-timer scaling) |

Per-case timing parameters are listed in each bucket file's header. CROSS frames use the parameter set described in `DV_CROSS.md`.

## 7. Coverage Targets (per `DV_COV.md`)

| metric | target | excludes |
|---|---|---|
| Statement | ≥ 95% | `RESET` branches reachable only from `rsi_reset_reset` glitches handled separately |
| Branch | ≥ 90% | dead `when others => null` clauses in `core_*_state_t` and `*_flow_t` |
| Toggle (port) | ≥ 80% | `coe_sense_dq_*` lines that the bridge holds at constant level for the FEB topology |
| FSM state | 100% on `core_state`, `core_tx_state`, `core_rx_state`, `tx_flow`, `rx_flow`, `tx_engine.flow`, `rx_engine.flow`, `interface_hub.pipe_handler`, `ticket_lock_state` | none |
| FSM arc | 100% on the named transitions in Section 1 (every `=>`-style transition cited in a case must be hit) | none |
| Functional (covergroups in `DV_CROSS.md`) | ≥ 95% | none |

## 8. Closure Commands

The DV gate is accepted only when every command below is green on the same SHA:

1. `make -C onewire_temp_sense/tb/uvm clean compile`
2. `make -C onewire_temp_sense/tb/uvm regress` (current implemented isolated slice)
3. `make -C onewire_temp_sense/tb/uvm run CASE=ALL_BUCKETS_FRAME MODE=all_buckets_frame` (current implemented continuous slice)
4. `make -C onewire_temp_sense/tb/uvm regress BUCKET=BASIC SEEDS=1` (future full bucket)
5. `make -C onewire_temp_sense/tb/uvm regress BUCKET=EDGE SEEDS=1` (future full bucket)
6. `make -C onewire_temp_sense/tb/uvm regress BUCKET=PROF SEEDS=1` (future full bucket)
7. `make -C onewire_temp_sense/tb/uvm regress BUCKET=ERROR SEEDS=1` (future full bucket)
8. `make -C onewire_temp_sense/tb/uvm coverage` (UCDB merge + report; pending)
9. `python3 ~/.codex/skills/dv-workflow/scripts/dv_report_gen.py --tb onewire_temp_sense/tb/doc`
10. `python3 ~/.codex/skills/rtl-doc-style/scripts/rtl_doc_style_check.py onewire_temp_sense/tb/doc`

## 9. Non-Claims

- No bucket is signed off until every cited case in Section 1 passes with primary checks observed in the harness, the CRC hookup landed in RTL, and the Read-ROM serial probe RTL is reviewed and merged.
- The live harness currently closes a named 55-case BASIC/ERROR slice (`B001-B033`, `B121-B125`, `X001-X012`, `X121-X125`) plus the implemented `ALL_BUCKETS_FRAME`. This is scoreboard evidence, not full bucket closure.
- Board correlation (System Console + SignalTap) is a separate signoff phase. It is required for the temperature path to graduate from "unit-pass" to "board-pass" before Read-ROM probing is exercised in the field.
- The `crc_err` controller status bit resets to 0 on the present RTL, but the CRC checker is not yet wired between the master scratchpad readback and the controller; cases `X116-X120` are catalog-ready but blocked on the RTL fix.
- The Read-ROM serial probe is RTL-fix-gated. Cases `B126-B130`, `E126-E130`, `P121-P130`, `X126-X130` are catalog-ready and will activate when the new state machine and CSR map of Section 5.2 are merged.

## 10. Companion Documents

- [DV_HARNESS.md](DV_HARNESS.md) — UVM harness boundary, agent topology, DS18B20 model, bind targets
- [DV_BASIC.md](DV_BASIC.md) — bucket catalog: basic functional sanity (`B001-B130`)
- [DV_EDGE.md](DV_EDGE.md) — bucket catalog: boundary and timing-margin (`E001-E130`)
- [DV_PROF.md](DV_PROF.md) — bucket catalog: throughput, soak, fairness (`P001-P130`)
- [DV_ERROR.md](DV_ERROR.md) — bucket catalog: faults, resets, illegal access (`X001-X130`)
- [DV_CROSS.md](DV_CROSS.md) — crossing coverage and continuous-frame baselines
- [DV_REPORT.md](DV_REPORT.md) — generated dashboard
- [DV_COV.md](DV_COV.md) — generated coverage rollup
- [BUG_HISTORY.md](BUG_HISTORY.md) — append-only bug ledger
