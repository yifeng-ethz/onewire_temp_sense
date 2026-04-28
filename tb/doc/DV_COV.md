# DV Coverage Summary — onewire_master_controller

This page is the live coverage scoreboard. Functional coverage is derived from
bucket-level features, not from a forced one-case/one-feature mapping. A case may
cover multiple features, and several cases may only reinforce the same feature.

Latest evidence:

- Isolated named-case slice: `../REPORT/onewire_regression_summary.json`
  (`2026-04-28T17:27:17`, 55/55 implemented BASIC+ERROR cases PASS)
- Continuous no-restart frame: `../REPORT/ALL_BUCKETS_FRAME.md`
- DS18B20 SystemC trace: `../../model/artifacts/ds18b20_systemc_trace.csv`

## Legend

✅ pass / closed &middot; ⚠️ partial / below target or known limitation &middot; ❌ failed / missing evidence &middot; ❓ pending &middot; ℹ️ informational

## Coverage Targets

| status | metric | target | current | note |
|:---:|---|---:|---:|---|
| ⚠️ | functional BASIC features | 95.0 | 14/18 | master CSR descriptor/commit, TX FSM, and controller CSR/temp slices closed; ticket-lock, RX/FIFO, CRC, and Read-ROM remain open |
| ⚠️ | functional ERROR features | 95.0 | 6/10 | master and controller CSR negative slices closed; presence/FIFO/reset/CRC/Read-ROM faults remain open |
| ❓ | functional EDGE features | 95.0 | 0/8 | timing-boundary and sample-valid transient cases still need dedicated probes |
| ❓ | functional PROF features | 95.0 | 0/8 | soak/rate/fairness cases need long-run harness mode |
| ❓ | statement | 95.0 | n/a | UCDB collection not enabled yet |
| ❓ | branch/FSM/toggle | 90.0/90.0/80.0 | n/a | UCDB collection not enabled yet |

## Functional Feature Catalog

### BASIC Features

| feature_id | status | evidence | notes |
|---|---|---|---|
| BASIC.common_uid_meta_header | PASS | `B003`, `ALL_BUCKETS_FRAME` | UID/META pages VERSION/DATE/GIT/INSTANCE_ID |
| BASIC.csr_scratchpad | PASS | `B003`, `ALL_BUCKETS_FRAME` | common scratch/readback word |
| BASIC.capability_readback | PASS | `B003`, `B121`, `ALL_BUCKETS_FRAME` | `N_DQ_LINES=6`, `n_sensors=0` until probe feature lands |
| BASIC.per_line_status_select | PASS | `B122`, `B002`, `ALL_BUCKETS_FRAME` | `sel_line` echo and selected-line status view |
| BASIC.processor_go_per_line | PASS | `B123`, `ALL_BUCKETS_FRAME` | one-line start does not start another line |
| BASIC.sample_valid_status | PASS | `B001`, `ALL_BUCKETS_FRAME` | `STATUS[26]` asserts after scratchpad capture |
| BASIC.temperature_read_zero_until_valid | PASS | `B124`, `ALL_BUCKETS_FRAME` | temp read is zero before selected line has a valid sample |
| BASIC.temperature_float32_readback | PASS | `B001`, `B124`, `B002`, `ALL_BUCKETS_FRAME` | exact model float32 words |
| BASIC.zero_temperature_guard | PASS | `B001`, `ALL_BUCKETS_FRAME` | valid 0 C returns `0x00000000`, not `0x3f800000` |
| BASIC.six_line_temperature_loop | PASS | `B002`, `ALL_BUCKETS_FRAME` | six DS18B20 lines complete one monitor loop |
| BASIC.master_csr_descriptor_range | PASS | `B004-B008`, `B011-B019`, `B021-B022`, `ALL_BUCKETS_FRAME` | master reset status, descriptor defaults, valid/invalid `tot_bytes` and `wire_id`, protected writes, undecoded accesses |
| BASIC.master_commit_busy_irq | PASS | `B009`, `B010`, `B020`, `B023`, `B024`, `ALL_BUCKETS_FRAME` | commit-to-busy handoff, TX/RX empty command completion, waitrequest contract, clear-on-touch IRQ |
| BASIC.master_tx_engine_fsm | PASS | `B025-B033`, `ALL_BUCKETS_FRAME` | direct master TX commits through empty, 1..7 byte payloads, init/no-init, selected-line DQ activity, channel-driven wire selection |
| BASIC.controller_init_err_readback | PASS | `B125`, `ALL_BUCKETS_FRAME` | selected-line `STATUS[25]` reflects the missing-presence path and does not mirror unrelated lines |
| BASIC.ticket_lock_round_robin | planned | none | cases `B081-B090` |
| BASIC.master_rx_engine_fsm | planned | none | cases `B045-B060` need RX byte-order and scratchpad capture probes |
| BASIC.master_fifo_bypass | planned | none | cases `B071-B080` need addr 3/4 two-phase bypass and AVST contention probes |
| BASIC.crc_checker_status | planned | none | `csr.crc_err` reset fixed; checker hookup still absent |
| BASIC.read_rom_probe | blocked | none | gated until serial-number RTL is implemented |

### ERROR Features

| feature_id | status | evidence | notes |
|---|---|---|---|
| ERROR.read_only_csr_noop | PASS | `X121`, `ALL_BUCKETS_FRAME` | UID and CAPABILITY writes are ignored |
| ERROR.master_csr_illegal_access | PASS | `X001-X012`, `ALL_BUCKETS_FRAME` | descriptor range rejects, busy-guard behavior, undefined reads, empty RX bypass, TX FIFO full auto-flush, and parasitic generic guard |
| ERROR.temperature_write_noop | PASS | `X122`, `ALL_BUCKETS_FRAME` | SENSOR temp words remain RTL-owned |
| ERROR.invalid_sel_line_status_zero | PASS | `X123`, `ALL_BUCKETS_FRAME` | invalid selected line returns zero status bits |
| ERROR.invalid_sel_line_go_suppressed | PASS | `X124`, `ALL_BUCKETS_FRAME` | invalid `processor_go` write starts no line |
| ERROR.unmapped_temperature_slot_zero | PASS | `X125`, `ALL_BUCKETS_FRAME` | temp word beyond line 5 reads zero |
| ERROR.missing_presence_init_err | planned | none | absent-sensor model control required |
| ERROR.fifo_underflow_overflow | planned | none | master-only or internal-probe harness required |
| ERROR.mid_transfer_reset | planned | none | needs reset sequencer and boundary assertions |
| ERROR.crc_mismatch | planned | none | blocked on CRC checker RTL hookup |
| ERROR.read_rom_faults | blocked | none | blocked on Read-ROM probe RTL |

### EDGE Features

| feature_id | status | evidence | notes |
|---|---|---|---|
| EDGE.onewire_slot_boundaries | planned | none | `E010-E036` |
| EDGE.presence_window_edges | planned | none | `E041-E055` |
| EDGE.fifo_depth_edges | planned | none | `E056-E075` |
| EDGE.ticket_lock_priority_edges | planned | none | `E076-E090` |
| EDGE.waiting_error_timer_edges | planned | none | `E091-E094` |
| EDGE.pipe_handshake_edges | planned | none | `E095-E122` |
| EDGE.sample_valid_clear_set | planned | none | `E123-E125` |
| EDGE.read_rom_edges | blocked | none | `E126-E130` |

### PROF Features

| feature_id | status | evidence | notes |
|---|---|---|---|
| PROF.single_line_rate | planned | none | `P001-P020` |
| PROF.six_line_saturation | planned | none | `P021-P040` |
| PROF.waiting_error_parallelism | planned | none | `P041-P060` |
| PROF.irq_scaling | planned | none | `P061-P080` |
| PROF.ticket_lock_fairness_soak | planned | none | `P081-P100` |
| PROF.temperature_value_sweep | planned | none | mixed/sign/range cases |
| PROF.csr_poll_under_load | planned | none | polling during live loops |
| PROF.read_rom_probe_soak | blocked | none | `P121-P130` |

## Case Execution Tables

### BASIC

| case_id | type (d/r) | coverage_by_this_case | executed random txn | coverage_incr_per_txn |
|---|---|---|---:|---|
| B001 | d | zero-temp guard, sample_valid, temp f32 | 0 | feature delta +3 |
| B002 | d | six-line loop, status select reinforcement | 0 | feature delta +1 |
| B003 | d | common UID/META, scratch, capability | 0 | feature delta +3 |
| B004 | d | master reset status bits | 0 | feature delta +1 (`master_csr_descriptor_range`) |
| B005 | d | master descriptor reset with parasitic generic true | 0 | reinforces `master_csr_descriptor_range` |
| B006 | d | master descriptor reset with parasitic generic false | 0 | reinforces `master_csr_descriptor_range` |
| B007 | d | master FIFO fill reset read | 0 | reinforces `master_csr_descriptor_range` |
| B008 | d | master undecoded read fall-through | 0 | reinforces `master_csr_descriptor_range` |
| B009 | d | master TX commit-to-busy path | 0 | feature delta +1 (`master_commit_busy_irq`) |
| B010 | d | master RX commit-to-busy path | 0 | reinforces `master_commit_busy_irq` |
| B011 | d | master valid descriptor write/readback | 0 | reinforces `master_csr_descriptor_range` |
| B012 | d | master descriptor boundary `tot_bytes=16`, `wire_id=5` | 0 | reinforces `master_csr_descriptor_range` |
| B013 | d | master invalid `tot_bytes=17` range reject | 0 | reinforces `master_csr_descriptor_range` |
| B014 | d | master non-parasitic descriptor bit write/readback | 0 | reinforces `master_csr_descriptor_range` |
| B015 | d | master parasitic generic write guard | 0 | reinforces `master_csr_descriptor_range` |
| B016 | d | master invalid `wire_id=6` range reject | 0 | reinforces `master_csr_descriptor_range` |
| B017 | d | master invalid `wire_id=7` range reject | 0 | reinforces `master_csr_descriptor_range` |
| B018 | d | master descriptor protected while busy | 0 | reinforces `master_csr_descriptor_range` |
| B019 | d | master FIFO fill write no-op | 0 | reinforces `master_csr_descriptor_range` |
| B020 | d | master commit read-after-write | 0 | reinforces `master_commit_busy_irq` |
| B021 | d | master undecoded write fall-through | 0 | reinforces `master_csr_descriptor_range` |
| B022 | d | master read high-byte leakage scan | 0 | reinforces `master_csr_descriptor_range` |
| B023 | d | master read waitrequest contract | 0 | reinforces `master_commit_busy_irq` |
| B024 | d | master complete IRQ clear-on-read/write | 0 | reinforces `master_commit_busy_irq` |
| B025 | d | empty TX commit path | 0 | feature delta +1 (`master_tx_engine_fsm`) |
| B026 | d | one-byte TX, no init, line 0 | 0 | reinforces `master_tx_engine_fsm` |
| B027 | d | one-byte TX with init/presence, line 0 | 0 | reinforces `master_tx_engine_fsm` |
| B028 | d | two-byte TX with init/presence, line 0 | 0 | reinforces `master_tx_engine_fsm` |
| B029 | d | three-byte TX, no init, line 0 | 0 | reinforces `master_tx_engine_fsm` |
| B030 | d | four-byte TX, no init, RX FIFO drained precondition | 0 | reinforces `master_tx_engine_fsm` |
| B031 | d | five-byte TX channel-driven wire selection to line 1 | 0 | reinforces `master_tx_engine_fsm` |
| B032 | d | six-byte TX with init/presence, line 1 | 0 | reinforces `master_tx_engine_fsm` |
| B033 | d | seven-byte TX with parasitic-power direct master, line 2 | 0 | reinforces `master_tx_engine_fsm` |
| B121 | d | capability reinforcement | 0 | redundant by design |
| B122 | d | selected-line CSR round-trip | 0 | feature delta +1 |
| B123 | d | per-line `processor_go` | 0 | feature delta +1 |
| B124 | d | read-zero-until-valid plus temp f32 | 0 | feature delta +1 |
| B125 | d | selected-line init_err readback | 0 | feature delta +1 |

### ERROR

| case_id | type (d/r) | coverage_by_this_case | executed random txn | coverage_incr_per_txn |
|---|---|---|---:|---|
| X001 | d | invalid master descriptor `tot_bytes=17` | 0 | feature delta +1 (`master_csr_illegal_access`) |
| X002 | d | invalid master descriptor `tot_bytes=255` | 0 | reinforces `master_csr_illegal_access` |
| X003 | d | invalid master descriptor `wire_id=6` | 0 | reinforces `master_csr_illegal_access` |
| X004 | d | invalid master descriptor `wire_id=255` | 0 | reinforces `master_csr_illegal_access` |
| X005 | d | invalid master descriptor `tot_bytes=17`, `wire_id=6` | 0 | reinforces `master_csr_illegal_access` |
| X006 | d | busy-guarded descriptor write suppression | 0 | reinforces `master_csr_illegal_access` |
| X007 | d | back-to-back commit while busy | 0 | reinforces `master_csr_illegal_access` |
| X008 | d | undefined master addr 5 read returns zero | 0 | reinforces `master_csr_illegal_access` |
| X009 | d | undefined master addr 15 read returns zero | 0 | reinforces `master_csr_illegal_access` |
| X010 | d | empty RX bypass read returns zero | 0 | reinforces `master_csr_illegal_access` |
| X011 | d | TX bypass full edge auto-flush | 0 | reinforces `master_csr_illegal_access`; partial FIFO-fault evidence only |
| X012 | d | parasitic generic blocks CSR bit clear | 0 | reinforces `master_csr_illegal_access` |
| X121 | d | read-only UID/CAPABILITY no-op | 0 | feature delta +1 |
| X122 | d | temp write no-op | 0 | feature delta +1 |
| X123 | d | invalid selected-line status zero | 0 | feature delta +1 |
| X124 | d | invalid selected-line `processor_go` suppress | 0 | feature delta +1 |
| X125 | d | unmapped temperature slot zero | 0 | feature delta +1 |

## Continuous-Frame Runs

| run_id | kind | state | notes |
|---|---|---|---|
| basic_frame | bucket_frame | PASS | included inside `ALL_BUCKETS_FRAME`; no reset between implemented BASIC cases |
| error_frame | bucket_frame | PASS | included inside `ALL_BUCKETS_FRAME`; no reset between implemented ERROR cases |
| ow_all_buckets_frame | all_buckets_frame | PASS | `../REPORT/ALL_BUCKETS_FRAME.md`; BASIC then ERROR implemented slice |
| full_catalog_all_buckets_frame | all_buckets_frame | planned | waits for EDGE/PROF and blocked Read-ROM/CRC features |

_Regenerate with `make -C onewire_temp_sense/tb/uvm regress` for the implemented slice; broader bucket automation will replace this once coverage collection is enabled._
