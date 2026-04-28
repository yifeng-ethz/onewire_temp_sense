# ⚠️ DV Report — onewire_master_controller mixed_vhdl_sv

**DUT:** `onewire_master_controller` and `onewire_master` &nbsp; **Date:** `2026-04-28` &nbsp; **RTL variant:** `26.2.0 common-csr-header` &nbsp; **Seed:** `1`

This page is the chief-architect dashboard for the six-line DS18B20 monitor
path. Case-level evidence lives under [`../REPORT/`](../REPORT/). Functional coverage
is derived from bucket-level features, not from a forced one-case/one-feature
mapping.

## Legend

✅ pass / closed &middot; ⚠️ partial / below target / known limitation &middot; ❌ failed / missing evidence &middot; ❓ pending &middot; ℹ️ informational

## Health

| status | field | value |
|:---:|---|---|
| ✅ | failed_cases | `0` |
| ✅ | signoff_runs_with_failures | `0` |
| ⚠️ | catalog_backlog_cases | `465` |
| ⚠️ | unimplemented_cases | `465` |
| ✅ | stale_artifacts | `0` |

## Signoff Scope

| field | claimed value |
|---|---|
| DUT_IMPL | `mixed_vhdl_sv` |
| ONEWIRE_N_DQ_LINES | `6` |
| SENSOR_TOPOLOGY | `one DS18B20-compatible device per DQ line` |
| CONTROLLER_CSR_MAP | `UID=0, META=1, SCRATCH=2, CAPABILITY=3, STATUS=4, SENSOR0..5_TEMP_F32=5..10` |
| probe_only_exclusions | `Read-ROM serial probe, scratchpad CRC checker hookup, UCDB code coverage` |

## Non-Claims

- No full DV bucket is signed off yet; current evidence is a 55-case BASIC/ERROR slice.
- Read-ROM serial-number probing is planned with selected-line `SENSOR_ROM_LO/HI` registers after temperature path closure.
- `crc_err` is reset-clean, but the scratchpad CRC checker is still not wired into the controller datapath.
- Board correlation through System Console and SignalTap remains pending for this unit DV gate.

## Bucket Summary

| status | bucket | catalog_planned | promoted | evidenced | backlog | merged | promoted functional |
|:---:|---|---:|---:|---:|---:|---|---|
| ⚠️ | [`BASIC`](DV_BASIC.md) | 130 | 38 | 38 | 92 | stmt=n/a, branch=n/a, cond=n/a, expr=n/a, fsm_state=n/a, fsm_trans=n/a, toggle=n/a | 77.8% (14/18) |
| ❓ | [`EDGE`](DV_EDGE.md) | 130 | 0 | 0 | 130 | stmt=n/a, branch=n/a, cond=n/a, expr=n/a, fsm_state=n/a, fsm_trans=n/a, toggle=n/a | 0.0% (0/8) |
| ❓ | [`PROF`](DV_PROF.md) | 130 | 0 | 0 | 130 | stmt=n/a, branch=n/a, cond=n/a, expr=n/a, fsm_state=n/a, fsm_trans=n/a, toggle=n/a | 0.0% (0/8) |
| ⚠️ | [`ERROR`](DV_ERROR.md) | 130 | 17 | 17 | 113 | stmt=n/a, branch=n/a, cond=n/a, expr=n/a, fsm_state=n/a, fsm_trans=n/a, toggle=n/a | 60.0% (6/10) |

## Totals

| status | metric | pct | target |
|:---:|---|---|---|
| ❓ | stmt | n/a | 95.0 |
| ❓ | branch | n/a | 90.0 |
| ❓ | cond | n/a | - |
| ❓ | expr | n/a | - |
| ❓ | fsm_state | n/a | 95.0 |
| ❓ | fsm_trans | n/a | 90.0 |
| ❓ | toggle | n/a | 80.0 |

- catalog_planned_cases: `520`
- promoted_signoff_cases: `55`
- evidenced_promoted_cases: `55`
- promoted functional coverage: `45.5% (20/44)`

## Signoff Runs

| status | run_id | kind | build | seq | txns | cross_pct |
|:---:|---|---|---|---|---:|---:|
| ✅ | [`onewire_named_isolated_slice`](../REPORT/onewire_regression_summary.json) | isolated | 26.2.0 | implemented BASIC/ERROR slice | 55 | 45.5 |
| ✅ | [`ow_all_buckets_frame`](../REPORT/ALL_BUCKETS_FRAME.md) | all_buckets_frame | 26.2.0 | BASIC then ERROR implemented slice | 55 | 45.5 |

## Index

- [`../REPORT/`](../REPORT/) — case and run evidence
- [`DV_PLAN.md`](DV_PLAN.md) — verification intent and CSR contract
- [`DV_COV.md`](DV_COV.md) — feature-derived coverage scoreboard
- [`DV_REPORT.json`](DV_REPORT.json) — machine-readable scoreboard summary
- [`BUG_HISTORY.md`](BUG_HISTORY.md) — active bug ledger

_Regenerate with `make -C onewire_temp_sense/tb/uvm regress` for the implemented slice. Replace with a generator-owned `dv_report_gen.py` refresh once the full bucket regression is automated._
