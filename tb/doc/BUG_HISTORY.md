# BUG_HISTORY.md - onewire_master_controller DV bug ledger

Class legend:
- `R` = RTL / DUT bug
- `H` = harness / testcase / reporting bug

Severity legend:
- `soft error` = the bad packet/data flushes through the stream and does not leave the later datapath stuck
- `hard stuck error` = the bug poisons later packet handling and typically needs a functional reset / fresh restart to recover
- `non-datapath-refactor` = observability, reporting, harness, or naming/accounting consistency work with no direct packet-contract effect

Encounterability legend:
- practical severity is `severity x encounterability`, so the index must say how likely a reader is to hit the bug in normal use rather than only when it first appeared in one simulation log
- nominal datapath operation = legal traffic, about `50%` link load, iid per-lane behavior, and no forced error injection or artificially pathological stalls
- nominal control-path operation = routine bring-up / CSR program / readback / clear-counter sequences
- `common (...)` = readily hit in nominal operation
- `occasional (...)` = hit in nominal operation without heroic setup, but not in every short run
- `rare (...)` = legal in nominal operation, but usually needs long runtime or unlucky alignment
- `corner-only (...)` = requires a legal but non-nominal stress or corner profile
- `directed-only (...)` = requires targeted error injection, formal/probe flow, reporting-only flow, or another non-operational stimulus
- detailed `min / p50 / max` first-hit sim-time studies may still appear inside individual bug sections when useful

Fix status detail contract for active entries and future updates:
- `state` = fixed / open / partial plus the current verification gate
- `mechanism` = how the implemented repair changes the RTL or harness behavior
- `before_fix_outcome` and `after_fix_outcome` = concise evidence showing what changed
- `potential_hazard` = whether the fix looks permanent or is still provisional / profile-limited
- `Claude Opus 4.7 xhigh review decision` = explicit review state; use `pending / not run` until that review has actually happened

Historical formal note:
- No historical formal runs are recorded for this IP yet.

## Index

| bug_id | class | severity | encounterability | status | first seen | commit | summary |
|---|---|---|---|---|---|---|---|
| [BUG-001-R](#bug-001-r-zero-scratchpad-converts-to-10-c) | R | soft error | `common (control-path routine)` | fixed in unit sim; board pending | Phase-5 environment readback on `2026-04-28` | `pending` | Zeroed scratchpad bytes converted to IEEE-754 `1.0`; RTL now reports zero for raw zero and exposes `sample_valid`. |
| [BUG-002-H](#bug-002-h-ds18b20-model-skipped-command-slots-after-master-written-one-bits) | H | non-datapath-refactor | `directed-only (new unit harness)` | fixed | OneWire smoke tb on `2026-04-28` | `pending` | DS18B20 model waited for a posedge that already happened on write-1 slots, causing command decode drift. |
| [BUG-003-R](#bug-003-r-controller-crc_err-status-bit-was-uninitialized) | R | soft error | `common (control-path routine)` | fixed in unit sim; board pending | `B001` after 4-state checker tightening on `2026-04-28` | `pending` | Controller `STATUS[24] crc_err` was declared but not reset/driven, so stricter DV saw `X` on healthy reads. |

## 2026-04-28

### BUG-001-R: Zero scratchpad converts to 1.0 C
- First seen in:
  - Phase-5 on-board environment monitor readback on `2026-04-28`
  - all six controller lines showed `processor_go=1`, `crc_err=0`,
    `init_err=0`, and temperature word `0x3f800000`
- Symptom:
  - a zeroed DS18B20 scratchpad maps to float32 `1.0 C`
  - this makes no-data/stale-data look like a plausible low temperature
- Root cause:
  - the format converter defaulted exponent to `127` before scanning the raw
    DS18B20 fraction bits
  - when all raw fraction bits are zero, the default remains `1.0` instead of
    the zero float
- Fix status:
  - state:
    - fixed in unit simulation; on-board reflash/readback pending
  - mechanism:
    - raw-zero temperature input now maps to IEEE-754 `0x00000000`
    - controller status bit `STATUS[26]` now reports `sample_valid` after a
      full scratchpad capture; temperature reads return zero until valid
  - before_fix_outcome:
    - board readback returns `0x3f800000` for all six lines
  - after_fix_outcome:
    - `make -C onewire_temp_sense/tb/uvm clean compile run`
      passes; line 0 is an intentional valid 0 C sample returning
      `0x00000000`, lines 1..5 return `0x41b88000`, `0x41c10000`,
      `0x41c98000`, `0x41d20000`, `0x41da8000`, and every line has
      `sample_valid=1`
  - potential_hazard:
    - board path still needs reflash and System Console evidence; the unit
      repair is expected to be permanent for the converter/stale-data class
  - Claude Opus 4.7 xhigh review decision:
    - pending / not run
- Runtime / coverage context:
  - unit harness now classifies a valid 0 C sample separately from no-data
    status through `sample_valid`
- Commit:
  - pending

### BUG-002-H: DS18B20 model skipped command slots after master-written one bits
- First seen in:
  - `make -C onewire_temp_sense/tb/uvm compile run`
  - debug trace decoded expected `0xCC` / `0x44` controller commands as
    `0x24`, then returned all-ones scratchpad data
- Symptom:
  - all six simulated temperature words became `0xc2ffe000` before the
    scoreboard was tightened
- Root cause:
  - the DS18B20 behavioral model sampled a master-written `1` correctly, but
    then waited for a future `posedge dq`; that posedge had already occurred
    before the 15 us sample point, so the model consumed the next slot boundary
    as the release for the current bit
- Fix status:
  - state:
    - fixed in the tb model; smoke rerun passes
  - mechanism:
    - only low sampled command bits wait for the release edge; high sampled
      bits wait to the end of the slot and then arm for the next falling edge
  - before_fix_outcome:
    - commands decoded as `0x24` / `0xa4`; temperature mismatch
      `got 0xc2ffe000`
  - after_fix_outcome:
    - commands decode as `0xCC`, `0x44`, and `0xBE`; six exact float32
      compares pass: `0x41b00000`, `0x41b88000`, `0x41c10000`,
      `0x41c98000`, `0x41d20000`, `0x41da8000`
  - potential_hazard:
    - low; this is isolated to the new behavioral model and preserves the
      datasheet slot direction
  - Claude Opus 4.7 xhigh review decision:
    - pending / not run
- Runtime / coverage context:
  - this bug prevented the new unit harness from being a trustworthy board
    debug reference
- Commit:
  - pending

### BUG-003-R: Controller crc_err status bit was uninitialized
- First seen in:
  - `make -C onewire_temp_sense/tb/uvm compile run CASE=B001`
  - after the harness moved from permissive smoke checks to explicit 4-state
    status checks
- Symptom:
  - line 0 reached `sample_valid=1` and returned the correct `0x00000000`
    valid 0 C temperature, but the case failed because `STATUS[24] crc_err`
    was `X` rather than `0`
- Root cause:
  - `csr.crc_err` existed in the controller CSR record but had no reset
    assignment and no active CRC checker drive in the current RTL
- Fix status:
  - state:
    - fixed in unit simulation; board reflash/readback pending
  - mechanism:
    - reset now initializes `csr.processor_go` and `csr.crc_err` to zero
    - the future CRC checker hookup remains a planned feature and must replace
      the fixed-zero behavior when implemented
  - before_fix_outcome:
    - `B001` failed with `status=0x0X010000`
  - after_fix_outcome:
    - `B001` passes; line 0 returns `0x00000000`, `sample_valid=1`,
      `init_err=0`, and `crc_err=0`
  - potential_hazard:
    - medium; the reset is permanent, but the CRC checker is still not
      implemented, so CRC mismatch cases remain planned rather than closed
  - Claude Opus 4.7 xhigh review decision:
    - pending / not run
- Runtime / coverage context:
  - this was found by using the bucket case as functional feature evidence
    instead of accepting the older smoke log
- Commit:
  - pending
