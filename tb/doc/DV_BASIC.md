# onewire_master_controller DV — Basic Functional Cases

**Companion docs:** [DV_PLAN.md](DV_PLAN.md), [DV_HARNESS.md](DV_HARNESS.md), [DV_EDGE.md](DV_EDGE.md), [DV_PROF.md](DV_PROF.md), [DV_ERROR.md](DV_ERROR.md), [DV_CROSS.md](DV_CROSS.md), [DV_REPORT.md](DV_REPORT.md), [BUG_HISTORY.md](BUG_HISTORY.md)
**Parent:** [DV_PLAN.md](DV_PLAN.md)
**ID Range:** `B001-B130`
**Total:** 130 cases (38 implemented / 0 waived)

This document covers the bring-up and protocol-correctness sanity for the six-line FEB DS18B20 controller path: master link layer, controller transaction layer, ticket-lock arbiter, format convertor, sample_valid guard, and the planned single-device Read-ROM serial-number probe. Every case here must pass before EDGE / PROF / ERROR cases are meaningful.

**Methodology key:**
- **D** = Directed (hand-crafted stimulus, deterministic expected output)
- **R** = Constrained-random (LCG-based PRNG; per ~/CLAUDE.md the harness uses the full Mentor floating license, so SystemVerilog `rand` and constrained-random are available)

---

## 1. Summary

<!-- Column legend
Section          : the per-functional-group section name elsewhere in this file
Cases            : how many cases the section contains
ID Range         : Bxxx-Bxxx span
What it Proves   : one-line elevator-pitch of the section's coverage target
Current Case     : `<n>/<n>` (implemented/total), or `implemented` / `waived` / `partial(<reason>)` / `pending` / `n/a`
-->

| Section | Cases | ID Range | What it Proves | Current Case |
|---|---|---|---|---|
| Master CSR Identity & Range Checker (CSR) | 24 | B001-B024 | master CSR identity header, descriptor range checker, waitrequest contract, IRQ clear-on-touch | 24/24 |
| Master TX Engine FSM (TX) | 20 | B025-B044 | TX engine `IDLE -> LOAD -> INIT -> SEND -> EVAL -> ACK` flow across all six lines, paracitic powering modes | 9/20 |
| Master RX Engine FSM (RX) | 16 | B045-B060 | RX engine `IDLE -> EVAL_CMD -> RECV -> STORE -> ACK` flow, full DS18B20 scratchpad capture per line | 0/16 |
| Init Pulse, Presence, Paracitic Powering (INIT) | 10 | B061-B070 | init pulse timing, presence sample window, paracitic vs HIGH_Z idle, init-only commands | 0/10 |
| TX/RX FIFO Bypass Two-Phase (FIFO) | 10 | B071-B080 | master CSR addr 3/4 two-phase bypass into the TX/RX scfifos, arbiter contention with AVST | 0/10 |
| Six-Line Ticket-Lock Arbitration (ARB) | 10 | B081-B090 | controller ticket-lock priority rotation across all six lines, IRQ correlation | 0/10 |
| Controller Core/TX/RX State Spines (STATE) | 10 | B091-B100 | controller per-line `core_state`/`core_tx_state`/`core_rx_state` walk and per-line independence | 0/10 |
| Pipe IPC Handshakes (PIPE) | 8 | B101-B108 | four-phase `main2tx`/`main2rx`/`tx2flow`/`rx2flow` handshakes per line, concurrency on different lines | 0/8 |
| 850 ms WAITING Hold (WAIT) | 5 | B109-B113 | controller WAITING hold uses `slow_timer > 850_000`, semaphore release, JOINING-entry counter reset | 0/5 |
| Format Convertor Happy Path (FMT) | 7 | B114-B120 | raw S(5).F(11) -> IEEE-754 float32 across the DS18B20 spec range, zero-fraction guard | 0/7 |
| Controller CSR Identity (CSRC) | 5 | B121-B125 | controller CSR identity header (CAPABILITY, sel_line, processor_go, init_err, per-line SENSOR_TEMP) | 5/5 |
| Read-ROM Serial-Number Probe (PROBE) | 5 | B126-B130 | planned single-device Read-ROM serial probe per line, ROM shadow stability, n_sensors update | 0/5 |

---

## 2. Master CSR Identity & Range Checker (CSR) -- 24 cases

These cases prove the master AVMM slave: identity header (UID/META/SCRATCH/CAPABILITY), `MASTER_CTRL_STATUS` (addr 0) busy/error sticky bits, `MASTER_DESC` (addr 1) descriptor RW with the `proc_helper_range_checker` `tot_bytes` and `wire_id` bound checks, `MASTER_FIFOFILL` (addr 2) usedw exposure, undecoded address fall-through, waitrequest contract, and the IRQ clear-on-touch behavior of `ins_complete_irq`. Cases B001/B002/B003 also exercise the controller-level smoke loop end-to-end and are the live UVM bring-up cases.

<!-- Column legend
ID                 : Bxxx canonical id, stable across edits
Method             : D (directed) or R (constrained-random)
Scenario           : one-line description of what the case does
Iter               : number of iterations (1 for directed; N for random; or e.g. `1 (smoke)`)
Stimulus           : the exact CSR / AVST / DQ stimulus the case applies — pin numbers
Pass Criteria      : the assertions the case must pass — pin signal/value
Function Reference : `TBD` (reserved for the future derived functional-list generator)
-->

| ID | Method | Scenario | Iter | Stimulus | Pass Criteria | Function Reference |
|---|---|---|---|---|---|---|
| B001 | D | start line 0, drive a real DS18B20 model at +0 C, read one full transaction loop, validate float32 word | 1 | CSR write STATUS=0x00010000 (sel_line=0, processor_go=1); wait 1 conversion period; CSR read SENSOR0_TEMP_F32 | STATUS[26]=1 within 1 conversion; SENSOR0_TEMP_F32 == 0x00000000 not 0x3F800000; init_err=0; crc_err=0 | TBD |
| B002 | D | walk all six lines through full Skip-ROM/Convert-T/Read-Scratchpad once, capture each float32 | 1 | CSR write STATUS sel_line=N processor_go=1 for each N in 0..5; wait one conversion per line; CSR read SENSORn_TEMP_F32 | per line N: STATUS[16]=1 sticks, STATUS[26]=1 sticks, SENSORn_TEMP_F32 matches expected_temp_word(N) (smoke harness golden) | TBD |
| B003 | D | reset, read common UID/META/SCRATCH, then read CAPABILITY at csr addr 3 before any processor_go | 1 | CSR reads at addr 0..3 in sequence; CSR write SCRATCH and read back | UID == `0x4F574D43`; META VERSION/DATE/GIT/INSTANCE pages read back; SCRATCH writes round-trip; CAPABILITY[15:0] == N_DQ_LINES (0x0006); CAPABILITY[31:16] == 0 (n_sensors RAZ); avs_csr_waitrequest deasserts within 1 cycle | TBD |
| B004 | D | master CSR addr 0 read after reset (no commit, no transaction in flight) | 1 | CSR read at master addr 0 immediately after reset release | readdata[0]=csr.busy=0; readdata[8]=out_of_range=0; readdata[9]=init_fail=0; readdata[10]=tx_fifo_empty=0; readdata[11]=rx_fifo_full=0; waitrequest deasserts in 1 cycle | TBD |
| B005 | D | master CSR addr 1 read after reset, with PARACITIC_POWERING=true generic | 1 | instantiate with PARACITIC_POWERING=true; CSR read at master addr 1 | readdata[2]=paracitic_pw=1 (bound by generic at reset); direction/init/usewire_id/tot_bytes/wire_id all 0 | TBD |
| B006 | D | master CSR addr 1 read after reset, with PARACITIC_POWERING=false generic | 1 | instantiate with PARACITIC_POWERING=false; CSR read at master addr 1 | readdata[2]=paracitic_pw=0; descriptor fields zero; field width matches mmap.csr_w_1 ranges | TBD |
| B007 | D | master CSR addr 2 read after reset (txfifo_usedw + rxfifo_usedw) | 1 | CSR read at master addr 2 immediately after reset release | tx_fifo.usedw=0 in [TX_FIFO_WIDTHU-1:0]; rx_fifo.usedw=0 in [RX_FIFO_WIDTHU-1+16:16]; other bits 0 | TBD |
| B008 | D | master CSR addr 5..15 read after reset (undecoded address fall-through) | 1 | CSR reads at master addr 5,6,...,15 in sequence | when others branch executes; readdata=0; waitrequest=0 (reads complete) | TBD |
| B009 | D | master CSR write to addr 0 with commit=1 while csr.busy=0 and descriptor configured for tx wire_id=0 tot_bytes=0 init=1 | 1 | CSR write addr 1 to load descriptor, then CSR write addr 0 with writedata[0]=1 | csr.commit latches to 1; pipe_handler enters TX_ACK; pipe.tx.start asserts; csr.busy goes to 1 in same cycle | TBD |
| B010 | D | master CSR write to addr 0 with commit=1 while descriptor is direction=0 (rx) | 1 | CSR write addr 1 with direction=0; CSR write addr 0 commit=1 | pipe_handler enters RX_ACK; pipe.rx.start asserts; csr.busy=1 | TBD |
| B011 | D | master CSR write to addr 1 with tot_bytes=8 wire_id=2 direction=tx init=1 usewire_id=1 while csr.busy=0 | 1 | CSR write addr 1 with descriptor `{wire_id=2, tot_bytes=8, usewire_id=1, init=1, direction=1}` | csr.tot_bytes=8; csr.wire_id=2; csr.direction=1; csr.init=1; csr.usewire_id=1; check_list.result=0; out_of_range=0 | TBD |
| B012 | D | master CSR write to addr 1 with tot_bytes=16 (= MAX_BUFFER_DEPTH boundary) wire_id=5 (= N_DQ_LINES-1) | 1 | CSR write addr 1 with `{tot_bytes=16, wire_id=5}` | check_list.result=0 (16 == MAX_RXTX_BYTES is allowed by `>` test); fields update; out_of_range stays 0 | TBD |
| B013 | D | master CSR write to addr 1 with tot_bytes=17 (over MAX_BUFFER_DEPTH=16) | 1 | CSR write addr 1 with `tot_bytes=17` | check_list.result=1; csr.tot_bytes NOT updated (stays at prior value); csr.error.out_of_range latches to 1 (visible at addr 0 readdata[8]) | TBD |
| B014 | D | master CSR addr 1 descriptor write while csr.busy=0: write tot_bytes=8, wire_id=3, direction=tx, init=1, paracitic_pw=1 (only when generic PARACITIC_POWERING=false), then read addr 1 back | 1 | with PARACITIC_POWERING=false, CSR write addr 1 then CSR read addr 1 | readback matches, out_of_range stays 0, csr.busy stays 0 until commit | TBD |
| B015 | D | master CSR addr 1 paracitic_pw write attempt with PARACITIC_POWERING=true generic | 1 | with PARACITIC_POWERING=true, CSR write addr 1 with writedata[2]=0 then with writedata[2]=1 | csr.paracitic_pw stays at 1 regardless of writedata[2] value (the `if (not PARACITIC_POWERING)` guard blocks the write); other fields update normally | TBD |
| B016 | D | master CSR write to addr 1 with wire_id=6 (over N_DQ_LINES-1=5 boundary) | 1 | CSR write addr 1 with `wire_id=6` | check_list.result=1 via wire_id branch; csr.error.out_of_range latches to 1; csr.wire_id NOT updated | TBD |
| B017 | D | master CSR write to addr 1 with wire_id=7 (= 2^AVST_CHANNEL_WIDTH-1 max) | 1 | CSR write addr 1 with `wire_id=7` | check_list.result=1; out_of_range latches to 1; csr.wire_id NOT updated | TBD |
| B018 | D | master CSR write to addr 1 while csr.busy=1 (descriptor protection) | 1 | drive csr.busy=1 (commit a prior transaction); CSR write addr 1 with bad writedata | csr.tot_bytes/wire_id/direction/init/paracitic_pw/usewire_id all hold their pre-busy values; out_of_range NOT updated even with bad writedata; range-checker write path is fully gated | TBD |
| B019 | D | master CSR write to addr 2 (no fields defined, when 2 is null in write decoder) | 1 | CSR write addr 2 with arbitrary writedata, then CSR read addr 2 | when 2 path executes; no register updates; waitrequest=0; readback at addr 2 returns fifo fillness only | TBD |
| B020 | D | master CSR addr 0 read-after-write of commit bit | 1 | CSR write addr 0 with writedata[0]=1; then CSR read addr 0 | first write sets csr.commit=1 (pipe_handler advances); read at addr 0 returns busy=1, commit not exposed; ins_complete_irq cleared on the read | TBD |
| B021 | D | master CSR write to addr 5..15 (undecoded address fall-through, write side) | 1 | CSR writes at master addr 5,6,...,15 in sequence | when others branch executes; no field updates; waitrequest=0 | TBD |
| B022 | D | master CSR readdata[31] is zero in every default branch (no driver leakage from prior reads) | 1 | scan reads at master addr 0..15 with reset held released and no prior writes | scan addresses 0..15 in reset, no field overlap above mmap.csr_w_1.wire_id'high; readdata[31:24] remains 0 except where mmap defines wire_id | TBD |
| B023 | D | master CSR waitrequest contract for read addr 0..2: deassert for one cycle, latch readdata, then re-assert | 1 | CSR reads at master addr 0,1,2 sampling avs_ctrl_waitrequest each cycle | for each addr in {0,1,2}: avs_ctrl_waitrequest=1 default; on read pulse waitrequest=0 for the cycle the readdata is valid; subsequent cycles return waitrequest=1 | TBD |
| B024 | D | master CSR ins_complete_irq cleared by both read and write, separate visits | 1 | force pipe handler to assert ins_complete_irq=1; CSR read addr 0; one cycle later CSR write addr 1 | drive ins_complete_irq=1 from pipe handler completion; first read at addr 0 clears it; second cycle a write at addr 1 keeps it 0 (not re-armed) | TBD |

---

## 3. Master TX Engine FSM (TX) -- 20 cases

These cases prove `proc_tx_engine`: the `IDLE -> LOAD_TX_DATA -> SEND_INIT? -> SEND_BIT -> EVAL -> ACK` flow with byte_cnt and bit_cnt walks, the empty-tx fast path, all six wire_id values, both paracitic-powering modes, and the AVST channel-driven wire selection mode (`usewire_id=0`). The all-zero data path and the `tot_bytes=16 == MAX_BUFFER_DEPTH` boundary are also covered here.

| ID | Method | Scenario | Iter | Stimulus | Pass Criteria | Function Reference |
|---|---|---|---|---|---|---|
| B025 | D | TX engine happy path tot_bytes=0 wire_id=0 init=0 (descriptor commits an empty tx, exercises empty-TX ACK) | 1 | CSR write descriptor `{tot_bytes=0, wire_id=0, init=0, dir=tx}`; CSR commit with TX FIFO empty | complete IRQ asserts; STATUS[0] busy clears; STATUS[10] tx_fifo_empty asserts; TX/RX FIFO fill returns to zero; no non-selected DQ low pulse | TBD |
| B026 | D | TX engine happy path tot_bytes=1 wire_id=0 init=0 with one byte preloaded into tx_fifo by bypass | 1 | preload one byte through master CSR addr 4; CSR commit descriptor `{tot_bytes=1, wire_id=0, init=0}` | complete IRQ asserts; STATUS[0]=0; STATUS[10]=0; TX FIFO drains to zero; DQ line 0 shows low TX slots and other DQ lines do not | TBD |
| B027 | D | TX engine happy path tot_bytes=1 wire_id=0 init=1 | 1 | preload one byte; commit descriptor `{tot_bytes=1, wire_id=0, init=1}`; DS18B20 model present on line 0 | complete IRQ asserts; STATUS[9]=0 init presence seen; STATUS[10]=0; TX FIFO drains; DQ line 0 shows init/TX low activity only | TBD |
| B028 | D | TX engine tot_bytes=2 wire_id=0 init=1 | 1 | preload two bytes; commit descriptor `{tot_bytes=2, wire_id=0, init=1}` | complete IRQ asserts; STATUS[9:10]=0; TX FIFO drains from 2 to 0; selected DQ line 0 toggles low, non-selected lines stay high | TBD |
| B029 | D | TX engine tot_bytes=3 wire_id=0 init=0 | 1 | preload three bytes; commit descriptor `{tot_bytes=3, wire_id=0, init=0}` | complete IRQ asserts; STATUS[10]=0; TX FIFO drains from 3 to 0; selected line 0 emits TX low slots | TBD |
| B030 | D | TX engine tot_bytes=4 wire_id=0 init=0 | 1 | preload four bytes; commit descriptor `{tot_bytes=4, wire_id=0, init=0}` | complete IRQ asserts; STATUS[10]=0; TX FIFO drains from 4 to 0; no residual RX FIFO data before or after the case | TBD |
| B031 | D | TX engine tot_bytes=5 with descriptor wire_id=0 but usewire_id=0 and FIFO channel=1 | 1 | preload five bytes through bypass with channel field=1; commit descriptor `{tot_bytes=5, wire_id=0, usewire_id=0, init=0}` | complete IRQ asserts; STATUS[10]=0; DQ line 1, not descriptor line 0, shows low TX slots; channel-driven wire selection path exercised | TBD |
| B032 | D | TX engine tot_bytes=6 wire_id=1 init=1 | 1 | preload six bytes; commit descriptor `{tot_bytes=6, wire_id=1, init=1}` | complete IRQ asserts; STATUS[9:10]=0; TX FIFO drains; DQ line 1 shows init/TX low activity and non-selected lines stay high | TBD |
| B033 | D | TX engine tot_bytes=7 wire_id=2 init=0 with PARACITIC_POWERING=true | 1 | instantiate direct master with PARACITIC_POWERING=true; preload seven bytes; commit descriptor `{tot_bytes=7, wire_id=2, init=0}` | complete IRQ asserts; STATUS[10]=0; TX FIFO drains from 7 to 0; DQ line 2 shows TX low slots while other lines remain non-low | TBD |
| B034 | D | TX engine tot_bytes=8 wire_id=2 init=1 with PARACITIC_POWERING=true | 1 | instantiate with PARACITIC_POWERING=true; preload eight bytes; commit descriptor `{tot_bytes=8, wire_id=2, init=1}` | init pulse on line 2; MASTER_EVAL of init_timing leaves dq(2)=PULL_HIGH; bytes follow with PULL_HIGH idle | TBD |
| B035 | D | TX engine tot_bytes=9 wire_id=3 init=0 with PARACITIC_POWERING=false (paracitic_pw=0) | 1 | instantiate with PARACITIC_POWERING=false; preload nine bytes; commit descriptor `{tot_bytes=9, wire_id=3, init=0}` | tx_engine.dq(3) restored to HIGH_Z after each MASTER_IDLE; coe_sense_dq_oe(3)=0 between bits | TBD |
| B036 | D | TX engine tot_bytes=10 wire_id=3 init=1 with paracitic_pw=0 | 1 | with PARACITIC_POWERING=false; preload ten bytes; commit descriptor `{tot_bytes=10, wire_id=3, init=1}` | init MASTER_EVAL leaves dq(3) at HIGH_Z; bytes follow with HIGH_Z idle | TBD |
| B037 | D | TX engine tot_bytes=11 wire_id=4 init=0 | 1 | preload eleven bytes; commit descriptor `{tot_bytes=11, wire_id=4, init=0}` | wire_id 4 selected; only dq(4) toggles, dq(0..3,5) untouched (prove per-line selection in sense_bidir_io_group) | TBD |
| B038 | D | TX engine tot_bytes=12 wire_id=4 init=1 | 1 | preload twelve bytes; commit descriptor `{tot_bytes=12, wire_id=4, init=1}`; DS18B20 model present on line 4 | init pulse on line 4; sample_presence reads coe_sense_dq_in(4) at the right window | TBD |
| B039 | D | TX engine tot_bytes=13 wire_id=5 init=0 | 1 | preload thirteen bytes; commit descriptor `{tot_bytes=13, wire_id=5, init=0}` | wire_id at upper boundary; tx_engine.wire_id=5 select correct; no out-of-range error | TBD |
| B040 | D | TX engine tot_bytes=14 wire_id=5 init=1 | 1 | preload fourteen bytes; commit descriptor `{tot_bytes=14, wire_id=5, init=1}` | init on line 5; full byte count; ACK returns to IDLE; csr.busy clears | TBD |
| B041 | D | TX engine tot_bytes=15 wire_id=0 init=0 | 1 | preload fifteen bytes; commit descriptor `{tot_bytes=15, wire_id=0, init=0}` | next-to-MAX byte count; tx_fifo drains across 15 LOAD_TX_DATA visits | TBD |
| B042 | D | TX engine tot_bytes=16 wire_id=0 init=1 (= MAX_BUFFER_DEPTH boundary) | 1 | preload sixteen bytes; commit descriptor `{tot_bytes=16, wire_id=0, init=1}` | maximum supported byte count; tx_fifo.full not asserted along the way (depth is sufficient); EVAL ACK after 16 bytes; byte_cnt resets to 0 in EVAL | TBD |
| B043 | D | TX engine tot_bytes=8 wire_id=0 init=1 with usewire_id=0 and AVST channel=3 (channel-driven wire selection) | 1 | preload eight bytes via AVST with channel field=3; commit descriptor `{tot_bytes=8, wire_id=0, usewire_id=0, init=1}` | tx_engine.wire_id=3 sourced from tx_fifo.q msb bits; csr.wire_id (0) ignored; init pulse on line 3 | TBD |
| B044 | D | TX engine tot_bytes=2 wire_id=0 init=1 paracitic_pw=1 with both bytes preloaded as 0x00 (all-zero data path) | 1 | preload `{0x00, 0x00}` into tx_fifo; commit descriptor `{tot_bytes=2, wire_id=0, init=1, paracitic_pw=1}` | every SEND_BIT slot drives PULL_LOW; bit_cnt increments correctly even when tx_data bit is 0; bit_timing MASTER_SEND_BIT branches down the `else` path per bit | TBD |

---

## 4. Master RX Engine FSM (RX) -- 16 cases

These cases prove `proc_rx_engine`: the `IDLE -> EVAL_CMD -> RECV_BIT -> STORE_RX_DATA -> ACK` flow, byte_cnt and bit_cnt walks, the empty-cmd fast path, the full DS18B20 9-byte scratchpad capture on every line including CRC, the `tot_bytes=16 == RX_BUFFER_DEPTH` boundary, and the HIGH_Z idle behavior with `paracitic_pw=0`.

| ID | Method | Scenario | Iter | Stimulus | Pass Criteria | Function Reference |
|---|---|---|---|---|---|---|
| B045 | D | RX engine happy path tot_bytes=0 wire_id=0 (empty-cmd fast path) | 1 | CSR commit descriptor `{tot_bytes=0, wire_id=0, dir=rx}` | rx_engine.flow IDLE->EVAL_CMD->ACK (no RECV_BIT, no STORE_RX_DATA); pipe.rx.done asserts; csr.busy clears | TBD |
| B046 | D | RX engine happy path tot_bytes=1 wire_id=0 with DS18B20 returning 0x55 | 1 | DS18B20 model on line 0 driving 0x55; CSR commit descriptor `{tot_bytes=1, wire_id=0, dir=rx}` | rx_engine.flow walks IDLE->EVAL_CMD->RECV_BIT->STORE_RX_DATA->ACK; rx_fifo q == channel(0) ‖ 0x55 | TBD |
| B047 | D | RX engine tot_bytes=2 wire_id=0 with DS18B20 returning {0xAA,0x55} | 1 | DS18B20 model on line 0 driving {0xAA,0x55}; commit descriptor `{tot_bytes=2, wire_id=0, dir=rx}` | bytes arrive in transmission order (LSB first per Maxim 1-Wire); rx_fifo holds {0x00‖0xAA, 0x00‖0x55} in order | TBD |
| B048 | D | RX engine tot_bytes=3 wire_id=0 | 1 | DS18B20 model returning 3 bytes; commit descriptor `{tot_bytes=3, wire_id=0, dir=rx}` | byte_cnt walks 1..3; STORE_RX_DATA visited 3 times; final ACK on byte 3 | TBD |
| B049 | D | RX engine tot_bytes=4 wire_id=0 | 1 | DS18B20 model returning 4 bytes; commit descriptor `{tot_bytes=4, wire_id=0, dir=rx}` | RECV_BIT bit_timing MASTER_PULL_LOW->RELEASE->SAMPLE_BIT->IDLE->EVAL traversed for every bit | TBD |
| B050 | D | RX engine tot_bytes=5 wire_id=0 | 1 | DS18B20 model returning 5 bytes; commit descriptor `{tot_bytes=5, wire_id=0, dir=rx}` | bit_cnt goes to 8 then resets to 0 in MASTER_EVAL on byte boundary; STORE_RX_DATA correctly clears bit_cnt | TBD |
| B051 | D | RX engine tot_bytes=6 wire_id=1 | 1 | DS18B20 model on line 1 returning 6 bytes; commit descriptor `{tot_bytes=6, wire_id=1, dir=rx}` | csr.wire_id=1 used in RECV_BIT to drive rx_engine.dq(1) PULL_LOW pulse and to read coe_sense_dq_in(1); rx_fifo channel field = 0x1 | TBD |
| B052 | D | RX engine tot_bytes=7 wire_id=2 | 1 | DS18B20 model on line 2 returning 7 bytes; commit descriptor `{tot_bytes=7, wire_id=2, dir=rx}` | wire_id=2 selection; only dq(2) toggles in RECV_BIT | TBD |
| B053 | D | RX engine tot_bytes=8 wire_id=2 with DS18B20 returning the 9-byte scratchpad first 8 bytes | 1 | DS18B20 model on line 2 with scratchpad payload; commit descriptor `{tot_bytes=8, wire_id=2, dir=rx}` | 8 bytes captured into rx_fifo with channel=2; first byte LSB matches scratchpad[0][7:0] | TBD |
| B054 | D | RX engine tot_bytes=9 wire_id=3 (full DS18B20 scratchpad including CRC) | 1 | DS18B20 model on line 3 with full scratchpad; commit descriptor `{tot_bytes=9, wire_id=3, dir=rx}` | 9 bytes into rx_fifo; final byte = computed CRC8 of bytes 0..7; channel field = 3 throughout | TBD |
| B055 | D | RX engine tot_bytes=9 wire_id=4 | 1 | DS18B20 model on line 4 with full scratchpad; commit descriptor `{tot_bytes=9, wire_id=4, dir=rx}` | full scratchpad on line 4; bytes arrive in correct order; rx_fifo.usedw=9 at end of capture | TBD |
| B056 | D | RX engine tot_bytes=9 wire_id=5 | 1 | DS18B20 model on line 5 with full scratchpad; commit descriptor `{tot_bytes=9, wire_id=5, dir=rx}` | full scratchpad on line 5; channel=5 throughout; rx_fifo.empty=0 after capture | TBD |
| B057 | D | RX engine tot_bytes=12 wire_id=0 (over scratchpad length but within fifo depth) | 1 | DS18B20 model returning scratchpad then 0xFF idle; commit descriptor `{tot_bytes=12, wire_id=0, dir=rx}` | engine reads beyond scratchpad; DS18B20 model returns 0xFF after end (idle high); rx_fifo holds 12 entries | TBD |
| B058 | D | RX engine tot_bytes=15 wire_id=0 | 1 | DS18B20 model returning 15 bytes; commit descriptor `{tot_bytes=15, wire_id=0, dir=rx}` | 15 bytes captured; rx_fifo.usedw=15; rx_fifo.full not asserted (depth=16) | TBD |
| B059 | D | RX engine tot_bytes=16 wire_id=0 (= RX_BUFFER_DEPTH max) | 1 | DS18B20 model returning 16 bytes; commit descriptor `{tot_bytes=16, wire_id=0, dir=rx}` | 16 bytes captured; rx_fifo.usedw=16; rx_fifo.full=1 after last STORE_RX_DATA, but ACK already taken so no error_fifo_full | TBD |
| B060 | D | RX engine tot_bytes=8 wire_id=0 with paracitic_pw=0 (HIGH_Z idle between bits) | 1 | with PARACITIC_POWERING=false; DS18B20 model on line 0 returning 8 bytes; commit descriptor `{tot_bytes=8, wire_id=0, dir=rx, paracitic_pw=0}` | rx_engine.dq(0) returns HIGH_Z in MASTER_IDLE/RELEASE; coe_sense_dq_oe(0)=0; sample at MASTER_SAMPLE_BIT still latches coe_sense_dq_in(0) | TBD |

---

## 5. Init Pulse, Presence, Paracitic Powering (INIT) -- 10 cases

These cases prove the `init_timing_t` micro-state machine: the 500 us PULL_LOW window, 45 us release-to-sample window, presence detection on every line, the missing-presence latching of `error_init_fail`, the init-only command (`tot_bytes=0` after init), and the paracitic-vs-HIGH_Z idle behavior bridging the SEND_INIT and SEND_BIT phases.

| ID | Method | Scenario | Iter | Stimulus | Pass Criteria | Function Reference |
|---|---|---|---|---|---|---|
| B061 | D | TX engine SEND_INIT with DS18B20 model present and paracitic_pw=1 (default for FEB) | 1 | DS18B20 model present; commit descriptor `{init=1, paracitic_pw=1}` | error_init_fail=1 set in MASTER_RELEASE then cleared in MASTER_SENSE_PRESENCE when coe_sense_dq_in(wire_id)=0 within sample window; csr.error.init_fail latches 0 in interface_hub | TBD |
| B062 | D | TX engine SEND_INIT with DS18B20 model present and paracitic_pw=0 | 1 | DS18B20 model present; commit descriptor `{init=1, paracitic_pw=0}` | same flow as B061 but dq idle returns to HIGH_Z in MASTER_EVAL; presence pulse still detected; init_fail clears | TBD |
| B063 | D | TX engine SEND_INIT with DS18B20 model removed (no presence pulse) | 1 | no DS18B20 model on selected line; commit descriptor `{init=1}` | error_init_fail stays 1 through MASTER_SENSE_PRESENCE; MASTER_EVAL takes the `error or tot_bytes=0` branch -> ACK; csr.error.init_fail latches 1 | TBD |
| B064 | D | TX engine SEND_INIT timing alignment: MASTER_INIT_RESET_US duration matches MASTER_INIT_RESET_US_U slow-tick boundary | 1 | commit descriptor `{init=1}`; sample slow_timer_cnt_unsigned each cycle | slow_timer_cnt_unsigned reaches MASTER_INIT_RESET_US_U exactly when MASTER_RELEASE asserts; PULL_LOW window measured equals 500 us in real time | TBD |
| B065 | D | TX engine SEND_INIT timing: MASTER_WAIT_PRESENCE_US elapses before sample window | 1 | commit descriptor `{init=1}`; sample slow_timer at MASTER_SENSE_PRESENCE entry | slow_timer reaches MASTER_INIT_RESET_US_U + MASTER_WAIT_PRESENCE_US_U = 545 before MASTER_SENSE_PRESENCE; sample window opens 45 us after MASTER_RELEASE | TBD |
| B066 | D | TX engine SEND_INIT timing: MASTER_SAMPLE_PRESENCE_TIMEOUT_US fully consumed before MASTER_EVAL | 1 | commit descriptor `{init=1}`; sample slow_timer at MASTER_EVAL entry | slow_timer reaches MASTER_INIT_RESET_US_U + MASTER_WAIT_PRESENCE_US_U + MASTER_SAMPLE_PRESENCE_TIMEOUT_US_U = 1545 before MASTER_EVAL; total init phase = ~1.55 ms | TBD |
| B067 | D | sense_dq_in(N) sampled exactly at the nominal window for line N=0 (presence) | 1 | DS18B20 model on line 0 holds presence; commit descriptor `{init=1, wire_id=0}` | DS18B20 model on line 0 holds dq low during sample window; tx_engine.error_init_fail sees `0` and clears | TBD |
| B068 | D | sense_dq_in(N) sample for N=1..5 (per-line presence sampling, sequential) | 1 | for each line N in 1..5, drive presence on line N only; commit descriptor `{init=1, wire_id=N}` | for each line, drive presence and verify the SENSE_PRESENCE branch reads coe_sense_dq_in(N) (and not other indices); only the wire_id index resolves | TBD |
| B069 | D | TX engine SEND_INIT MASTER_EVAL when csr.tot_bytes=0 (init-only command) | 1 | commit descriptor `{tot_bytes=0, init=1}` | MASTER_EVAL branch takes the `error_init_fail=1 or tot_bytes=0` path -> ACK with no SEND_BIT; pipe.tx.done asserts; init pulse used as bus-reset only | TBD |
| B070 | D | TX engine SEND_INIT followed by SEND_BIT exits MASTER_EVAL with paracitic_pw=1 idle | 1 | commit descriptor `{tot_bytes=1, init=1, paracitic_pw=1}` | dq(wire_id)=PULL_HIGH in MASTER_EVAL; bit_timing immediately pre-loaded to MASTER_PULL_LOW; first SEND_BIT iteration starts with timer reset | TBD |

---

## 6. TX/RX FIFO Bypass Two-Phase (FIFO) -- 10 cases

These cases prove the master CSR addr 3/4 two-phase bypass paths into the TX/RX scfifos through `interface_hub.tx_wrreq` / `rx_rdreq`, the `proc_interface_hub_comb` arbiter routing for the bypass-vs-AVST contention, and the channel field round-tripping. Both write-side (addr 4) and read-side (addr 3) are covered.

| ID | Method | Scenario | Iter | Stimulus | Pass Criteria | Function Reference |
|---|---|---|---|---|---|---|
| B071 | D | master CSR write to addr 4 (TX FIFO bypass) one byte channel=0 data=0xCC: first write asserts waitrequest=1 | 1 | CSR write addr 4 with `{channel=0, data=0xCC}`; sample avs_ctrl_waitrequest in cycle 1 | interface_hub.tx_wrreq=1 in cycle 1; interface_hub.write_one_done=1; tx_fifo.wrreq follows interface_hub.tx_wrreq via comb arbiter; avs_ctrl_waitrequest=1 holds master | TBD |
| B072 | D | master CSR addr 4 second cycle of two-phase write completes with waitrequest=0 | 1 | continue B071 sequence into cycle 2 | interface_hub.tx_wrreq=0; avs_ctrl_waitrequest=0; tx_fifo.usedw=1; data field == channel(0)‖0xCC | TBD |
| B073 | D | master CSR addr 4 bypass write three bytes back-to-back (each two-phase) | 1 | CSR writes addr 4 three times in sequence with distinct channel/data pairs | tx_fifo.usedw advances 0->1->2->3; channel field on each entry matches writedata[high:8] bits | TBD |
| B074 | D | master CSR addr 4 bypass write with channel=5 data=0x55 | 1 | CSR write addr 4 with `{channel=5, data=0x55}` | tx_fifo entry q[high:AVST_DATA_WIDTH] reads back 5 (binary 101); confirms AVST_CHANNEL_WIDTH=3 routing | TBD |
| B075 | D | master CSR addr 4 bypass write while asi_tx_valid=0 (no contention with avst tx) | 1 | drive asi_tx_valid=0; CSR write addr 4 | proc_interface_hub_comb arbiter routes write path: tx_fifo.wrreq=interface_hub.tx_wrreq; the `else` (avst tx) branch is fully gated when avs_ctrl_write=1 | TBD |
| B076 | D | master CSR read from addr 3 (RX FIFO bypass) when rx_fifo is empty: first read returns 0 | 1 | rx_fifo empty; CSR read addr 3 | interface_hub.read_one_done=1; rx_fifo.empty=1 path taken; readdata=0; rx_fifo.rdreq stays 0 | TBD |
| B077 | D | master CSR read from addr 3 when rx_fifo holds one entry channel=2 data=0xA5 | 1 | preload one entry `{channel=2, data=0xA5}` into rx_fifo via AVST rx; CSR read addr 3 | first read returns rx_fifo.q (= 0x2 ‖ 0xA5); interface_hub.rx_rdreq=1; rx_fifo.rdreq follows it via comb arbiter | TBD |
| B078 | D | master CSR addr 3 second cycle of two-phase read completes with waitrequest=1 | 1 | continue B077 sequence into cycle 2 | interface_hub.rx_rdreq=0 in cycle 2; avs_ctrl_waitrequest=1; readdata=0; rx_fifo.usedw decremented by 1 | TBD |
| B079 | D | master CSR addr 3 bypass read drains 4 entries back-to-back | 1 | preload four entries; CSR reads addr 3 four times in sequence | sequence of 4 two-phase reads pops 4 entries; rx_fifo.usedw walks 4->3->2->1->0; channel field of each pop matches its push | TBD |
| B080 | D | master CSR addr 3 read while aso_rx_ready=0 (no contention with avst rx) | 1 | drive aso_rx_ready=0; preload one entry; CSR read addr 3 | proc_interface_hub_comb arbiter routes read path: rx_fifo.rdreq=interface_hub.rx_rdreq; aso_rx_valid=0 (gated by avs_ctrl_read=1 branch); avst rx port silent | TBD |

---

## 7. Six-Line Ticket-Lock Arbitration (ARB) -- 10 cases

These cases prove `proc_ticket_lock` and `proc_ticket_lock_comb`: rising-edge `lock_req` detection, the IDLE -> DECIDING -> LOCKED state walk, the rotating priority across all six lines, the skip-on-no-req behavior, the unlock_req -> IDLE return, and the IRQ correlation with the granted line completing one TX leg.

| ID | Method | Scenario | Iter | Stimulus | Pass Criteria | Function Reference |
|---|---|---|---|---|---|---|
| B081 | D | controller ticket lock: line 0 alone asserts processor_req | 1 | drive processor_req(0)=1, all other processor_req=0 | ticket_lock.lock_req(0) detected on rising edge of processor_req(0); ticket_lock_state goes IDLE->DECIDING->LOCKED; ticket_lock.gnt(0)=1; processor_grant(0)=1 | TBD |
| B082 | D | controller ticket lock: line 5 alone asserts processor_req | 1 | drive processor_req(5)=1, all other processor_req=0 | only ticket_lock.gnt(5)=1; sense_dq_oe routed via port_mux to line 5; vector bridge passes temp_dab1_dq_* through (proves bridge index 5 -> dab1) | TBD |
| B083 | D | controller ticket lock: lines 0 and 1 simultaneously assert processor_req | 1 | drive processor_req(0)=1 and processor_req(1)=1 in same cycle | priority init (1<<0) wins line 0 first; ticket_lock.gnt(0)=1 in cycle K; line 1 waits in ticket_lock.req=1; after unlock, DECIDING grants line 1 | TBD |
| B084 | D | controller ticket lock: lines 2 and 3 simultaneously assert | 1 | drive processor_req(2)=1 and processor_req(3)=1 in same cycle | DECIDING result_comb selects the lower-index in-priority candidate (line 2 if priority bit covers it); next cycle priority rotates; line 3 grant follows | TBD |
| B085 | D | controller ticket lock: all 6 lines simultaneously assert processor_req | 1 | drive processor_req(0..5)=1 all in same cycle | round-robin walks 0->1->2->3->4->5 across 6 successive LOCKED/IDLE cycles; ticket_lock.priority shifts left by one each grant; every line wins exactly once | TBD |
| B086 | D | controller ticket lock: lines 0,2,4 assert simultaneously, then 1,3,5 next | 1 | drive processor_req on lines 0,2,4 first, then on lines 1,3,5 after the first triplet drains | first triplet wins in priority order (0,2,4); priority lands on bit 5 after the third grant; second triplet (1,3,5) granted in priority order from there | TBD |
| B087 | D | controller ticket lock: rotating priority skips a non-requesting line | 1 | force priority bit on bit 1; drive processor_req on lines 0,2,3 only | with lines 0,2,3 requesting and priority on bit 1, grant_comb skips line 1 (no req) and grants line 2 first; line 1 never granted in this window | TBD |
| B088 | D | controller ticket lock LOCKED -> IDLE on unlock_req | 1 | grant a line; drive falling edge of processor_req from RECV_BYTES exit | unlock_req asserted (falling edge of processor_req from RECV_BYTES exit); ticket_lock.gnt cleared; ticket_lock_state -> IDLE within 1 cycle | TBD |
| B089 | D | IRQ assertion correlates with the granted line completing one TX leg | 1 | grant line N alone; close pipe.tx handshake | for line N alone in flight, ins_complete_irq=1 at the cycle pipe.tx handshake closes; correlates with that line's tx_flow READBACK->IDLE transition | TBD |
| B090 | D | IRQ clearing on master CSR addr 0 read after transaction completes | 1 | drive ins_complete_irq=1 from pipe handler; CSR read addr 0 then CSR read addr 0 again | ins_complete_irq=1 from pipe handler; first avs_ctrl_read pulse on the master CSR drives ins_complete_irq <= 0; subsequent reads keep it 0 | TBD |

---

## 8. Controller Core/TX/RX State Spines (STATE) -- 10 cases

These cases prove `proc_processor`'s per-line `core_state_t`, `core_tx_state_t`, and `core_rx_state_t` walks: IDLE -> SEND_CONVT -> WAITING -> JOINING -> SEND_READT -> RECV_BYTES -> IDLE, plus the per-line independence guarantee. The line-0 walk is exhaustive; B100 mirrors the full walk on line 5.

| ID | Method | Scenario | Iter | Stimulus | Pass Criteria | Function Reference |
|---|---|---|---|---|---|---|
| B091 | D | controller core_state walk for line 0: IDLE -> SEND_CONVT after CSR processor_go=1 | 1 | CSR write controller addr 4 STATUS sel_line=0 processor_go=1 | csr.processor_go(0)=1 sets processor_req(0)=1; on grant core_state(0) IDLE->SEND_CONVT and processor_flow(0)<=SEND_CONVT in same cycle | TBD |
| B092 | D | controller core_state SEND_CONVT -> WAITING after main2tx handshake closes | 1 | continue B091; close main2tx four-phase | pipe.main2tx(0).start=1 -> done=1 -> start=0 sequence; on slave-ack core_state(0)=WAITING and processor_flow(0)<=SEND_READT | TBD |
| B093 | D | controller core_state WAITING -> JOINING after slow_timer_cnt_unsigned > 850_000 | 1 | continue B092; let slow_timer reach 850_001 | slow_timer_on(0)=1 in WAITING; processor_req(0)=0 to release semaphore; once cnt > 850_000 core_state(0)<=JOINING and slow_timer_on(0)=0 | TBD |
| B094 | D | controller core_state JOINING -> SEND_READT after re-acquiring grant | 1 | continue B093 | processor_req(0)<=1 in JOINING; ticket_lock grants again; core_state(0)<=SEND_READT | TBD |
| B095 | D | controller core_state SEND_READT -> RECV_BYTES after second main2tx handshake | 1 | continue B094; close second main2tx four-phase | pipe.main2tx(0) start/done four-phase closes; processor_flow(0)<=RECV_BYTES; core_state(0)<=RECV_BYTES in same cycle | TBD |
| B096 | D | controller core_state RECV_BYTES -> IDLE after main2rx handshake closes | 1 | continue B095; close main2rx four-phase | pipe.main2rx(0) start/done four-phase closes; processor_req(0)<=0 (release semaphore); core_state(0)<=IDLE; processor_flow(0)<=RESET | TBD |
| B097 | D | controller core_tx_state walk per line 0: IDLE -> TX_ROM_SKIP -> TX_FUNC_CMD -> IDLE | 1 | drive pipe.main2tx start on line 0; observe core_tx_state | on pipe.main2tx start core_tx_state(0)=TX_ROM_SKIP; sub-routine via pipe.tx2flow returns; transitions to TX_FUNC_CMD then IDLE; pipe.main2tx.done asserts on exit | TBD |
| B098 | D | controller core_rx_state walk per line 0: IDLE -> RX_BYTES -> IDLE | 1 | drive pipe.main2rx start on line 0; observe core_rx_state | on pipe.main2rx start core_rx_state(0)=RX_BYTES; sub-routine via pipe.rx2flow closes; pipe.main2rx.done asserts on exit | TBD |
| B099 | D | controller per-line independence: lines 1..5 stay in IDLE while line 0 walks SEND_CONVT->WAITING with processor_go(0)=1 only | 1 | CSR write STATUS sel_line=0 processor_go=1; observe core_state(1..5) | core_state(1..5)=IDLE throughout; processor_flow(1..5)=RESET; tx_flow(1..5)=IDLE; per-line gen_processor isolation confirmed | TBD |
| B100 | D | controller core_state walk on line 5 (mirror of B091..B096 on the highest-indexed line) | 1 | CSR write STATUS sel_line=5 processor_go=1; close all four-phase handshakes | full state spine traversal on line 5; processor_grant(5) routes through ticket_lock; mm_address.tx(5) drives wire_id=5 in writedata[23:16] | TBD |

---

## 9. Pipe IPC Handshakes (PIPE) -- 8 cases

These cases prove the four-phase request/done IPC contracts between the controller's processor, tx, and rx state machines: `pipe.main2tx`, `pipe.main2rx`, `pipe.tx2flow`, and `pipe.rx2flow`. Each handshake is exercised on both line 0 (canonical) and one mid-/upper-index line, plus a concurrency case across two lines.

| ID | Method | Scenario | Iter | Stimulus | Pass Criteria | Function Reference |
|---|---|---|---|---|---|---|
| B101 | D | pipe.main2tx four-phase contract (line 0): start=0/done=0 -> start=1/done=0 -> start=1/done=1 -> start=0/done=1 -> idle | 1 | drive controller line 0 through SEND_CONVT and observe pipe.main2tx(0) per cycle | each cycle in sequence is observed in core_state SEND_CONVT branch and core_tx_state IDLE/TX_ROM_SKIP branches; no skipped phase | TBD |
| B102 | D | pipe.main2rx four-phase contract (line 0) | 1 | drive controller line 0 through RECV_BYTES and observe pipe.main2rx(0) per cycle | start=0/done=0 -> start=1/done=0 -> start=1/done=1 -> start=0/done=1 sequence in RECV_BYTES branch; pipe.main2rx.done sourced by core_rx_state RX_BYTES exit; cleared in IDLE | TBD |
| B103 | D | pipe.tx2flow four-phase contract (line 0) for TX_ROM_SKIP leg | 1 | drive controller line 0 into core_tx_state TX_ROM_SKIP and observe pipe.tx2flow(0) per cycle | start asserted by core_tx_state TX_ROM_SKIP; done sourced by tx_flow READBACK->IDLE transition; full four-phase observed | TBD |
| B104 | D | pipe.tx2flow four-phase contract (line 0) for TX_FUNC_CMD leg | 1 | drive controller line 0 into core_tx_state TX_FUNC_CMD and observe pipe.tx2flow(0) per cycle | second start in core_tx_state TX_FUNC_CMD; done in tx_flow READBACK->IDLE; pipe.main2tx.done set on exit | TBD |
| B105 | D | pipe.rx2flow four-phase contract (line 0) | 1 | drive controller line 0 into core_rx_state RX_BYTES and observe pipe.rx2flow(0) per cycle | start asserted by core_rx_state RX_BYTES; done sourced by rx_flow ST_GET_DATA tail (when rx_flow_byte_cnt=8); full four-phase observed | TBD |
| B106 | D | pipe.main2tx contract on line 3 (mid-index line) | 1 | drive controller line 3 through SEND_CONVT and observe pipe.main2tx(3) | independent state of pipe.main2tx(3) does not interact with pipe.main2tx(0..2,4,5); per-line array element isolation confirmed | TBD |
| B107 | D | pipe.main2rx contract on line 4 | 1 | drive controller line 4 through RECV_BYTES and observe pipe.main2rx(4) | per-line independence; pipe.main2rx(4) walks four-phase without disturbing pipe.main2rx(0..3,5) | TBD |
| B108 | D | pipe.tx2flow plus pipe.rx2flow concurrent on different lines | 1 | drive line 0 into TX leg and line 1 into RX leg simultaneously after ticket_lock arbitrates | line 0 in tx2flow active and line 1 in rx2flow active in overlapping cycles (after ticket_lock arbitrates); each pipe handshake closes independently | TBD |

---

## 10. 850 ms WAITING Hold (WAIT) -- 5 cases

These cases prove the controller's WAITING hold: the `slow_timer_cnt_unsigned > 850_000` comparator (with REF_CLOCK_RATE shrink), the semaphore release while waiting, the JOINING-entry counter reset, the `>` (not `>=`) comparator boundary, and the absence of an ERROR transition during the hold.

| ID | Method | Scenario | Iter | Stimulus | Pass Criteria | Function Reference |
|---|---|---|---|---|---|---|
| B109 | D | controller WAITING hold uses slow_timer_cnt_unsigned > 850_000 (with REF_CLOCK_RATE=1_000_000 fast-clock) | 1 | enter WAITING on a selected line; let slow_tick at 1 MHz advance slow_timer_cnt | slow_tick(N) at 1 MHz drives slow_timer_cnt(N); cnt reaches 850_001 then JOINING is taken; total simulated WAITING window = 850 ms (compressed by REF_CLOCK_RATE shrink) | TBD |
| B110 | D | controller WAITING releases processor_req while waiting | 1 | enter WAITING; observe processor_req(N) and ticket_lock per cycle | processor_req(N)=0 throughout WAITING; ticket_lock sees no req on this line during the hold; other lines may take grants | TBD |
| B111 | D | controller WAITING resets slow_timer_on=0 on JOINING entry | 1 | enter WAITING then exit to JOINING; sample slow_timer_on(N) | slow_timer_on(N) goes high in WAITING entry; falls to 0 in same cycle core_state(N)<=JOINING; counter resets via the `slow_timer_on=0` branch in proc_slow_timer | TBD |
| B112 | D | controller WAITING boundary: cnt = 850_000 stays in WAITING, cnt = 850_001 transitions to JOINING | 1 | force slow_timer_cnt to 850_000 then to 850_001 across two cycles | confirms `>` (not `>=`) comparator semantics in core_state WAITING branch | TBD |
| B113 | D | controller WAITING does not arm core_state ERROR within the hold window | 1 | enter WAITING with prior tx_flow READBACK clean; sample core_state and sensor_error throughout the hold | core_state(N) stays WAITING throughout; sensor_error(N).init_fail=0 (latched from prior tx_flow READBACK); ERROR branch not entered | TBD |

---

## 11. Format Convertor Happy Path (FMT) -- 7 cases

These cases prove `proc_format_convertor_comb`: the raw S(5).F(11) -> IEEE-754 float32 mapping across the DS18B20 spec range (+0 C, +1 C, +4 C, +25 C, +85 C, +125 C), the `temp_frac=0` zero-output guard (which B001 also indirectly exercises), and the j-4+127 exponent calculation across leading-1 positions.

| ID | Method | Scenario | Iter | Stimulus | Pass Criteria | Function Reference |
|---|---|---|---|---|---|---|
| B114 | D | format_convertor positive 0.0625 C (raw 0x0001, S=00000, F=000_0000_0001) | 1 | drive raw scratchpad temp word 0x0001 into format_convertor input | sense_temp_comb(i) = 0x3D800000 (sign=0, expo=01111011=123, frac=23'h000000); SENSORn_TEMP_F32 reads back 0x3D800000 | TBD |
| B115 | D | format_convertor positive 1 C (raw 0x0010, F=000_0001_0000) | 1 | drive raw temp word 0x0010 into format_convertor input | sense_temp_comb = 0x3F800000 (sign=0, expo=127, frac=0); confirms exponent calc j-4+127 with j=4 | TBD |
| B116 | D | format_convertor positive 4 C (raw 0x0040, F=000_0100_0000) | 1 | drive raw temp word 0x0040; ensure sample_valid=1 captured before the SENSOR read | sense_temp_comb = 0x40800000 (sign=0, expo=129, frac=0); confirms j=6 leading-1 detect; sample_valid=1 captured before read | TBD |
| B117 | D | format_convertor positive 25 C (raw 0x0190, F=001_1001_0000) | 1 | drive raw temp word 0x0190 (line 2 = 25 C in smoke harness) | sense_temp_comb = 0x41C80000 (sign=0, expo=131, frac=0x480000); matches expected_temp_word(2) for the smoke harness when line 2 has 25 C | TBD |
| B118 | D | format_convertor positive 85 C (raw 0x0550, F=101_0101_0000) | 1 | drive raw temp word 0x0550 (DS18B20 power-on default) | sense_temp_comb = 0x42AA0000 (sign=0, expo=133, frac=0x2A0000); within DS18B20 specified +85 C "power-on default"; SENSORn_TEMP_F32 reads back | TBD |
| B119 | D | format_convertor positive 125 C (raw 0x07D0, F=111_1101_0000) | 1 | drive raw temp word 0x07D0 (DS18B20 spec upper rail) | sense_temp_comb = 0x42FA0000 (sign=0, expo=133, frac=0x7A0000); maxes the +125 C upper rail of DS18B20 | TBD |
| B120 | D | format_convertor zero-fraction zero-output guard (raw 0x0000) | 1 | drive raw temp word 0x0000 into format_convertor input | unsigned(temp_frac)=0 branch taken; f32_sign=0; f32_expo=00000000; f32_frac=0; sense_temp_comb=0x00000000 (NOT 0x3F800000 stale-marker); covers B001 reading | TBD |

---

## 12. Controller CSR Identity (CSRC) -- 5 cases

These cases prove the controller's frozen CSR header: CAPABILITY at addr 3 (n_dq_lines / n_sensors RAZ), STATUS at addr 4 (sel_line round-trip, processor_go per-line stickiness, init_err per-selected-line readback), and per-line SENSORn_TEMP_F32 at addr 5..10. B121-B125 are the live UVM cases that pass today.

| ID | Method | Scenario | Iter | Stimulus | Pass Criteria | Function Reference |
|---|---|---|---|---|---|---|
| B121 | D | controller CSR addr 3 read returns CAPABILITY n_dq_lines | 1 | CSR read controller addr 3 immediately after reset | avs_csr_readdata[15:0]=N_DQ_LINES=6; avs_csr_readdata[31:16]=0 (n_sensors RAZ pre-Read-ROM); avs_csr_waitrequest deasserts in 1 cycle | TBD |
| B122 | D | controller CSR addr 4 sel_line round-trip write/read | 1 | CSR write controller addr 4 with `{sel_line=3, processor_go=0}`; CSR read addr 4 | write {sel_line=3, processor_go=0} -> read returns sel_line=3 in [15:0]; processor_go(3) bit at [16] reflects current value; per-line iteration in proc_csr_hub correct | TBD |
| B123 | D | controller CSR addr 4 processor_go set on sel_line=4 sticks | 1 | CSR write controller addr 4 with `{sel_line=4, processor_go=1}`; CSR read addr 4 with sel_line=4 then with sel_line=0..3,5 | write {sel_line=4, processor_go=1}; csr.processor_go(4) latches 1; read at addr 4 with sel_line=4 returns [16]=1; csr.processor_go(0..3,5) unchanged | TBD |
| B124 | D | controller CSR per-line read of SENSORn_TEMP_F32 at addr 5..10 after sample_valid set | 1 | for each line N in 0..5 run a full transaction loop; CSR read addr 5+N before and after sample_valid | for each line N in 0..5, after a full transaction loop, read at addr 5+N returns csr.sense_temp(N); read at addr 5+N before sample_valid returns 0 | TBD |
| B125 | D | controller CSR addr 4 init_err read for selected line | 1 | drive sensor_error(N).init_fail=1 via missing-presence stub; CSR read addr 4 with sel_line=N then with sel_line!=N | drive sensor_error(N).init_fail=1 via missing-presence stub on selected line; STATUS[25]=1 visible on sel_line=N read; STATUS[25] reads 0 on other sel_line values | TBD |

---

## 13. Read-ROM Serial-Number Probe (PROBE) -- 5 cases

These cases prove the planned single-device Read-ROM serial-number probe (RTL fix gated, see DV_PLAN Section 5.2): the per-line probe flow (init + 0x33 + 8 byte read + CRC validation), the cycle-through-all-six-lines scan, idempotent re-probe stability, ROM shadow stability across temperature loop activity, and the CAPABILITY n_sensors update after a multi-line probe scan.

| ID | Method | Scenario | Iter | Stimulus | Pass Criteria | Function Reference |
|---|---|---|---|---|---|---|
| B126 | D | NEW Read-ROM probe: single-device flow on line 0 (STATUS sel_line=0, probe_request rising edge) | 1 | CSR write STATUS sel_line=0 probe_request=1; wait for probe_busy to clear; CSR reads SENSOR_ROM_LO/HI and STATUS bits | probe_busy ([27]) rises within 1 cycle of probe_request edge; controller issues init + 0x33 + 8 byte read; probe_valid ([28]) sets; selected-line SENSOR_ROM_LO/HI hold 0x2800000000000100 (model ROM_CODE for line 0); probe_crc_err ([29]) = 0 | TBD |
| B127 | D | NEW Read-ROM probe: cycle through all six lines one-by-one (sel_line walks 0..5 with probe_request per line) | 1 | for each line N in 0..5, CSR write STATUS sel_line=N probe_request=1, wait for probe_busy clear, CSR read SENSOR_ROM_LO/HI | for each line N: selected-line SENSOR_ROM_LO/HI = 0x2800000000000100 + N after selecting N; probe_crc_err=0 for every line; CAPABILITY n_sensors = 6 after the scan | TBD |
| B128 | D | NEW Read-ROM probe: idempotent re-probe on same line keeps shadow stable | 1 | probe sel_line=2 twice back-to-back; CSR read SENSOR_ROM_LO/HI and STATUS between the two probes | probe sel_line=2 twice back-to-back; selected-line ROM shadow unchanged across both probes; probe_valid stays 1; probe_crc_err stays 0; n_sensors unchanged | TBD |
| B129 | D | NEW Read-ROM probe: ROM shadow on line 0 stable across temperature loop activity on lines 1..5 | 1 | probe sel_line=0; then enable processor_go on lines 1..5 for one full 850 ms cycle; CSR read SENSOR_ROM_LO/HI with sel_line=0 again | probe sel_line=0; then enable processor_go on lines 1..5 for one full 850 ms cycle; selected-line SENSOR_ROM_LO/HI unchanged after selecting line 0 again; probe_valid(0) stays 1 | TBD |
| B130 | D | NEW Read-ROM probe: CAPABILITY n_sensors updates after multi-line probe scan | 1 | probe lines 0,2,4 only and read CAPABILITY[31:16]; then probe lines 1,3,5 and read CAPABILITY[31:16] again | probe lines 0,2,4 only; CAPABILITY[31:16] reads 3; probe lines 1,3,5 next; CAPABILITY[31:16] reads 6; per-line probe_valid bits set independently | TBD |
