# DV_CROSS.md — onewire_master_controller

**Companion to:** [DV_PLAN.md](DV_PLAN.md), [DV_HARNESS.md](DV_HARNESS.md)
**Canonical ID Range:** `C001-C099`
**Intent:** merged continuous-frame runs that reuse the same build phase across all accepted buckets.

Continuous-frame runs use the same compiled DUT image as isolated cases. They
do not replace per-case evidence; they prove that accepted cases survive ordered
execution without inter-case reset.

Required continuous-frame baselines:

| run | state | notes |
|---|---|---|
| `basic_frame` | PASS for implemented slice | currently included inside `ALL_BUCKETS_FRAME`; no reset between `B003-B033`, `B121-B125`, `B001-B002` |
| `error_frame` | PASS for implemented slice | currently included inside `ALL_BUCKETS_FRAME`; no reset between `X001-X012`, `X121-X125` |
| `all_buckets_frame` | PASS for implemented slice | `../REPORT/ALL_BUCKETS_FRAME.md`; BASIC then ERROR implemented slice |
| `full_catalog_all_buckets_frame` | planned | all BASIC/EDGE/PROF/ERROR cases in order after remaining buckets are implemented |
