# ALL_BUCKETS_FRAME

| field | value |
|---|---|
| status | PASS |
| command | `make -C onewire_temp_sense/tb/uvm run CASE=ALL_BUCKETS_FRAME MODE=all_buckets_frame` |
| log | `REPORT/ALL_BUCKETS_FRAME.log` |
| cases in frame | `B003`, `B004`..`B033`, `B121`..`B125`, `B001`, `B002`, `X001`..`X012`, `X121`..`X125` |
| features covered | BASIC common header/scratch/capability/master CSR descriptor range/master commit-busy-IRQ/master TX FSM/status/processor_go/init_err/sample_valid/temperature/zero guard/six-line loop; ERROR master CSR illegal access/read-only no-op/temperature write no-op/invalid selection/unmapped temperature |

This is a continuous no-restart frame over the currently implemented BASIC and
ERROR slice. EDGE, PROF, CRC-fault, FIFO-fault, and Read-ROM-probe features
remain planned or blocked as recorded in `../doc/DV_COV.md`.
