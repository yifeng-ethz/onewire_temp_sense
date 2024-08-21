# Onewire Temperature Sensor Controller IP

## IP Description

**Name**: Onewire temperature sensor IP  
**Description**: Periodically read the temperature sensor with onewire protocol, especially the Dallas DS18B20. Provide the readout temperature in floating point (32bit) format in the CSR registers. Currently, only support one sensor on a `dq` line. Automatically perform `init`, send `rom_skip` command, send `convert` command, send `read_temp` command and receive the slave scratch pad (contains temperature info) and verify with crc check.   
**Usage**: Read once after FPGA power-up. The user can write `0x1` to start periodic reading or write `0x0` to halt. Default is halt. Read back temperature through address 0 of CSR.   

