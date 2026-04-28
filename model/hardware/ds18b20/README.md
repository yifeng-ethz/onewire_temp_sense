# DS18B20 Datasheet Reference

Local reference copy:

- `DS18B20_rev6_2019.pdf`
- `DS18B20_rev6_2019.pdf.sha256`

Source: Analog Devices / Maxim Integrated, `DS18B20: Programmable Resolution
1-Wire Digital Thermometer`, Rev. 6, dated 2019-08-09.

The DV models use this PDF as the truth source for:

- reset low/high timing (`tRSTL`, `tRSTH`)
- presence-detect timing (`tPDHIGH`, `tPDLOW`)
- write slot timing (`tSLOT`, `tLOW0`, `tLOW1`, `tREC`)
- read slot timing (`tRDV`)
- pull-up topology, nominal 4.7 kOhm to `VPU`
- input thresholds (`VIL`, `VIH`)
- maximum DS18B20 pin capacitance (`CIN/OUT`)
- temperature conversion time and scratchpad layout

Do not tune the model only to make the current RTL pass. If RTL simulation and
this datasheet disagree, record the mismatch in `tb/doc/BUG_HISTORY.md` and
debug the lower layer first.
