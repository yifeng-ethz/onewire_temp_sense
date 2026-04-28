# OneWire Synthesis And Packaging

Root-level `rtl/` contains the active RTL source. Root-level `script/` contains
the active Platform Designer package files (`*_hw.tcl`) and SVD helpers.

`syn/` is reserved for synthesis projects, generated helper IP, Quartus output,
and synthesis evidence. `alt_ip/` contains archived Intel-generated helper IP
files kept for reference.
