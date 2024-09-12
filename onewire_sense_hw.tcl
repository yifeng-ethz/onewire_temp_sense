# TCL File Generated by Component Editor 18.1
# Fri Sep 06 16:54:17 CEST 2024
# DO --NOT-- MODIFY


# 
# onewire_master "OneWire Master" v24.0.911
# Yifeng Wang 2024.09.06.16:54:17


# 
# request TCL package from ACDS 16.1
# 
package require -exact qsys 16.1


# 
# module onewire_master
# 
set_module_property DESCRIPTION \
"<html>
Handles link layer transaction in <b>1-Wire</b> protocol. <br>
<br>
Usage:
<ul>	
	<li><b>TX Flow</b>:</li> 
	<ol>
		<li>send data through <b>tx</b> port. <br> </li>
		<li>issue command through <b>ctrl</b> port. <br> </li>
	</ol>
	<li><b>RX Flow</b>:</li> 
	<ol>
		<li>issue command through <b>ctrl</b> port. <br> </li>
		<li>retrieve data from <b>rx</b> port. <br> </li>
	</ol>
</ul>
</html>"
set_module_property NAME onewire_master
set_module_property VERSION 24.0.911.1
set_module_property INTERNAL false
set_module_property OPAQUE_ADDRESS_MAP true
set_module_property GROUP "Mu3e Control Plane/Modules"
set_module_property AUTHOR "Yifeng Wang"
set_module_property ICON_PATH ../figures/mu3e_logo.png
set_module_property DISPLAY_NAME "OneWire Master"
set_module_property INSTANTIATE_IN_SYSTEM_MODULE true
set_module_property EDITABLE false
set_module_property REPORT_TO_TALKBACK false
set_module_property ALLOW_GREYBOX_GENERATION false
set_module_property REPORT_HIERARCHY false
set_module_property ELABORATION_CALLBACK elaborate


# 
# file sets
# 
add_fileset QUARTUS_SYNTH QUARTUS_SYNTH "" ""
set_fileset_property QUARTUS_SYNTH TOP_LEVEL onewire_master
set_fileset_property QUARTUS_SYNTH ENABLE_RELATIVE_INCLUDE_PATHS false
set_fileset_property QUARTUS_SYNTH ENABLE_FILE_OVERWRITE_MODE false
add_fileset_file crc_checker_maxim_crc8.vhd VHDL PATH crc_checker_maxim_crc8.vhd
add_fileset_file pseudo_clock_down_convertor.vhd VHDL PATH pseudo_clock_down_convertor.vhd
add_fileset_file onewire_master.vhd VHDL PATH onewire_master.vhd TOP_LEVEL_FILE


# 
# parameters
# 
# Reference for html codes used in this section
 # ----------------------------------------------
 # &lt = less than (<)
 # &gt = greater than (>)
 # <b></b> = bold text
 # <ul></ul> = defines an unordered list
 # <li></li> = bullet list
 # <br> = line break
add_parameter MASTER_INIT_RESET_US NATURAL 
set_parameter_property MASTER_INIT_RESET_US DEFAULT_VALUE 500
set_parameter_property MASTER_INIT_RESET_US DISPLAY_NAME "Reset pulse width (master)"
set_parameter_property MASTER_INIT_RESET_US TYPE NATURAL
set_parameter_property MASTER_INIT_RESET_US UNITS Microseconds
set_parameter_property MASTER_INIT_RESET_US ALLOWED_RANGES 400:1_000_000
set_parameter_property MASTER_INIT_RESET_US HDL_PARAMETER true

add_parameter MASTER_WAIT_PRESENCE_US NATURAL 
set_parameter_property MASTER_WAIT_PRESENCE_US DEFAULT_VALUE 45
set_parameter_property MASTER_WAIT_PRESENCE_US DISPLAY_NAME "Time to start sampling presence (slave)"
set_parameter_property MASTER_WAIT_PRESENCE_US TYPE NATURAL
set_parameter_property MASTER_WAIT_PRESENCE_US UNITS Microseconds
set_parameter_property MASTER_WAIT_PRESENCE_US ALLOWED_RANGES 0:1000
set_parameter_property MASTER_WAIT_PRESENCE_US HDL_PARAMETER true

add_parameter MASTER_SAMPLE_PRESENCE_TIMEOUT_US NATURAL 
set_parameter_property MASTER_SAMPLE_PRESENCE_TIMEOUT_US DEFAULT_VALUE 1000
set_parameter_property MASTER_SAMPLE_PRESENCE_TIMEOUT_US DISPLAY_NAME "Reset pulse detection timeout"
set_parameter_property MASTER_SAMPLE_PRESENCE_TIMEOUT_US TYPE NATURAL
set_parameter_property MASTER_SAMPLE_PRESENCE_TIMEOUT_US UNITS Microseconds
set_parameter_property MASTER_SAMPLE_PRESENCE_TIMEOUT_US ALLOWED_RANGES 0:1_000_000
set_parameter_property MASTER_SAMPLE_PRESENCE_TIMEOUT_US HDL_PARAMETER true

add_parameter SLOTS_SEPERATION_US NATURAL 
set_parameter_property SLOTS_SEPERATION_US DEFAULT_VALUE 5
set_parameter_property SLOTS_SEPERATION_US DISPLAY_NAME "RX and TX bit transaction slot seperation"
set_parameter_property SLOTS_SEPERATION_US TYPE NATURAL
set_parameter_property SLOTS_SEPERATION_US UNITS Microseconds
set_parameter_property SLOTS_SEPERATION_US ALLOWED_RANGES 0:1_000_000
set_parameter_property SLOTS_SEPERATION_US HDL_PARAMETER true

add_parameter RX_SLOT_US NATURAL 
set_parameter_property RX_SLOT_US DEFAULT_VALUE 90
set_parameter_property RX_SLOT_US DISPLAY_NAME "RX (bit) slot total time"
set_parameter_property RX_SLOT_US TYPE NATURAL
set_parameter_property RX_SLOT_US UNITS Microseconds
set_parameter_property RX_SLOT_US ALLOWED_RANGES 0:500
set_parameter_property RX_SLOT_US HDL_PARAMETER true

add_parameter RX_PULL_LOW_US NATURAL 
set_parameter_property RX_PULL_LOW_US DEFAULT_VALUE 5
set_parameter_property RX_PULL_LOW_US DISPLAY_NAME "RX pull low timestamp"
set_parameter_property RX_PULL_LOW_US TYPE NATURAL
set_parameter_property RX_PULL_LOW_US UNITS Microseconds
set_parameter_property RX_PULL_LOW_US ALLOWED_RANGES 0:15
set_parameter_property RX_PULL_LOW_US HDL_PARAMETER true

add_parameter RX_MASTER_SAMPLE_US NATURAL 
set_parameter_property RX_MASTER_SAMPLE_US DEFAULT_VALUE 10 
set_parameter_property RX_MASTER_SAMPLE_US DISPLAY_NAME "RX master sample bit timestamp"
set_parameter_property RX_MASTER_SAMPLE_US TYPE NATURAL
set_parameter_property RX_MASTER_SAMPLE_US UNITS Microseconds
set_parameter_property RX_MASTER_SAMPLE_US ALLOWED_RANGES 0:100
set_parameter_property RX_MASTER_SAMPLE_US HDL_PARAMETER true

add_parameter TX_SLOT_US NATURAL 
set_parameter_property TX_SLOT_US DEFAULT_VALUE 90
set_parameter_property TX_SLOT_US DISPLAY_NAME "TX (bit) slot total time" 
set_parameter_property TX_SLOT_US TYPE NATURAL
set_parameter_property TX_SLOT_US UNITS Microseconds
set_parameter_property TX_SLOT_US ALLOWED_RANGES 0:500
set_parameter_property TX_SLOT_US HDL_PARAMETER true

add_parameter TX_PULL_LOW_US NATURAL 
set_parameter_property TX_PULL_LOW_US DEFAULT_VALUE 5
set_parameter_property TX_PULL_LOW_US DISPLAY_NAME "TX pull low timstamp"
set_parameter_property TX_PULL_LOW_US TYPE NATURAL
set_parameter_property TX_PULL_LOW_US UNITS Microseconds
set_parameter_property TX_PULL_LOW_US ALLOWED_RANGES 0:15
set_parameter_property TX_PULL_LOW_US HDL_PARAMETER true

add_parameter PARACITIC_POWERING boolean 
set_parameter_property PARACITIC_POWERING DEFAULT_VALUE true
set_parameter_property PARACITIC_POWERING DISPLAY_NAME "Sensor powering scheme"
set_parameter_property PARACITIC_POWERING TYPE boolean
set_parameter_property PARACITIC_POWERING UNITS None
set_parameter_property PARACITIC_POWERING ALLOWED_RANGES {"true:Paracitic Powering" "false:Direct Powering (Default)"}
set_parameter_property PARACITIC_POWERING DISPLAY_HINT "RADIO"
set_parameter_property PARACITIC_POWERING HDL_PARAMETER true
set dscpt \
"<html>
Select Sensor Powering Scheme (affects the tri-state buffer status). <br>
<br>
<b>Note</b>: <br>
<i><b>Paracitic Powering</b></i> will override <b>ctrl</b> port command. <br> Select <i><b>Direct</b></i> as you may set it later through <b>ctrl</b> port.
	<ul>
	<li><b>Paracitic</b> : Powered from the master side.<br> You have to remove the pull-up resistor in the slave side. <br>Strong pull up during line idle, might increase crosstalk and EMI.</li>
	<li><b>Direct (Default)</b> : Powered by external 3.3V through sensor VCC pin (<b>preferred in Mu3e</b>). <br>Tri-state line during idle, preventing crosstalk and EMI.</li>
	</ul>
</html>"
set_parameter_property PARACITIC_POWERING LONG_DESCRIPTION $dscpt
set_parameter_property PARACITIC_POWERING DESCRIPTION $dscpt
    

add_parameter REF_CLOCK_RATE NATURAL
set_parameter_property REF_CLOCK_RATE DEFAULT_VALUE 156_250_000
set_parameter_property REF_CLOCK_RATE DISPLAY_NAME "Reference clock rate"
set_parameter_property REF_CLOCK_RATE TYPE NATURAL
set_parameter_property REF_CLOCK_RATE UNITS Hertz
set_parameter_property REF_CLOCK_RATE ALLOWED_RANGES 1_000_000:10_000_000_000
set_parameter_property REF_CLOCK_RATE ENABLED true
set_parameter_property REF_CLOCK_RATE HDL_PARAMETER true
set dscpt \
"<html>
Set the reference input clock rate. <br>
This IP has internal clock down-convertor for transaction purpose of 1-Wire protocol. <br>
The input must be greater <b>(&ge 1 MHz)</b> than the requirement of the timing parameter granularity, which is 1 us. <br>
All link layer timing are strictly following the timing parameters you set. <br>
<b> You only need to set this parameter according to your system bus clock rate. </b> 
</html>"
set_parameter_property REF_CLOCK_RATE LONG_DESCRIPTION $dscpt
set_parameter_property REF_CLOCK_RATE DESCRIPTION $dscpt

add_parameter EN_STREAMING BOOLEAN true
set_parameter_property EN_STREAMING DISPLAY_NAME "Enable Streaming port" 
set_parameter_property EN_STREAMING HDL_PARAMETER 0
set_parameter_property EN_STREAMING DISPLAY_HINT "boolean"

add_parameter AVST_DATA_WIDTH NATURAL 
set_parameter_property AVST_DATA_WIDTH DEFAULT_VALUE 8
set_parameter_property AVST_DATA_WIDTH DISPLAY_NAME "Avalon Streaming data width"
set_parameter_property AVST_DATA_WIDTH TYPE NATURAL
set_parameter_property AVST_DATA_WIDTH UNITS Bits
set_parameter_property AVST_DATA_WIDTH ALLOWED_RANGES 1:1023
set_parameter_property AVST_DATA_WIDTH HDL_PARAMETER true
set dscpt \
"<html>
Set the data width of the Avalon Streaming interface. <br>
This parameter also sets <i>dataBitsPerSymbol</i> of the <b>rx</b> and <b>tx</b> interface. <br>
You have to set this paramter according to the length of a command of your sensor. <br>
Also referred to as <i>symbol</i> in the RTL code.  
<b>Default is 8.</b>
</html>"
set_parameter_property AVST_DATA_WIDTH LONG_DESCRIPTION $dscpt
set_parameter_property AVST_DATA_WIDTH DESCRIPTION $dscpt

add_parameter AVMM_DATA_WIDTH NATURAL 
set_parameter_property AVMM_DATA_WIDTH DEFAULT_VALUE 32
set_parameter_property AVMM_DATA_WIDTH DISPLAY_NAME "Avalon Memory-Mapped data width"
set_parameter_property AVMM_DATA_WIDTH TYPE NATURAL
set_parameter_property AVMM_DATA_WIDTH UNITS Bits
set_parameter_property AVMM_DATA_WIDTH ALLOWED_RANGES 1:1023
set_parameter_property AVMM_DATA_WIDTH HDL_PARAMETER true
set_parameter_property AVMM_DATA_WIDTH ENABLED false 
set dscpt \
"<html>
Set the data width of the Avalon Memory-Mapped interface. <br>
<b>Default is 32.</b>
</html>"
set_parameter_property AVMM_DATA_WIDTH LONG_DESCRIPTION $dscpt
set_parameter_property AVMM_DATA_WIDTH DESCRIPTION $dscpt

add_parameter N_DQ_LINES NATURAL 
set_parameter_property N_DQ_LINES DEFAULT_VALUE 6
set_parameter_property N_DQ_LINES DISPLAY_NAME "Number of DQ lines"
set_parameter_property N_DQ_LINES TYPE NATURAL
set_parameter_property N_DQ_LINES UNITS None
set_parameter_property N_DQ_LINES ALLOWED_RANGES 1:255
set_parameter_property N_DQ_LINES HDL_PARAMETER true
set dscpt \
"<html>
Set the number of dq lines to connect to sensors. <br>
<b>Note:</b> this parameter has nothing to do with number of sensors on the line. <br>
This will instantiate the same number of groups of tri-state control signals in the <b>sense</b> conduit interface. <br>
To select the line, 1) use the channel signal from the <b>rx</b> or <b>tx</b> interface or <br>2) use <i>wire_id</i> from the <b>ctrl</b> interface.
</html>"
set_parameter_property N_DQ_LINES LONG_DESCRIPTION $dscpt
set_parameter_property N_DQ_LINES DESCRIPTION $dscpt

add_parameter AVST_CHANNEL_WIDTH NATURAL 
set_parameter_property AVST_CHANNEL_WIDTH DEFAULT_VALUE 3
set_parameter_property AVST_CHANNEL_WIDTH DISPLAY_NAME "Avalon Steaming channel width"
set_parameter_property AVST_CHANNEL_WIDTH TYPE NATURAL
set_parameter_property AVST_CHANNEL_WIDTH UNITS Bits
set_parameter_property AVST_CHANNEL_WIDTH ALLOWED_RANGES 0:7
set_parameter_property AVST_CHANNEL_WIDTH HDL_PARAMETER true
set dscpt \
"<html>
Set the channel width of the out-band signal of Avalon Streaming interface. <br>
<br>
<b>Note</b>: Must be larger than number of dq lines.
</html>"
set_parameter_property AVST_CHANNEL_WIDTH LONG_DESCRIPTION $dscpt
set_parameter_property AVST_CHANNEL_WIDTH DESCRIPTION $dscpt

add_parameter RX_BUFFER_DEPTH NATURAL 
set_parameter_property RX_BUFFER_DEPTH DEFAULT_VALUE 16
set_parameter_property RX_BUFFER_DEPTH DISPLAY_NAME "RX buffer depth"
set_parameter_property RX_BUFFER_DEPTH TYPE NATURAL
set_parameter_property RX_BUFFER_DEPTH UNITS None
set_parameter_property RX_BUFFER_DEPTH ALLOWED_RANGES 1:1024
set_parameter_property RX_BUFFER_DEPTH HDL_PARAMETER true
set dscpt \
"<html>
Set the maximum number of entries for the <b>RX buffer</b>. The buffer is implemented as the scfifo. <br>
For smaller number entries (<b> N &lt 32 </b>), this fifo is intantiated from <b>MLAB</b>. <br>
For larger number of entries (<b> N &ge 32 </b>), this fifo is instantiated from <b>M10K/M20K</b> (block RAM). <br>
<br>
<b>Note</b>: for best resource utilization (symbol=8 bit), see the following advice: <br>
	When instantiated from <b>M10K</b>, use multiple of <b>1k</b> entry . <br>
	When instantiated from <b>M20K</b>, use multiple of <b>2k</b> entry. <br>
	<i> resize the above if symbol is not 8 bit </i>
</html>"
set_parameter_property RX_BUFFER_DEPTH LONG_DESCRIPTION $dscpt
set_parameter_property RX_BUFFER_DEPTH DESCRIPTION $dscpt

add_parameter TX_BUFFER_DEPTH NATURAL 
set_parameter_property TX_BUFFER_DEPTH DEFAULT_VALUE 8
set_parameter_property TX_BUFFER_DEPTH DISPLAY_NAME "TX buffer depth"
set_parameter_property TX_BUFFER_DEPTH TYPE NATURAL
set_parameter_property TX_BUFFER_DEPTH UNITS None
set_parameter_property TX_BUFFER_DEPTH ALLOWED_RANGES 1:1024
set_parameter_property TX_BUFFER_DEPTH HDL_PARAMETER true
set_parameter_property TX_BUFFER_DEPTH HDL_PARAMETER true
set dscpt \
"<html>
Set the maximum number of entries for the <b>TX buffer</b>. The buffer is implemented as the scfifo. <br>
For smaller number entries (<b> N &lt 32 </b>), this fifo is intantiated from <b>MLAB</b>. <br>
For larger number of entries (<b> N &ge 32 </b>), this fifo is instantiated from <b>M10K/M20K</b> (block RAM). <br>
<br>
<b>Note</b>: for best resource utilization (symbol=8 bit), see the following advice: <br>
	When instantiated from <b>M10K</b>, use multiple of <b>1k</b> entry . <br>
	When instantiated from <b>M20K</b>, use multiple of <b>2k</b> entry. <br>
	<i> Resize the above if symbol is not 8 bit. </i>
</html>"
set_parameter_property TX_BUFFER_DEPTH LONG_DESCRIPTION $dscpt
set_parameter_property TX_BUFFER_DEPTH DESCRIPTION $dscpt

add_parameter MAX_BUFFER_DEPTH NATURAL
set_parameter_property MAX_BUFFER_DEPTH DEFAULT_VALUE 16
set_parameter_property MAX_BUFFER_DEPTH DISPLAY_NAME "Max buffer depth"
set_parameter_property MAX_BUFFER_DEPTH TYPE NATURAL
set_parameter_property MAX_BUFFER_DEPTH UNITS None
set_parameter_property MAX_BUFFER_DEPTH VISIBLE false
set_parameter_property MAX_BUFFER_DEPTH ALLOWED_RANGES 0:1024
set_parameter_property MAX_BUFFER_DEPTH HDL_PARAMETER true
set_parameter_property MAX_BUFFER_DEPTH DERIVED true

add_parameter VARIANT string
set_parameter_property VARIANT DEFAULT_VALUE "lite"
set_parameter_property VARIANT TYPE string
set_parameter_property VARIANT DISPLAY_NAME "Variant"
set_parameter_property VARIANT ENABLED true
set_parameter_property VARIANT UNITS None
set_parameter_property VARIANT ALLOWED_RANGES {"lite:Lite" "full:Full"}
set_parameter_property VARIANT HDL_PARAMETER true
set dscpt \
"<html>
Select the IP variant.<br>
<ul>
	<li><b>Lite</b> : <i>(Basic transactions)</i> Descriptor-based RX and TX engines.</li>
	<li><b>Full</b> : <i>(Advance feature)</i> Adding probe engine for discovering and <br> reporting the <b>ROM code</b> of the sensors on all dq lines. </li>
</ul>
</html>"
set_parameter_property VARIANT LONG_DESCRIPTION $dscpt
set_parameter_property VARIANT DESCRIPTION $dscpt

add_parameter DEBUG_LV NATURAL 
set_parameter_property DEBUG_LV DEFAULT_VALUE 0
set_parameter_property DEBUG_LV DISPLAY_NAME "Debug level"
set_parameter_property DEBUG_LV TYPE NATURAL
set_parameter_property DEBUG_LV UNITS None
set_parameter_property DEBUG_LV ALLOWED_RANGES {0 1 2}
set_parameter_property DEBUG_LV HDL_PARAMETER true
set dscpt \
"<html>
Select the debug level of the IP (affects generation).<br>
<ul>
	<li><b>0</b> : off <br> </li>
	<li><b>1</b> : on, synthesizble <br> </li>
	<li><b>2</b> : on, non-synthesizble, simulation-only <br> </li>
</ul>
</html>"
set_parameter_property DEBUG_LV LONG_DESCRIPTION $dscpt
set_parameter_property DEBUG_LV DESCRIPTION $dscpt

add_parameter RX_FIFO_TYPE string "MLAB"
set_parameter_property RX_FIFO_TYPE DISPLAY_NAME "RAM block type of RX FIFO"
set_parameter_property RX_FIFO_TYPE ALLOWED_RANGES {"M10K" "MLAB" "M20K"}
set_parameter_property RX_FIFO_TYPE HDL_PARAMETER true
set_parameter_property RX_FIFO_TYPE ENABLED false
set_parameter_property RX_FIFO_TYPE DERIVED true
set dscpt \
"<html>
Inferred from device family and buffer depth.
</html>"
set_parameter_property RX_FIFO_TYPE LONG_DESCRIPTION $dscpt
set_parameter_property RX_FIFO_TYPE DESCRIPTION $dscpt

add_parameter TX_FIFO_TYPE string "MLAB"
set_parameter_property TX_FIFO_TYPE DISPLAY_NAME "RAM block type of TX FIFO"
set_parameter_property TX_FIFO_TYPE ALLOWED_RANGES {"M10K" "MLAB" "M20K"}
set_parameter_property TX_FIFO_TYPE HDL_PARAMETER true
set_parameter_property TX_FIFO_TYPE ENABLED false
set_parameter_property TX_FIFO_TYPE DERIVED true
set dscpt \
"<html>
Inferred from device family and buffer depth.
</html>"
set_parameter_property TX_FIFO_TYPE LONG_DESCRIPTION $dscpt
set_parameter_property TX_FIFO_TYPE DESCRIPTION $dscpt

add_parameter DEVICE_FAMILY string
set_parameter_property DEVICE_FAMILY SYSTEM_INFO {DEVICE_FAMILY}
set_parameter_property DEVICE_FAMILY DISPLAY_NAME "Device family"
set_parameter_property DEVICE_FAMILY HDL_PARAMETER false
set dscpt \
"<html>
Auto detection of the device family.<br>
This parameter sets the supported RAM block type.
</html>"
set_parameter_property DEVICE_FAMILY LONG_DESCRIPTION $dscpt
set_parameter_property DEVICE_FAMILY DESCRIPTION $dscpt

add_parameter DEVICE_FEATURES string
set_parameter_property DEVICE_FEATURES SYSTEM_INFO {DEVICE_FEATURES}
set_parameter_property DEVICE_FEATURES HDL_PARAMETER false
set_parameter_property DEVICE_FEATURES VISIBLE 0


# 
# display items
# 

# --------------------------------------------------------------- 
add_display_item "" "Transmission Timing Parameter" GROUP ""

add_display_item "Transmission Timing Parameter" "Initialization" GROUP tab
add_display_item "Initialization" MASTER_INIT_RESET_US PARAMETER 
add_display_item "Initialization" MASTER_WAIT_PRESENCE_US PARAMETER 
add_display_item "Initialization" MASTER_SAMPLE_PRESENCE_TIMEOUT_US PARAMETER 

add_display_item "Transmission Timing Parameter" "RX/TX" GROUP tab
add_display_item "RX/TX" SLOTS_SEPERATION_US PARAMETER 

add_display_item "Transmission Timing Parameter" "RX" GROUP tab
add_display_item "RX" RX_SLOT_US PARAMETER 
add_display_item "RX" RX_PULL_LOW_US PARAMETER 
add_display_item "RX" RX_MASTER_SAMPLE_US PARAMETER

add_display_item "Transmission Timing Parameter" "TX" GROUP tab
add_display_item "TX" TX_SLOT_US PARAMETER 
add_display_item "TX" TX_PULL_LOW_US PARAMETER 

# --------------------------------------------------------------- 
add_display_item "" "IP Setting" GROUP ""
## ----
add_display_item "IP Setting" "System" GROUP ""
add_display_item "System" DEVICE_FAMILY PARAMETER
add_display_item "System" REF_CLOCK_RATE PARAMETER
add_display_item "System" N_DQ_LINES PARAMETER 
## ----
add_display_item "IP Setting" "Ports" GROUP ""
add_display_item "Ports" EN_STREAMING PARAMETER 
add_display_item "Ports" AVST_DATA_WIDTH PARAMETER 
add_display_item "Ports" AVST_CHANNEL_WIDTH PARAMETER
add_display_item "Ports" AVMM_DATA_WIDTH PARAMETER 
## ---- 
add_display_item "IP Setting" "Buffers" GROUP ""
add_display_item "Buffers" RX_BUFFER_DEPTH PARAMETER
add_display_item "Buffers" RX_FIFO_TYPE PARAMETER  
add_display_item "Buffers" TX_BUFFER_DEPTH PARAMETER 
add_display_item "Buffers" TX_FIFO_TYPE PARAMETER  
add_display_item "Buffers" MAX_BUFFER_DEPTH PARAMETER 
## ----
add_display_item "IP Setting" "Version" GROUP ""
add_display_item "Version" VARIANT PARAMETER 
add_display_item "Version" DEBUG_LV PARAMETER 


# --------------------------------------------------------------- 
add_display_item "" "Periphial Circuitry Setting" GROUP ""
add_display_item "Periphial Circuitry Setting" PARACITIC_POWERING PARAMETER 



# --------------------------------------------------------------- 
add_display_item "" "Timing Diagram" GROUP ""

add_display_item "Timing Diagram" "RX and TX" GROUP tab
add_display_item "RX and TX" rx_tx_timing_icon ICON ./application_note/rx_and_tx_bit_timing.png

add_display_item "Timing Diagram" "Init." GROUP tab
add_display_item "Init." init_timing_icon ICON ./application_note/init_timing.png






#add_display_item "" "Address Routing Table" group "table"

#add_display_item "Transmission Timing Parameter" actionName ACTION buttonClickCallbackProc
add_display_item "" "Physical Layer Settings" GROUP ""
add_display_item "" "Optional " GROUP ""
add_display_item "" Optional GROUP ""


#send_message INFO "<b>Hello World!</b> Byte~"



# 
# connection point ctrl
# 
add_interface ctrl avalon end
set_interface_property ctrl addressUnits WORDS
set_interface_property ctrl associatedClock clock
set_interface_property ctrl associatedReset reset
set_interface_property ctrl bitsPerSymbol 8
set_interface_property ctrl burstOnBurstBoundariesOnly false
set_interface_property ctrl burstcountUnits WORDS
set_interface_property ctrl explicitAddressSpan 0
set_interface_property ctrl holdTime 0
set_interface_property ctrl linewrapBursts false
set_interface_property ctrl maximumPendingReadTransactions 0
set_interface_property ctrl maximumPendingWriteTransactions 0
set_interface_property ctrl readLatency 0
set_interface_property ctrl readWaitTime 1
set_interface_property ctrl setupTime 0
set_interface_property ctrl timingUnits Cycles
set_interface_property ctrl writeWaitTime 0
set_interface_property ctrl ENABLED true
set_interface_property ctrl EXPORT_OF ""
set_interface_property ctrl PORT_NAME_MAP ""
set_interface_property ctrl CMSIS_SVD_VARIABLES ""
set_interface_property ctrl SVD_ADDRESS_GROUP ""

add_interface_port ctrl avs_ctrl_read read Input 1
add_interface_port ctrl avs_ctrl_readdata readdata Output avmm_data_width
add_interface_port ctrl avs_ctrl_writedata writedata Input avmm_data_width
add_interface_port ctrl avs_ctrl_address address Input 4
add_interface_port ctrl avs_ctrl_waitrequest waitrequest Output 1
add_interface_port ctrl avs_ctrl_write write Input 1
set_interface_assignment ctrl embeddedsw.configuration.isFlash 0
set_interface_assignment ctrl embeddedsw.configuration.isMemoryDevice 0
set_interface_assignment ctrl embeddedsw.configuration.isNonVolatileStorage 0
set_interface_assignment ctrl embeddedsw.configuration.isPrintableDevice 0


# 
# connection point rx
# 
add_interface rx avalon_streaming start
set_interface_property rx associatedClock clock
set_interface_property rx associatedReset reset
set_interface_property rx dataBitsPerSymbol 8
set_interface_property rx errorDescriptor ""
set_interface_property rx firstSymbolInHighOrderBits true
set_interface_property rx maxChannel 0
set_interface_property rx readyLatency 0
set_interface_property rx ENABLED true
set_interface_property rx EXPORT_OF ""
set_interface_property rx PORT_NAME_MAP ""
set_interface_property rx CMSIS_SVD_VARIABLES ""
set_interface_property rx SVD_ADDRESS_GROUP ""

add_interface_port rx aso_rx_data data Output avst_data_width
add_interface_port rx aso_rx_valid valid Output 1
add_interface_port rx aso_rx_ready ready Input 1
add_interface_port rx aso_rx_channel channel Output avst_channel_width


# 
# connection point tx
# 
add_interface tx avalon_streaming end
set_interface_property tx associatedClock clock
set_interface_property tx associatedReset reset
set_interface_property tx dataBitsPerSymbol 8
set_interface_property tx errorDescriptor ""
set_interface_property tx firstSymbolInHighOrderBits true
set_interface_property tx maxChannel 0
set_interface_property tx readyLatency 0
set_interface_property tx ENABLED true
set_interface_property tx EXPORT_OF ""
set_interface_property tx PORT_NAME_MAP ""
set_interface_property tx CMSIS_SVD_VARIABLES ""
set_interface_property tx SVD_ADDRESS_GROUP ""

add_interface_port tx asi_tx_data data Input avst_data_width
add_interface_port tx asi_tx_valid valid Input 1
add_interface_port tx asi_tx_ready ready Output 1
add_interface_port tx asi_tx_channel channel Input avst_channel_width


# 
# connection point complete
# 
add_interface complete interrupt end
set_interface_property complete associatedAddressablePoint ctrl
set_interface_property complete associatedClock clock
set_interface_property complete associatedReset reset
set_interface_property complete bridgedReceiverOffset 0
set_interface_property complete bridgesToReceiver ""
set_interface_property complete ENABLED true
set_interface_property complete EXPORT_OF ""
set_interface_property complete PORT_NAME_MAP ""
set_interface_property complete CMSIS_SVD_VARIABLES ""
set_interface_property complete SVD_ADDRESS_GROUP ""

add_interface_port complete ins_complete_irq irq Output 1


# 
# connection point sense_dq
# 
add_interface sense_dq conduit end
set_interface_property sense_dq associatedClock clock
set_interface_property sense_dq associatedReset reset
set_interface_property sense_dq ENABLED true
set_interface_property sense_dq EXPORT_OF ""
set_interface_property sense_dq PORT_NAME_MAP ""
set_interface_property sense_dq CMSIS_SVD_VARIABLES ""
set_interface_property sense_dq SVD_ADDRESS_GROUP ""

add_interface_port sense_dq coe_sense_dq_in in Input n_dq_lines
add_interface_port sense_dq coe_sense_dq_out out Output n_dq_lines
add_interface_port sense_dq coe_sense_dq_oe oe Output n_dq_lines


# 
# connection point clock
# 
add_interface clock clock end
set_interface_property clock clockRate 0
set_interface_property clock ENABLED true
set_interface_property clock EXPORT_OF ""
set_interface_property clock PORT_NAME_MAP ""
set_interface_property clock CMSIS_SVD_VARIABLES ""
set_interface_property clock SVD_ADDRESS_GROUP ""

add_interface_port clock csi_clock_clk clk Input 1


# 
# connection point reset
# 
add_interface reset reset end
set_interface_property reset associatedClock clock
set_interface_property reset synchronousEdges BOTH
set_interface_property reset ENABLED true
set_interface_property reset EXPORT_OF ""
set_interface_property reset PORT_NAME_MAP ""
set_interface_property reset CMSIS_SVD_VARIABLES ""
set_interface_property reset SVD_ADDRESS_GROUP ""

add_interface_port reset rsi_reset_reset reset Input 1


#
# callbacks
#
proc elaborate {} {
	global GLOBAL_PARAMS
	send_message INFO "<b>Hello World!</b> Byte~"
	
	# ----
	# derive max fifo depth
	set rx_d [get_parameter_value "RX_BUFFER_DEPTH"]
	set tx_d [get_parameter_value "TX_BUFFER_DEPTH"]
	if {[expr $rx_d > $tx_d]} {
		set_parameter_value "MAX_BUFFER_DEPTH" $rx_d
	} else {
		set_parameter_value "MAX_BUFFER_DEPTH" $tx_d
	}
	
	# ---- 
	# grab device memory type support
	set device_features_list [get_parameter_value "DEVICE_FEATURES"]
	set supp_m10k_idx [lsearch $device_features_list "M10K_MEMORY"]
	set supp_m10k [lindex $device_features_list [expr $supp_m10k_idx+1]]
	set supp_m20k_idx [lsearch $device_features_list "M20K_MEMORY"]
	set supp_m20k [lindex $device_features_list [expr $supp_m20k_idx+1]]

	
	# ----
	# set ram block type
	if {[expr $rx_d >= 32]} {
		if {$supp_m10k} {
			set_parameter_value "RX_FIFO_TYPE" "M10K"
		} elseif {$supp_m20k} {
			set_parameter_value "RX_FIFO_TYPE" "M20K"
		} else {
			send_message ERROR "Memory type cannot be set for this device"
		}
	} else { 
		set_parameter_value "RX_FIFO_TYPE" "MLAB"
	}
	
	if {[expr $tx_d >= 32]} {
		if {$supp_m10k} {
			set_parameter_value "TX_FIFO_TYPE" "M10K"
		} elseif {$supp_m20k} {
			set_parameter_value "TX_FIFO_TYPE" "M20K"
		} else {
			send_message ERROR "Memory type cannot be set for this device"
		}
	} else { 
		set_parameter_value "TX_FIFO_TYPE" "MLAB"
	}
	
	# ----
	# set maxchannel
	set_interface_property rx maxChannel [get_parameter_value "N_DQ_LINES"]
	set_interface_property tx maxChannel [get_parameter_value "N_DQ_LINES"]
	
	# ----
	# set clock rate
	set_interface_property clock clockRate [get_parameter_value "REF_CLOCK_RATE"]
	
	# ----
	# set beats per symbol
	set_interface_property rx dataBitsPerSymbol [get_parameter_value "AVST_DATA_WIDTH"]
	set_interface_property tx dataBitsPerSymbol [get_parameter_value "AVST_DATA_WIDTH"]
	
	# ---- 
	# hide avst port
	set enable_avst [get_parameter_value EN_STREAMING]
	set_parameter_property AVST_CHANNEL_WIDTH ENABLED $enable_avst
	set_parameter_property AVST_DATA_WIDTH ENABLED $enable_avst
	# close interface rx
	set_port_property aso_rx_data termination [expr {!$enable_avst}]
	set_port_property aso_rx_valid termination [expr {!$enable_avst}]
	set_port_property aso_rx_ready termination [expr {!$enable_avst}]
	set_port_property aso_rx_channel termination [expr {!$enable_avst}]
	set_port_property aso_rx_ready termination_value "0"
	# close interface tx
	set_port_property asi_tx_data termination [expr {!$enable_avst}]
	set_port_property asi_tx_valid termination [expr {!$enable_avst}]
	set_port_property asi_tx_ready termination [expr {!$enable_avst}]
	set_port_property asi_tx_channel termination [expr {!$enable_avst}]
	set_port_property asi_tx_valid termination_value "0"
	
	
	
}




