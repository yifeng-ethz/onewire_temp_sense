# TCL File Generated by Component Editor 18.1
# Wed Aug 21 17:02:13 CEST 2024
# DO NOT MODIFY


# 
# onewire_sense "OneWire Temperature Sensor Controller" v4.0.0
# Yifeng Wang 2024.08.21.17:02:13
# Periodically read onewire sensor. Can be halted from CSR.
# 

# 
# request TCL package from ACDS 16.1
# 
package require -exact qsys 16.1


# 
# module onewire_sense
# 
set_module_property DESCRIPTION "Periodically read onewire sensor. Can be halted from CSR."
set_module_property NAME onewire_sense
set_module_property VERSION 4.0.0
set_module_property INTERNAL false
set_module_property OPAQUE_ADDRESS_MAP true
set_module_property GROUP "Mu3e Control Plane/Modules"
set_module_property AUTHOR "Yifeng Wang"
set_module_property ICON_PATH ../drawings/20121123203030!Logo_drawing100.png
set_module_property DISPLAY_NAME "OneWire Temperature Sensor Controller"
set_module_property INSTANTIATE_IN_SYSTEM_MODULE true
set_module_property EDITABLE true
set_module_property REPORT_TO_TALKBACK false
set_module_property ALLOW_GREYBOX_GENERATION false
set_module_property REPORT_HIERARCHY false


# 
# file sets
# 
add_fileset QUARTUS_SYNTH QUARTUS_SYNTH "" ""
set_fileset_property QUARTUS_SYNTH TOP_LEVEL onewire_dri
set_fileset_property QUARTUS_SYNTH ENABLE_RELATIVE_INCLUDE_PATHS false
set_fileset_property QUARTUS_SYNTH ENABLE_FILE_OVERWRITE_MODE false
add_fileset_file checker_crc8_maxim.vhd VHDL PATH checker_crc8_maxim.vhd
add_fileset_file onewire_dri.vhd VHDL PATH onewire_temp_sense.vhd TOP_LEVEL_FILE
add_fileset_file clkdiv.vhd VHDL PATH clkdiv.vhd


# 
# parameters
# 
add_parameter SENSOR_TYPE STRING DS18B20 "Sensor family number; only specific sensor are supported; other families requires modificiations of rtl code"
set_parameter_property SENSOR_TYPE DEFAULT_VALUE DS18B20
set_parameter_property SENSOR_TYPE DISPLAY_NAME SENSOR_TYPE
set_parameter_property SENSOR_TYPE TYPE STRING
set_parameter_property SENSOR_TYPE ENABLED false
set_parameter_property SENSOR_TYPE UNITS None
set_parameter_property SENSOR_TYPE DESCRIPTION "Sensor family number; only specific sensor are supported; other families requires modificiations of rtl code"
set_parameter_property SENSOR_TYPE HDL_PARAMETER true
add_parameter PARACITIC_POWERING BOOLEAN true "Power scheme of the slave sensor; pull-high DQ line if in paracitic powering scheme"
set_parameter_property PARACITIC_POWERING DEFAULT_VALUE true
set_parameter_property PARACITIC_POWERING DISPLAY_NAME PARACITIC_POWERING
set_parameter_property PARACITIC_POWERING TYPE BOOLEAN
set_parameter_property PARACITIC_POWERING ENABLED false
set_parameter_property PARACITIC_POWERING UNITS None
set_parameter_property PARACITIC_POWERING DESCRIPTION "Power scheme of the slave sensor; pull-high DQ line if in paracitic powering scheme"
set_parameter_property PARACITIC_POWERING HDL_PARAMETER true
add_parameter BAUD_RATE INTEGER 10000 "Baud rate for one-wire protocol"
set_parameter_property BAUD_RATE DEFAULT_VALUE 10000
set_parameter_property BAUD_RATE DISPLAY_NAME BAUD_RATE
set_parameter_property BAUD_RATE TYPE INTEGER
set_parameter_property BAUD_RATE ENABLED false
set_parameter_property BAUD_RATE UNITS None
set_parameter_property BAUD_RATE ALLOWED_RANGES -2147483648:2147483647
set_parameter_property BAUD_RATE DESCRIPTION "Baud rate for one-wire protocol"
set_parameter_property BAUD_RATE HDL_PARAMETER true
add_parameter CONVERSION_WAIT_T_MS NATURAL 800
set_parameter_property CONVERSION_WAIT_T_MS DEFAULT_VALUE 800
set_parameter_property CONVERSION_WAIT_T_MS DISPLAY_NAME CONVERSION_WAIT_T_MS
set_parameter_property CONVERSION_WAIT_T_MS TYPE NATURAL
set_parameter_property CONVERSION_WAIT_T_MS UNITS None
set_parameter_property CONVERSION_WAIT_T_MS ALLOWED_RANGES 0:2147483647
set_parameter_property CONVERSION_WAIT_T_MS HDL_PARAMETER true
add_parameter TEMPERATURE_RESOLUTION INTEGER 12 "The conversion precision; lower precision will half the conversion time"
set_parameter_property TEMPERATURE_RESOLUTION DEFAULT_VALUE 12
set_parameter_property TEMPERATURE_RESOLUTION DISPLAY_NAME TEMPERATURE_RESOLUTION
set_parameter_property TEMPERATURE_RESOLUTION TYPE INTEGER
set_parameter_property TEMPERATURE_RESOLUTION ENABLED false
set_parameter_property TEMPERATURE_RESOLUTION UNITS None
set_parameter_property TEMPERATURE_RESOLUTION ALLOWED_RANGES -2147483648:2147483647
set_parameter_property TEMPERATURE_RESOLUTION DESCRIPTION "The conversion precision; lower precision will half the conversion time"
set_parameter_property TEMPERATURE_RESOLUTION HDL_PARAMETER true


# 
# display items
# 
add_display_item "" Basics GROUP ""
add_display_item "" "Physical Layer Settings" GROUP ""
add_display_item "" "Optional " GROUP ""
add_display_item "" Optional GROUP ""


# 
# connection point sensor_dq
# 
add_interface sensor_dq conduit end
set_interface_property sensor_dq associatedClock clock_interface
set_interface_property sensor_dq associatedReset reset_interface
set_interface_property sensor_dq ENABLED true
set_interface_property sensor_dq EXPORT_OF ""
set_interface_property sensor_dq PORT_NAME_MAP ""
set_interface_property sensor_dq CMSIS_SVD_VARIABLES ""
set_interface_property sensor_dq SVD_ADDRESS_GROUP ""

add_interface_port sensor_dq io_dq export Bidir 1


# 
# connection point csr
# 
add_interface csr avalon end
set_interface_property csr addressUnits WORDS
set_interface_property csr associatedClock clock_interface
set_interface_property csr associatedReset reset_interface
set_interface_property csr bitsPerSymbol 8
set_interface_property csr burstOnBurstBoundariesOnly false
set_interface_property csr burstcountUnits WORDS
set_interface_property csr explicitAddressSpan 0
set_interface_property csr holdTime 0
set_interface_property csr linewrapBursts false
set_interface_property csr maximumPendingReadTransactions 0
set_interface_property csr maximumPendingWriteTransactions 0
set_interface_property csr readLatency 1
set_interface_property csr readWaitTime 1
set_interface_property csr setupTime 0
set_interface_property csr timingUnits Cycles
set_interface_property csr writeWaitTime 0
set_interface_property csr ENABLED true
set_interface_property csr EXPORT_OF ""
set_interface_property csr PORT_NAME_MAP ""
set_interface_property csr CMSIS_SVD_VARIABLES ""
set_interface_property csr SVD_ADDRESS_GROUP ""

add_interface_port csr avs_ow_write write Input 1
add_interface_port csr avs_ow_writedata writedata Input 32
add_interface_port csr avs_ow_read read Input 1
add_interface_port csr avs_ow_readdata readdata Output 32
add_interface_port csr avs_ow_waitrequest waitrequest Output 1
set_interface_assignment csr embeddedsw.configuration.isFlash 0
set_interface_assignment csr embeddedsw.configuration.isMemoryDevice 0
set_interface_assignment csr embeddedsw.configuration.isNonVolatileStorage 0
set_interface_assignment csr embeddedsw.configuration.isPrintableDevice 0


# 
# connection point reset_interface
# 
add_interface reset_interface reset end
set_interface_property reset_interface associatedClock clock_interface
set_interface_property reset_interface synchronousEdges BOTH
set_interface_property reset_interface ENABLED true
set_interface_property reset_interface EXPORT_OF ""
set_interface_property reset_interface PORT_NAME_MAP ""
set_interface_property reset_interface CMSIS_SVD_VARIABLES ""
set_interface_property reset_interface SVD_ADDRESS_GROUP ""

add_interface_port reset_interface i_rst_n reset_n Input 1


# 
# connection point clock_interface
# 
add_interface clock_interface clock end
set_interface_property clock_interface clockRate 125000000
set_interface_property clock_interface ENABLED true
set_interface_property clock_interface EXPORT_OF ""
set_interface_property clock_interface PORT_NAME_MAP ""
set_interface_property clock_interface CMSIS_SVD_VARIABLES ""
set_interface_property clock_interface SVD_ADDRESS_GROUP ""

add_interface_port clock_interface i_clk_125 clk Input 1

