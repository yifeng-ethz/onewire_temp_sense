package require Tcl 8.5

set script_dir [file dirname [info script]]
set helper_file [file normalize [file join $script_dir .. toolkit infra cmsis_svd lib mu3e_cmsis_svd.tcl]]
source $helper_file

namespace eval ::mu3e::cmsis::spec {}

proc ::mu3e::cmsis::spec::build_device {} {
    set registers [list \
        [::mu3e::cmsis::svd::register CAPABILITY 0x00 \
            -description {1-Wire controller capability register.} \
            -access read-only \
            -fields [list \
                [::mu3e::cmsis::svd::field n_dq_lines 0 16 \
                    -description {Number of 1-Wire DQ lines synthesized into this instance.} \
                    -access read-only] \
                [::mu3e::cmsis::svd::field n_sensors 16 16 \
                    -description {Reserved for future sensor-count reporting; currently reads zero.} \
                    -access read-only]]] \
        [::mu3e::cmsis::svd::register STATUS 0x04 \
            -description {Line select and per-line control/status register. Reads return the selected line and its sticky error summary. Writes update sel_line and processor_go for the addressed line.} \
            -access read-write \
            -fields [list \
                [::mu3e::cmsis::svd::field sel_line 0 16 \
                    -description {Selected DQ line index for readback and processor_go writes.} \
                    -access read-write] \
                [::mu3e::cmsis::svd::field processor_go 16 1 \
                    -description {Enable or disable the background measurement processor for the selected line.} \
                    -access read-write] \
                [::mu3e::cmsis::svd::field reserved0 17 7 \
                    -description {Reserved, read as zero.} \
                    -access read-only] \
                [::mu3e::cmsis::svd::field crc_err 24 1 \
                    -description {CRC error sticky flag for the selected line.} \
                    -access read-only] \
                [::mu3e::cmsis::svd::field init_err 25 1 \
                    -description {Initialization error sticky flag for the selected line.} \
                    -access read-only] \
                [::mu3e::cmsis::svd::field reserved1 26 6 \
                    -description {Reserved, read as zero.} \
                    -access read-only]]] \
        [::mu3e::cmsis::svd::register SENSOR0_TEMP_F32 0x08 \
            -description {IEEE-754 float32 temperature reading for sensor 0.} \
            -access read-only \
            -fields [list [::mu3e::cmsis::svd::field value 0 32 -description {Raw float32 temperature sample for sensor 0.} -access read-only]]] \
        [::mu3e::cmsis::svd::register SENSOR1_TEMP_F32 0x0C \
            -description {IEEE-754 float32 temperature reading for sensor 1.} \
            -access read-only \
            -fields [list [::mu3e::cmsis::svd::field value 0 32 -description {Raw float32 temperature sample for sensor 1.} -access read-only]]] \
        [::mu3e::cmsis::svd::register SENSOR2_TEMP_F32 0x10 \
            -description {IEEE-754 float32 temperature reading for sensor 2.} \
            -access read-only \
            -fields [list [::mu3e::cmsis::svd::field value 0 32 -description {Raw float32 temperature sample for sensor 2.} -access read-only]]] \
        [::mu3e::cmsis::svd::register SENSOR3_TEMP_F32 0x14 \
            -description {IEEE-754 float32 temperature reading for sensor 3.} \
            -access read-only \
            -fields [list [::mu3e::cmsis::svd::field value 0 32 -description {Raw float32 temperature sample for sensor 3.} -access read-only]]] \
        [::mu3e::cmsis::svd::register SENSOR4_TEMP_F32 0x18 \
            -description {IEEE-754 float32 temperature reading for sensor 4.} \
            -access read-only \
            -fields [list [::mu3e::cmsis::svd::field value 0 32 -description {Raw float32 temperature sample for sensor 4.} -access read-only]]] \
        [::mu3e::cmsis::svd::register SENSOR5_TEMP_F32 0x1C \
            -description {IEEE-754 float32 temperature reading for sensor 5.} \
            -access read-only \
            -fields [list [::mu3e::cmsis::svd::field value 0 32 -description {Raw float32 temperature sample for sensor 5.} -access read-only]]]]

    return [::mu3e::cmsis::svd::device MU3E_ONEWIRE_MASTER_CONTROLLER \
        -version 24.0.918 \
        -description {CMSIS-SVD description of the onewire_master_controller CSR aperture. BaseAddress is 0 because this file describes the relative CSR aperture of the IP; system integration supplies the live slave base address.} \
        -peripherals [list \
            [::mu3e::cmsis::svd::peripheral ONEWIRE_MASTER_CONTROLLER_CSR 0x0 \
                -description {Relative 8-word 1-Wire controller CSR aperture.} \
                -groupName MU3E_ONEWIRE \
                -addressBlockSize 0x20 \
                -registers $registers]]]
}

if {[info exists ::argv0] &&
    [file normalize $::argv0] eq [file normalize [info script]]} {
    set out_path [file join $script_dir onewire_master_controller.svd]
    if {[llength $::argv] >= 1} {
        set out_path [lindex $::argv 0]
    }
    ::mu3e::cmsis::svd::write_device_file \
        [::mu3e::cmsis::spec::build_device] $out_path
}
