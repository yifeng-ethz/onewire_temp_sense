`timescale 1ns/1ps

// DS18B20/1-Wire physical-line model scope:
// This is a first-order RTL-simulation abstraction for FEB OneWire DV. It
// models open-drain low drive and RC pull-up crossing of the digital VIH
// threshold. It does not model analog pad impedance, leakage, supply droop,
// reflections, distributed cable effects, temperature-dependent thresholds,
// parasite-power reservoir behavior, or full SPICE/SystemC-AMS waveforms from
// the DS18B20 datasheet. Use it to catch read-slot and pull-up/capacitance
// margin bugs, not as a complete electrical signoff model.

module onewire_rc_line_model #(
    parameter real VPU_V = 3.3,
    parameter real VIL_MAX_V = 0.8,
    parameter real VIH_MIN_V = 2.2,
    parameter real R_PULLUP_OHM = 4700.0,
    parameter real R_STRONG_PULLUP_OHM = 100.0,
    parameter real C_BUS_PF = 125.0,
    parameter bit DEBUG = 1'b0
) (
    input logic master_drive_low,
    input logic master_drive_high,
    input logic sensor_drive_low,
    output logic dq_logic,
    output real dq_voltage_v
);
    int unsigned rc_generation;

    function automatic real rise_delay_ns(input real target_v, input real resistance_ohm);
        real capacitance_f;
        real ratio;
        begin
            capacitance_f = C_BUS_PF * 1.0e-12;
            if (target_v <= 0.0) begin
                rise_delay_ns = 0.0;
            end else if (target_v >= VPU_V) begin
                rise_delay_ns = 1.0e12;
            end else begin
                ratio = 1.0 - (target_v / VPU_V);
                rise_delay_ns = -resistance_ohm * capacitance_f * $ln(ratio) * 1.0e9;
            end
        end
    endfunction

    task automatic schedule_release(input real resistance_ohm);
        int unsigned release_generation;
        real vih_delay_ns;
        begin
            rc_generation++;
            release_generation = rc_generation;
            vih_delay_ns = rise_delay_ns(VIH_MIN_V, resistance_ohm);
            dq_voltage_v = 0.0;
            dq_logic = 1'b0;
            if (DEBUG) begin
                $display(
                    "%0t ONEWIRE_RC_RELEASE r=%0.1f c=%0.1fpF vih_delay_ns=%0.3f",
                    $time,
                    resistance_ohm,
                    C_BUS_PF,
                    vih_delay_ns
                );
            end
            fork
                begin
                    #(vih_delay_ns);
                    if (release_generation == rc_generation &&
                        master_drive_low !== 1'b1 &&
                        sensor_drive_low !== 1'b1) begin
                        dq_voltage_v = VPU_V;
                        dq_logic = 1'b1;
                    end
                end
            join_none
        end
    endtask

    initial begin
        rc_generation = 0;
        dq_voltage_v = VPU_V;
        dq_logic = 1'b1;
    end

    always @(master_drive_low or master_drive_high or sensor_drive_low) begin
        if (master_drive_low === 1'b1 || sensor_drive_low === 1'b1) begin
            rc_generation++;
            dq_voltage_v = 0.0;
            dq_logic = 1'b0;
        end else if (master_drive_high === 1'b1) begin
            schedule_release(R_STRONG_PULLUP_OHM);
        end else begin
            schedule_release(R_PULLUP_OHM);
        end
    end
endmodule
