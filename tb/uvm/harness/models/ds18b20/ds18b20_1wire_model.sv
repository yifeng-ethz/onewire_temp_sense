`timescale 1ns/1ps

module ds18b20_1wire_model #(
    parameter longint unsigned ROM_CODE = 64'h2800000000000001,
    parameter int signed TEMP_C_X16 = 25 * 16,
    parameter int CONVERT_TIME_US = 1000,
    parameter bit DEBUG = 1'b0
) (
    inout tri1 dq,
    input logic present_enable
);
    localparam byte CMD_READ_ROM        = 8'h33;
    localparam byte CMD_SKIP_ROM        = 8'hcc;
    localparam byte CMD_CONVERT_T       = 8'h44;
    localparam byte CMD_READ_SCRATCHPAD = 8'hbe;

    bit drive_low;
    bit conversion_done;
    byte scratchpad[9];

    assign dq = drive_low ? 1'b0 : 1'bz;

    function automatic byte crc8_byte(input byte crc_in, input byte data_in);
        byte crc;
        byte data;
        crc = crc_in;
        data = data_in;
        for (int bit_idx = 0; bit_idx < 8; bit_idx++) begin
            if ((crc[0] ^ data[0]) == 1'b1) begin
                crc = (crc >> 1) ^ 8'h8c;
            end else begin
                crc = crc >> 1;
            end
            data = data >> 1;
        end
        return crc;
    endfunction

    function automatic byte crc8_scratchpad();
        byte crc;
        crc = 8'h00;
        for (int idx = 0; idx < 8; idx++) begin
            crc = crc8_byte(crc, scratchpad[idx]);
        end
        return crc;
    endfunction

    function automatic byte rom_byte(input int idx);
        return byte'((ROM_CODE >> (8 * idx)) & 64'hff);
    endfunction

    task automatic refresh_scratchpad();
        shortint signed raw_temp;
        raw_temp = shortint'(TEMP_C_X16);
        scratchpad[0] = byte'(raw_temp[7:0]);
        scratchpad[1] = byte'(raw_temp[15:8]);
        scratchpad[2] = 8'h4b;
        scratchpad[3] = 8'h46;
        scratchpad[4] = 8'h7f;
        scratchpad[5] = 8'hff;
        scratchpad[6] = 8'h0c;
        scratchpad[7] = 8'h10;
        scratchpad[8] = crc8_scratchpad();
    endtask

    task automatic wait_reset(output bit got_reset);
        realtime low_start;
        realtime low_width;
        got_reset = 1'b0;
        @(negedge dq);
        low_start = $realtime;
        @(posedge dq);
        low_width = $realtime - low_start;
        if (low_width >= 480_000.0) begin
            got_reset = 1'b1;
        end
    endtask

    task automatic send_presence();
        #(30_000);
        drive_low = 1'b1;
        #(120_000);
        drive_low = 1'b0;
    endtask

    task automatic read_master_byte(output byte value, output bit got_reset);
        value = 8'h00;
        got_reset = 1'b0;
        for (int bit_idx = 0; bit_idx < 8; bit_idx++) begin
            realtime low_start;
            realtime low_width;
            @(negedge dq);
            low_start = $realtime;
            #(15_000);
            value[bit_idx] = dq;
            if (DEBUG) begin
                $display("%0t DS18B20 rom=0x%016x rx_bit[%0d]=%0b", $time, ROM_CODE, bit_idx, dq);
            end
            if (dq === 1'b0) begin
                @(posedge dq);
                low_width = $realtime - low_start;
                if (low_width >= 480_000.0) begin
                    got_reset = 1'b1;
                    return;
                end
                #(2_000);
            end else begin
                #(80_000);
            end
        end
    endtask

    task automatic write_slave_byte(input byte value);
        for (int bit_idx = 0; bit_idx < 8; bit_idx++) begin
            @(negedge dq);
            if (value[bit_idx] == 1'b0) begin
                #(2_000);
                drive_low = 1'b1;
                #(58_000);
                drive_low = 1'b0;
            end else begin
                #(60_000);
            end
            #(2_000);
        end
    endtask

    task automatic convert_temperature();
        conversion_done = 1'b0;
        #(CONVERT_TIME_US * 1000);
        refresh_scratchpad();
        conversion_done = 1'b1;
    endtask

    initial begin
        byte cmd;
        bit got_reset;

        drive_low = 1'b0;
        conversion_done = 1'b1;
        refresh_scratchpad();

        forever begin
            wait_reset(got_reset);
            if (!got_reset) begin
                continue;
            end
            if (!present_enable) begin
                drive_low = 1'b0;
                continue;
            end
            send_presence();

            forever begin
                if (!present_enable) begin
                    drive_low = 1'b0;
                    break;
                end
                read_master_byte(cmd, got_reset);
                if (got_reset) begin
                    if (present_enable) begin
                        send_presence();
                    end
                    continue;
                end
                if (DEBUG) begin
                    $display("%0t DS18B20 rom=0x%016x cmd=0x%02x", $time, ROM_CODE, cmd);
                end

                unique case (cmd)
                    CMD_SKIP_ROM: begin
                    end
                    CMD_CONVERT_T: begin
                        fork
                            convert_temperature();
                        join_none
                    end
                    CMD_READ_SCRATCHPAD: begin
                        if (!conversion_done) begin
                            wait (conversion_done);
                        end
                        for (int idx = 0; idx < 9; idx++) begin
                            write_slave_byte(scratchpad[idx]);
                        end
                    end
                    CMD_READ_ROM: begin
                        for (int idx = 0; idx < 8; idx++) begin
                            write_slave_byte(rom_byte(idx));
                        end
                    end
                    default: begin
                    end
                endcase
            end
        end
    end
endmodule
