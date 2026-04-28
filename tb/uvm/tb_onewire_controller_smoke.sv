`timescale 1ns/1ps

module tb_onewire_controller_smoke;
    localparam int N_DQ_LINES = 6;
    localparam int AVST_CHANNEL_WIDTH = 3;
    localparam int REF_CLOCK_RATE = 1_000_000;

    localparam logic [3:0] CSR_UID        = 4'd0;
    localparam logic [3:0] CSR_META       = 4'd1;
    localparam logic [3:0] CSR_SCRATCH    = 4'd2;
    localparam logic [3:0] CSR_CAPABILITY = 4'd3;
    localparam logic [3:0] CSR_STATUS     = 4'd4;
    localparam logic [3:0] CSR_TEMP0      = 4'd5;

    localparam logic [3:0] MASTER_CSR_STATUS    = 4'd0;
    localparam logic [3:0] MASTER_CSR_DESC      = 4'd1;
    localparam logic [3:0] MASTER_CSR_FIFO_FILL = 4'd2;

    localparam logic [31:0] EXPECTED_UID          = 32'h4f57_4d43; // OWMC
    localparam logic [31:0] EXPECTED_META_VERSION = 32'h1a02_11ac; // 26.2.1.0428
    localparam logic [31:0] EXPECTED_META_DATE    = 32'd20260428;
    localparam logic [31:0] EXPECTED_META_GIT     = 32'd0;
    localparam logic [31:0] EXPECTED_META_INST    = 32'd0;

    logic clk;
    logic reset;

    logic avm_ctrl_read;
    logic [31:0] avm_ctrl_readdata;
    logic avm_ctrl_write;
    logic [31:0] avm_ctrl_writedata;
    logic [3:0] avm_ctrl_address;
    logic avm_ctrl_waitrequest;

    logic avs_csr_read;
    logic [31:0] avs_csr_readdata;
    logic avs_csr_write;
    logic [31:0] avs_csr_writedata;
    logic [3:0] avs_csr_address;
    logic avs_csr_waitrequest;

    logic [7:0] asi_rx_data;
    logic asi_rx_valid;
    logic asi_rx_ready;
    logic [AVST_CHANNEL_WIDTH-1:0] asi_rx_channel;

    logic [7:0] aso_tx_data;
    logic aso_tx_valid;
    logic aso_tx_ready;
    logic [AVST_CHANNEL_WIDTH-1:0] aso_tx_channel;

    logic complete_irq;

    logic [N_DQ_LINES-1:0] dq_in;
    logic [N_DQ_LINES-1:0] dq_out;
    logic [N_DQ_LINES-1:0] dq_oe;
    logic [N_DQ_LINES-1:0] sensor_present;
    logic [N_DQ_LINES-1:0] sensor_drive_low;
    real dq_voltage_v[N_DQ_LINES];

    logic m_ctrl_read;
    logic [31:0] m_ctrl_readdata;
    logic m_ctrl_write;
    logic [31:0] m_ctrl_writedata;
    logic [3:0] m_ctrl_address;
    logic m_ctrl_waitrequest;
    logic [7:0] m_rx_data;
    logic m_rx_valid;
    logic m_rx_ready;
    logic [AVST_CHANNEL_WIDTH-1:0] m_rx_channel;
    logic [7:0] m_tx_data;
    logic m_tx_valid;
    logic m_tx_ready;
    logic [AVST_CHANNEL_WIDTH-1:0] m_tx_channel;
    logic m_complete_irq;
    logic [N_DQ_LINES-1:0] m_dq_in;
    logic [N_DQ_LINES-1:0] m_dq_out;
    logic [N_DQ_LINES-1:0] m_dq_oe;
    logic [N_DQ_LINES-1:0] master_sensor_present;
    logic [N_DQ_LINES-1:0] master_sensor_drive_low;
    real m_dq_voltage_v[N_DQ_LINES];

    logic mnp_ctrl_read;
    logic [31:0] mnp_ctrl_readdata;
    logic mnp_ctrl_write;
    logic [31:0] mnp_ctrl_writedata;
    logic [3:0] mnp_ctrl_address;
    logic mnp_ctrl_waitrequest;
    logic [7:0] mnp_rx_data;
    logic mnp_rx_valid;
    logic mnp_rx_ready;
    logic [AVST_CHANNEL_WIDTH-1:0] mnp_rx_channel;
    logic [7:0] mnp_tx_data;
    logic mnp_tx_valid;
    logic mnp_tx_ready;
    logic [AVST_CHANNEL_WIDTH-1:0] mnp_tx_channel;
    logic mnp_complete_irq;
    logic [N_DQ_LINES-1:0] mnp_dq_in;
    logic [N_DQ_LINES-1:0] mnp_dq_out;
    logic [N_DQ_LINES-1:0] mnp_dq_oe;
    logic [N_DQ_LINES-1:0] mnp_sensor_present;
    logic [N_DQ_LINES-1:0] mnp_sensor_drive_low;
    real mnp_dq_voltage_v[N_DQ_LINES];

    string ow_case;
    string ow_mode;
    int error_count;

    genvar dq_idx;
    generate
        for (dq_idx = 0; dq_idx < N_DQ_LINES; dq_idx++) begin : gen_dq
            localparam int signed SENSOR_TEMP_C_X16 = (dq_idx == 0) ? 0 : ((22 + dq_idx) * 16 + dq_idx);

            onewire_rc_line_model controller_line (
                .master_drive_low(dq_oe[dq_idx] && !dq_out[dq_idx]),
                .master_drive_high(dq_oe[dq_idx] && dq_out[dq_idx]),
                .sensor_drive_low(sensor_drive_low[dq_idx]),
                .dq_logic(dq_in[dq_idx]),
                .dq_voltage_v(dq_voltage_v[dq_idx])
            );

            ds18b20_1wire_model #(
                .ROM_CODE(64'h2800000000000100 + dq_idx),
                .TEMP_C_X16(SENSOR_TEMP_C_X16)
            ) sensor (
                .dq_in(dq_in[dq_idx]),
                .dq_drive_low(sensor_drive_low[dq_idx]),
                .present_enable(sensor_present[dq_idx])
            );

            onewire_rc_line_model direct_line (
                .master_drive_low(m_dq_oe[dq_idx] && !m_dq_out[dq_idx]),
                .master_drive_high(m_dq_oe[dq_idx] && m_dq_out[dq_idx]),
                .sensor_drive_low(master_sensor_drive_low[dq_idx]),
                .dq_logic(m_dq_in[dq_idx]),
                .dq_voltage_v(m_dq_voltage_v[dq_idx])
            );

            ds18b20_1wire_model #(
                .ROM_CODE(64'h2800000000000200 + dq_idx),
                .TEMP_C_X16(SENSOR_TEMP_C_X16)
            ) master_sensor (
                .dq_in(m_dq_in[dq_idx]),
                .dq_drive_low(master_sensor_drive_low[dq_idx]),
                .present_enable(master_sensor_present[dq_idx])
            );

            onewire_rc_line_model no_parasitic_line (
                .master_drive_low(mnp_dq_oe[dq_idx] && !mnp_dq_out[dq_idx]),
                .master_drive_high(mnp_dq_oe[dq_idx] && mnp_dq_out[dq_idx]),
                .sensor_drive_low(mnp_sensor_drive_low[dq_idx]),
                .dq_logic(mnp_dq_in[dq_idx]),
                .dq_voltage_v(mnp_dq_voltage_v[dq_idx])
            );

            ds18b20_1wire_model #(
                .ROM_CODE(64'h2800000000000300 + dq_idx),
                .TEMP_C_X16(SENSOR_TEMP_C_X16)
            ) master_np_sensor (
                .dq_in(mnp_dq_in[dq_idx]),
                .dq_drive_low(mnp_sensor_drive_low[dq_idx]),
                .present_enable(mnp_sensor_present[dq_idx])
            );
        end
    endgenerate

    initial begin
        clk = 1'b0;
        forever #500 clk = ~clk;
    end

    onewire_master #(
        .REF_CLOCK_RATE(REF_CLOCK_RATE),
        .N_DQ_LINES(N_DQ_LINES),
        .AVST_CHANNEL_WIDTH(AVST_CHANNEL_WIDTH),
        .PARACITIC_POWERING(1'b1),
        .DEBUG_LV(0)
    ) u_master (
        .avs_ctrl_read(avm_ctrl_read),
        .avs_ctrl_readdata(avm_ctrl_readdata),
        .avs_ctrl_write(avm_ctrl_write),
        .avs_ctrl_writedata(avm_ctrl_writedata),
        .avs_ctrl_address(avm_ctrl_address),
        .avs_ctrl_waitrequest(avm_ctrl_waitrequest),
        .aso_rx_data(asi_rx_data),
        .aso_rx_valid(asi_rx_valid),
        .aso_rx_ready(asi_rx_ready),
        .aso_rx_channel(asi_rx_channel),
        .asi_tx_data(aso_tx_data),
        .asi_tx_valid(aso_tx_valid),
        .asi_tx_ready(aso_tx_ready),
        .asi_tx_channel(aso_tx_channel),
        .ins_complete_irq(complete_irq),
        .coe_sense_dq_in(dq_in),
        .coe_sense_dq_out(dq_out),
        .coe_sense_dq_oe(dq_oe),
        .rsi_reset_reset(reset),
        .csi_clock_clk(clk)
    );

    onewire_master_controller #(
        .REF_CLOCK_RATE(REF_CLOCK_RATE),
        .N_DQ_LINES(N_DQ_LINES),
        .AVST_CHANNEL_WIDTH(AVST_CHANNEL_WIDTH),
        .DEBUG_LV(0)
    ) u_controller (
        .avm_ctrl_read(avm_ctrl_read),
        .avm_ctrl_readdata(avm_ctrl_readdata),
        .avm_ctrl_write(avm_ctrl_write),
        .avm_ctrl_writedata(avm_ctrl_writedata),
        .avm_ctrl_address(avm_ctrl_address),
        .avm_ctrl_waitrequest(avm_ctrl_waitrequest),
        .avs_csr_read(avs_csr_read),
        .avs_csr_readdata(avs_csr_readdata),
        .avs_csr_write(avs_csr_write),
        .avs_csr_writedata(avs_csr_writedata),
        .avs_csr_address(avs_csr_address),
        .avs_csr_waitrequest(avs_csr_waitrequest),
        .asi_rx_data(asi_rx_data),
        .asi_rx_valid(asi_rx_valid),
        .asi_rx_ready(asi_rx_ready),
        .asi_rx_channel(asi_rx_channel),
        .aso_tx_data(aso_tx_data),
        .aso_tx_valid(aso_tx_valid),
        .aso_tx_ready(aso_tx_ready),
        .aso_tx_channel(aso_tx_channel),
        .inr_complete_irq(complete_irq),
        .rsi_reset_reset(reset),
        .csi_clock_clk(clk)
    );

    onewire_master #(
        .REF_CLOCK_RATE(REF_CLOCK_RATE),
        .N_DQ_LINES(N_DQ_LINES),
        .AVST_CHANNEL_WIDTH(AVST_CHANNEL_WIDTH),
        .PARACITIC_POWERING(1'b1),
        .DEBUG_LV(0)
    ) u_master_direct (
        .avs_ctrl_read(m_ctrl_read),
        .avs_ctrl_readdata(m_ctrl_readdata),
        .avs_ctrl_write(m_ctrl_write),
        .avs_ctrl_writedata(m_ctrl_writedata),
        .avs_ctrl_address(m_ctrl_address),
        .avs_ctrl_waitrequest(m_ctrl_waitrequest),
        .aso_rx_data(m_rx_data),
        .aso_rx_valid(m_rx_valid),
        .aso_rx_ready(m_rx_ready),
        .aso_rx_channel(m_rx_channel),
        .asi_tx_data(m_tx_data),
        .asi_tx_valid(m_tx_valid),
        .asi_tx_ready(m_tx_ready),
        .asi_tx_channel(m_tx_channel),
        .ins_complete_irq(m_complete_irq),
        .coe_sense_dq_in(m_dq_in),
        .coe_sense_dq_out(m_dq_out),
        .coe_sense_dq_oe(m_dq_oe),
        .rsi_reset_reset(reset),
        .csi_clock_clk(clk)
    );

    onewire_master #(
        .REF_CLOCK_RATE(REF_CLOCK_RATE),
        .N_DQ_LINES(N_DQ_LINES),
        .AVST_CHANNEL_WIDTH(AVST_CHANNEL_WIDTH),
        .PARACITIC_POWERING(1'b0),
        .DEBUG_LV(0)
    ) u_master_no_parasitic (
        .avs_ctrl_read(mnp_ctrl_read),
        .avs_ctrl_readdata(mnp_ctrl_readdata),
        .avs_ctrl_write(mnp_ctrl_write),
        .avs_ctrl_writedata(mnp_ctrl_writedata),
        .avs_ctrl_address(mnp_ctrl_address),
        .avs_ctrl_waitrequest(mnp_ctrl_waitrequest),
        .aso_rx_data(mnp_rx_data),
        .aso_rx_valid(mnp_rx_valid),
        .aso_rx_ready(mnp_rx_ready),
        .aso_rx_channel(mnp_rx_channel),
        .asi_tx_data(mnp_tx_data),
        .asi_tx_valid(mnp_tx_valid),
        .asi_tx_ready(mnp_tx_ready),
        .asi_tx_channel(mnp_tx_channel),
        .ins_complete_irq(mnp_complete_irq),
        .coe_sense_dq_in(mnp_dq_in),
        .coe_sense_dq_out(mnp_dq_out),
        .coe_sense_dq_oe(mnp_dq_oe),
        .rsi_reset_reset(reset),
        .csi_clock_clk(clk)
    );

    task automatic reset_dut();
        avs_csr_read = 1'b0;
        avs_csr_write = 1'b0;
        avs_csr_writedata = '0;
        avs_csr_address = '0;
        m_ctrl_read = 1'b0;
        m_ctrl_write = 1'b0;
        m_ctrl_writedata = '0;
        m_ctrl_address = '0;
        m_rx_ready = 1'b0;
        m_tx_data = '0;
        m_tx_valid = 1'b0;
        m_tx_channel = '0;
        mnp_ctrl_read = 1'b0;
        mnp_ctrl_write = 1'b0;
        mnp_ctrl_writedata = '0;
        mnp_ctrl_address = '0;
        mnp_rx_ready = 1'b0;
        mnp_tx_data = '0;
        mnp_tx_valid = 1'b0;
        mnp_tx_channel = '0;
        sensor_present = {N_DQ_LINES{1'b1}};
        master_sensor_present = {N_DQ_LINES{1'b1}};
        mnp_sensor_present = {N_DQ_LINES{1'b1}};
        reset = 1'b1;
        repeat (20) @(posedge clk);
        reset = 1'b0;
        repeat (20) @(posedge clk);
    endtask

    task automatic csr_write(input logic [3:0] addr, input logic [31:0] data);
        @(posedge clk);
        avs_csr_address <= addr;
        avs_csr_writedata <= data;
        avs_csr_write <= 1'b1;
        do begin
            @(posedge clk);
        end while (avs_csr_waitrequest);
        avs_csr_write <= 1'b0;
        avs_csr_address <= '0;
        avs_csr_writedata <= '0;
    endtask

    task automatic csr_read(input logic [3:0] addr, output logic [31:0] data);
        @(posedge clk);
        avs_csr_address <= addr;
        avs_csr_read <= 1'b1;
        do begin
            @(posedge clk);
        end while (avs_csr_waitrequest);
        data = avs_csr_readdata;
        avs_csr_read <= 1'b0;
        avs_csr_address <= '0;
    endtask

    function automatic logic [31:0] master_desc(
        input bit direction,
        input bit init,
        input bit paracitic_pw,
        input bit usewire_id,
        input int unsigned tot_bytes,
        input int unsigned wire_id
    );
        logic [31:0] desc;
        desc = 32'h0000_0000;
        desc[0] = direction;
        desc[1] = init;
        desc[2] = paracitic_pw;
        desc[3] = usewire_id;
        desc[15:8] = tot_bytes[7:0];
        desc[23:16] = wire_id[7:0];
        return desc;
    endfunction

    task automatic master_csr_write(input logic [3:0] addr, input logic [31:0] data);
        @(posedge clk);
        m_ctrl_address <= addr;
        m_ctrl_writedata <= data;
        m_ctrl_write <= 1'b1;
        do begin
            @(posedge clk);
            #1;
        end while (m_ctrl_waitrequest);
        m_ctrl_write <= 1'b0;
        m_ctrl_address <= '0;
        m_ctrl_writedata <= '0;
    endtask

    task automatic master_csr_read(input logic [3:0] addr, output logic [31:0] data);
        @(posedge clk);
        m_ctrl_address <= addr;
        m_ctrl_read <= 1'b1;
        do begin
            @(posedge clk);
            #1;
        end while (m_ctrl_waitrequest);
        data = m_ctrl_readdata;
        m_ctrl_read <= 1'b0;
        m_ctrl_address <= '0;
    endtask

    task automatic master_np_csr_write(input logic [3:0] addr, input logic [31:0] data);
        @(posedge clk);
        mnp_ctrl_address <= addr;
        mnp_ctrl_writedata <= data;
        mnp_ctrl_write <= 1'b1;
        do begin
            @(posedge clk);
            #1;
        end while (mnp_ctrl_waitrequest);
        mnp_ctrl_write <= 1'b0;
        mnp_ctrl_address <= '0;
        mnp_ctrl_writedata <= '0;
    endtask

    task automatic master_np_csr_read(input logic [3:0] addr, output logic [31:0] data);
        @(posedge clk);
        mnp_ctrl_address <= addr;
        mnp_ctrl_read <= 1'b1;
        do begin
            @(posedge clk);
            #1;
        end while (mnp_ctrl_waitrequest);
        data = mnp_ctrl_readdata;
        mnp_ctrl_read <= 1'b0;
        mnp_ctrl_address <= '0;
    endtask

    task automatic master_read_wait_contract(input logic [3:0] addr, output logic [31:0] data);
        @(posedge clk);
        m_ctrl_address <= addr;
        m_ctrl_read <= 1'b1;
        @(posedge clk);
        #1;
        expect_true($sformatf("master read addr %0d waitrequest should deassert", addr), m_ctrl_waitrequest === 1'b0);
        data = m_ctrl_readdata;
        m_ctrl_read <= 1'b0;
        m_ctrl_address <= '0;
        @(posedge clk);
        #1;
        expect_true($sformatf("master read addr %0d waitrequest should reassert when idle", addr), m_ctrl_waitrequest === 1'b1);
    endtask

    task automatic master_wait_busy(input bit expected_busy, input int max_cycles, output logic [31:0] status_word);
        for (int cycle_idx = 0; cycle_idx < max_cycles; cycle_idx++) begin
            master_csr_read(MASTER_CSR_STATUS, status_word);
            if (status_word[0] === expected_busy) begin
                return;
            end
            repeat (2) @(posedge clk);
        end
        $error("master busy did not become %0b within %0d poll cycles", expected_busy, max_cycles);
        error_count++;
    endtask

    task automatic master_wait_irq(input int max_cycles);
        for (int cycle_idx = 0; cycle_idx < max_cycles; cycle_idx++) begin
            if (m_complete_irq === 1'b1) begin
                return;
            end
            @(posedge clk);
        end
        $error("master complete IRQ did not assert within %0d cycles", max_cycles);
        error_count++;
    endtask

    task automatic master_drain_rx_fifo(input int max_reads);
        logic [31:0] fill_word;

        for (int read_idx = 0; read_idx < max_reads; read_idx++) begin
            master_csr_read(MASTER_CSR_FIFO_FILL, fill_word);
            if (fill_word[31:16] == 16'd0) begin
                m_rx_ready <= 1'b0;
                return;
            end
            m_rx_ready <= 1'b1;
            @(posedge clk);
            #1;
            m_rx_ready <= 1'b0;
            repeat (2) @(posedge clk);
        end

        master_csr_read(MASTER_CSR_FIFO_FILL, fill_word);
        expect_eq("master rx fifo drain timeout", {16'd0, fill_word[31:16]}, 32'h0000_0000);
    endtask

    task automatic master_push_tx_bypass(input int channel, input logic [7:0] data);
        logic [31:0] write_word;
        write_word = {21'd0, channel[2:0], data};
        master_csr_write(4'd4, write_word);
    endtask

    task automatic master_expect_fifo_fill(input string check_name, input int tx_usedw, input int rx_usedw);
        logic [31:0] fill_word;
        master_csr_read(MASTER_CSR_FIFO_FILL, fill_word);
        expect_eq({check_name, " tx usedw"}, {24'd0, fill_word[7:0]}, tx_usedw);
        expect_eq({check_name, " rx usedw"}, {16'd0, fill_word[31:16]}, rx_usedw);
    endtask

    task automatic master_wait_irq_and_check_line(input int line, input int max_cycles, input bit expect_selected_low);
        int selected_low_count;
        int other_low_count;

        selected_low_count = 0;
        other_low_count = 0;
        for (int cycle_idx = 0; cycle_idx < max_cycles; cycle_idx++) begin
            @(posedge clk);
            #1;
            for (int probe_line = 0; probe_line < N_DQ_LINES; probe_line++) begin
                if (m_dq_oe[probe_line] === 1'b1 && m_dq_out[probe_line] === 1'b0) begin
                    if (probe_line == line) begin
                        selected_low_count++;
                    end else begin
                        other_low_count++;
                    end
                end
            end
            if (m_complete_irq === 1'b1) begin
                break;
            end
        end

        expect_true("master complete IRQ did not assert during line monitor", m_complete_irq === 1'b1);
        if (expect_selected_low) begin
            expect_true($sformatf("line %0d did not show a TX low pulse", line), selected_low_count > 0);
        end
        expect_eq("non-selected line low-pulse count", other_low_count[31:0], 32'h0000_0000);
    endtask

    task automatic status_write(input int line, input bit go);
        logic [31:0] write_data;
        write_data = (go ? 32'h0001_0000 : 32'h0000_0000) | (line & 32'h0000_ffff);
        csr_write(CSR_STATUS, write_data);
    endtask

    task automatic read_selected_status(input int line, input bit go, output logic [31:0] data);
        status_write(line, go);
        csr_read(CSR_STATUS, data);
    endtask

    task automatic expect_eq(input string check_name, input logic [31:0] got, input logic [31:0] expected);
        if (got !== expected) begin
            $error("%s: got 0x%08x expected 0x%08x", check_name, got, expected);
            error_count++;
        end
    endtask

    task automatic expect_true(input string check_name, input bit ok);
        if (!ok) begin
            $error("%s", check_name);
            error_count++;
        end
    endtask

    task automatic begin_case(input string case_id, output int start_errors);
        start_errors = error_count;
        $display("OW_CASE_BEGIN %s", case_id);
    endtask

    task automatic end_case(input string case_id, input int start_errors);
        if (error_count == start_errors) begin
            $display("OW_CASE_PASS %s", case_id);
        end else begin
            $display("OW_CASE_FAIL %s errors=%0d", case_id, error_count - start_errors);
        end
    endtask

    task automatic feature_pass(input string feature_id);
        $display("OW_FEATURE_PASS %s", feature_id);
    endtask

    function automatic logic [31:0] expected_temp_word(input int line);
        int temp_x16;
        real temp_c;
        shortreal temp_s;

        temp_x16 = (line == 0) ? 0 : ((22 + line) * 16) + line;
        temp_c = temp_x16 / 16.0;
        temp_s = shortreal'(temp_c);
        return $shortrealtobits(temp_s);
    endfunction

    task automatic check_common_header();
        logic [31:0] read_data;

        csr_read(CSR_UID, read_data);
        expect_eq("UID", read_data, EXPECTED_UID);

        csr_write(CSR_META, 32'd0);
        csr_read(CSR_META, read_data);
        expect_eq("META VERSION", read_data, EXPECTED_META_VERSION);

        csr_write(CSR_META, 32'd1);
        csr_read(CSR_META, read_data);
        expect_eq("META DATE", read_data, EXPECTED_META_DATE);

        csr_write(CSR_META, 32'd2);
        csr_read(CSR_META, read_data);
        expect_eq("META GIT", read_data, EXPECTED_META_GIT);

        csr_write(CSR_META, 32'd3);
        csr_read(CSR_META, read_data);
        expect_eq("META INSTANCE", read_data, EXPECTED_META_INST);

        csr_write(CSR_SCRATCH, 32'h5aa5_c33c);
        csr_read(CSR_SCRATCH, read_data);
        expect_eq("SCRATCH readback", read_data, 32'h5aa5_c33c);
    endtask

    task automatic check_capability();
        logic [31:0] read_data;

        csr_read(CSR_CAPABILITY, read_data);
        expect_true($sformatf("CAPABILITY n_dq_lines got 0x%04x", read_data[15:0]), read_data[15:0] == N_DQ_LINES);
        expect_true($sformatf("CAPABILITY n_sensors reset got 0x%04x", read_data[31:16]), read_data[31:16] == 16'd0);
    endtask

    task automatic wait_for_sample_valid(input int line, input time timeout_ns);
        logic [31:0] status_word;
        time deadline;

        deadline = $time + timeout_ns;
        while ($time < deadline) begin
            read_selected_status(line, 1'b1, status_word);
            if (status_word[26] === 1'b1) begin
                return;
            end
            repeat (5000) @(posedge clk);
        end
        $error("line %0d sample_valid timeout", line);
        error_count++;
    endtask

    task automatic check_line_temperature(input int line);
        logic [31:0] status_word;
        logic [31:0] read_data;
        logic [31:0] expected_data;
        logic [3:0] temp_addr;

        read_selected_status(line, 1'b1, status_word);
        expect_true($sformatf("line %0d processor_go did not stick: status=0x%08x", line, status_word), status_word[16] === 1'b1);
        expect_true($sformatf("line %0d error flags set: status=0x%08x", line, status_word), status_word[25:24] == 2'b00);
        expect_true($sformatf("line %0d sample_valid was not set: status=0x%08x", line, status_word), status_word[26] === 1'b1);

        temp_addr = CSR_TEMP0 + line;
        csr_read(temp_addr, read_data);
        expected_data = expected_temp_word(line);
        if (read_data == 32'h3f80_0000) begin
            $error("line %0d stale/reset-like temperature: 0x%08x", line, read_data);
            error_count++;
        end else if (read_data != expected_data) begin
            $error("line %0d temperature mismatch: got 0x%08x expected 0x%08x", line, read_data, expected_data);
            error_count++;
        end else begin
            $display("line %0d temperature word 0x%08x", line, read_data);
        end
    endtask

`include "sequence/onewire_basic_sequences.svh"
`include "sequence/onewire_error_sequences.svh"

    function automatic string canonical_case(input string case_id);
        return case_id;
    endfunction

    task automatic run_named_case(input string case_id);
        string resolved_case;

        resolved_case = canonical_case(case_id);
        if (resolved_case != case_id) begin
            $display("OW_CASE_ALIAS %s -> %s", case_id, resolved_case);
        end
        if (resolved_case == "B001") begin
            run_b001();
        end else if (resolved_case == "B002") begin
            run_b002();
        end else if (resolved_case == "B003") begin
            run_b003();
        end else if (resolved_case == "B004") begin
            run_b004();
        end else if (resolved_case == "B005") begin
            run_b005();
        end else if (resolved_case == "B006") begin
            run_b006();
        end else if (resolved_case == "B007") begin
            run_b007();
        end else if (resolved_case == "B008") begin
            run_b008();
        end else if (resolved_case == "B009") begin
            run_b009();
        end else if (resolved_case == "B010") begin
            run_b010();
        end else if (resolved_case == "B011") begin
            run_b011();
        end else if (resolved_case == "B012") begin
            run_b012();
        end else if (resolved_case == "B013") begin
            run_b013();
        end else if (resolved_case == "B014") begin
            run_b014();
        end else if (resolved_case == "B015") begin
            run_b015();
        end else if (resolved_case == "B016") begin
            run_b016();
        end else if (resolved_case == "B017") begin
            run_b017();
        end else if (resolved_case == "B018") begin
            run_b018();
        end else if (resolved_case == "B019") begin
            run_b019();
        end else if (resolved_case == "B020") begin
            run_b020();
        end else if (resolved_case == "B021") begin
            run_b021();
        end else if (resolved_case == "B022") begin
            run_b022();
        end else if (resolved_case == "B023") begin
            run_b023();
        end else if (resolved_case == "B024") begin
            run_b024();
        end else if (resolved_case == "B025") begin
            run_b025();
        end else if (resolved_case == "B026") begin
            run_b026();
        end else if (resolved_case == "B027") begin
            run_b027();
        end else if (resolved_case == "B028") begin
            run_b028();
        end else if (resolved_case == "B029") begin
            run_b029();
        end else if (resolved_case == "B030") begin
            run_b030();
        end else if (resolved_case == "B031") begin
            run_b031();
        end else if (resolved_case == "B032") begin
            run_b032();
        end else if (resolved_case == "B033") begin
            run_b033();
        end else if (resolved_case == "B121") begin
            run_b121();
        end else if (resolved_case == "B122") begin
            run_b122();
        end else if (resolved_case == "B123") begin
            run_b123();
        end else if (resolved_case == "B124") begin
            run_b124();
        end else if (resolved_case == "B125") begin
            run_b125();
        end else if (resolved_case == "X001") begin
            run_x001();
        end else if (resolved_case == "X002") begin
            run_x002();
        end else if (resolved_case == "X003") begin
            run_x003();
        end else if (resolved_case == "X004") begin
            run_x004();
        end else if (resolved_case == "X005") begin
            run_x005();
        end else if (resolved_case == "X006") begin
            run_x006();
        end else if (resolved_case == "X007") begin
            run_x007();
        end else if (resolved_case == "X008") begin
            run_x008();
        end else if (resolved_case == "X009") begin
            run_x009();
        end else if (resolved_case == "X010") begin
            run_x010();
        end else if (resolved_case == "X011") begin
            run_x011();
        end else if (resolved_case == "X012") begin
            run_x012();
        end else if (resolved_case == "X121") begin
            run_x121();
        end else if (resolved_case == "X122") begin
            run_x122();
        end else if (resolved_case == "X123") begin
            run_x123();
        end else if (resolved_case == "X124") begin
            run_x124();
        end else if (resolved_case == "X125") begin
            run_x125();
        end else begin
            $error("unknown OW_CASE '%s'", resolved_case);
            error_count++;
        end
    endtask

    task automatic run_basic_frame();
        run_b003();
        run_b004();
        run_b005();
        run_b006();
        run_b007();
        run_b008();
        run_b009();
        run_b010();
        run_b011();
        run_b012();
        run_b013();
        run_b014();
        run_b015();
        run_b016();
        run_b017();
        run_b018();
        run_b019();
        run_b020();
        run_b021();
        run_b022();
        run_b023();
        run_b024();
        run_b025();
        run_b026();
        run_b027();
        run_b028();
        run_b029();
        run_b030();
        run_b031();
        run_b032();
        run_b033();
        run_b121();
        run_b122();
        run_b123();
        run_b124();
        run_b125();
        run_b001();
        run_b002();
    endtask

    task automatic run_error_frame();
        run_x001();
        run_x002();
        run_x003();
        run_x004();
        run_x005();
        run_x006();
        run_x007();
        run_x008();
        run_x009();
        run_x010();
        run_x011();
        run_x012();
        run_x121();
        run_x122();
        run_x123();
        run_x124();
        run_x125();
    endtask

    initial begin
        error_count = 0;
        if (!$value$plusargs("OW_CASE=%s", ow_case)) begin
            ow_case = "OW_SMOKE";
        end
        if (!$value$plusargs("OW_MODE=%s", ow_mode)) begin
            ow_mode = "isolated";
        end

        reset_dut();
        $display("OW_RUN_BEGIN case=%s mode=%s", ow_case, ow_mode);

        if (ow_case == "OW_SMOKE") begin
            run_basic_frame();
        end else if (ow_case == "BASIC" || ow_case == "BASIC_FRAME") begin
            run_basic_frame();
        end else if (ow_case == "ERROR" || ow_case == "ERROR_FRAME") begin
            run_error_frame();
        end else if (ow_case == "ALL" || ow_case == "ALL_BUCKETS_FRAME") begin
            run_basic_frame();
            run_error_frame();
        end else begin
            run_named_case(ow_case);
        end

        if (error_count == 0) begin
            $display("ONEWIRE_CONTROLLER_SMOKE_PASS");
        end else begin
            $fatal(1, "ONEWIRE_CONTROLLER_SMOKE_FAIL errors=%0d", error_count);
        end
        $finish;
    end
endmodule
