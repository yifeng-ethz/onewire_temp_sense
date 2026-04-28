    task automatic check_invalid_master_descriptor(
        input string case_id,
        input logic [31:0] bad_desc,
        input string check_label
    );
        int start_errors;
        logic [31:0] desc_before;
        logic [31:0] desc_after;
        logic [31:0] status_word;

        begin_case(case_id, start_errors);
        master_csr_write(MASTER_CSR_DESC, master_desc(1'b1, 1'b0, 1'b1, 1'b1, 8, 2));
        master_csr_read(MASTER_CSR_DESC, desc_before);
        master_csr_write(MASTER_CSR_DESC, bad_desc);
        master_csr_read(MASTER_CSR_DESC, desc_after);
        master_csr_read(MASTER_CSR_STATUS, status_word);
        expect_eq({check_label, " descriptor hold"}, desc_after[23:8], desc_before[23:8]);
        expect_true({check_label, " should latch out_of_range"}, status_word[8] === 1'b1);
        if (error_count == start_errors) begin
            feature_pass("ERROR.master_csr_illegal_access");
        end
        end_case(case_id, start_errors);
    endtask

    task automatic run_x001();
        check_invalid_master_descriptor("X001", master_desc(1'b1, 1'b0, 1'b1, 1'b1, 17, 0), "tot_bytes=17");
    endtask

    task automatic run_x002();
        check_invalid_master_descriptor("X002", master_desc(1'b1, 1'b0, 1'b1, 1'b1, 255, 0), "tot_bytes=255");
    endtask

    task automatic run_x003();
        check_invalid_master_descriptor("X003", master_desc(1'b1, 1'b0, 1'b1, 1'b1, 4, 6), "wire_id=6");
    endtask

    task automatic run_x004();
        check_invalid_master_descriptor("X004", master_desc(1'b1, 1'b0, 1'b1, 1'b1, 4, 255), "wire_id=255");
    endtask

    task automatic run_x005();
        check_invalid_master_descriptor("X005", master_desc(1'b1, 1'b0, 1'b1, 1'b1, 17, 6), "tot_bytes=17 wire_id=6");
    endtask

    task automatic run_x006();
        int start_errors;
        logic [31:0] desc_before;
        logic [31:0] desc_after;
        logic [31:0] status_before;
        logic [31:0] status_after;

        begin_case("X006", start_errors);
        master_csr_write(MASTER_CSR_DESC, master_desc(1'b0, 1'b0, 1'b1, 1'b1, 1, 0));
        master_csr_read(MASTER_CSR_DESC, desc_before);
        master_csr_write(MASTER_CSR_STATUS, 32'h0000_0001);
        repeat (1) @(posedge clk);
        master_csr_read(MASTER_CSR_STATUS, status_before);
        expect_true("X006 busy precondition", status_before[0] === 1'b1);
        master_csr_write(MASTER_CSR_DESC, master_desc(1'b1, 1'b1, 1'b1, 1'b1, 8, 5));
        master_wait_busy(1'b0, 5000, status_after);
        master_csr_read(MASTER_CSR_DESC, desc_after);
        expect_eq("busy descriptor valid write should be refused", desc_after & 32'h00ff_ff0f, desc_before & 32'h00ff_ff0f);
        expect_true("busy descriptor write should not raise out_of_range", status_after[8] === status_before[8]);
        if (error_count == start_errors) begin
            feature_pass("ERROR.master_csr_illegal_access");
        end
        end_case("X006", start_errors);
    endtask

    task automatic run_x007();
        int start_errors;
        int irq_count;
        bit prev_irq;
        logic [31:0] status_word;

        begin_case("X007", start_errors);
        irq_count = 0;
        prev_irq = 1'b0;
        master_csr_write(MASTER_CSR_DESC, master_desc(1'b0, 1'b0, 1'b1, 1'b1, 1, 0));
        master_csr_write(MASTER_CSR_STATUS, 32'h0000_0001);
        repeat (2) @(posedge clk);
        master_csr_write(MASTER_CSR_STATUS, 32'h0000_0001);
        for (int cycle_idx = 0; cycle_idx < 5000; cycle_idx++) begin
            @(posedge clk);
            if (m_complete_irq && !prev_irq) begin
                irq_count++;
            end
            prev_irq = m_complete_irq;
            if (m_complete_irq) begin
                break;
            end
        end
        expect_eq("back-to-back commit should produce one pending IRQ", irq_count, 1);
        master_csr_read(MASTER_CSR_STATUS, status_word);
        @(posedge clk);
        #1;
        expect_true("IRQ clear after back-to-back commit status read", m_complete_irq === 1'b0);
        if (error_count == start_errors) begin
            feature_pass("ERROR.master_csr_illegal_access");
        end
        end_case("X007", start_errors);
    endtask

    task automatic run_x008();
        int start_errors;
        logic [31:0] read_data;
        begin_case("X008", start_errors);
        master_csr_read(4'd5, read_data);
        expect_eq("undefined addr5 read", read_data, 32'h0000_0000);
        if (error_count == start_errors) begin
            feature_pass("ERROR.master_csr_illegal_access");
        end
        end_case("X008", start_errors);
    endtask

    task automatic run_x009();
        int start_errors;
        logic [31:0] read_data;
        begin_case("X009", start_errors);
        master_csr_read(4'd15, read_data);
        expect_eq("undefined addr15 read", read_data, 32'h0000_0000);
        if (error_count == start_errors) begin
            feature_pass("ERROR.master_csr_illegal_access");
        end
        end_case("X009", start_errors);
    endtask

    task automatic run_x010();
        int start_errors;
        logic [31:0] read_data;
        begin_case("X010", start_errors);
        master_drain_rx_fifo(32);
        master_csr_read(4'd3, read_data);
        expect_eq("empty RX bypass read data", read_data, 32'h0000_0000);
        if (error_count == start_errors) begin
            feature_pass("ERROR.master_csr_illegal_access");
        end
        end_case("X010", start_errors);
    endtask

    task automatic run_x011();
        int start_errors;
        logic [31:0] fill_word;

        begin_case("X011", start_errors);
        for (int idx = 0; idx < 8; idx++) begin
            master_csr_write(4'd4, {21'd0, 3'd0, idx[7:0]});
        end
        repeat (20) @(posedge clk);
        master_csr_read(MASTER_CSR_FIFO_FILL, fill_word);
        expect_eq("tx fifo auto-flush after full", fill_word[7:0], 32'h0000_0000);
        if (error_count == start_errors) begin
            feature_pass("ERROR.master_csr_illegal_access");
        end
        end_case("X011", start_errors);
    endtask

    task automatic run_x012();
        int start_errors;
        logic [31:0] desc_word;

        begin_case("X012", start_errors);
        master_csr_write(MASTER_CSR_DESC, master_desc(1'b1, 1'b0, 1'b0, 1'b1, 4, 1));
        master_csr_read(MASTER_CSR_DESC, desc_word);
        expect_true("parasitic generic should force bit2 high", desc_word[2] === 1'b1);
        if (error_count == start_errors) begin
            feature_pass("ERROR.master_csr_illegal_access");
        end
        end_case("X012", start_errors);
    endtask

    task automatic run_x121();
        int start_errors;
        logic [31:0] uid_before;
        logic [31:0] uid_after;
        logic [31:0] cap_before;
        logic [31:0] cap_after;
        begin_case("X121", start_errors);
        csr_read(CSR_UID, uid_before);
        csr_read(CSR_CAPABILITY, cap_before);
        csr_write(CSR_UID, 32'hffff_ffff);
        csr_write(CSR_CAPABILITY, 32'hffff_ffff);
        csr_read(CSR_UID, uid_after);
        csr_read(CSR_CAPABILITY, cap_after);
        expect_eq("UID write no-op", uid_after, uid_before);
        expect_eq("CAPABILITY write no-op", cap_after, cap_before);
        if (error_count == start_errors) begin
            feature_pass("ERROR.read_only_csr_noop");
        end
        end_case("X121", start_errors);
    endtask

    task automatic run_x122();
        int start_errors;
        logic [31:0] before_word;
        logic [31:0] after_word;
        begin_case("X122", start_errors);
        csr_read(CSR_TEMP0, before_word);
        csr_write(CSR_TEMP0, 32'hffff_ffff);
        csr_read(CSR_TEMP0, after_word);
        expect_eq("SENSOR0 write no-op", after_word, before_word);
        if (error_count == start_errors) begin
            feature_pass("ERROR.temperature_write_noop");
        end
        end_case("X122", start_errors);
    endtask

    task automatic run_x123();
        int start_errors;
        logic [31:0] status_word;
        begin_case("X123", start_errors);
        status_write(255, 1'b0);
        csr_read(CSR_STATUS, status_word);
        expect_eq("invalid sel_line echo", status_word[15:0], 16'd255);
        expect_true("invalid sel_line processor_go should read zero", status_word[16] === 1'b0);
        expect_true("invalid sel_line status bits should read zero", status_word[26:24] == 3'b000);
        if (error_count == start_errors) begin
            feature_pass("ERROR.invalid_sel_line_status_zero");
        end
        end_case("X123", start_errors);
    endtask

    task automatic run_x124();
        int start_errors;
        logic [31:0] status_word;
        logic prior_go;
        begin_case("X124", start_errors);
        read_selected_status(0, 1'b0, status_word);
        prior_go = status_word[16];
        status_write(255, 1'b1);
        csr_read(CSR_STATUS, status_word);
        expect_true("invalid sel_line go write should not report active line", status_word[16] === 1'b0);
        read_selected_status(0, prior_go, status_word);
        expect_true("invalid sel_line go write changed line0", status_word[16] === prior_go);
        if (error_count == start_errors) begin
            feature_pass("ERROR.invalid_sel_line_go_suppressed");
        end
        end_case("X124", start_errors);
    endtask

    task automatic run_x125();
        int start_errors;
        logic [31:0] read_data;
        logic [3:0] temp_addr;
        begin_case("X125", start_errors);
        temp_addr = CSR_TEMP0 + N_DQ_LINES;
        csr_read(temp_addr, read_data);
        expect_eq("out-of-range temp slot", read_data, 32'h0000_0000);
        if (error_count == start_errors) begin
            feature_pass("ERROR.unmapped_temperature_slot_zero");
        end
        end_case("X125", start_errors);
    endtask
