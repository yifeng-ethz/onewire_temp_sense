    task automatic run_b001();
        int start_errors;
        begin_case("B001", start_errors);
        status_write(0, 1'b1);
        wait_for_sample_valid(0, 1_300_000_000ns);
        check_line_temperature(0);
        if (error_count == start_errors) begin
            feature_pass("BASIC.zero_temperature_guard");
            feature_pass("BASIC.sample_valid_status");
            feature_pass("BASIC.temperature_float32_readback");
        end
        end_case("B001", start_errors);
    endtask

    task automatic run_b002();
        int start_errors;
        begin_case("B002", start_errors);
        for (int line = 0; line < N_DQ_LINES; line++) begin
            status_write(line, 1'b1);
        end
        #(1_250_000_000);
        for (int line = 0; line < N_DQ_LINES; line++) begin
            check_line_temperature(line);
        end
        if (error_count == start_errors) begin
            feature_pass("BASIC.six_line_temperature_loop");
            feature_pass("BASIC.per_line_status_select");
        end
        end_case("B002", start_errors);
    endtask

    task automatic run_b003();
        int start_errors;
        begin_case("B003", start_errors);
        check_common_header();
        check_capability();
        if (error_count == start_errors) begin
            feature_pass("BASIC.common_uid_meta_header");
            feature_pass("BASIC.csr_scratchpad");
            feature_pass("BASIC.capability_readback");
        end
        end_case("B003", start_errors);
    endtask

    task automatic run_b004();
        int start_errors;
        logic [31:0] status_word;
        begin_case("B004", start_errors);
        master_csr_read(MASTER_CSR_STATUS, status_word);
        expect_true("master busy should reset low", status_word[0] === 1'b0);
        expect_true("master out_of_range should reset low", status_word[8] === 1'b0);
        expect_true("master init_fail should reset low", status_word[9] === 1'b0);
        expect_true("master tx_fifo_empty error should reset low", status_word[10] === 1'b0);
        expect_true("master rx_fifo_full error should reset low", status_word[11] === 1'b0);
        if (error_count == start_errors) begin
            feature_pass("BASIC.master_csr_descriptor_range");
        end
        end_case("B004", start_errors);
    endtask

    task automatic run_b005();
        int start_errors;
        logic [31:0] desc_word;
        begin_case("B005", start_errors);
        master_csr_read(MASTER_CSR_DESC, desc_word);
        expect_true("master parasitic-power generic should force descriptor bit2 high", desc_word[2] === 1'b1);
        expect_eq("master descriptor reset fields with parasitic power", desc_word & 32'h00ff_ff0f, 32'h0000_0004);
        if (error_count == start_errors) begin
            feature_pass("BASIC.master_csr_descriptor_range");
        end
        end_case("B005", start_errors);
    endtask

    task automatic run_b006();
        int start_errors;
        logic [31:0] desc_word;
        begin_case("B006", start_errors);
        master_np_csr_read(MASTER_CSR_DESC, desc_word);
        expect_eq("master descriptor reset fields without parasitic power", desc_word & 32'h00ff_ff0f, 32'h0000_0000);
        if (error_count == start_errors) begin
            feature_pass("BASIC.master_csr_descriptor_range");
        end
        end_case("B006", start_errors);
    endtask

    task automatic run_b007();
        int start_errors;
        logic [31:0] fill_word;
        begin_case("B007", start_errors);
        master_csr_read(MASTER_CSR_FIFO_FILL, fill_word);
        expect_eq("master fifo fill reset", fill_word, 32'h0000_0000);
        if (error_count == start_errors) begin
            feature_pass("BASIC.master_csr_descriptor_range");
        end
        end_case("B007", start_errors);
    endtask

    task automatic run_b008();
        int start_errors;
        logic [31:0] read_data;
        logic [3:0] addr4;
        begin_case("B008", start_errors);
        for (int addr = 5; addr < 16; addr++) begin
            addr4 = addr[3:0];
            master_csr_read(addr4, read_data);
            expect_eq($sformatf("master undecoded read addr %0d", addr), read_data, 32'h0000_0000);
        end
        if (error_count == start_errors) begin
            feature_pass("BASIC.master_csr_descriptor_range");
        end
        end_case("B008", start_errors);
    endtask

    task automatic run_b009();
        int start_errors;
        logic [31:0] status_word;
        begin_case("B009", start_errors);
        master_csr_write(MASTER_CSR_DESC, master_desc(1'b1, 1'b1, 1'b1, 1'b1, 0, 0));
        master_csr_write(MASTER_CSR_STATUS, 32'h0000_0001);
        repeat (1) @(posedge clk);
        master_csr_read(MASTER_CSR_STATUS, status_word);
        expect_true("master tx commit should make busy visible", status_word[0] === 1'b1);
        master_wait_busy(1'b0, 2000, status_word);
        expect_true("empty tx command should report tx_fifo_empty", status_word[10] === 1'b1);
        if (error_count == start_errors) begin
            feature_pass("BASIC.master_commit_busy_irq");
        end
        end_case("B009", start_errors);
    endtask

    task automatic run_b010();
        int start_errors;
        logic [31:0] status_word;
        begin_case("B010", start_errors);
        master_csr_write(MASTER_CSR_DESC, master_desc(1'b0, 1'b0, 1'b1, 1'b1, 0, 0));
        master_csr_write(MASTER_CSR_STATUS, 32'h0000_0001);
        repeat (1) @(posedge clk);
        master_csr_read(MASTER_CSR_STATUS, status_word);
        expect_true("master rx commit should make busy visible", status_word[0] === 1'b1);
        master_wait_busy(1'b0, 2000, status_word);
        if (error_count == start_errors) begin
            feature_pass("BASIC.master_commit_busy_irq");
        end
        end_case("B010", start_errors);
    endtask

    task automatic run_b011();
        int start_errors;
        logic [31:0] desc_word;
        logic [31:0] status_word;
        logic [31:0] expected_desc;
        begin_case("B011", start_errors);
        expected_desc = master_desc(1'b1, 1'b1, 1'b1, 1'b1, 8, 2);
        master_csr_write(MASTER_CSR_DESC, expected_desc);
        master_csr_read(MASTER_CSR_DESC, desc_word);
        master_csr_read(MASTER_CSR_STATUS, status_word);
        expect_eq("master descriptor valid write tot8 wire2", desc_word & 32'h00ff_ff0f, expected_desc & 32'h00ff_ff0f);
        expect_true("valid descriptor should not set out_of_range", status_word[8] === 1'b0);
        if (error_count == start_errors) begin
            feature_pass("BASIC.master_csr_descriptor_range");
        end
        end_case("B011", start_errors);
    endtask

    task automatic run_b012();
        int start_errors;
        logic [31:0] desc_word;
        logic [31:0] status_word;
        logic [31:0] expected_desc;
        begin_case("B012", start_errors);
        expected_desc = master_desc(1'b1, 1'b0, 1'b1, 1'b1, 16, 5);
        master_csr_write(MASTER_CSR_DESC, expected_desc);
        master_csr_read(MASTER_CSR_DESC, desc_word);
        master_csr_read(MASTER_CSR_STATUS, status_word);
        expect_eq("master descriptor boundary tot16 wire5", desc_word & 32'h00ff_ff0f, expected_desc & 32'h00ff_ff0f);
        expect_true("boundary descriptor should not set out_of_range", status_word[8] === 1'b0);
        if (error_count == start_errors) begin
            feature_pass("BASIC.master_csr_descriptor_range");
        end
        end_case("B012", start_errors);
    endtask

    task automatic run_b013();
        int start_errors;
        logic [31:0] desc_before;
        logic [31:0] desc_after;
        logic [31:0] status_word;
        begin_case("B013", start_errors);
        master_csr_write(MASTER_CSR_DESC, master_desc(1'b1, 1'b0, 1'b1, 1'b1, 8, 2));
        master_csr_read(MASTER_CSR_DESC, desc_before);
        master_csr_write(MASTER_CSR_DESC, master_desc(1'b1, 1'b0, 1'b1, 1'b1, 17, 2));
        master_csr_read(MASTER_CSR_DESC, desc_after);
        master_csr_read(MASTER_CSR_STATUS, status_word);
        expect_eq("invalid tot_bytes should not update tot/wire fields", desc_after[23:8], desc_before[23:8]);
        expect_true("invalid tot_bytes should latch out_of_range", status_word[8] === 1'b1);
        if (error_count == start_errors) begin
            feature_pass("BASIC.master_csr_descriptor_range");
        end
        end_case("B013", start_errors);
    endtask

    task automatic run_b014();
        int start_errors;
        logic [31:0] desc_word;
        logic [31:0] expected_desc;
        begin_case("B014", start_errors);
        expected_desc = master_desc(1'b1, 1'b1, 1'b1, 1'b1, 8, 3);
        master_np_csr_write(MASTER_CSR_DESC, expected_desc);
        master_np_csr_read(MASTER_CSR_DESC, desc_word);
        expect_eq("non-parasitic descriptor write/readback", desc_word & 32'h00ff_ff0f, expected_desc & 32'h00ff_ff0f);
        if (error_count == start_errors) begin
            feature_pass("BASIC.master_csr_descriptor_range");
        end
        end_case("B014", start_errors);
    endtask

    task automatic run_b015();
        int start_errors;
        logic [31:0] desc_word;
        begin_case("B015", start_errors);
        master_csr_write(MASTER_CSR_DESC, master_desc(1'b1, 1'b0, 1'b0, 1'b1, 4, 1));
        master_csr_read(MASTER_CSR_DESC, desc_word);
        expect_true("parasitic generic should block clearing desc bit2", desc_word[2] === 1'b1);
        master_csr_write(MASTER_CSR_DESC, master_desc(1'b1, 1'b0, 1'b1, 1'b1, 4, 1));
        master_csr_read(MASTER_CSR_DESC, desc_word);
        expect_true("parasitic generic should keep desc bit2 high", desc_word[2] === 1'b1);
        if (error_count == start_errors) begin
            feature_pass("BASIC.master_csr_descriptor_range");
        end
        end_case("B015", start_errors);
    endtask

    task automatic run_b016();
        int start_errors;
        logic [31:0] desc_before;
        logic [31:0] desc_after;
        logic [31:0] status_word;
        begin_case("B016", start_errors);
        master_csr_write(MASTER_CSR_DESC, master_desc(1'b1, 1'b0, 1'b1, 1'b1, 8, 2));
        master_csr_read(MASTER_CSR_DESC, desc_before);
        master_csr_write(MASTER_CSR_DESC, master_desc(1'b1, 1'b0, 1'b1, 1'b1, 8, 6));
        master_csr_read(MASTER_CSR_DESC, desc_after);
        master_csr_read(MASTER_CSR_STATUS, status_word);
        expect_eq("invalid wire6 should not update tot/wire fields", desc_after[23:8], desc_before[23:8]);
        expect_true("invalid wire6 should latch out_of_range", status_word[8] === 1'b1);
        if (error_count == start_errors) begin
            feature_pass("BASIC.master_csr_descriptor_range");
        end
        end_case("B016", start_errors);
    endtask

    task automatic run_b017();
        int start_errors;
        logic [31:0] desc_before;
        logic [31:0] desc_after;
        logic [31:0] status_word;
        begin_case("B017", start_errors);
        master_csr_write(MASTER_CSR_DESC, master_desc(1'b1, 1'b0, 1'b1, 1'b1, 8, 2));
        master_csr_read(MASTER_CSR_DESC, desc_before);
        master_csr_write(MASTER_CSR_DESC, master_desc(1'b1, 1'b0, 1'b1, 1'b1, 8, 7));
        master_csr_read(MASTER_CSR_DESC, desc_after);
        master_csr_read(MASTER_CSR_STATUS, status_word);
        expect_eq("invalid wire7 should not update tot/wire fields", desc_after[23:8], desc_before[23:8]);
        expect_true("invalid wire7 should latch out_of_range", status_word[8] === 1'b1);
        if (error_count == start_errors) begin
            feature_pass("BASIC.master_csr_descriptor_range");
        end
        end_case("B017", start_errors);
    endtask

    task automatic run_b018();
        int start_errors;
        logic [31:0] desc_before;
        logic [31:0] desc_after;
        logic [31:0] status_before;
        logic [31:0] status_after;
        begin_case("B018", start_errors);
        master_csr_write(MASTER_CSR_DESC, master_desc(1'b0, 1'b0, 1'b1, 1'b1, 1, 0));
        master_csr_read(MASTER_CSR_DESC, desc_before);
        master_csr_write(MASTER_CSR_STATUS, 32'h0000_0001);
        repeat (1) @(posedge clk);
        master_csr_read(MASTER_CSR_STATUS, status_before);
        expect_true("busy setup for protected descriptor write", status_before[0] === 1'b1);
        master_csr_write(MASTER_CSR_DESC, master_desc(1'b1, 1'b1, 1'b1, 1'b1, 17, 7));
        master_wait_busy(1'b0, 5000, status_after);
        master_csr_read(MASTER_CSR_DESC, desc_after);
        expect_eq("busy descriptor write should hold descriptor", desc_after & 32'h00ff_ff0f, desc_before & 32'h00ff_ff0f);
        expect_true("busy descriptor write should not change out_of_range", status_after[8] === status_before[8]);
        if (error_count == start_errors) begin
            feature_pass("BASIC.master_csr_descriptor_range");
        end
        end_case("B018", start_errors);
    endtask

    task automatic run_b019();
        int start_errors;
        logic [31:0] fill_before;
        logic [31:0] fill_after;
        begin_case("B019", start_errors);
        master_csr_read(MASTER_CSR_FIFO_FILL, fill_before);
        master_csr_write(MASTER_CSR_FIFO_FILL, 32'hffff_ffff);
        master_csr_read(MASTER_CSR_FIFO_FILL, fill_after);
        expect_eq("master fifo-fill write should be no-op", fill_after, fill_before);
        if (error_count == start_errors) begin
            feature_pass("BASIC.master_csr_descriptor_range");
        end
        end_case("B019", start_errors);
    endtask

    task automatic run_b020();
        int start_errors;
        logic [31:0] status_word;
        begin_case("B020", start_errors);
        master_csr_write(MASTER_CSR_DESC, master_desc(1'b0, 1'b0, 1'b1, 1'b1, 0, 0));
        master_csr_write(MASTER_CSR_STATUS, 32'h0000_0001);
        repeat (1) @(posedge clk);
        master_csr_read(MASTER_CSR_STATUS, status_word);
        expect_true("commit read-after-write should expose busy, not commit", status_word[0] === 1'b1);
        expect_true("commit bit should not be exposed above busy bit", status_word[1] === 1'b0);
        master_wait_busy(1'b0, 2000, status_word);
        if (error_count == start_errors) begin
            feature_pass("BASIC.master_commit_busy_irq");
        end
        end_case("B020", start_errors);
    endtask

    task automatic run_b021();
        int start_errors;
        logic [31:0] desc_before;
        logic [31:0] desc_after;
        logic [3:0] addr4;
        begin_case("B021", start_errors);
        master_csr_read(MASTER_CSR_DESC, desc_before);
        for (int addr = 5; addr < 16; addr++) begin
            addr4 = addr[3:0];
            master_csr_write(addr4, 32'h55aa_0000 | addr);
        end
        master_csr_read(MASTER_CSR_DESC, desc_after);
        expect_eq("master undecoded writes should not touch descriptor", desc_after & 32'h00ff_ff0f, desc_before & 32'h00ff_ff0f);
        if (error_count == start_errors) begin
            feature_pass("BASIC.master_csr_descriptor_range");
        end
        end_case("B021", start_errors);
    endtask

    task automatic run_b022();
        int start_errors;
        logic [31:0] read_data;
        logic [3:0] addr4;
        begin_case("B022", start_errors);
        for (int addr = 0; addr < 16; addr++) begin
            addr4 = addr[3:0];
            master_csr_read(addr4, read_data);
            expect_true($sformatf("master read addr %0d leaked into [31:24]: 0x%08x", addr, read_data), read_data[31:24] == 8'h00);
        end
        if (error_count == start_errors) begin
            feature_pass("BASIC.master_csr_descriptor_range");
        end
        end_case("B022", start_errors);
    endtask

    task automatic run_b023();
        int start_errors;
        logic [31:0] read_data;
        logic [3:0] addr4;
        begin_case("B023", start_errors);
        for (int addr = 0; addr < 3; addr++) begin
            addr4 = addr[3:0];
            master_read_wait_contract(addr4, read_data);
        end
        if (error_count == start_errors) begin
            feature_pass("BASIC.master_commit_busy_irq");
        end
        end_case("B023", start_errors);
    endtask

    task automatic run_b024();
        int start_errors;
        logic [31:0] status_word;
        begin_case("B024", start_errors);
        master_csr_read(MASTER_CSR_STATUS, status_word);
        repeat (1) @(posedge clk);
        expect_true("precondition: complete IRQ clear before irq-clear check", m_complete_irq === 1'b0);

        master_csr_write(MASTER_CSR_DESC, master_desc(1'b0, 1'b0, 1'b1, 1'b1, 0, 0));
        master_csr_write(MASTER_CSR_STATUS, 32'h0000_0001);
        master_wait_irq(2000);
        master_csr_read(MASTER_CSR_STATUS, status_word);
        @(posedge clk);
        #1;
        expect_true("master status read should clear complete IRQ", m_complete_irq === 1'b0);

        master_csr_write(MASTER_CSR_STATUS, 32'h0000_0001);
        master_wait_irq(2000);
        master_csr_write(MASTER_CSR_DESC, master_desc(1'b0, 1'b0, 1'b1, 1'b1, 0, 0));
        @(posedge clk);
        #1;
        expect_true("master descriptor write should clear complete IRQ", m_complete_irq === 1'b0);
        if (error_count == start_errors) begin
            feature_pass("BASIC.master_commit_busy_irq");
        end
        end_case("B024", start_errors);
    endtask

    task automatic run_master_tx_case(
        input string case_id,
        input int tot_bytes,
        input int descriptor_wire,
        input bit init,
        input bit usewire_id,
        input int fifo_channel,
        input int expected_wire
    );
        int start_errors;
        logic [31:0] status_word;

        begin_case(case_id, start_errors);
        master_drain_rx_fifo(32);
        master_expect_fifo_fill({case_id, " precondition"}, 0, 0);
        for (int byte_idx = 0; byte_idx < tot_bytes; byte_idx++) begin
            master_push_tx_bypass(fifo_channel, 8'h00);
        end
        master_expect_fifo_fill({case_id, " preload"}, tot_bytes, 0);
        master_csr_write(MASTER_CSR_DESC, master_desc(1'b1, init, 1'b1, usewire_id, tot_bytes, descriptor_wire));
        master_csr_write(MASTER_CSR_STATUS, 32'h0000_0001);
        master_wait_irq_and_check_line(expected_wire, 50_000, tot_bytes > 0);
        master_csr_read(MASTER_CSR_STATUS, status_word);
        expect_true({case_id, " busy should clear"}, status_word[0] === 1'b0);
        if (tot_bytes == 0) begin
            expect_true({case_id, " empty tx should flag fifo empty"}, status_word[10] === 1'b1);
        end else begin
            expect_true({case_id, " non-empty tx should clear fifo-empty flag"}, status_word[10] === 1'b0);
        end
        if (init) begin
            expect_true({case_id, " init should see presence"}, status_word[9] === 1'b0);
        end
        master_expect_fifo_fill({case_id, " post"}, 0, 0);
        if (error_count == start_errors) begin
            feature_pass("BASIC.master_tx_engine_fsm");
        end
        end_case(case_id, start_errors);
    endtask

    task automatic run_b025();
        run_master_tx_case("B025", 0, 0, 1'b0, 1'b1, 0, 0);
    endtask

    task automatic run_b026();
        run_master_tx_case("B026", 1, 0, 1'b0, 1'b1, 0, 0);
    endtask

    task automatic run_b027();
        run_master_tx_case("B027", 1, 0, 1'b1, 1'b1, 0, 0);
    endtask

    task automatic run_b028();
        run_master_tx_case("B028", 2, 0, 1'b1, 1'b1, 0, 0);
    endtask

    task automatic run_b029();
        run_master_tx_case("B029", 3, 0, 1'b0, 1'b1, 0, 0);
    endtask

    task automatic run_b030();
        run_master_tx_case("B030", 4, 0, 1'b0, 1'b1, 0, 0);
    endtask

    task automatic run_b031();
        run_master_tx_case("B031", 5, 0, 1'b0, 1'b0, 1, 1);
    endtask

    task automatic run_b032();
        run_master_tx_case("B032", 6, 1, 1'b1, 1'b1, 1, 1);
    endtask

    task automatic run_b033();
        run_master_tx_case("B033", 7, 2, 1'b0, 1'b1, 2, 2);
    endtask

    task automatic run_b121();
        int start_errors;
        begin_case("B121", start_errors);
        check_capability();
        if (error_count == start_errors) begin
            feature_pass("BASIC.capability_readback");
        end
        end_case("B121", start_errors);
    endtask

    task automatic run_b122();
        int start_errors;
        logic [31:0] status_word;
        begin_case("B122", start_errors);
        status_write(3, 1'b0);
        csr_read(CSR_STATUS, status_word);
        expect_eq("STATUS sel_line round-trip", status_word[15:0], 16'd3);
        expect_true("STATUS processor_go should be 0 after go=0 write", status_word[16] === 1'b0);
        if (error_count == start_errors) begin
            feature_pass("BASIC.per_line_status_select");
        end
        end_case("B122", start_errors);
    endtask

    task automatic run_b123();
        int start_errors;
        logic [31:0] status_word;
        begin_case("B123", start_errors);
        read_selected_status(4, 1'b1, status_word);
        expect_eq("STATUS sel_line line4", status_word[15:0], 16'd4);
        expect_true("STATUS processor_go line4 should stick", status_word[16] === 1'b1);
        read_selected_status(5, 1'b0, status_word);
        expect_true("STATUS processor_go line5 should stay 0", status_word[16] === 1'b0);
        if (error_count == start_errors) begin
            feature_pass("BASIC.processor_go_per_line");
        end
        end_case("B123", start_errors);
    endtask

    task automatic run_b124();
        int start_errors;
        logic [31:0] read_data;
        logic [3:0] temp_addr;
        begin_case("B124", start_errors);
        temp_addr = CSR_TEMP0 + 2;
        csr_read(temp_addr, read_data);
        expect_eq("SENSOR2 before sample_valid", read_data, 32'h0000_0000);
        status_write(2, 1'b1);
        wait_for_sample_valid(2, 1_300_000_000ns);
        check_line_temperature(2);
        if (error_count == start_errors) begin
            feature_pass("BASIC.temperature_read_zero_until_valid");
            feature_pass("BASIC.temperature_float32_readback");
        end
        end_case("B124", start_errors);
    endtask

    task automatic wait_for_init_err(input int line, input time timeout_ns);
        logic [31:0] status_word;
        time deadline;

        deadline = $time + timeout_ns;
        while ($time < deadline) begin
            csr_read(CSR_STATUS, status_word);
            if (status_word[25] === 1'b1) begin
                return;
            end
            repeat (1000) @(posedge clk);
        end
        $error("line %0d init_err timeout", line);
        error_count++;
    endtask

    task automatic run_b125();
        int start_errors;
        logic [31:0] status_word;
        begin_case("B125", start_errors);
        sensor_present[5] = 1'b0;
        status_write(5, 1'b1);
        wait_for_init_err(5, 20_000_000ns);
        csr_read(CSR_STATUS, status_word);
        expect_true("line5 init_err should be visible on selected line", status_word[25] === 1'b1);
        status_write(4, 1'b0);
        csr_read(CSR_STATUS, status_word);
        expect_true("line4 init_err should not mirror line5", status_word[25] === 1'b0);
        status_write(5, 1'b0);
        sensor_present[5] = 1'b1;
        if (error_count == start_errors) begin
            feature_pass("BASIC.controller_init_err_readback");
        end
        end_case("B125", start_errors);
    endtask
