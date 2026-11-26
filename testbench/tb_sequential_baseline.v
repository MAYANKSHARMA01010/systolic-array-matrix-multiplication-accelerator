`timescale 1ns/1ps

// ============================================================================
// Testbench: tb_sequential_baseline
// Description:
//   Unit and verification testbench for 'sequential_matmul_baseline'.
//
//   Test Coverage:
//   1. Functional Mathematical Verification:
//      - Identity matrix multiplication (A * I = A)
//      - Zero matrix multiplication (A * 0 = 0)
//      - Mixed positive and negative values
//      - Signed boundary values (min_val, max_val)
//      - Extended precision (16-bit operands)
//   2. Protocol & Timing Verification:
//      - Strict latency check: confirms execution takes exactly N^3 clock cycles
//      - Handshake: 'busy' asserts immediately on 'start', 'done' pulses for 1 cycle
//      - Input latching: altering 'a_flat' / 'b_flat' mid-computation does not affect result
//      - Start ignore: applying 'start' while 'busy' is high is safely ignored
//      - Output stability: 'result' stays unchanged across idle cycles after 'done'
//      - Back-to-back operations: consecutive starts without idle gaps
//      - Asynchronous reset: asserting 'rst_n' mid-flight cleanly wipes state
// ============================================================================

module tb_sequential_baseline;

    // ------------------------------------------------------------------------
    // Top-Level Testbench Parameters
    // ------------------------------------------------------------------------
    parameter ARRAY_SIZE = 4;   // Matrix dimension N (N x N matrices)
    parameter DATA_WIDTH = 8;   // Input element bit-width (signed)
    parameter ACC_WIDTH  = 32;  // Accumulator bit-width (signed)

    // ------------------------------------------------------------------------
    // Testbench Stimulus and Monitor Signals
    // ------------------------------------------------------------------------
    reg clk;    // Testbench clock generator
    reg rst_n;  // Active-low asynchronous reset stimulus
    reg start;  // 1-cycle active-high start command

    // Packed 1D input buses driving the DUT
    reg signed [ARRAY_SIZE * ARRAY_SIZE * DATA_WIDTH - 1 : 0] a_flat;
    reg signed [ARRAY_SIZE * ARRAY_SIZE * DATA_WIDTH - 1 : 0] b_flat;

    // DUT output wires
    wire busy;  // High while computing
    wire done;  // 1-cycle pulse upon completion
    wire signed [ARRAY_SIZE * ARRAY_SIZE * ACC_WIDTH - 1 : 0] result;  // Output matrix

    // ------------------------------------------------------------------------
    // Device Under Test (DUT) Instantiation
    // ------------------------------------------------------------------------
    sequential_matmul_baseline #(
        .ARRAY_SIZE(ARRAY_SIZE),
        .DATA_WIDTH(DATA_WIDTH),
        .ACC_WIDTH (ACC_WIDTH)
    ) dut (
        .clk   (clk),
        .rst_n (rst_n),
        .start (start),
        .a_flat(a_flat),
        .b_flat(b_flat),
        .busy  (busy),
        .done  (done),
        .result(result)
    );

    // ------------------------------------------------------------------------
    // Clock Generation: 10ns period -> 100 MHz clock frequency
    // ------------------------------------------------------------------------
    always #5 clk = ~clk;

    // ------------------------------------------------------------------------
    // 2D Matrix Storage for Test Data Generation and Software Golden Checks
    // ------------------------------------------------------------------------
    reg signed [DATA_WIDTH-1:0] mat_a [0:ARRAY_SIZE-1][0:ARRAY_SIZE-1];  // 2D representation of Matrix A
    reg signed [DATA_WIDTH-1:0] mat_b [0:ARRAY_SIZE-1][0:ARRAY_SIZE-1];  // 2D representation of Matrix B
    reg signed [ACC_WIDTH-1:0]  exp_c [0:ARRAY_SIZE-1][0:ARRAY_SIZE-1];  // Software golden expected Matrix C

    // Test tracking variables
    reg signed [ARRAY_SIZE * ARRAY_SIZE * ACC_WIDTH - 1 : 0] saved_result;
    reg signed [DATA_WIDTH-1:0] min_val;
    reg signed [DATA_WIDTH-1:0] max_val;

    integer cycle_count;    // Measured clock cycles between start and done
    integer timeout_count;  // Watchdog counter to avoid infinite simulation loops
    integer max_cycles;     // Maximum allowed cycles before timeout
    integer latency_ns;     // Measured latency converted to nanoseconds
    integer mac_count;      // Total MAC operations = N^3
    integer r, c, k;        // Loop indices for matrix setup

    // ------------------------------------------------------------------------
    // Task: pack_inputs
    // Description:
    //   Packs the 2D matrices 'mat_a' and 'mat_b' into the flattened 1D buses
    //   'a_flat' and 'b_flat' using row-major ordering.
    // ------------------------------------------------------------------------
    task pack_inputs;
        integer pr, pc;
        begin
            for (pr = 0; pr < ARRAY_SIZE; pr = pr + 1) begin
                for (pc = 0; pc < ARRAY_SIZE; pc = pc + 1) begin
                    a_flat[(pr * ARRAY_SIZE + pc) * DATA_WIDTH +: DATA_WIDTH] = mat_a[pr][pc];
                    b_flat[(pr * ARRAY_SIZE + pc) * DATA_WIDTH +: DATA_WIDTH] = mat_b[pr][pc];
                end
            end
        end
    endtask

    // ------------------------------------------------------------------------
    // Task: compute_expected
    // Description:
    //   Computes the golden mathematical matrix product C = A * B in software:
    //       exp_c[er][ec] = sum_{ek=0}^{N-1} (mat_a[er][ek] * mat_b[ek][ec])
    // ------------------------------------------------------------------------
    task compute_expected;
        integer er, ec, ek;
        reg signed [ACC_WIDTH-1:0] sum;
        begin
            for (er = 0; er < ARRAY_SIZE; er = er + 1) begin
                for (ec = 0; ec < ARRAY_SIZE; ec = ec + 1) begin
                    sum = {ACC_WIDTH{1'b0}};
                    for (ek = 0; ek < ARRAY_SIZE; ek = ek + 1) begin
                        sum = sum + $signed(mat_a[er][ek]) * $signed(mat_b[ek][ec]);
                    end
                    exp_c[er][ec] = sum;
                end
            end
        end
    endtask

    // ------------------------------------------------------------------------
    // Task: run_matmul
    // Description:
    //   Sends a 1-cycle 'start' pulse to the DUT and measures the exact number
    //   of clock cycles until 'done' is asserted. Confirms cycles == N^3.
    // ------------------------------------------------------------------------
    task run_matmul;
        begin
            @(negedge clk);
            start = 1'b1;  // Assert start pulse
            @(negedge clk);
            start = 1'b0;  // Deassert start pulse
            cycle_count = 0;
            timeout_count = 0;
            max_cycles = (ARRAY_SIZE * ARRAY_SIZE * ARRAY_SIZE) + 100;

            // Wait until DUT asserts 'done'
            while (!done) begin
                @(negedge clk);
                cycle_count = cycle_count + 1;
                timeout_count = timeout_count + 1;
                if (timeout_count >= max_cycles) begin
                    $fatal(1, "Simulation timeout: done was not asserted within %0d cycles!", max_cycles);
                end
            end

            mac_count = ARRAY_SIZE * ARRAY_SIZE * ARRAY_SIZE;
            latency_ns = cycle_count * 10;

            // Strict latency check: Must match N^3 exactly!
            if (cycle_count !== mac_count) begin
                $fatal(1, "Latency check failed: Expected %0d cycles, measured %0d cycles!",
                       mac_count, cycle_count);
            end

            $display("Matrix dimension: %0dx%0d", ARRAY_SIZE, ARRAY_SIZE);
            $display("Number of MAC operations: %0d", mac_count);
            $display("Measured clock cycles: %0d", cycle_count);
            $display("Clock period: 10 ns");
            $display("Measured latency: %0d ns", latency_ns);
        end
    endtask

    // ------------------------------------------------------------------------
    // Task: verify_result
    // Description:
    //   Compares each element of the hardware output 'result' against the
    //   software golden matrix 'exp_c'.
    // ------------------------------------------------------------------------
    task verify_result;
        input [511:0] test_name;
        integer vr, vc;
        reg signed [ACC_WIDTH-1:0] act_val;
        begin
            for (vr = 0; vr < ARRAY_SIZE; vr = vr + 1) begin
                for (vc = 0; vc < ARRAY_SIZE; vc = vc + 1) begin
                    // Extract element [vr][vc] from packed 1D result bus
                    act_val = result[(vr * ARRAY_SIZE + vc) * ACC_WIDTH +: ACC_WIDTH];
                    if (act_val !== exp_c[vr][vc]) begin
                        $fatal(1, "MISMATCH in %0s at [%0d][%0d]: expected %0d, got %0d",
                               test_name, vr, vc, exp_c[vr][vc], act_val);
                    end
                end
            end
            $display("PASS: %0s", test_name);
        end
    endtask

    // ------------------------------------------------------------------------
    // Main Test Stimulus Process
    // ------------------------------------------------------------------------
    initial begin
        // Configure VCD waveform file
        if (DATA_WIDTH == 16)
            $dumpfile("sim/waveforms/seq_baseline_4x4_16bit.vcd");
        else if (ARRAY_SIZE == 2)
            $dumpfile("sim/waveforms/seq_baseline_2x2.vcd");
        else if (ARRAY_SIZE == 4)
            $dumpfile("sim/waveforms/seq_baseline_4x4.vcd");
        else
            $dumpfile("sim/waveforms/seq_baseline_8x8.vcd");
        $dumpvars(0, tb_sequential_baseline);

        // Initialize signals and apply reset pulse
        clk    = 0;
        rst_n  = 0;
        start  = 0;
        a_flat = 0;
        b_flat = 0;

        repeat (2) @(negedge clk);
        rst_n = 1;  // Deassert reset
        @(negedge clk);

        // --------------------------------------------------------------------
        // Test 1: Identity Matrix Multiplication (A * I = A)
        // --------------------------------------------------------------------
        for (r = 0; r < ARRAY_SIZE; r = r + 1) begin
            for (c = 0; c < ARRAY_SIZE; c = c + 1) begin
                mat_a[r][c] = r * ARRAY_SIZE + c + 1;    // Ramp values: 1, 2, 3...
                mat_b[r][c] = (r == c) ? 1 : 0;         // Identity matrix: 1 on diagonal, 0 elsewhere
            end
        end
        compute_expected();
        pack_inputs();
        run_matmul();
        verify_result("Identity matrix test");

        // --------------------------------------------------------------------
        // Test 2: Zero Matrix Multiplication (A * 0 = 0)
        // --------------------------------------------------------------------
        for (r = 0; r < ARRAY_SIZE; r = r + 1) begin
            for (c = 0; c < ARRAY_SIZE; c = c + 1) begin
                mat_a[r][c] = r + c + 2;
                mat_b[r][c] = 0;  // All zeros
            end
        end
        compute_expected();
        pack_inputs();
        run_matmul();
        verify_result("Zero matrix test");

        // --------------------------------------------------------------------
        // Test 3: Mixed Positive and Negative Numbers
        // --------------------------------------------------------------------
        for (r = 0; r < ARRAY_SIZE; r = r + 1) begin
            for (c = 0; c < ARRAY_SIZE; c = c + 1) begin
                mat_a[r][c] = ((r + c) % 2 == 0) ? (r - c + 2) : (c - r - 3);
                mat_b[r][c] = ((r * c) % 2 == 0) ? (c + 1) : (-r - 2);
            end
        end
        compute_expected();
        pack_inputs();
        run_matmul();
        verify_result("Positive and negative values test");

        // --------------------------------------------------------------------
        // Test 4: Signed Boundary Values (min_val and max_val)
        // --------------------------------------------------------------------
        min_val = - (1 <<< (DATA_WIDTH - 1));       // e.g. -128
        max_val = (1 <<< (DATA_WIDTH - 1)) - 1;    // e.g. +127
        for (r = 0; r < ARRAY_SIZE; r = r + 1) begin
            for (c = 0; c < ARRAY_SIZE; c = c + 1) begin
                mat_a[r][c] = ((r + c) % 4 == 0) ? min_val :
                              ((r + c) % 4 == 1) ? max_val :
                              ((r + c) % 4 == 2) ? -1 : 1;
                mat_b[r][c] = ((r * 2 + c) % 4 == 0) ? max_val :
                              ((r * 2 + c) % 4 == 1) ? min_val :
                              ((r * 2 + c) % 4 == 2) ? 1 : -1;
            end
        end
        compute_expected();
        pack_inputs();
        run_matmul();
        verify_result("Boundary values test");

        // --------------------------------------------------------------------
        // Test 5: Handshake Protocol, Start Ignore, and Bus Tamper Immunity
        // --------------------------------------------------------------------
        @(negedge clk);
        if (busy !== 1'b0)
            $fatal(1, "Handshake test failed: busy should be 0 in idle!");
        if (done !== 1'b0)
            $fatal(1, "Handshake test failed: done should be 0 in idle!");

        for (r = 0; r < ARRAY_SIZE; r = r + 1) begin
            for (c = 0; c < ARRAY_SIZE; c = c + 1) begin
                mat_a[r][c] = r * 2 + c + 1;
                mat_b[r][c] = (r == c) ? 2 : -1;
            end
        end
        compute_expected();
        pack_inputs();

        @(negedge clk);
        start = 1'b1;  // Issue start pulse
        @(negedge clk);
        start = 1'b0;
        cycle_count = 0;

        if (busy !== 1'b1)
            $fatal(1, "Handshake test failed: busy did not become 1 after start accepted!");
        if (done !== 1'b0)
            $fatal(1, "Handshake test failed: done should be 0 during computation!");

        repeat (2) begin
            @(negedge clk);
            cycle_count = cycle_count + 1;
        end
        if (busy !== 1'b1)
            $fatal(1, "Handshake test failed: busy dropped prematurely!");

        // Inject second 'start' pulse mid-flight -> MUST BE SAFELY IGNORED!
        start = 1'b1;
        @(negedge clk);
        start = 1'b0;
        cycle_count = cycle_count + 1;

        // Corrupt external input buses mid-flight -> LATCHED INPUTS MUST BE IMMUNE!
        a_flat = ~a_flat;
        b_flat = ~b_flat;

        timeout_count = 0;
        max_cycles = (ARRAY_SIZE * ARRAY_SIZE * ARRAY_SIZE) + 100;

        while (!done) begin
            @(negedge clk);
            cycle_count = cycle_count + 1;
            timeout_count = timeout_count + 1;
            if (timeout_count >= max_cycles)
                $fatal(1, "Handshake test timeout: done not asserted!");
            if (!done && busy !== 1'b1)
                $fatal(1, "Handshake test failed: busy dropped before done was asserted!");
        end

        if (busy !== 1'b0)
            $fatal(1, "Handshake test failed: busy did not become low when done was asserted!");
        if (done !== 1'b1)
            $fatal(1, "Handshake test failed: done is not 1 when result is ready!");

        verify_result("Handshake start-ignore and bus-change test");

        if (cycle_count !== (ARRAY_SIZE * ARRAY_SIZE * ARRAY_SIZE))
            $fatal(1, "Handshake test failed: cycle count %0d did not match expected %0d!",
                   cycle_count, (ARRAY_SIZE * ARRAY_SIZE * ARRAY_SIZE));

        @(negedge clk);
        if (done !== 1'b0)
            $fatal(1, "Handshake test failed: done did not return to 0 after exactly one cycle!");
        if (busy !== 1'b0)
            $fatal(1, "Handshake test failed: second start request was queued or started unexpectedly!");

        // --------------------------------------------------------------------
        // Test 6: Output Stability Across Idle Cycles
        // Result must hold steady when system returns to IDLE.
        // --------------------------------------------------------------------
        saved_result = result;
        repeat (10) begin
            @(negedge clk);
            if (busy !== 1'b0)
                $fatal(1, "Handshake test failed: busy became 1 unexpectedly during idle!");
            if (done !== 1'b0)
                $fatal(1, "Handshake test failed: done asserted unexpectedly during idle!");
            if (result !== saved_result)
                $fatal(1, "Handshake test failed: result changed during idle!");
        end
        $display("PASS: Start handshake and busy/done protocol verification");

        // --------------------------------------------------------------------
        // Test 7: Input Latching Verification
        // Inverting a_flat and b_flat immediately after start must not alter output.
        // --------------------------------------------------------------------
        for (r = 0; r < ARRAY_SIZE; r = r + 1) begin
            for (c = 0; c < ARRAY_SIZE; c = c + 1) begin
                mat_a[r][c] = r + 1;
                mat_b[r][c] = c + 1;
            end
        end
        compute_expected();
        pack_inputs();

        @(negedge clk);
        start = 1'b1;
        @(negedge clk);
        start = 1'b0;
        a_flat = ~a_flat;  // Invert external bus immediately
        b_flat = ~b_flat;

        timeout_count = 0;
        while (!done) begin
            @(negedge clk);
            timeout_count = timeout_count + 1;
            if (timeout_count >= max_cycles)
                $fatal(1, "Input latching test timeout!");
        end
        verify_result("Input latching test");

        saved_result = result;
        repeat (10) begin
            @(negedge clk);
            if (result !== saved_result)
                $fatal(1, "Output stability test failed: result changed during idle!");
        end
        $display("PASS: Output stability test");

        // --------------------------------------------------------------------
        // Test 8: Back-to-Back Matrix Operations
        // Launching operation 2 immediately without an idle gap.
        // --------------------------------------------------------------------
        for (r = 0; r < ARRAY_SIZE; r = r + 1) begin
            for (c = 0; c < ARRAY_SIZE; c = c + 1) begin
                mat_a[r][c] = r + 1;
                mat_b[r][c] = c + 1;
            end
        end
        compute_expected();
        pack_inputs();

        @(negedge clk);
        start = 1'b1;
        @(negedge clk);
        start = 1'b0;

        timeout_count = 0;
        while (!done) begin
            @(negedge clk);
            timeout_count = timeout_count + 1;
            if (timeout_count >= max_cycles)
                $fatal(1, "Back-to-back op 1 timeout!");
        end
        verify_result("Back-to-back operation 1");

        // Immediately setup and launch operation 2
        for (r = 0; r < ARRAY_SIZE; r = r + 1) begin
            for (c = 0; c < ARRAY_SIZE; c = c + 1) begin
                mat_a[r][c] = r + 3;
                mat_b[r][c] = c + 2;
            end
        end
        compute_expected();
        pack_inputs();

        start = 1'b1;
        @(negedge clk);
        start = 1'b0;

        timeout_count = 0;
        while (!done) begin
            @(negedge clk);
            timeout_count = timeout_count + 1;
            if (timeout_count >= max_cycles)
                $fatal(1, "Back-to-back op 2 timeout!");
        end
        verify_result("Back-to-back operation 2");
        $display("PASS: Back-to-back matrix operations");

        // --------------------------------------------------------------------
        // Test 9: Asynchronous Reset Recovery Test
        // Assert reset mid-computation, verify clean shutdown, then verify normal operation.
        // --------------------------------------------------------------------
        @(negedge clk);
        start = 1'b1;
        @(negedge clk);
        start = 1'b0;

        repeat (3) @(negedge clk);
        if (!busy)
            $fatal(1, "Reset test failed: DUT should be busy!");

        #2 rst_n = 1'b0;  // Assert asynchronous reset mid-computation
        #1;
        if (busy !== 1'b0 || done !== 1'b0 || result !== 0)
            $fatal(1, "Reset test failed: registers not cleared!");

        repeat (2) @(negedge clk);
        rst_n = 1'b1;  // Deassert reset
        @(negedge clk);

        // Run post-reset operation to prove healthy recovery
        for (r = 0; r < ARRAY_SIZE; r = r + 1) begin
            for (c = 0; c < ARRAY_SIZE; c = c + 1) begin
                mat_a[r][c] = r + 2;
                mat_b[r][c] = (r == c) ? 1 : 0;
            end
        end
        compute_expected();
        pack_inputs();
        run_matmul();
        verify_result("Post-reset operation test");
        $display("PASS: Reset behaviour test");

        // --------------------------------------------------------------------
        // Test 10: Extended Precision Large Accumulation Test (when 16-bit)
        // --------------------------------------------------------------------
        if (DATA_WIDTH == 16) begin
            for (r = 0; r < ARRAY_SIZE; r = r + 1) begin
                for (c = 0; c < ARRAY_SIZE; c = c + 1) begin
                    mat_a[r][c] = ((r + c) % 2 == 0) ? 16'sd15000 : -16'sd20000;
                    mat_b[r][c] = ((r * c) % 2 == 0) ? 16'sd12000 : -16'sd18000;
                end
            end
            compute_expected();
            pack_inputs();
            run_matmul();
            verify_result("Extended precision large accumulation test");
        end

        // --------------------------------------------------------------------
        // Testbench Summary
        // --------------------------------------------------------------------
        @(negedge clk);
        $display("========================================");
        $display("ALL BASELINE TESTS PASSED SUCCESSFULLY (ARRAY_SIZE=%0d, DATA_WIDTH=%0d, ACC_WIDTH=%0d)",
                 ARRAY_SIZE, DATA_WIDTH, ACC_WIDTH);
        $display("========================================");
        $finish;
    end

endmodule
