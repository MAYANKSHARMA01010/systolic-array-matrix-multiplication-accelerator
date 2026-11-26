`timescale 1ns/1ps

// ============================================================================
// Testbench: tb_processing_element
// Description:
//   Comprehensive unit testbench for verifying a single Processing Element (PE).
//
//   Verifies 14 distinct test scenarios:
//     1. Positive signed multiplication (e.g. 3 * 4 = 12)
//     2. Negative signed multiplication (e.g. -3 * 5 = -15)
//     3. Multiplication with both operands negative (e.g. -3 * -4 = 12)
//     4. Multiplication by zero (e.g. 0 * 7 = 0, -5 * 0 = 0)
//     5. Multiple consecutive MAC operations (accumulating successive products)
//     6. Validity gating: invalid A with valid B (acc must NOT change)
//     7. Validity gating: valid A with invalid B (acc must NOT change)
//     8. Synchronous accumulator clear ('clear_acc' resets accumulator to 0)
//     9. Clear priority over simultaneous MAC ('clear_acc' overrides valid MAC)
//    10. Asynchronous reset ('rst_n' immediately clears all outputs & registers)
//    11. Data forwarding verification ('a_out' and 'b_out' delayed by exactly 1 cycle)
//    12. Valid signal forwarding ('a_valid_out' and 'b_valid_out' delayed by 1 cycle)
//    13. Extreme signed boundary values (min_val * max_val, min_val * min_val)
//    14. Accumulator overflow wrapping (validates two's complement wrap behavior)
// ============================================================================

module tb_processing_element;

    // ------------------------------------------------------------------------
    // Testbench Parameters
    // ------------------------------------------------------------------------
    parameter DATA_WIDTH = 8;   // Input data bit-width (signed, e.g. 8-bit: -128 to +127)
    parameter ACC_WIDTH  = 32;  // Accumulator bit-width (signed, e.g. 32-bit)

    // ------------------------------------------------------------------------
    // Testbench Signals (reg drives inputs, wire monitors outputs)
    // ------------------------------------------------------------------------
    reg clk;        // Testbench clock generator
    reg rst_n;      // Active-low asynchronous reset stimulus
    reg clear_acc;  // Synchronous accumulator clear stimulus

    reg signed [DATA_WIDTH-1:0] a_in;  // Horizontal input operand stimulus
    reg signed [DATA_WIDTH-1:0] b_in;  // Vertical input operand stimulus

    reg a_valid_in;  // Handshake validity flag for operand A
    reg b_valid_in;  // Handshake validity flag for operand B

    wire signed [DATA_WIDTH-1:0] a_out;  // Forwarded horizontal output from DUT
    wire signed [DATA_WIDTH-1:0] b_out;  // Forwarded vertical output from DUT

    wire a_valid_out;  // Forwarded valid flag for A from DUT
    wire b_valid_out;  // Forwarded valid flag for B from DUT

    wire signed [ACC_WIDTH-1:0] acc_out;  // Accumulated MAC output from DUT

    // ------------------------------------------------------------------------
    // Device Under Test (DUT) Instantiation
    // ------------------------------------------------------------------------
    processing_element #(
        .DATA_WIDTH(DATA_WIDTH),
        .ACC_WIDTH (ACC_WIDTH)
    ) uut (
        .clk        (clk),
        .rst_n      (rst_n),
        .clear_acc  (clear_acc),
        .a_in       (a_in),
        .b_in       (b_in),
        .a_valid_in (a_valid_in),
        .b_valid_in (b_valid_in),
        .a_out      (a_out),
        .b_out      (b_out),
        .a_valid_out(a_valid_out),
        .b_valid_out(b_valid_out),
        .acc_out    (acc_out)
    );

    // ------------------------------------------------------------------------
    // Clock Generation:
    // Toggles every 5ns -> Period = 10ns -> Frequency = 100 MHz
    // ------------------------------------------------------------------------
    always #5 clk = ~clk;

    // Helper variables for tests
    reg signed [DATA_WIDTH-1:0] min_val;       // Minimum representable signed value (-2^(DATA_WIDTH-1))
    reg signed [DATA_WIDTH-1:0] max_val;       // Maximum representable signed value (2^(DATA_WIDTH-1) - 1)
    reg signed [ACC_WIDTH-1:0]  exp_boundary;  // Expected result for boundary multiplication
    reg signed [ACC_WIDTH-1:0]  exp_wrap;      // Expected result during overflow wrap test
    integer wrap_step;                         // Loop index for overflow wrap iterations

    // ------------------------------------------------------------------------
    // Main Test Stimulus Process
    // ------------------------------------------------------------------------
    initial begin
        // Configure VCD waveform dump for GTKWave debugging
        if (DATA_WIDTH == 8)
            $dumpfile("sim/waveforms/pe_8bit.vcd");
        else
            $dumpfile("sim/waveforms/pe_16bit.vcd");
        $dumpvars(0, tb_processing_element);

        // --------------------------------------------------------------------
        // Step 0: Initial Signal Setup & Reset Pulse
        // --------------------------------------------------------------------
        clk        = 0;
        rst_n      = 0;  // Assert active-low reset
        clear_acc  = 0;
        a_in       = {DATA_WIDTH{1'b0}};
        b_in       = {DATA_WIDTH{1'b0}};
        a_valid_in = 1'b0;
        b_valid_in = 1'b0;

        // Hold reset for 2 clock cycles, then release on negedge clk
        // (Applying changes on negedge avoids setup/hold race conditions with posedge)
        repeat (2) @(negedge clk);
        rst_n = 1;  // Deassert reset (DUT is now active)
        @(negedge clk);

        // Clear accumulator to ensure clean starting state
        clear_acc = 1;
        @(negedge clk);
        clear_acc = 0;

        // --------------------------------------------------------------------
        // Test 1: Positive signed multiplication (3 * 4 = 12)
        // --------------------------------------------------------------------
        a_in       = 3;
        b_in       = 4;
        a_valid_in = 1'b1;
        b_valid_in = 1'b1;
        @(negedge clk);     // Clock edge causes PE to compute: acc = 0 + (3 * 4) = 12
        a_valid_in = 1'b0;  // Stop feeding
        b_valid_in = 1'b0;

        if (acc_out !== 12)
            $fatal(1, "Test 1 Failed: Positive mult. Expected 12, got %0d", acc_out);
        $display("PASS: Test 1 - Positive multiplication");

        // --------------------------------------------------------------------
        // Test 2: Negative signed multiplication (-3 * 5 = -15)
        // --------------------------------------------------------------------
        clear_acc = 1;      // Clear accumulator
        @(negedge clk);
        clear_acc = 0;

        a_in       = -3;
        b_in       = 5;
        a_valid_in = 1'b1;
        b_valid_in = 1'b1;
        @(negedge clk);     // Clock edge: acc = 0 + (-3 * 5) = -15
        a_valid_in = 1'b0;
        b_valid_in = 1'b0;

        if (acc_out !== -15)
            $fatal(1, "Test 2 Failed: Negative mult. Expected -15, got %0d", acc_out);
        $display("PASS: Test 2 - Negative multiplication");

        // --------------------------------------------------------------------
        // Test 3: Multiplication with both operands negative (-3 * -4 = +12)
        // --------------------------------------------------------------------
        clear_acc = 1;
        @(negedge clk);
        clear_acc = 0;

        a_in       = -3;
        b_in       = -4;
        a_valid_in = 1'b1;
        b_valid_in = 1'b1;
        @(negedge clk);     // Clock edge: acc = 0 + (-3 * -4) = 12
        a_valid_in = 1'b0;
        b_valid_in = 1'b0;

        if (acc_out !== 12)
            $fatal(1, "Test 3 Failed: Both negative. Expected 12, got %0d", acc_out);
        $display("PASS: Test 3 - Both negative operands");

        // --------------------------------------------------------------------
        // Test 4: Multiplication by zero (0 * 7 = 0, and -5 * 0 = 0)
        // --------------------------------------------------------------------
        clear_acc = 1;
        @(negedge clk);
        clear_acc = 0;

        a_in       = 0;
        b_in       = 7;
        a_valid_in = 1'b1;
        b_valid_in = 1'b1;
        @(negedge clk);

        if (acc_out !== 0)
            $fatal(1, "Test 4 Failed: Expected 0 for 0 * 7, got %0d", acc_out);

        a_in       = -5;
        b_in       = 0;
        a_valid_in = 1'b1;
        b_valid_in = 1'b1;
        @(negedge clk);
        a_valid_in = 1'b0;
        b_valid_in = 1'b0;

        if (acc_out !== 0)
            $fatal(1, "Test 4 Failed: Expected 0 for -5 * 0, got %0d", acc_out);
        $display("PASS: Test 4 - Multiplication by zero");

        // --------------------------------------------------------------------
        // Test 5: Multiple consecutive MAC operations
        // Computes: (2*3) + (4*5) + (-2*6) + (-3*-5)
        //         = 6     + 20    - 12     + 15 = 29
        // --------------------------------------------------------------------
        clear_acc = 1;
        @(negedge clk);
        clear_acc = 0;

        // Step 1: 2 * 3 = 6
        a_in = 2; b_in = 3; a_valid_in = 1'b1; b_valid_in = 1'b1;
        @(negedge clk);

        // Step 2: + (4 * 5 = 20) -> acc = 26
        a_in = 4; b_in = 5;
        @(negedge clk);

        // Step 3: + (-2 * 6 = -12) -> acc = 14
        a_in = -2; b_in = 6;
        @(negedge clk);

        // Step 4: + (-3 * -5 = 15) -> acc = 29
        a_in = -3; b_in = -5;
        @(negedge clk);
        a_valid_in = 1'b0;
        b_valid_in = 1'b0;

        if (acc_out !== 29)
            $fatal(1, "Test 5 Failed: Multiple MAC. Expected 29, got %0d", acc_out);
        $display("PASS: Test 5 - Multiple consecutive MAC operations");

        // --------------------------------------------------------------------
        // Test 6: Validity Gating - Invalid A and Valid B
        // Even though B is valid, A is invalid (valid_in = 0). Accumulator must hold 29!
        // --------------------------------------------------------------------
        a_in       = 10;
        b_in       = 10;
        a_valid_in = 1'b0;  // Invalid A!
        b_valid_in = 1'b1;
        @(negedge clk);

        if (acc_out !== 29)
            $fatal(1, "Test 6 Failed: Accumulator changed when A was invalid! Expected 29, got %0d", acc_out);
        $display("PASS: Test 6 - Invalid A and valid B");

        // --------------------------------------------------------------------
        // Test 7: Validity Gating - Valid A and Invalid B
        // Even though A is valid, B is invalid (valid_in = 0). Accumulator must hold 29!
        // --------------------------------------------------------------------
        a_in       = 10;
        b_in       = 10;
        a_valid_in = 1'b1;
        b_valid_in = 1'b0;  // Invalid B!
        @(negedge clk);
        a_valid_in = 1'b0;
        b_valid_in = 1'b0;

        if (acc_out !== 29)
            $fatal(1, "Test 7 Failed: Accumulator changed when B was invalid! Expected 29, got %0d", acc_out);
        $display("PASS: Test 7 - Valid A and invalid B");

        // --------------------------------------------------------------------
        // Test 8: Accumulator Clear after Nonzero Result
        // Asserting 'clear_acc' must immediately reset acc_out from 29 back to 0.
        // --------------------------------------------------------------------
        clear_acc = 1'b1;
        @(negedge clk);
        clear_acc = 1'b0;

        if (acc_out !== 0)
            $fatal(1, "Test 8 Failed: clear_acc did not reset accumulator to 0! Got %0d", acc_out);
        $display("PASS: Test 8 - Accumulator clear after nonzero result");

        // --------------------------------------------------------------------
        // Test 9: Clear Priority over Simultaneous MAC
        // If 'clear_acc' is asserted at the EXACT SAME cycle as valid inputs,
        // clear must take priority (acc_out <= 0, NOT 0 + 7*7).
        // --------------------------------------------------------------------
        a_in       = 7;
        b_in       = 7;
        a_valid_in = 1'b1;
        b_valid_in = 1'b1;
        clear_acc  = 1'b1;  // Simultaneous clear!
        @(negedge clk);
        clear_acc  = 1'b0;

        if (acc_out !== 0)
            $fatal(1, "Test 9 Failed: clear_acc did not take priority over MAC! Expected 0, got %0d", acc_out);

        // Next cycle: verify normal MAC resumes after clear_acc is deasserted
        a_in       = 3;
        b_in       = 7;
        a_valid_in = 1'b1;
        b_valid_in = 1'b1;
        @(negedge clk);
        a_valid_in = 1'b0;
        b_valid_in = 1'b0;

        if (acc_out !== 21)
            $fatal(1, "Test 9 Failed: MAC operation after clear failed! Expected 21, got %0d", acc_out);
        $display("PASS: Test 9 - Clear priority over simultaneous MAC");

        // --------------------------------------------------------------------
        // Test 10: Asynchronous Reset Verification
        // Reset (rst_n = 0) must immediately clear all registers asynchronously
        // without waiting for the next clock edge.
        // --------------------------------------------------------------------
        a_in       = 42;
        b_in       = -84;
        a_valid_in = 1'b1;
        b_valid_in = 1'b1;
        @(negedge clk);

        rst_n = 1'b0;  // Assert async reset
        #1;            // Wait 1ns for propagation
        if (acc_out !== 0 || a_out !== 0 || b_out !== 0 || a_valid_out !== 1'b0 || b_valid_out !== 1'b0)
            $fatal(1, "Test 10 Failed: Reset did not immediately clear all registers!");

        repeat (2) @(negedge clk);
        rst_n = 1'b1;  // Release reset
        a_in = 0; b_in = 0; a_valid_in = 1'b0; b_valid_in = 1'b0;
        @(negedge clk);

        if (acc_out !== 0 || a_out !== 0 || b_out !== 0 || a_valid_out !== 1'b0 || b_valid_out !== 1'b0)
            $fatal(1, "Test 10 Failed: Registers nonzero after reset release!");
        $display("PASS: Test 10 - Reset functionality");

        // --------------------------------------------------------------------
        // Test 11: Data Forwarding Verification
        // Ensures that a_in and b_in appear at a_out and b_out after exactly
        // 1 clock cycle of register delay (systolic pipeline flow).
        // --------------------------------------------------------------------
        a_in       = 55;
        b_in       = -77;
        a_valid_in = 1'b0;
        b_valid_in = 1'b0;
        @(negedge clk);

        if (a_out !== 55 || b_out !== -77)
            $fatal(1, "Test 11 Failed: Data forwarding incorrect! a_out=%0d, b_out=%0d", a_out, b_out);

        a_in = -110;
        b_in = 99;
        @(negedge clk);

        if (a_out !== -110 || b_out !== 99)
            $fatal(1, "Test 11 Failed: Data forwarding incorrect! a_out=%0d, b_out=%0d", a_out, b_out);
        $display("PASS: Test 11 - Data forwarding");

        // --------------------------------------------------------------------
        // Test 12: Valid-Signal Forwarding Verification
        // Ensures that a_valid_out and b_valid_out faithfully reflect
        // a_valid_in and b_valid_in delayed by 1 clock cycle.
        // --------------------------------------------------------------------
        a_valid_in = 1'b1; b_valid_in = 1'b0;
        @(negedge clk);
        if (a_valid_out !== 1'b1 || b_valid_out !== 1'b0)
            $fatal(1, "Test 12 Failed: Valid forwarding failed for 1/0! a_valid_out=%0b, b_valid_out=%0b", a_valid_out, b_valid_out);

        a_valid_in = 1'b0; b_valid_in = 1'b1;
        @(negedge clk);
        if (a_valid_out !== 1'b0 || b_valid_out !== 1'b1)
            $fatal(1, "Test 12 Failed: Valid forwarding failed for 0/1! a_valid_out=%0b, b_valid_out=%0b", a_valid_out, b_valid_out);

        a_valid_in = 1'b0; b_valid_in = 1'b0;
        @(negedge clk);
        if (a_valid_out !== 1'b0 || b_valid_out !== 1'b0)
            $fatal(1, "Test 12 Failed: Valid forwarding failed for 0/0! a_valid_out=%0b, b_valid_out=%0b", a_valid_out, b_valid_out);
        $display("PASS: Test 12 - Valid-signal forwarding");

        // --------------------------------------------------------------------
        // Test 13: Signed Boundary Extremes
        // Tests edge cases of two's complement integer arithmetic:
        //   - For 8-bit: min_val = -128, max_val = +127
        //   - Checks: max*max, min*max, min*min, min*-1
        // --------------------------------------------------------------------
        min_val = - (1 <<< (DATA_WIDTH - 1));       // -128 for 8-bit
        max_val = (1 <<< (DATA_WIDTH - 1)) - 1;    // +127 for 8-bit

        // Check max * max
        clear_acc = 1'b1;
        @(negedge clk);
        clear_acc = 1'b0;
        a_in = max_val; b_in = max_val; a_valid_in = 1'b1; b_valid_in = 1'b1;
        exp_boundary = $signed(max_val) * $signed(max_val);
        @(negedge clk);
        if (acc_out !== exp_boundary)
            $fatal(1, "Test 13 Failed: max * max. Expected %0d, got %0d", exp_boundary, acc_out);

        // Check min * max
        clear_acc = 1'b1;
        @(negedge clk);
        clear_acc = 1'b0;
        a_in = min_val; b_in = max_val; a_valid_in = 1'b1; b_valid_in = 1'b1;
        exp_boundary = $signed(min_val) * $signed(max_val);
        @(negedge clk);
        if (acc_out !== exp_boundary)
            $fatal(1, "Test 13 Failed: min * max. Expected %0d, got %0d", exp_boundary, acc_out);

        // Check min * min
        clear_acc = 1'b1;
        @(negedge clk);
        clear_acc = 1'b0;
        a_in = min_val; b_in = min_val; a_valid_in = 1'b1; b_valid_in = 1'b1;
        exp_boundary = $signed(min_val) * $signed(min_val);
        @(negedge clk);
        if (acc_out !== exp_boundary)
            $fatal(1, "Test 13 Failed: min * min. Expected %0d, got %0d", exp_boundary, acc_out);

        // Check min * -1 (potential asymmetry in two's complement)
        clear_acc = 1'b1;
        @(negedge clk);
        clear_acc = 1'b0;
        a_in = min_val; b_in = -1; a_valid_in = 1'b1; b_valid_in = 1'b1;
        exp_boundary = $signed(min_val) * -1;
        @(negedge clk);
        a_valid_in = 1'b0; b_valid_in = 1'b0;
        if (acc_out !== exp_boundary)
            $fatal(1, "Test 13 Failed: min * -1. Expected %0d, got %0d", exp_boundary, acc_out);

        $display("PASS: Test 13 - Signed boundary values");

        // --------------------------------------------------------------------
        // Test 14: Accumulator Overflow Wrapping (Two's Complement Arithmetic)
        // Accumulates (-128 * -128 = 16384) over 131,073 iterations to cross
        // the 32-bit signed boundary (2^31 - 1 = 2,147,483,647) and verify that
        // hardware accumulator wraps correctly according to standard two's complement.
        // --------------------------------------------------------------------
        clear_acc = 1'b1;
        @(negedge clk);
        clear_acc = 1'b0;

        exp_wrap   = {ACC_WIDTH{1'b0}};
        a_in       = min_val;
        b_in       = min_val;
        a_valid_in = 1'b1;
        b_valid_in = 1'b1;

        for (wrap_step = 0; wrap_step < 131073; wrap_step = wrap_step + 1) begin
            exp_wrap = exp_wrap + ($signed(min_val) * $signed(min_val));
            @(negedge clk);
        end

        a_valid_in = 1'b0;
        b_valid_in = 1'b0;

        $display("Test 14: acc_out = %0d, exp_wrap = %0d", acc_out, exp_wrap);

        if (acc_out !== exp_wrap)
            $fatal(1, "Test 14 Failed: Accumulator overflow wrap mismatch! Expected %0d, got %0d", exp_wrap, acc_out);

        $display("PASS: Test 14 - Accumulator overflow wraps (two's complement)");

        // --------------------------------------------------------------------
        // Testbench Summary
        // --------------------------------------------------------------------
        @(negedge clk);
        $display("========================================");
        $display("ALL 14 PE TESTS PASSED SUCCESSFULLY (DATA_WIDTH=%0d, ACC_WIDTH=%0d)", DATA_WIDTH, ACC_WIDTH);
        $display("========================================");
        $finish;  // End simulation successfully
    end

endmodule
