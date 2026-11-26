`timescale 1ns/1ps

// ============================================================================
// Module: systolic_array_top
// Description:
//   Top-level System Integration Module for the Systolic Array Matrix Multiplier.
//
//   Structural Composition:
//   1. matrix_controller (ctrl):
//      - Handles top-level handshake (start / busy / done).
//      - Latches incoming packed matrices (a_flat, b_flat).
//      - Generates the cycle-by-cycle parallel row/col feeds (a_out, b_out)
//        and validity signals (a_valid, b_valid).
//      - Issues the initial 'clear_acc' pulse.
//      - Counts completion latency and latches final results from the array into 'result'.
//   2. systolic_array (array):
//      - Contains input skewing delay registers for wave-front timing alignment.
//      - Contains the 2D grid (N x N) of Processing Elements (PEs) that execute MAC operations.
//      - Outputs accumulated values across all PEs (acc_w) back to the controller.
// ============================================================================

module systolic_array_top #(
    parameter ARRAY_SIZE = 4,  // Square matrix dimension N (e.g., 4 for a 4x4 matrix)
    parameter DATA_WIDTH = 8,  // Input element bit-width (signed integer)
    parameter ACC_WIDTH  = 32  // Output element bit-width (signed integer accumulator)
) (
    // Global Clock and Active-Low Reset
    input  wire clk,    // Main clock input
    input  wire rst_n,  // Active-low asynchronous reset

    // Control Handshake Interface
    input  wire start,  // 1-cycle active-high pulse: starts matrix multiplication
    output wire busy,   // High while computation is in progress
    output wire done,   // Active-high 1-cycle pulse indicating valid matrix result

    // Flattened Matrix Inputs (Row-major packing: N*N elements * DATA_WIDTH bits)
    input  wire signed [ARRAY_SIZE*ARRAY_SIZE*DATA_WIDTH-1:0] a_flat,  // Flattened Matrix A input
    input  wire signed [ARRAY_SIZE*ARRAY_SIZE*DATA_WIDTH-1:0] b_flat,  // Flattened Matrix B input

    // Flattened Matrix Output (Row-major packing: N*N elements * ACC_WIDTH bits)
    // C = A * B
    output wire signed [ARRAY_SIZE*ARRAY_SIZE*ACC_WIDTH-1:0] result   // Latched final matrix result
);

    // ------------------------------------------------------------------------
    // Elaboration Parameter Validation
    // ------------------------------------------------------------------------
    initial begin
        if (ARRAY_SIZE < 1)
            $fatal(1, "systolic_array_top requires ARRAY_SIZE >= 1");
        if (DATA_WIDTH < 1)
            $fatal(1, "systolic_array_top requires DATA_WIDTH >= 1");
        if (ACC_WIDTH < 2 * DATA_WIDTH)
            $fatal(1, "systolic_array_top requires ACC_WIDTH >= 2*DATA_WIDTH");
    end

    // ------------------------------------------------------------------------
    // Internal Structural Interconnect Wires
    // (Connecting matrix_controller and systolic_array)
    // ------------------------------------------------------------------------
    wire                                              clear_acc_w;  // Clear pulse from controller to all PEs
    wire signed [ARRAY_SIZE*DATA_WIDTH-1:0]           a_lane_w;     // Parallel row operands from controller to array
    wire signed [ARRAY_SIZE*DATA_WIDTH-1:0]           b_lane_w;     // Parallel col operands from controller to array
    wire [ARRAY_SIZE-1:0]                             a_valid_w;    // Valid flags for row lanes
    wire [ARRAY_SIZE-1:0]                             b_valid_w;    // Valid flags for col lanes
    wire signed [ARRAY_SIZE*ARRAY_SIZE*ACC_WIDTH-1:0] acc_w;        // Tap of all PE accumulators from array to controller

    // ------------------------------------------------------------------------
    // Instance 1: Matrix Controller FSM & Data Sequencer
    // ------------------------------------------------------------------------
    matrix_controller #(
        .ARRAY_SIZE(ARRAY_SIZE),
        .DATA_WIDTH(DATA_WIDTH),
        .ACC_WIDTH (ACC_WIDTH)
    ) ctrl (
        // System connections
        .clk       (clk),
        .rst_n     (rst_n),

        // User / Host Handshake
        .start     (start),
        .busy      (busy),
        .done      (done),

        // Matrix Inputs from external host
        .a_flat    (a_flat),
        .b_flat    (b_flat),

        // Signals driving the systolic array core
        .clear_acc (clear_acc_w),  // Asserted in IDLE on 'start' to flush accumulators
        .a_out     (a_lane_w),     // Fed into array row inputs
        .b_out     (b_lane_w),     // Fed into array col inputs
        .a_valid   (a_valid_w),    // High during LOAD phase
        .b_valid   (b_valid_w),    // High during LOAD phase

        // Tapping accumulator outputs from the array
        .acc_in    (acc_w),        // Raw accumulator values from array PEs

        // Final Latched Output
        .result    (result)        // Stable final result captured at completion
    );

    // ------------------------------------------------------------------------
    // Instance 2: 2D Systolic Array Mesh with Skewing Registers
    // ------------------------------------------------------------------------
    systolic_array #(
        .ARRAY_SIZE(ARRAY_SIZE),
        .DATA_WIDTH(DATA_WIDTH),
        .ACC_WIDTH (ACC_WIDTH)
    ) array (
        // System connections
        .clk        (clk),
        .rst_n      (rst_n),

        // Control from controller
        .clear_acc  (clear_acc_w),

        // Streamed operands from controller
        .a_in       (a_lane_w),
        .b_in       (b_lane_w),
        .a_valid_in (a_valid_w),
        .b_valid_in (b_valid_w),

        // PE Accumulator Bus routed to controller
        .acc_out    (acc_w)
    );

endmodule
