`timescale 1ns/1ps

// Top-level system module connecting matrix_controller and systolic_array
module systolic_array_top #(
    parameter ARRAY_SIZE = 4,   // Matrix dimension N (N x N)
    parameter DATA_WIDTH = 8,   // Input element width
    parameter ACC_WIDTH  = 32   // Output accumulator width
) (
    input  wire                                         clk,    // System clock
    input  wire                                         rst_n,  // Active-low async reset
    input  wire                                         start,  // 1-cycle start pulse

    input  wire signed [ARRAY_SIZE*ARRAY_SIZE*DATA_WIDTH-1:0] a_flat, // Packed input matrix A
    input  wire signed [ARRAY_SIZE*ARRAY_SIZE*DATA_WIDTH-1:0] b_flat, // Packed input matrix B

    output wire                                         busy,   // High during computation
    output wire                                         done,   // 1-cycle pulse when result is valid
    output wire signed [ARRAY_SIZE*ARRAY_SIZE*ACC_WIDTH-1:0] result   // Final output matrix C = A * B
);

    initial begin
        if (ARRAY_SIZE < 1)
            $fatal(1, "systolic_array_top requires ARRAY_SIZE >= 1");
        if (DATA_WIDTH < 1)
            $fatal(1, "systolic_array_top requires DATA_WIDTH >= 1");
        if (ACC_WIDTH < 2 * DATA_WIDTH)
            $fatal(1, "systolic_array_top requires ACC_WIDTH >= 2*DATA_WIDTH");
    end

    // Internal wires linking controller to array core
    wire                                                 clear_acc_w;
    wire signed [ARRAY_SIZE*DATA_WIDTH-1:0]              a_lane_w;
    wire signed [ARRAY_SIZE*DATA_WIDTH-1:0]              b_lane_w;
    wire [ARRAY_SIZE-1:0]                                a_valid_w;
    wire [ARRAY_SIZE-1:0]                                b_valid_w;
    wire signed [ARRAY_SIZE*ARRAY_SIZE*ACC_WIDTH-1:0]    acc_w;

    // FSM controller: manages handshake, input slicing, timing, and result capture
    matrix_controller #(
        .ARRAY_SIZE(ARRAY_SIZE),
        .DATA_WIDTH(DATA_WIDTH),
        .ACC_WIDTH (ACC_WIDTH)
    ) ctrl (
        .clk       (clk),
        .rst_n     (rst_n),
        .start     (start),
        .a_flat    (a_flat),
        .b_flat    (b_flat),
        .busy      (busy),
        .done      (done),
        .clear_acc (clear_acc_w),
        .a_out     (a_lane_w),
        .b_out     (b_lane_w),
        .a_valid   (a_valid_w),
        .b_valid   (b_valid_w),
        .acc_in    (acc_w),
        .result    (result)
    );

    // 2D PE grid with input skew registers
    systolic_array #(
        .ARRAY_SIZE(ARRAY_SIZE),
        .DATA_WIDTH(DATA_WIDTH),
        .ACC_WIDTH (ACC_WIDTH)
    ) array (
        .clk        (clk),
        .rst_n      (rst_n),
        .clear_acc  (clear_acc_w),
        .a_in       (a_lane_w),
        .b_in       (b_lane_w),
        .a_valid_in (a_valid_w),
        .b_valid_in (b_valid_w),
        .acc_out    (acc_w)
    );

endmodule
