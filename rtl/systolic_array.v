`timescale 1ns/1ps

// ============================================================================
// Module: systolic_array
// Description:
//   2D Mesh Systolic Array with Input Skewing Registers for Matrix Multiplication.
//
//   Architecture Overview:
//   - An N x N grid of Processing Elements (PEs).
//   - Data Flow:
//       * Matrix A streams horizontally from West to East (row by row).
//       * Matrix B streams vertically from North to South (col by col).
//   - Why Skewing Registers are Required:
//       In a systolic array, PE(row, col) computes its product at time:
//           t = k + row + col
//       To ensure that corresponding operands A[row][k] and B[k][col] arrive
//       at PE(row, col) simultaneously at cycle 't', the input for row 'r' must
//       be delayed by 'r' clock cycles, and column 'c' by 'c' clock cycles.
//       This module contains built-in shift registers (input_skew) that delay
//       each input lane by exactly 'lane' clock cycles.
// ============================================================================

module systolic_array #(
    parameter ARRAY_SIZE = 4,  // Array dimension N (creates an N x N PE grid)
    parameter DATA_WIDTH = 8,  // Operand bit-width (signed)
    parameter ACC_WIDTH  = 32  // Accumulator bit-width (signed)
) (
    // Global Clock and Control
    input  wire clk,        // System clock
    input  wire rst_n,      // Active-low asynchronous reset
    input  wire clear_acc,  // Synchronous accumulator clear (resets all PEs to 0)

    // Parallel Input Buses from Controller (Unskewed, directly from controller)
    input  wire [ARRAY_SIZE*DATA_WIDTH-1:0] a_in,  // Packed row operands for Matrix A [lane 0 .. lane N-1]
    input  wire [ARRAY_SIZE*DATA_WIDTH-1:0] b_in,  // Packed col operands for Matrix B [lane 0 .. lane N-1]

    // Valid flags for each input lane
    input  wire [ARRAY_SIZE-1:0] a_valid_in,  // Valid mask for A row inputs
    input  wire [ARRAY_SIZE-1:0] b_valid_in,  // Valid mask for B col inputs

    // Packed Accumulator Outputs from all N x N PEs
    // Flattened array of results: PE(row, col) output is at index (row*N + col)*ACC_WIDTH
    output wire [ARRAY_SIZE*ARRAY_SIZE*ACC_WIDTH-1:0] acc_out
);

    // ------------------------------------------------------------------------
    // Inter-PE Interconnection Wires (Horizontal and Vertical Links)
    // ------------------------------------------------------------------------
    // a_link[row][col]: carries Matrix A data from PE(row, col-1) output to PE(row, col) input
    // Column 0 is the skewed input from the west boundary; column ARRAY_SIZE is the east boundary output.
    wire signed [DATA_WIDTH-1:0] a_link [0:ARRAY_SIZE-1][0:ARRAY_SIZE];

    // b_link[row][col]: carries Matrix B data from PE(row-1, col) output to PE(row, col) input
    // Row 0 is the skewed input from the north boundary; row ARRAY_SIZE is the south boundary output.
    wire signed [DATA_WIDTH-1:0] b_link [0:ARRAY_SIZE][0:ARRAY_SIZE-1];

    // Valid handshake signals propagating alongside data through the mesh
    wire a_valid_link [0:ARRAY_SIZE-1][0:ARRAY_SIZE];
    wire b_valid_link [0:ARRAY_SIZE][0:ARRAY_SIZE-1];

    // ------------------------------------------------------------------------
    // Elaboration Assertions
    // ------------------------------------------------------------------------
    initial begin
        if (ARRAY_SIZE < 1 || DATA_WIDTH < 1 || ACC_WIDTH < 2*DATA_WIDTH)
            $fatal(1, "Require ARRAY_SIZE >= 1, DATA_WIDTH >= 1, ACC_WIDTH >= 2*DATA_WIDTH");
    end

    // ------------------------------------------------------------------------
    // Generate Block 1: Input Skewing Registers
    // ------------------------------------------------------------------------
    // Lane 0: 0 delay stages (direct connection to array boundary)
    // Lane 1: 1 flip-flop delay stage
    // Lane 2: 2 flip-flop delay stages in series ...
    // Lane 'lane': 'lane' flip-flop delay stages in series.
    // ------------------------------------------------------------------------
    genvar lane, stage, row, col;
    generate
        for (lane = 0; lane < ARRAY_SIZE; lane = lane + 1) begin : input_skew
            if (lane == 0) begin : direct
                // ------------------------------------------------------------
                // Lane 0: No delay needed (0 clock cycles of skew)
                // Connect directly to the mesh edge:
                // a_link[0][0] = West edge of PE(0,0)
                // b_link[0][0] = North edge of PE(0,0)
                // ------------------------------------------------------------
                assign a_link[lane][0]       = a_in[DATA_WIDTH*lane +: DATA_WIDTH];
                assign b_link[0][lane]       = b_in[DATA_WIDTH*lane +: DATA_WIDTH];
                assign a_valid_link[lane][0] = a_valid_in[lane];
                assign b_valid_link[0][lane] = b_valid_in[lane];
            end else begin : delayed
                // ------------------------------------------------------------
                // Lanes 1 to N-1: Need 'lane' flip-flop delay stages
                // ------------------------------------------------------------
                // Shift register arrays to hold intermediate delay stages
                reg signed [DATA_WIDTH-1:0] a_delay [0:lane-1];
                reg signed [DATA_WIDTH-1:0] b_delay [0:lane-1];
                reg [lane-1:0] a_valid_delay;
                reg [lane-1:0] b_valid_delay;

                // Chain 'lane' flip-flops in series: stage 0 takes external input,
                // stage 's' takes output of stage 's-1'.
                for (stage = 0; stage < lane; stage = stage + 1) begin : delay_stage
                    wire signed [DATA_WIDTH-1:0] a_source, b_source;
                    wire av_source, bv_source;

                    if (stage == 0) begin : first_stage
                        // Stage 0 connects to the external input from controller
                        assign a_source  = a_in[DATA_WIDTH*lane +: DATA_WIDTH];
                        assign b_source  = b_in[DATA_WIDTH*lane +: DATA_WIDTH];
                        assign av_source = a_valid_in[lane];
                        assign bv_source = b_valid_in[lane];
                    end else begin : subsequent_stage
                        // Subsequent stages connect to the previous flip-flop output
                        assign a_source  = a_delay[stage-1];
                        assign b_source  = b_delay[stage-1];
                        assign av_source = a_valid_delay[stage-1];
                        assign bv_source = b_valid_delay[stage-1];
                    end

                    // Synchronous shift register update
                    always @(posedge clk or negedge rst_n) begin
                        if (!rst_n) begin
                            // Reset clears delay line
                            a_delay[stage]       <= {DATA_WIDTH{1'b0}};
                            b_delay[stage]       <= {DATA_WIDTH{1'b0}};
                            a_valid_delay[stage] <= 1'b0;
                            b_valid_delay[stage] <= 1'b0;
                        end else if (clear_acc) begin
                            // clear_acc also flushes any leftover skewed operands
                            a_delay[stage]       <= {DATA_WIDTH{1'b0}};
                            b_delay[stage]       <= {DATA_WIDTH{1'b0}};
                            a_valid_delay[stage] <= 1'b0;
                            b_valid_delay[stage] <= 1'b0;
                        end else begin
                            // Shift data through delay chain on each clock tick
                            a_delay[stage]       <= a_source;
                            b_delay[stage]       <= b_source;
                            a_valid_delay[stage] <= av_source;
                            b_valid_delay[stage] <= bv_source;
                        end
                    end
                end

                // Connect the final delay stage (lane-1) to the array mesh boundary
                assign a_link[lane][0]       = a_delay[lane-1];
                assign b_link[0][lane]       = b_delay[lane-1];
                assign a_valid_link[lane][0] = a_valid_delay[lane-1];
                assign b_valid_link[0][lane] = b_valid_delay[lane-1];
            end
        end

        // --------------------------------------------------------------------
        // Generate Block 2: 2D Grid of Processing Elements (PEs)
        // --------------------------------------------------------------------
        // Instantiates N x N PEs arranged in rows and columns:
        // - Row 'row' runs from 0 to N-1
        // - Col 'col' runs from 0 to N-1
        // --------------------------------------------------------------------
        for (row = 0; row < ARRAY_SIZE; row = row + 1) begin : rows
            for (col = 0; col < ARRAY_SIZE; col = col + 1) begin : cols
                processing_element #(
                    .DATA_WIDTH(DATA_WIDTH),
                    .ACC_WIDTH(ACC_WIDTH)
                ) pe (
                    .clk(clk),
                    .rst_n(rst_n),
                    .clear_acc(clear_acc),

                    // Inputs from West (A) and North (B)
                    .a_in(a_link[row][col]),
                    .b_in(b_link[row][col]),

                    // Valid inputs: masked with ~clear_acc so clear cycle doesn't compute
                    .a_valid_in(a_valid_link[row][col] & ~clear_acc),
                    .b_valid_in(b_valid_link[row][col] & ~clear_acc),

                    // Forwarded outputs to East (a_out) and South (b_out)
                    .a_out(a_link[row][col+1]),
                    .b_out(b_link[row+1][col]),

                    // Forwarded valid flags to East and South
                    .a_valid_out(a_valid_link[row][col+1]),
                    .b_valid_out(b_valid_link[row+1][col]),

                    // Tap internal accumulator output into the packed result bus
                    // Linear coordinate for PE(row, col) = (ARRAY_SIZE * row) + col
                    .acc_out(acc_out[ACC_WIDTH*(ARRAY_SIZE*row+col) +: ACC_WIDTH])
                );
            end
        end
    endgenerate

endmodule
