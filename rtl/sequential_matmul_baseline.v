`timescale 1ns/1ps

// Single-MAC sequential matrix multiplier baseline (takes N^3 cycles; used for speedup comparison)
module sequential_matmul_baseline #(
    parameter ARRAY_SIZE = 4,   // Matrix dimension N (N x N)
    parameter DATA_WIDTH = 8,   // Input element width
    parameter ACC_WIDTH = 32    // Accumulator width
) (
    input wire clk,             // System clock
    input wire rst_n,           // Active-low async reset
    input wire start,           // 1-cycle pulse to start

    input wire signed [ARRAY_SIZE * ARRAY_SIZE * DATA_WIDTH - 1 : 0] a_flat, // Packed matrix A
    input wire signed [ARRAY_SIZE * ARRAY_SIZE * DATA_WIDTH - 1 : 0] b_flat, // Packed matrix B

    output reg busy,            // High while computing
    output reg done,            // 1-cycle pulse when finished
    output reg signed [ARRAY_SIZE * ARRAY_SIZE * ACC_WIDTH - 1 : 0] result   // Final matrix C
);

    // FSM states
    localparam STATE_IDLE    = 2'd0;
    localparam STATE_COMPUTE = 2'd1;
    localparam STATE_DONE    = 2'd2;

    reg [1:0] state;

    // Latched input matrices
    reg signed [ARRAY_SIZE * ARRAY_SIZE * DATA_WIDTH - 1 : 0] a_reg;
    reg signed [ARRAY_SIZE * ARRAY_SIZE * DATA_WIDTH - 1 : 0] b_reg;

    // 3 nested loop counters: r (row), c (col), k (dot-product index)
    reg [15:0] r_cnt;
    reg [15:0] c_cnt;
    reg [15:0] k_cnt;

    reg signed [ACC_WIDTH-1:0] acc; // Single shared MAC accumulator

    wire signed [DATA_WIDTH-1:0] a_val;       // Current A[r][k]
    wire signed [DATA_WIDTH-1:0] b_val;       // Current B[k][c]
    wire signed [2*DATA_WIDTH-1:0] mult_res;  // Product a_val * b_val
    wire signed [ACC_WIDTH-1:0] mult_ext;     // Sign-extended product
    wire signed [ACC_WIDTH-1:0] current_sum;  // Running dot-product sum

    // Fetch A[r_cnt][k_cnt] and B[k_cnt][c_cnt] from row-major flat buses
    assign a_val = a_reg[(r_cnt * ARRAY_SIZE + k_cnt) * DATA_WIDTH +: DATA_WIDTH];
    assign b_val = b_reg[(k_cnt * ARRAY_SIZE + c_cnt) * DATA_WIDTH +: DATA_WIDTH];
    assign mult_res = a_val * b_val;

    // Sign-extend product to accumulator width
    generate
        if (ACC_WIDTH > 2 * DATA_WIDTH) begin : gen_ext
            assign mult_ext = {{(ACC_WIDTH - 2*DATA_WIDTH){mult_res[2*DATA_WIDTH-1]}}, mult_res};
        end else begin : gen_no_ext
            assign mult_ext = mult_res;
        end
    endgenerate

    // Start fresh sum when k_cnt == 0, otherwise add to previous acc
    assign current_sum = (k_cnt == 16'd0) ? mult_ext : (acc + mult_ext);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= STATE_IDLE;
            busy <= 1'b0;
            done <= 1'b0;
            r_cnt <= 16'd0;
            c_cnt <= 16'd0;
            k_cnt <= 16'd0;
            acc <= {ACC_WIDTH{1'b0}};
            a_reg <= {(ARRAY_SIZE * ARRAY_SIZE * DATA_WIDTH){1'b0}};
            b_reg <= {(ARRAY_SIZE * ARRAY_SIZE * DATA_WIDTH){1'b0}};
            result <= {(ARRAY_SIZE * ARRAY_SIZE * ACC_WIDTH){1'b0}};
        end else begin
            case (state)
                STATE_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy <= 1'b1;
                        a_reg <= a_flat;
                        b_reg <= b_flat;
                        r_cnt <= 16'd0;
                        c_cnt <= 16'd0;
                        k_cnt <= 16'd0;
                        acc <= {ACC_WIDTH{1'b0}};
                        state <= STATE_COMPUTE;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                STATE_COMPUTE: begin
                    if (k_cnt < ARRAY_SIZE - 1) begin
                        // Accumulate inner loop (k)
                        acc <= current_sum;
                        k_cnt <= k_cnt + 16'd1;
                    end else begin
                        // Dot product finished for C[r][c]: save result and reset k & acc
                        result[(r_cnt * ARRAY_SIZE + c_cnt) * ACC_WIDTH +: ACC_WIDTH] <= current_sum;
                        k_cnt <= 16'd0;
                        acc <= {ACC_WIDTH{1'b0}};
                        if (c_cnt < ARRAY_SIZE - 1) begin
                            c_cnt <= c_cnt + 16'd1; // Next column
                        end else begin
                            c_cnt <= 16'd0;
                            if (r_cnt < ARRAY_SIZE - 1) begin
                                r_cnt <= r_cnt + 16'd1; // Next row
                            end else begin
                                // All N^3 MAC cycles finished
                                r_cnt <= 16'd0;
                                busy <= 1'b0;
                                done <= 1'b1;
                                state <= STATE_DONE;
                            end
                        end
                    end
                end

                STATE_DONE: begin
                    done <= 1'b0;
                    if (start) begin
                        // Support immediate back-to-back start
                        busy <= 1'b1;
                        a_reg <= a_flat;
                        b_reg <= b_flat;
                        r_cnt <= 16'd0;
                        c_cnt <= 16'd0;
                        k_cnt <= 16'd0;
                        acc <= {ACC_WIDTH{1'b0}};
                        state <= STATE_COMPUTE;
                    end else begin
                        busy <= 1'b0;
                        state <= STATE_IDLE;
                    end
                end

                default: begin
                    state <= STATE_IDLE;
                end
            endcase
        end
    end

endmodule
