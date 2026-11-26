`timescale 1ns/1ps

// ============================================================================
// Module: sequential_matmul_baseline
// Description:
//   Sequential (Iterative Single-MAC) Reference Matrix Multiplier Baseline.
//
//   Purpose in Project:
//   - Serves as the performance and hardware baseline to benchmark against
//     the 2D systolic array accelerator.
//   - While the systolic array uses N x N = N^2 processing elements in parallel
//     to finish in O(N) cycles (specifically 3N-1 cycles), this sequential
//     baseline uses a SINGLE multiply-accumulate unit that computes the 3 nested
//     matrix loops sequentially:
//         for r = 0 to N-1
//             for c = 0 to N-1
//                 for k = 0 to N-1
//                     C[r][c] += A[r][k] * B[k][c]
//     This requires exactly N^3 clock cycles (e.g. 64 cycles for 4x4, 512 for 8x8).
//   - Identical external interface: uses the same 'start', 'busy', 'done', and
//     packed matrices (a_flat, b_flat, result) for a fair, direct comparison.
// ============================================================================

module sequential_matmul_baseline #(
    parameter ARRAY_SIZE = 4,  // Square matrix dimension N (N x N matrices)
    parameter DATA_WIDTH = 8,  // Input element bit-width (signed integer)
    parameter ACC_WIDTH  = 32  // Accumulator bit-width (signed integer)
) (
    // Global Clock and Active-Low Reset
    input  wire clk,    // Main system clock
    input  wire rst_n,  // Active-low asynchronous reset

    // Control Handshake Interface
    input  wire start,  // 1-cycle active-high pulse: triggers sequential matrix multiplication
    output reg  busy,   // High while sequentially computing products
    output reg  done,   // 1-cycle active-high pulse when all N^3 MACs are done and 'result' is valid

    // Flattened Matrix Inputs (Row-major packing: N*N * DATA_WIDTH)
    input  wire signed [ARRAY_SIZE * ARRAY_SIZE * DATA_WIDTH - 1 : 0] a_flat,  // Flattened Matrix A input
    input  wire signed [ARRAY_SIZE * ARRAY_SIZE * DATA_WIDTH - 1 : 0] b_flat,  // Flattened Matrix B input

    // Flattened Matrix Output (Row-major packing: N*N * ACC_WIDTH)
    output reg  signed [ARRAY_SIZE * ARRAY_SIZE * ACC_WIDTH - 1 : 0] result   // Latched final matrix result
);

    // ------------------------------------------------------------------------
    // FSM State Encoding
    // ------------------------------------------------------------------------
    localparam STATE_IDLE    = 2'd0;  // Waiting for 'start' command
    localparam STATE_COMPUTE = 2'd1;  // Iterating through the 3 nested loops (N^3 cycles)
    localparam STATE_DONE    = 2'd2;  // Asserting 'done' and holding stable results

    reg [1:0] state;  // Current FSM state register

    // Internal input latches to keep inputs stable during the long multi-cycle run
    reg signed [ARRAY_SIZE * ARRAY_SIZE * DATA_WIDTH - 1 : 0] a_reg;  // Latched copy of Matrix A
    reg signed [ARRAY_SIZE * ARRAY_SIZE * DATA_WIDTH - 1 : 0] b_reg;  // Latched copy of Matrix B

    // ------------------------------------------------------------------------
    // Loop Counter Registers (Implementing the 3 nested loops)
    // ------------------------------------------------------------------------
    reg [15:0] r_cnt;  // Outer loop counter: row index 'r' of output C[r][c] (0 to ARRAY_SIZE-1)
    reg [15:0] c_cnt;  // Middle loop counter: column index 'c' of output C[r][c] (0 to ARRAY_SIZE-1)
    reg [15:0] k_cnt;  // Inner loop counter: dot-product accumulation index 'k' (0 to ARRAY_SIZE-1)

    // Single accumulator register for the currently computed element C[r][c]
    reg signed [ACC_WIDTH-1:0] acc;

    // ------------------------------------------------------------------------
    // Combinational Single-MAC Datapath
    // ------------------------------------------------------------------------
    wire signed [DATA_WIDTH-1:0]   a_val;        // Current operand A[r][k]
    wire signed [DATA_WIDTH-1:0]   b_val;        // Current operand B[k][c]
    wire signed [2*DATA_WIDTH-1:0] mult_res;     // Multiplication product: a_val * b_val
    wire signed [ACC_WIDTH-1:0]    mult_ext;     // Sign-extended product to match ACC_WIDTH
    wire signed [ACC_WIDTH-1:0]    current_sum;  // Sum of accumulator and current product

    // Extract element A[r_cnt][k_cnt] from row-major flat register:
    // Linear index = (r_cnt * ARRAY_SIZE) + k_cnt
    assign a_val = a_reg[(r_cnt * ARRAY_SIZE + k_cnt) * DATA_WIDTH +: DATA_WIDTH];

    // Extract element B[k_cnt][c_cnt] from row-major flat register:
    // Linear index = (k_cnt * ARRAY_SIZE) + c_cnt
    assign b_val = b_reg[(k_cnt * ARRAY_SIZE + c_cnt) * DATA_WIDTH +: DATA_WIDTH];

    // Multiply operands (signed integer multiplication)
    assign mult_res = a_val * b_val;

    // Sign extend multiplier output to accumulator width
    generate
        if (ACC_WIDTH > 2 * DATA_WIDTH) begin : gen_ext
            // Replicate MSB sign bit
            assign mult_ext = {{(ACC_WIDTH - 2*DATA_WIDTH){mult_res[2*DATA_WIDTH-1]}}, mult_res};
        end else begin : gen_no_ext
            assign mult_ext = mult_res;
        end
    endgenerate

    // Compute updated accumulation sum:
    // When starting a new element (k_cnt == 0), the sum starts fresh with mult_ext;
    // otherwise it adds mult_ext to the running accumulator 'acc'.
    assign current_sum = (k_cnt == 16'd0) ? mult_ext : (acc + mult_ext);

    // ------------------------------------------------------------------------
    // Sequential State Machine & Counter Progression
    // ------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Asynchronous reset: initialize all registers to 0
            state   <= STATE_IDLE;
            busy    <= 1'b0;
            done    <= 1'b0;
            r_cnt   <= 16'd0;
            c_cnt   <= 16'd0;
            k_cnt   <= 16'd0;
            acc     <= {ACC_WIDTH{1'b0}};
            a_reg   <= {(ARRAY_SIZE * ARRAY_SIZE * DATA_WIDTH){1'b0}};
            b_reg   <= {(ARRAY_SIZE * ARRAY_SIZE * DATA_WIDTH){1'b0}};
            result  <= {(ARRAY_SIZE * ARRAY_SIZE * ACC_WIDTH){1'b0}};
        end else begin
            case (state)
                // ------------------------------------------------------------
                // STATE_IDLE: Awaiting computation trigger
                // ------------------------------------------------------------
                STATE_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        // Latch input matrices into internal storage
                        busy  <= 1'b1;
                        a_reg <= a_flat;
                        b_reg <= b_flat;
                        // Reset all three loop indices to 0
                        r_cnt <= 16'd0;
                        c_cnt <= 16'd0;
                        k_cnt <= 16'd0;
                        acc   <= {ACC_WIDTH{1'b0}};
                        state <= STATE_COMPUTE;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // ------------------------------------------------------------
                // STATE_COMPUTE: Step through the 3 nested loops
                // ------------------------------------------------------------
                STATE_COMPUTE: begin
                    if (k_cnt < ARRAY_SIZE - 1) begin
                        // Inner loop step: accumulate dot-product term and increment k
                        acc   <= current_sum;
                        k_cnt <= k_cnt + 16'd1;
                    end else begin
                        // Dot-product finished for C[r_cnt][c_cnt]!
                        // Store completed element into the result matrix
                        result[(r_cnt * ARRAY_SIZE + c_cnt) * ACC_WIDTH +: ACC_WIDTH] <= current_sum;
                        k_cnt <= 16'd0;                // Reset inner loop index
                        acc   <= {ACC_WIDTH{1'b0}};    // Reset accumulator for next element

                        // Advance middle loop (column counter)
                        if (c_cnt < ARRAY_SIZE - 1) begin
                            c_cnt <= c_cnt + 16'd1;
                        end else begin
                            c_cnt <= 16'd0;            // Row complete, reset column index

                            // Advance outer loop (row counter)
                            if (r_cnt < ARRAY_SIZE - 1) begin
                                r_cnt <= r_cnt + 16'd1;
                            end else begin
                                // All N * N elements calculated! (Total N^3 cycles completed)
                                r_cnt <= 16'd0;
                                busy  <= 1'b0;         // Computation done
                                done  <= 1'b1;         // Assert 1-cycle done flag
                                state <= STATE_DONE;   // Transition to DONE state
                            end
                        end
                    end
                end

                // ------------------------------------------------------------
                // STATE_DONE: Output valid phase
                // ------------------------------------------------------------
                STATE_DONE: begin
                    done <= 1'b0;  // Lower 'done' pulse after 1 cycle
                    if (start) begin
                        // Allow immediate back-to-back start without wasting a cycle
                        busy  <= 1'b1;
                        a_reg <= a_flat;
                        b_reg <= b_flat;
                        r_cnt <= 16'd0;
                        c_cnt <= 16'd0;
                        k_cnt <= 16'd0;
                        acc   <= {ACC_WIDTH{1'b0}};
                        state <= STATE_COMPUTE;
                    end else begin
                        busy  <= 1'b0;
                        state <= STATE_IDLE;
                    end
                end

                // ------------------------------------------------------------
                // Safe default catch
                // ------------------------------------------------------------
                default: begin
                    state <= STATE_IDLE;
                end
            endcase
        end
    end

endmodule
