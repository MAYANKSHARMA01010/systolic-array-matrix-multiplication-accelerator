`timescale 1ns/1ps

// ============================================================================
// Module: matrix_controller
// Description:
//   Finite State Machine (FSM) and data orchestration controller for the
//   systolic array matrix multiplier.
//
//   Responsibilities:
//   1. Handshake Interface: Receives a 'start' pulse, raises 'busy', and asserts
//      'done' when the output matrix multiplication result is ready.
//   2. Input Latching: Latches the entire input matrices (a_flat, b_flat) into
//      internal registers on 'start' so external memory/buses can change freely.
//   3. Skewed Feed Generation: During the LOAD state, extracts the appropriate
//      matrix elements cycle-by-cycle (k = 0 to ARRAY_SIZE-1) and presents them
//      to the systolic array input lanes.
//   4. Accumulator Clear: Generates a single-cycle 'clear_acc' pulse on 'start'
//      to reset all PE accumulators in the array before computation begins.
//   5. Drain Timing & Capture: During the DRAIN state, counts until the final
//      wave-front passes through the bottom-right PE (at 2*ARRAY_SIZE - 1 cycles),
//      then latches the final accumulated matrix from the array into 'result'.
// ============================================================================

module matrix_controller #(
    parameter ARRAY_SIZE = 4,  // Dimension of the square matrices (N for an N x N matrix)
    parameter DATA_WIDTH = 8,  // Bit-width of each matrix element (signed integer)
    parameter ACC_WIDTH  = 32  // Bit-width of each accumulated element in output matrix
) (
    // Clock and Reset
    input  wire clk,    // Main system clock
    input  wire rst_n,  // Active-low asynchronous reset

    // Control Handshake
    input  wire start,  // 1-cycle active-high pulse to initiate matrix multiplication

    // Packed 1D input buses containing the complete flattened N x N matrices
    // Elements are packed in row-major order: index = (row * ARRAY_SIZE) + col
    input  wire signed [ARRAY_SIZE*ARRAY_SIZE*DATA_WIDTH-1:0] a_flat,  // Flattened Matrix A input
    input  wire signed [ARRAY_SIZE*ARRAY_SIZE*DATA_WIDTH-1:0] b_flat,  // Flattened Matrix B input

    // Status outputs to external host/processor
    output reg  busy,  // High while the accelerator is executing matrix multiplication
    output reg  done,  // Active-high 1-cycle pulse signaling completion and valid 'result'

    // Control and Data connections to the systolic array core
    output wire clear_acc,  // Clear pulse sent to all PEs in the array during start
    output wire signed [ARRAY_SIZE*DATA_WIDTH-1:0] a_out,    // Parallel A operands for array row inputs [0..N-1]
    output wire signed [ARRAY_SIZE*DATA_WIDTH-1:0] b_out,    // Parallel B operands for array col inputs [0..N-1]
    output wire [ARRAY_SIZE-1:0]                  a_valid,  // Valid mask per row lane of A
    output wire [ARRAY_SIZE-1:0]                  b_valid,  // Valid mask per col lane of B

    // Array result collection
    input  wire signed [ARRAY_SIZE*ARRAY_SIZE*ACC_WIDTH-1:0] acc_in,  // Accumulated results tapped from all PEs
    output reg  signed [ARRAY_SIZE*ARRAY_SIZE*ACC_WIDTH-1:0] result   // Latched final output matrix (C = A * B)
);

    // ------------------------------------------------------------------------
    // Architectural Derived Parameters (Local Constants)
    // ------------------------------------------------------------------------
    localparam integer ELEMENTS     = ARRAY_SIZE * ARRAY_SIZE;  // Total elements in N x N matrix (e.g. 16 for 4x4)
    localparam integer IN_WIDTH     = ELEMENTS * DATA_WIDTH;    // Total bit-width for packed input matrices A and B
    localparam integer OUT_WIDTH    = ELEMENTS * ACC_WIDTH;     // Total bit-width for packed output matrix C

    // Scheduling cycle counts:
    // LOAD_CYCLES: Number of cycles spent streaming columns of A and rows of B (equals N)
    localparam integer LOAD_CYCLES  = ARRAY_SIZE;

    // DRAIN_CYCLES: Number of cycles after LOAD needed for the last computation wave
    // to propagate through the entire array mesh to PE(N-1, N-1). Formula: 2*N - 1.
    localparam integer DRAIN_CYCLES = 2 * ARRAY_SIZE - 1;

    // Counter bit-width: dynamically sized using $clog2 to hold up to DRAIN_CYCLES
    localparam integer COUNT_WIDTH  = (DRAIN_CYCLES <= 1) ? 1 : $clog2(DRAIN_CYCLES);

    // ------------------------------------------------------------------------
    // FSM State Encoding (2-bit state machine)
    // ------------------------------------------------------------------------
    localparam [1:0] STATE_IDLE  = 2'd0;  // System is idle, awaiting 'start' pulse
    localparam [1:0] STATE_LOAD  = 2'd1;  // Feeding matrix elements into array lanes (N cycles)
    localparam [1:0] STATE_DRAIN = 2'd2;  // Waiting for systolic wave to drain to all PEs (2N-1 cycles)

    // ------------------------------------------------------------------------
    // Parameter Sanity Assertions: Catch illegal configurations at simulation start
    // ------------------------------------------------------------------------
    initial begin
        if (ARRAY_SIZE < 1)
            $fatal(1, "matrix_controller requires ARRAY_SIZE >= 1");
        if (DATA_WIDTH < 1)
            $fatal(1, "matrix_controller requires DATA_WIDTH >= 1");
        if (ACC_WIDTH < 2 * DATA_WIDTH)
            $fatal(1, "matrix_controller requires ACC_WIDTH >= 2*DATA_WIDTH");
    end

    // ------------------------------------------------------------------------
    // Internal Registers
    // ------------------------------------------------------------------------
    reg [1:0]             state;      // Current FSM state register
    reg [COUNT_WIDTH-1:0] cycle_cnt;  // Multi-purpose cycle counter for LOAD and DRAIN phases

    // Input latches: capture a_flat and b_flat when 'start' arrives
    reg signed [IN_WIDTH-1:0] a_latched;  // Internally stored copy of Matrix A
    reg signed [IN_WIDTH-1:0] b_latched;  // Internally stored copy of Matrix B

    // ------------------------------------------------------------------------
    // Accumulator Clear Generation:
    // Generate a single-cycle pulse when 'start' is detected while in IDLE state.
    // This immediately clears PE accumulators in the array before computation starts.
    // ------------------------------------------------------------------------
    assign clear_acc = (state == STATE_IDLE) && start;

    // ------------------------------------------------------------------------
    // Data Slicing and Feeding Logic (Per-Lane Operand Routing):
    // For each row 'lane' in 0..N-1:
    //   At cycle 'cycle_cnt' (which acts as index 'k'):
    //   - a_out[lane] receives element A[lane][cycle_cnt]
    //   - b_out[lane] receives element B[cycle_cnt][lane]
    //
    // Note on Verilog syntax '+:' (Indexed Part-Select):
    //   base +: width selects a range of 'width' bits starting at 'base'.
    // ------------------------------------------------------------------------
    genvar lane;
    generate
        for (lane = 0; lane < ARRAY_SIZE; lane = lane + 1) begin : gen_operand_lanes
            // Slice A[lane][cycle_cnt] from latched matrix A:
            // Linear index in row-major matrix = (lane * ARRAY_SIZE) + cycle_cnt
            assign a_out[DATA_WIDTH*lane +: DATA_WIDTH] =
                a_latched[(lane*ARRAY_SIZE + cycle_cnt)*DATA_WIDTH +: DATA_WIDTH];

            // Slice B[cycle_cnt][lane] from latched matrix B:
            // Linear index in row-major matrix = (cycle_cnt * ARRAY_SIZE) + lane
            assign b_out[DATA_WIDTH*lane +: DATA_WIDTH] =
                b_latched[(cycle_cnt*ARRAY_SIZE + lane)*DATA_WIDTH +: DATA_WIDTH];

            // Valid signals are high only during the STATE_LOAD phase
            assign a_valid[lane] = (state == STATE_LOAD);
            assign b_valid[lane] = (state == STATE_LOAD);
        end
    endgenerate

    // ------------------------------------------------------------------------
    // Sequential Controller FSM: Controls execution progression and output capture
    // ------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Asynchronous reset: return to clean IDLE state and clear all outputs
            state     <= STATE_IDLE;
            cycle_cnt <= {COUNT_WIDTH{1'b0}};
            busy      <= 1'b0;
            done      <= 1'b0;
            a_latched <= {IN_WIDTH{1'b0}};
            b_latched <= {IN_WIDTH{1'b0}};
            result    <= {OUT_WIDTH{1'b0}};
        end else begin
            // Default: 'done' is a single-cycle pulse, so deassert by default
            done <= 1'b0;

            case (state)
                // ------------------------------------------------------------
                // STATE_IDLE: Awaiting computation trigger
                // ------------------------------------------------------------
                STATE_IDLE: begin
                    busy <= 1'b0;  // Controller is ready for work
                    if (start) begin
                        // Latch input matrices so external inputs can change safely
                        a_latched <= a_flat;
                        b_latched <= b_flat;
                        cycle_cnt <= {COUNT_WIDTH{1'b0}};  // Reset cycle counter
                        busy      <= 1'b1;                 // Mark accelerator as busy
                        state     <= STATE_LOAD;           // Transition to matrix feed phase
                    end
                end

                // ------------------------------------------------------------
                // STATE_LOAD: Feed matrix operands cycle-by-cycle (takes N cycles)
                // In cycle k (0 to N-1), column k of A and row k of B are injected.
                // ------------------------------------------------------------
                STATE_LOAD: begin
                    if (cycle_cnt == LOAD_CYCLES - 1) begin
                        // All N elements injected; move to DRAIN state to let pipeline finish
                        cycle_cnt <= {COUNT_WIDTH{1'b0}};
                        state     <= STATE_DRAIN;
                    end else begin
                        cycle_cnt <= cycle_cnt + 1'b1;  // Advance to next element index k
                    end
                end

                // ------------------------------------------------------------
                // STATE_DRAIN: Allow wave-front to traverse to the final PE(N-1, N-1).
                // Takes 2*N - 1 cycles.
                // ------------------------------------------------------------
                STATE_DRAIN: begin
                    if (cycle_cnt == DRAIN_CYCLES - 1) begin
                        // All accumulations across all PEs are complete!
                        // Latch the final values from array accumulators into 'result'
                        result    <= acc_in;
                        busy      <= 1'b0;        // Accelerator is no longer busy
                        done      <= 1'b1;        // Pulse 'done' high for 1 clock cycle
                        state     <= STATE_IDLE;  // Return to IDLE, ready for next command
                        cycle_cnt <= {COUNT_WIDTH{1'b0}};
                    end else begin
                        cycle_cnt <= cycle_cnt + 1'b1;  // Continue waiting for drain
                    end
                end

                // ------------------------------------------------------------
                // Safe default catch: Return to IDLE on any unexpected state
                // ------------------------------------------------------------
                default: begin
                    state     <= STATE_IDLE;
                    cycle_cnt <= {COUNT_WIDTH{1'b0}};
                    busy      <= 1'b0;
                end
            endcase
        end
    end

endmodule
