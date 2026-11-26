`timescale 1ns/1ps

// FSM controller: latches A & B, feeds N slices during LOAD, waits 2N-1 cycles in DRAIN, then captures result
module matrix_controller #(
    parameter ARRAY_SIZE = 4,   // Matrix dimension N (N x N)
    parameter DATA_WIDTH = 8,   // Input element width
    parameter ACC_WIDTH  = 32   // Output element width
) (
    input  wire                                         clk,    // System clock
    input  wire                                         rst_n,  // Active-low async reset
    input  wire                                         start,  // 1-cycle pulse to start matmul

    input  wire signed [ARRAY_SIZE*ARRAY_SIZE*DATA_WIDTH-1:0] a_flat, // Packed matrix A (row-major)
    input  wire signed [ARRAY_SIZE*ARRAY_SIZE*DATA_WIDTH-1:0] b_flat, // Packed matrix B (row-major)

    output reg                                          busy,   // High while processing
    output reg                                          done,   // 1-cycle pulse when result is ready

    output wire                                         clear_acc, // Clears PE accumulators at start
    output wire signed [ARRAY_SIZE*DATA_WIDTH-1:0]       a_out,    // N operands fed to array rows
    output wire signed [ARRAY_SIZE*DATA_WIDTH-1:0]       b_out,    // N operands fed to array cols
    output wire [ARRAY_SIZE-1:0]                        a_valid,   // Valid flags for A lanes
    output wire [ARRAY_SIZE-1:0]                        b_valid,   // Valid flags for B lanes

    input  wire signed [ARRAY_SIZE*ARRAY_SIZE*ACC_WIDTH-1:0] acc_in, // Raw PE accumulators from array
    output reg  signed [ARRAY_SIZE*ARRAY_SIZE*ACC_WIDTH-1:0] result  // Latched final C = A * B
);

    localparam integer ELEMENTS     = ARRAY_SIZE * ARRAY_SIZE;  // Total elements (N^2)
    localparam integer IN_WIDTH     = ELEMENTS * DATA_WIDTH;    // Bit-width of packed input bus
    localparam integer OUT_WIDTH    = ELEMENTS * ACC_WIDTH;     // Bit-width of packed output bus
    localparam integer LOAD_CYCLES  = ARRAY_SIZE;               // Cycles to inject N slices
    localparam integer DRAIN_CYCLES = 2 * ARRAY_SIZE - 1;       // Cycles for wave to reach bottom-right PE
    localparam integer COUNT_WIDTH  = (DRAIN_CYCLES <= 1) ? 1 : $clog2(DRAIN_CYCLES); // Counter bits

    // FSM states
    localparam [1:0] STATE_IDLE  = 2'd0;
    localparam [1:0] STATE_LOAD  = 2'd1;
    localparam [1:0] STATE_DRAIN = 2'd2;

    // Parameter sanity checks at simulation start
    initial begin
        if (ARRAY_SIZE < 1)
            $fatal(1, "matrix_controller requires ARRAY_SIZE >= 1");
        if (DATA_WIDTH < 1)
            $fatal(1, "matrix_controller requires DATA_WIDTH >= 1");
        if (ACC_WIDTH < 2 * DATA_WIDTH)
            $fatal(1, "matrix_controller requires ACC_WIDTH >= 2*DATA_WIDTH");
    end

    reg [1:0]             state;      // Current FSM state
    reg [COUNT_WIDTH-1:0] cycle_cnt;  // Cycle counter for LOAD and DRAIN

    reg signed [IN_WIDTH-1:0] a_latched; // Latched copy of A (immune to bus changes)
    reg signed [IN_WIDTH-1:0] b_latched; // Latched copy of B

    // Pulse clear_acc for 1 cycle when start arrives in IDLE
    assign clear_acc = (state == STATE_IDLE) && start;

    // Slice column k (=cycle_cnt) of A and row k of B for each lane; '+:' means [base +: width]
    genvar lane;
    generate
        for (lane = 0; lane < ARRAY_SIZE; lane = lane + 1) begin : gen_operand_lanes
            assign a_out[DATA_WIDTH*lane +: DATA_WIDTH] =
                a_latched[(lane*ARRAY_SIZE + cycle_cnt)*DATA_WIDTH +: DATA_WIDTH]; // A[lane][cycle_cnt]
            assign b_out[DATA_WIDTH*lane +: DATA_WIDTH] =
                b_latched[(cycle_cnt*ARRAY_SIZE + lane)*DATA_WIDTH +: DATA_WIDTH]; // B[cycle_cnt][lane]
            assign a_valid[lane] = (state == STATE_LOAD); // Valid only while feeding inputs
            assign b_valid[lane] = (state == STATE_LOAD);
        end
    endgenerate

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= STATE_IDLE;
            cycle_cnt <= {COUNT_WIDTH{1'b0}};
            busy      <= 1'b0;
            done      <= 1'b0;
            a_latched <= {IN_WIDTH{1'b0}};
            b_latched <= {IN_WIDTH{1'b0}};
            result    <= {OUT_WIDTH{1'b0}};
        end else begin
            done <= 1'b0; // Default low so done is only a 1-cycle pulse

            case (state)
                STATE_IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        // Latch inputs and start loading
                        a_latched <= a_flat;
                        b_latched <= b_flat;
                        cycle_cnt <= {COUNT_WIDTH{1'b0}};
                        busy      <= 1'b1;
                        state     <= STATE_LOAD;
                    end
                end

                STATE_LOAD: begin
                    // Feed one slice per cycle for N cycles, then drain
                    if (cycle_cnt == LOAD_CYCLES - 1) begin
                        cycle_cnt <= {COUNT_WIDTH{1'b0}};
                        state     <= STATE_DRAIN;
                    end else begin
                        cycle_cnt <= cycle_cnt + 1'b1;
                    end
                end

                STATE_DRAIN: begin
                    // Wait 2N-1 cycles for final PE to finish, then latch result
                    if (cycle_cnt == DRAIN_CYCLES - 1) begin
                        result    <= acc_in;
                        busy      <= 1'b0;
                        done      <= 1'b1;
                        state     <= STATE_IDLE;
                        cycle_cnt <= {COUNT_WIDTH{1'b0}};
                    end else begin
                        cycle_cnt <= cycle_cnt + 1'b1;
                    end
                end

                default: begin
                    state     <= STATE_IDLE;
                    cycle_cnt <= {COUNT_WIDTH{1'b0}};
                    busy      <= 1'b0;
                end
            endcase
        end
    end

endmodule
