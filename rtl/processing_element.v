`timescale 1ns/1ps

// ============================================================================
// Module: processing_element
// Description:
//   Fundamental building block (PE) of a 2D systolic array for matrix multiplication.
//   Each PE performs a Multiply-Accumulate (MAC) operation:
//       acc_out <= acc_out + (a_in * b_in)
//   It also forwards operand 'a' to the East neighbor (a_out) and operand 'b'
//   to the South neighbor (b_out) with a 1-clock-cycle register delay, which
//   creates the rhythmic "systolic" flow of data across the array grid.
// ============================================================================

module processing_element #(
    parameter DATA_WIDTH = 8,  // Bit-width of input data operands (signed, e.g., 8-bit: -128 to +127)
    parameter ACC_WIDTH  = 32  // Bit-width of accumulator register (signed, e.g., 32-bit to prevent overflow)
) (
    // Clock and Reset control signals
    input  wire clk,        // System clock (all registers update on rising edge)
    input  wire rst_n,      // Asynchronous active-low reset (clears all internal registers to 0 when low)
    input  wire clear_acc,  // Synchronous active-high clear signal: resets accumulator to 0 before a new matrix run

    // Data inputs (West and North ports)
    input  wire signed [DATA_WIDTH-1:0] a_in,  // Input operand A coming from the West (left neighbor or controller)
    input  wire signed [DATA_WIDTH-1:0] b_in,  // Input operand B coming from the North (top neighbor or controller)

    // Handshake / valid control inputs
    input  wire a_valid_in,  // High when 'a_in' contains valid data (used to prevent bogus multiplications)
    input  wire b_valid_in,  // High when 'b_in' contains valid data

    // Pipelined data outputs (East and South forwarding ports)
    output reg  signed [DATA_WIDTH-1:0] a_out,  // Forwarded operand A to the East (right neighbor) after 1 cycle delay
    output reg  signed [DATA_WIDTH-1:0] b_out,  // Forwarded operand B to the South (bottom neighbor) after 1 cycle delay

    // Pipelined valid flags forwarded along with the data
    output reg  a_valid_out,  // Forwarded valid flag for operand A (delayed by 1 cycle)
    output reg  b_valid_out,  // Forwarded valid flag for operand B (delayed by 1 cycle)

    // Current accumulated MAC result for this PE coordinate (row, col)
    output reg  signed [ACC_WIDTH-1:0]  acc_out  // Internal accumulator output holding the accumulated sum
);

    // ------------------------------------------------------------------------
    // Internal Wires for Multiplier Datapath
    // ------------------------------------------------------------------------
    // Multiplier product width: multiplying two signed N-bit numbers produces a (2*N)-bit result
    wire signed [2*DATA_WIDTH-1:0] product;

    // Sign-extended product matched to the wider accumulator bit-width (ACC_WIDTH)
    wire signed [ACC_WIDTH-1:0]    product_ext;

    // Combinational signed multiplication of input operands: product = a_in * b_in
    assign product = a_in * b_in;

    // ------------------------------------------------------------------------
    // Sign Extension Logic:
    // If ACC_WIDTH is wider than the 2*DATA_WIDTH product (e.g. 32 > 16),
    // we must replicate the Most Significant Bit (sign bit) so negative numbers
    // remain negative without altering their arithmetic value when added to acc_out.
    // ------------------------------------------------------------------------
    generate
        if (ACC_WIDTH > 2 * DATA_WIDTH) begin : gen_ext
            // Replicate product's MSB: product[2*DATA_WIDTH-1] for the remaining upper bits
            assign product_ext = {{(ACC_WIDTH - 2*DATA_WIDTH){product[2*DATA_WIDTH-1]}}, product};
        end else begin : gen_no_ext
            // Product already matches accumulator width
            assign product_ext = product;
        end
    endgenerate

    // ------------------------------------------------------------------------
    // Sequential Logic: Triggered on rising edge of clock or active-low reset
    // ------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // ----------------------------------------------------------------
            // Asynchronous Reset: Clear all pipeline and accumulator registers
            // ----------------------------------------------------------------
            a_out       <= {DATA_WIDTH{1'b0}};  // Clear forwarded A data
            b_out       <= {DATA_WIDTH{1'b0}};  // Clear forwarded B data
            a_valid_out <= 1'b0;               // Clear forwarded valid flags
            b_valid_out <= 1'b0;
            acc_out     <= {ACC_WIDTH{1'b0}};   // Clear MAC accumulator
        end else begin
            // ----------------------------------------------------------------
            // Systolic Pipeline Forwarding:
            // Register input operands and valid flags on every clock tick so
            // data flows in wave-front fashion to neighboring PEs.
            // ----------------------------------------------------------------
            a_out       <= a_in;         // Forward A to next column on next cycle
            b_out       <= b_in;         // Forward B to next row on next cycle
            a_valid_out <= a_valid_in;   // Forward valid flag A
            b_valid_out <= b_valid_in;   // Forward valid flag B

            // ----------------------------------------------------------------
            // Accumulator Update:
            // Priority 1: 'clear_acc' wipes the accumulator clean for a fresh matrix.
            // Priority 2: When both A and B are valid, accumulate product:
            //             acc_out <= acc_out + product_ext
            // If either input is invalid (pipeline bubble), maintain current accumulator value.
            // ----------------------------------------------------------------
            if (clear_acc) begin
                acc_out <= {ACC_WIDTH{1'b0}};  // Reset accumulator to zero
            end else if (a_valid_in && b_valid_in) begin
                acc_out <= acc_out + product_ext;  // Multiply-Accumulate step
            end
            // Note: If (a_valid_in && b_valid_in) is false, acc_out implicitly retains its value
        end
    end

endmodule
