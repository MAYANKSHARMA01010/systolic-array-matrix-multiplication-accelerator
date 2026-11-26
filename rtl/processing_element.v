`timescale 1ns/1ps

// Single Processing Element (PE): performs MAC (acc += a * b) and forwards inputs by 1 cycle
module processing_element #(
    parameter DATA_WIDTH = 8,   // Bit-width of signed input operands
    parameter ACC_WIDTH = 32    // Accumulator bit-width (wider to avoid overflow)
) (
    input wire clk,             // System clock
    input wire rst_n,           // Active-low async reset
    input wire clear_acc,       // Synchronous clear before a new matrix multiply

    input wire signed [DATA_WIDTH-1:0] a_in,        // Input A from left neighbor
    input wire signed [DATA_WIDTH-1:0] b_in,        // Input B from top neighbor

    input wire a_valid_in,      // High when a_in is valid
    input wire b_valid_in,      // High when b_in is valid

    output reg signed [DATA_WIDTH-1:0] a_out,       // Forwarded A to right neighbor
    output reg signed [DATA_WIDTH-1:0] b_out,       // Forwarded B to bottom neighbor

    output reg a_valid_out,     // Forwarded A valid flag
    output reg b_valid_out,     // Forwarded B valid flag

    output reg signed [ACC_WIDTH-1:0] acc_out       // Accumulated sum stored in this PE
);

    wire signed [2*DATA_WIDTH-1:0] product;         // Raw product (2 * DATA_WIDTH bits)
    wire signed [ACC_WIDTH-1:0] product_ext;        // Sign-extended product

    assign product = a_in * b_in;                   // Signed multiplication

    // Sign-extend product by replicating MSB so negative values stay negative in ACC_WIDTH
    generate
        if (ACC_WIDTH > 2 * DATA_WIDTH) begin : gen_ext
            assign product_ext = {{(ACC_WIDTH - 2*DATA_WIDTH){product[2*DATA_WIDTH-1]}}, product};
        end else begin : gen_no_ext
            assign product_ext = product;
        end
    endgenerate

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Async reset clears all outputs and accumulator
            a_out <= {DATA_WIDTH{1'b0}};
            b_out <= {DATA_WIDTH{1'b0}};
            a_valid_out <= 1'b0;
            b_valid_out <= 1'b0;
            acc_out <= {ACC_WIDTH{1'b0}};
        end else begin
            // Systolic forwarding: pass inputs and valid flags right/down with 1-cycle delay
            a_out <= a_in;
            b_out <= b_in;
            a_valid_out <= a_valid_in;
            b_valid_out <= b_valid_in;

            // clear_acc has priority; otherwise accumulate only when both operands are valid
            if (clear_acc) begin
                acc_out <= {ACC_WIDTH{1'b0}};
            end else if (a_valid_in && b_valid_in) begin
                acc_out <= acc_out + product_ext;
            end
        end
    end

endmodule
