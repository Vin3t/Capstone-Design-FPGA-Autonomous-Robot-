`timescale 1ns / 1ps
// Stores the 20x20 obstacle map received from the RPi over UART.
// Each cell is 1 bit: 0 = free, 1 = obstacle.
// 400 cells packed into a 400-bit register backed by BRAM inference.
//
// UART loading protocol (RPi must follow this):
//   1. Assert load_start HIGH for 1 cycle to reset the write pointer.
//   2. Send 400 bytes over UART (row-major, row 0 first).
//      Each byte: 0x00 = free, anything non-zero = obstacle.
//   3. load_done goes HIGH when all 400 bytes have been received.
//
// The A* core and VGA renderer both read from this memory.

module grid_memory #(
    parameter ROWS = 20,
    parameter COLS = 20
)(
    input wire clk,
    input wire rst,

    // UART write port
    input wire load_start,  // pulse to reset write pointer
    input wire [7:0] uart_data,  // byte from uart_rx
    input wire uart_valid,  // strobe from uart_rx
    output reg load_done,   // asserted when all 400 cells loaded

    // Read port A: A* core (combinational)
    input  wire [8:0] cell_addr_a,  // row*COLS + col  (0â€“399)
    output wire cell_data_a,  // 1 = obstacle

    // Read port B: VGA renderer (combinational)
    input  wire [8:0] cell_addr_b,
    output wire cell_data_b
);

    localparam TOTAL_CELLS = ROWS * COLS;  // 400

    reg [TOTAL_CELLS-1:0] grid_mem = 0;   // synthesises to BRAM on Nexys A7
    reg [8:0] wr_ptr = 0;

    // Combinational reads (1-cycle latency; fine for A* FSM and VGA)
    assign cell_data_a = grid_mem[cell_addr_a];
    assign cell_data_b = grid_mem[cell_addr_b];

    always @(posedge clk) begin
        if (rst) begin
            wr_ptr <= 0;
            load_done <= 0;
            grid_mem <= 0;
        end else begin

            // Reset write pointer when a new map is incoming
            if (load_start) begin
                wr_ptr <= 0;
                load_done <= 0;
            end

            // Write one cell per received UART byte
            if (uart_valid && !load_done) begin
                grid_mem[wr_ptr] <= (uart_data != 8'h00) ? 1'b1 : 1'b0;
                if (wr_ptr == TOTAL_CELLS - 1) begin
                    load_done <= 1'b1;
                    wr_ptr <= 0;
                end else begin
                    wr_ptr <= wr_ptr + 1;
                end
            end
        end
    end
endmodule