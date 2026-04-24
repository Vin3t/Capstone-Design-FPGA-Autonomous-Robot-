`timescale 1ns / 1ps
// uart_tx.v - 8N1 UART transmitter
//
// Transmits one byte whenever send_valid is asserted and the module is idle.
// If send_valid arrives while a byte is already being shifted out it is
// ignored; the caller must wait for tx_ready before presenting the next byte.
//
// Parameters:
//   CLK_FREQ  - system clock frequency in Hz   (default 100 MHz)
//   BAUD_RATE - desired baud rate              (default 115200)
//
// Timing:
//   One bit period = CLK_FREQ / BAUD_RATE clock cycles.
//   Full frame (start + 8 data + stop) = 10 bit-periods.
//   At 115200 baud on 100 MHz clock: bit_period = 868 cycles,
//   full byte ? 8680 cycles ? 86.8 µs.

module uart_tx #(
    parameter CLK_FREQ  = 100_000_000,
    parameter BAUD_RATE = 115_200
)(
    input  wire clk,
    input  wire rst,
 
    // Transmit interface
    input  wire [7:0] send_data,   // byte to send
    input  wire send_valid,  // pulse high for 1 cycle to start TX
    output reg  tx_ready,    // high when idle, ready for next byte
 
    // UART line
    output reg tx           // connect to RPi RX GPIO (GPIO15)
);
 
    // Bit-period counter - number of clocks per baud bit
    localparam BIT_PERIOD = CLK_FREQ / BAUD_RATE;   // 868 at 100 MHz/115200
 
    // Internal state
    localparam S_IDLE  = 2'd0;
    localparam S_START = 2'd1;
    localparam S_DATA  = 2'd2;
    localparam S_STOP  = 2'd3;
 
    reg [1:0]  state = S_IDLE;
    reg [31:0] baud_cnt  = 0;       // counts down to 0 per bit
    reg [2:0]  bit_idx   = 0;       // which data bit we are sending (0-7)
    reg [7:0]  shift_reg = 0;       // loaded on start, shifted LSB-first
 
    always @(posedge clk) begin
        if (rst) begin
            state    <= S_IDLE;
            tx       <= 1'b1;       // idle line is high (mark)
            tx_ready <= 1'b1;
            baud_cnt <= 0;
            bit_idx  <= 0;
        end else begin
            case (state)
                S_IDLE: begin
                    tx       <= 1'b1;
                    tx_ready <= 1'b1;
                    if (send_valid) begin
                        shift_reg <= send_data;
                        baud_cnt  <= BIT_PERIOD - 1;
                        tx        <= 1'b0;   // start bit (space)
                        tx_ready  <= 1'b0;
                        state     <= S_START;
                    end
                end
                 // Hold start bit for one full bit period
                S_START: begin
                    if (baud_cnt == 0) begin
                        baud_cnt <= BIT_PERIOD - 1;
                        tx       <= shift_reg[0];   // LSB first
                        bit_idx  <= 0;
                        state    <= S_DATA;
                    end else begin
                        baud_cnt <= baud_cnt - 1;
                    end
                end
                 // Shift out 8 data bits, LSB first
                S_DATA: begin
                    if (baud_cnt == 0) begin
                        if (bit_idx == 3'd7) begin
                            // All 8 bits sent - move to stop bit
                            baud_cnt <= BIT_PERIOD - 1;
                            tx       <= 1'b1;    // stop bit (mark)
                            state    <= S_STOP;
                        end else begin
                            bit_idx  <= bit_idx + 3'd1;
                            tx       <= shift_reg[bit_idx + 1];
                            baud_cnt <= BIT_PERIOD - 1;
                        end
                    end else begin
                        baud_cnt <= baud_cnt - 1;
                    end
                end
                // Hold stop bit for one full bit period, then go idle
                S_STOP: begin
                    if (baud_cnt == 0) begin
                        state    <= S_IDLE;
                        tx_ready <= 1'b1;
                    end else begin
                        baud_cnt <= baud_cnt - 1;
                    end
                end
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule