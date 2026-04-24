`timescale 1ns / 1ps
// Generates two independent PWM channels for the Roboclaw 2x15A motor controller.
// The Roboclaw accepts standard RC-style PWM (1000 ~ 2000 Âµs pulse, 50 Hz period)
// on its S1 (left motor) and S2 (right motor) signal pins.
//
// Pulse widths:
//   1000 Âµs: full reverse
//   1500 Âµs: stop
//   2000 Âµs: full forward
//
// speed_l / speed_r are signed 8-bit values:
//   -128:    full reverse  (1000 Âµs)
//      0:    stop          (1500 Âµs)
//   +127:    full forward  (2000 Âµs)
//
// At 100 MHz system clock:
//   1 Âµs  = 100 ticks
//   20 ms period (50 Hz) = 2,000,000 ticks
//   Pulse range: 100,000 ~ 200,000 ticks

module pwm_motor_ctrl (
    input wire clk, // 100 MHz
    input wire rst,
    input wire e_stop, // active HIGH: forces both channels to 1500 Âµs (stop)

    input wire signed [7:0] speed_l, // left motor  command
    input wire signed [7:0] speed_r, // right motor command

    output reg pwm_l, // S1 pin - Roboclaw left  channel
    output reg pwm_r // S2 pin - Roboclaw right channel
);

    // Period counter
    localparam PERIOD_TICKS = 2_000_000;  // 20 ms at 100 MHz

    // Pulse width mapping: 1500 Âµs centre Â± 500 Âµs for Â±128 speed
    // pulse_ticks = 150_000 + (speed * 500_000 / 128)
    // Simplified: pulse_ticks = 150_000 + speed * 3906  (âˆ500000/128)
    localparam CENTER = 150_000;
    localparam SCALE  = 3_906; // ticks per speed unit (500000/128)

    reg [20:0] cnt = 0;
    reg [20:0] pulse_l = CENTER;
    reg [20:0] pulse_r = CENTER;

    // Update pulse widths each period (registered to avoid glitches)
    wire [20:0] cmd_l = e_stop ? CENTER : CENTER + ($signed(speed_l) * SCALE);
    wire [20:0] cmd_r = e_stop ? CENTER : CENTER + ($signed(speed_r) * SCALE);

    always @(posedge clk) begin
        if (rst) begin
            cnt <= 0;
            pwm_l <= 0;
            pwm_r <= 0;
            pulse_l <= CENTER;
            pulse_r <= CENTER;
        end else begin
            if (cnt == PERIOD_TICKS - 1) begin
                cnt <= 0;
                pulse_l <= cmd_l;
                pulse_r <= cmd_r;
            end else begin
                cnt <= cnt + 1;
            end

            // Set HIGH at start of period, LOW when pulse width expires
            pwm_l <= (cnt < pulse_l) ? 1'b1 : 1'b0;
            pwm_r <= (cnt < pulse_r) ? 1'b1 : 1'b0;
        end
    end
endmodule