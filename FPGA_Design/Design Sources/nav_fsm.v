`timescale 1ns / 1ps
// Translates the ordered A* waypoint list into left/right motor speed commands
// for the Roboclaw via pwm_motor_ctrl.
//
// E-STOP NOTE:
//   The E-stop switch cuts power directly to the Roboclaw motor controller
//   the FPGA is never notified and does not need to be. Safety on resume is 
//   guaranteed by two decisions
//     1. nav_fsm sits in S_IDLE (outputs STOP) on reset and after arrival.
//        When the Roboclaw regains power it will find the FPGA already
//        outputting a neutral/zero duty cycle from pwm_motor_ctrl.
//     2. Motion only resumes when the Raspberry Pi explicitly sends a new
//        navigation packet, which requires deliberate user interaction on
//        the touchscreen. There is no automatic resume.
//
// State machine:
//   IDLE         waiting for path_ready
//   LOAD_WP      latch next waypoint from path buffer
//   TURN         rotate in place until heading aligns with waypoint direction
//   DRIVE        drive forward one cell length
//   ARRIVED      all waypoints consumed; signal done
//   ESTOP        emergency stop triggered
//
// Waypoint representation:
//   Each waypoint is a cell index (row*COLS + col).
//   The FSM computes the direction (N/S/E/W) from current to next cell
//   and issues differential drive commands accordingly.
//
// Timing constants assume the robot travels one 0.32 m cell in ~1 second
// at nominal speed, and turns 90° in ~0.5 seconds.
// Tune DRIVE_TICKS and TURN_TICKS to match your actual robot.

module nav_fsm #(
    parameter COLS = 20,
    parameter DRIVE_TICKS = 100_000_000,  // 1 second at 100 MHz per cell
    parameter TURN_TICKS = 50_000_000   // 0.5 second per 90° turn
)(
    input wire clk,
    input wire rst,

    // Path input (from path_tracer via top_level)
    input wire path_ready, // pulse when new path is ready
    input wire [8:0] wp_data, // current waypoint cell index (muxed externally)
    output reg [8:0] wp_addr, // request waypoint N from path buffer
    input wire [7:0] path_length, // total waypoints in path

    // Motor commands - pwm_motor_ctrl
    output reg signed [7:0] speed_l,
    output reg signed [7:0] speed_r,

    // Status
    output reg nav_done, // reached final waypoint
    output reg nav_busy
);

    // Heading encoding
    localparam NORTH = 2'd0;
    localparam EAST = 2'd1;
    localparam SOUTH = 2'd2;
    localparam WEST = 2'd3;

    // Speed constants
    localparam signed [7:0] FORWARD =  8'sd80;
    localparam signed [7:0] TURN_SPD =  8'sd60;
    localparam signed [7:0] STOP =  8'sd0;

    // FSM states
    localparam S_IDLE = 3'd0;
    localparam S_LOAD = 3'd1;
    localparam S_TURN = 3'd2;
    localparam S_DRIVE = 3'd3;
    localparam S_ARRIVED = 3'd4;

    reg [2:0] state = S_IDLE;
    reg [31:0] timer = 0;
    reg [7:0] wp_idx = 0; // which waypoint we're heading to
    reg [7:0] wp_count = 0;
    reg [8:0] cur_cell = 0;
    reg [8:0] nxt_cell = 0;
    reg [1:0] heading = NORTH; // robot's current heading
    reg [1:0] target_hdg = NORTH;

    // Compute required heading from cur_cell to nxt_cell
    function [1:0] required_heading;
        input [8:0] from_cell;
        input [8:0] to_cell;
        reg signed [9:0] delta;
        begin
            delta = $signed({1'b0, to_cell}) - $signed({1'b0, from_cell});
            if (delta ==  1) 
                required_heading = EAST;
            else if (delta == -1)          
                required_heading = WEST;
            else if (delta ==  COLS)       
                required_heading = SOUTH;
            else                           
                required_heading = NORTH;
        end
    endfunction

    always @(posedge clk) begin
        if (rst) begin
            // On reset: outputs are neutral so Roboclaw power-restore is safe
            state <= S_IDLE;
            speed_l <= STOP;
            speed_r <= STOP;
            nav_done <= 1'b0;
            nav_busy <= 1'b0;
            wp_addr <= 9'd0;
            timer <= 0;
            wp_idx <= 0;
            wp_count <= 0;
            heading <= NORTH;
        end else begin
            // Default: clear nav_done every cycle unless we're in S_ARRIVED
            nav_done <= 1'b0;
            case (state)
                //  IDLE: safe state; motors stopped; wait for RPi command
                //  The robot will NOT move again until the RPi GUI sends a
                //  fresh navigation packet.  This is the post-E-stop safe
                //  state as well - when Roboclaw power is restored the FPGA
                //  is here, outputting STOP.
                S_IDLE: begin
                    nav_busy <= 1'b0;
                    speed_l <= STOP;
                    speed_r <= STOP;
                    if (path_ready) begin
                        wp_count <= path_length;
                        wp_idx <= 8'd0;
                        wp_addr <= 9'd0;
                        cur_cell <= 9'd0;
                        nxt_cell <= 9'd0;
                        nav_busy <= 1'b1;
                        state <= S_LOAD;
                    end
                end
                 //  LOAD: fetch next waypoint from path buffer
                //  Two-cycle latency: address set this cycle, data valid
                //  next cycle.  The FSM spends one cycle here per waypoint.
                S_LOAD: begin
                    cur_cell <= nxt_cell;
                    nxt_cell <= wp_data;
                    target_hdg <= required_heading(nxt_cell, wp_data);
                    wp_idx <= wp_idx + 8'd1;
                    wp_addr <= {1'b0, wp_idx + 8'd1};
                    timer <= 0;
                    state <= S_TURN;
                end
                 //  TURN: rotate in-place until heading == target_hdg
                //  One TURN_TICKS period rotates the robot exactly 90° CW.
                //  Heading is updated after each 90° increment.
                S_TURN: begin
                    if (heading == target_hdg) begin
                        // Already facing the right way - skip straight to drive
                        speed_l <= STOP;
                        speed_r <= STOP;
                        timer <= 0;
                        state <= S_DRIVE;
                    end else begin
                        // Spin clockwise: left motor forward, right motor reverse
                        speed_l <= TURN_SPD;
                        speed_r <= -TURN_SPD;
                        if (timer == TURN_TICKS - 1) begin
                            heading <= (heading == WEST) ? NORTH : heading + 2'd1;
                            timer <= 0;
                            // Re-check heading on next cycle (stays in S_TURN)
                        end else begin
                            timer <= timer + 1;
                        end
                    end
                end
                 //  DRIVE: move forward one cell length (DRIVE_TICKS cycles)
                S_DRIVE: begin
                    speed_l <= FORWARD;
                    speed_r <= FORWARD;
                    if (timer == DRIVE_TICKS - 1) begin
                        speed_l <= STOP;
                        speed_r <= STOP;
                        timer <= 0;
                        if (wp_idx >= wp_count)
                            state <= S_ARRIVED;
                        else
                            state <= S_LOAD;
                    end else begin
                        timer <= timer + 1;
                    end
                end
                 //  ARRIVED:  assert nav_done for one cycle, return to IDLE
                //  top_level edge-detects nav_done to trigger UART 0xBB TX.
                //  After this the FSM returns to S_IDLE (motors stopped),
                //  so any subsequent Roboclaw power cycle is safe.
                S_ARRIVED: begin
                    nav_done <= 1'b1;   // single-cycle pulse
                    nav_busy <= 1'b0;
                    speed_l <= STOP;
                    speed_r <= STOP;
                    state <= S_IDLE;
                end
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule