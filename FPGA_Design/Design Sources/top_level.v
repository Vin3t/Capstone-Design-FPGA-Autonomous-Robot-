`timescale 1ns / 1ps
//     The Raspberry Pi should not talk to the motor controller.
//     RPi -> UART -> FPGA -> PWM/DIR ->  Roboclaw motor controller
//     go_pulse now asserted automatically when map load completes
// -----------------------------------------------------------------------
// UART protocol from Raspberry Pi (must match rpi_sender.py):
//   Byte 0        : start_row  
//   Byte 1        : start_col  
//   Byte 2        : goal_row   
//   Byte 3        : goal_col   
//   Bytes 4       : obstacle map, row-major (0x00=free, 0x01=obstacle)
//
// External pin connections:
//   UART_TXD_IN  <- RPi GPIO14 TX (USB-to-UART bridge or direct 3.3V GPIO)
//   BTNC         -> Start A* search after map loaded
//   CPU_RESETN   -> Active-low reset
//   E_STOP_PIN   <- Physical E-stop switch (active HIGH)
//   PWM_L        -> Roboclaw S1 (left  motor PWM)
//   PWM_R        -> Roboclaw S2 (right motor PWM)
//   VGA_*        -> VGA connector
// -----------------------------------------------------------------------

module top_level #(
    parameter CLK_FREQ  = 100_000_000,
    parameter BAUD_RATE = 115_200
    )(
    input wire CLK100MHZ,
    input wire CPU_RESETN,
 
    // UART from Raspberry Pi (RPi TX ? FPGA RX)
    input wire UART_TXD_IN,
    
    // UART back to Raspberry Pi (FPGA TX ? RPi RX)
    // Connect to RPi GPIO15 (UART0 RX, pin 10)
    // Status bytes sent:  
    // 0xAA = path found / navigating
    // 0xFF = no path exists
    // 0xBB = navigation complete (arrived)
    output wire UART_RXD_OUT,
 
    // Motor controller outputs - FPGA drives Roboclaw directly
    output wire PWM_L,
    output wire PWM_R
);
 
    wire rst = ~CPU_RESETN;
 
    //  UART RX
    wire [7:0] uart_byte;
    wire uart_valid;
 
    uart_rx #(
        .CLK_FREQ(CLK_FREQ),
        .BAUD_RATE(BAUD_RATE)
    ) u_uart (
        .clk(CLK100MHZ),
        .rst(rst),
        .rx(UART_TXD_IN),
        .data_out(uart_byte),
        .data_valid(uart_valid)
    );
 
     //  UART loader FSM
    //  Receives: start_row, start_col, goal_row, goal_col, then 400 map bytes
    //  Automatically pulses go_pulse when the full map has been received 
    localparam L_START_ROW = 3'd0;
    localparam L_START_COL = 3'd1;
    localparam L_GOAL_ROW = 3'd2;
    localparam L_GOAL_COL = 3'd3;
    localparam L_MAP = 3'd4;
 
    reg [2:0] loader_state = L_START_ROW;
    reg [8:0] start_cell_r = 0;
    reg [8:0] goal_cell_r = 0;
    reg [7:0] tmp_row = 0;
    reg load_start = 0;
    reg map_uart_valid = 0;
    reg [7:0]  map_uart_byte = 0;
 
    always @(posedge CLK100MHZ) begin
        load_start <= 1'b0;
        map_uart_valid <= 1'b0;
 
        if (rst) begin
            loader_state <= L_START_ROW;
        end else if (uart_valid) begin
            case (loader_state)
                L_START_ROW: begin
                    tmp_row <= uart_byte;
                    loader_state <= L_START_COL;
                end
                L_START_COL: begin
                    start_cell_r <= tmp_row * 20 + uart_byte;
                    loader_state <= L_GOAL_ROW;
                end
                L_GOAL_ROW: begin
                    tmp_row <= uart_byte;
                    loader_state <= L_GOAL_COL;
                end
                L_GOAL_COL: begin
                    goal_cell_r <= tmp_row * 20 + uart_byte;
                    load_start <= 1'b1;
                    loader_state <= L_MAP;
                end
                L_MAP: begin
                    map_uart_byte <= uart_byte;
                    map_uart_valid <= 1'b1;
                end
                default: ;
            endcase
        end
    end
 
    //  Grid memory
    wire load_done;
    wire [8:0] grid_addr_a;
    wire grid_obs_a;
 
    grid_memory #(.ROWS(20), .COLS(20)) u_grid (
        .clk(CLK100MHZ),
        .rst(rst),
        .load_start(load_start),
        .uart_data(map_uart_byte),
        .uart_valid(map_uart_valid),
        .load_done(load_done),
        .cell_addr_a(grid_addr_a),
        .cell_data_a(grid_obs_a),
        .cell_addr_b(9'd0),
        .cell_data_b()
    );
 
    //  go_pulse - fires automatically when map load completes.
    //  A single-cycle pulse is generated on the rising edge of load_done.
    reg map_load_started = 1'b0;
    always @(posedge CLK100MHZ) begin
        if (rst) map_load_started <= 1'b0;
        else if (load_start) map_load_started <= 1'b1;
    end
    
    reg load_done_prev = 1'b0;
    always @(posedge CLK100MHZ) load_done_prev <= load_done;
    
    wire go_pulse = load_done & ~load_done_prev & map_load_started;
 
    //  A* core
    wire astar_done;
    wire astar_no_path;
    wire astar_searching;
    wire [8:0] cf_rd_addr;
    wire [8:0] cf_rd_data;
 
    astar_core #(.ROWS(20), .COLS(20)) u_astar (
        .clk(CLK100MHZ),
        .rst(rst),
        .go(go_pulse),
        .start_cell(start_cell_r),
        .goal_cell(goal_cell_r),
        .grid_addr(grid_addr_a),
        .grid_obstacle(grid_obs_a),
        .done(astar_done),
        .no_path(astar_no_path),
        .searching(astar_searching),
        .cf_rd_addr(cf_rd_addr),
        .cf_rd_data(cf_rd_data)
    );
 
    //  Path tracer
    //  Walks came_from RAM from goal back to start; emits path_vis bitmap
    //  and signals trace_done when the ordered waypoint list is ready.
    wire [399:0] path_vis;
    wire trace_done;
 
    path_tracer #(.ROWS(20), .COLS(20)) u_tracer (
        .clk(CLK100MHZ),
        .rst(rst),
        .start_trace(astar_done),
        .goal_cell(goal_cell_r),
        .start_cell(start_cell_r),
        .cf_addr(cf_rd_addr),
        .cf_data(cf_rd_data),
        .path_valid(path_vis),
        .trace_done(trace_done)
    );
 
    //  Nav FSM
    //  Reads ordered waypoints from path_tracer and issues speed commands.
    //  wp_data / path_length are stubs - extend path_tracer to export an
    //  ordered waypoint RAM and length counter when ready.
    wire [8:0] wp_data = 9'd0;   // stub: replace with path_tracer waypoint RAM output
    wire [8:0] wp_addr;
    wire [7:0] path_length = 8'd0;  // stub: replace with path_tracer path_length output
    wire nav_done;
    wire nav_busy;
    wire signed [7:0] speed_l_w;
    wire signed [7:0] speed_r_w;
 
    nav_fsm #(
        .COLS(20),
        .DRIVE_TICKS(100_000_000),
        .TURN_TICKS(50_000_000)
    ) u_nav (
        .clk(CLK100MHZ),
        .rst(rst),
        .path_ready(trace_done),
        .wp_data(wp_data),
        .wp_addr(wp_addr),
        .path_length(path_length),
        .speed_l(speed_l_w),
        .speed_r(speed_r_w),
        .nav_done(nav_done),
        .nav_busy(nav_busy)
    );
 
    //  PWM motor controller - Should change so that it directly drives Roboclaw from FPGA
    pwm_motor_ctrl u_pwm (
        .clk(CLK100MHZ),
        .rst(rst),
        .speed_l(speed_l_w),
        .speed_r(speed_r_w),
        .pwm_l(PWM_L),
        .pwm_r(PWM_R)
    );
    
    //  Status arbiter - converts FPGA events to 1-byte UART replies
    //
    //  Three events can trigger a TX byte, in priority order:
    //    1. astar_done    ? 0xAA  (path found, motors about to run)
    //    2. astar_no_path ? 0xFF  (no path, user must replan)
    //    3. nav_done      ? 0xBB  (robot arrived at goal)
    //
    //  Each signal is edge-detected so a single byte is sent per event.
    //  If two events coincide (extremely unlikely), the higher-priority
    //  one wins; the lower-priority byte is dropped (not queued).
    //  This is acceptable because the RPi GUI only needs to know the
    //  latest state.
    //
    //  The arbiter waits for uart_tx to be ready (tx_ready) before
    //  asserting send_valid, so back-to-back events separated by less
    //  than one UART frame (~87 µs at 115200) are handled.
    
    // Edge detectors
    reg astar_done_prev = 1'b0;
    reg astar_no_path_prev = 1'b0;
    reg nav_done_prev = 1'b0;
    
    always @(posedge CLK100MHZ) begin
        astar_done_prev <= astar_done;
        astar_no_path_prev <= astar_no_path;
        nav_done_prev <= nav_done;
    end
    
    wire event_found = astar_done & ~astar_done_prev;    // rising edge
    wire event_nopath = astar_no_path & ~astar_no_path_prev; // rising edge
    wire event_arrived = nav_done & ~nav_done_prev;       // rising edge
    
    // Arbiter registers
    reg [7:0] tx_byte = 8'h00;
    reg tx_send = 1'b0;
    wire tx_ready_w;
    
    always @(posedge CLK100MHZ) begin
        tx_send <= 1'b0;   // default: no send request
 
        if (rst) begin
            tx_byte <= 8'h00;
        end else if (tx_ready_w) begin
            // Priority: found > no_path > arrived
            // (found and no_path are mutually exclusive by A* design,
            //  but the priority encoding is kept for safety)
            if (event_found) begin
                tx_byte <= 8'hAA;
                tx_send <= 1'b1;
            end else if (event_nopath) begin
                tx_byte <= 8'hFF;
                tx_send <= 1'b1;
            end else if (event_arrived) begin
                tx_byte <= 8'hBB;
                tx_send <= 1'b1;
            end
        end
    end
    
    //  UART TX - sends status bytes back to Raspberry Pi
    uart_tx #(
        .CLK_FREQ(CLK_FREQ),
        .BAUD_RATE(BAUD_RATE)
    ) u_uart_tx (
        .clk(CLK100MHZ),
        .rst(rst),
        .send_data(tx_byte),
        .send_valid(tx_send),
        .tx_ready(tx_ready_w),
        .tx(UART_RXD_OUT)
    );
endmodule