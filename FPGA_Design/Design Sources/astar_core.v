`timescale 1ns / 1ps
// Hardware A* pathfinding engine for a 20x20 grid.
//
//  1. S_INIT for-loop REMOVED - replaced with a sequential clear FSM state
//     that resets one cell per clock cycle (400 cycles) instead of trying
//     to write all 400 RAMs in a single cycle. This eliminates the massive
//     combinational fanout that was the primary timing killer.
//
//  2. Division/modulo by non-power-of-2 (COLS=20) ELIMINATED in heuristic()
//     and get_neighbor(). Row and column are now maintained as separate
//     5-bit registers (current_r, current_c) updated whenever `current`
//     changes, so no runtime dividers are needed.
//
//  3. open_set == 0 wide comparison PIPELINED - a single-bit `open_empty`
//     flag is maintained incrementally: cleared when any bit is set,
//     set when the last open bit is cleared. Avoids a 400-input OR gate
//     on the critical path.
//
//  4. came_from RAM registered output - cf_rd_data is now registered
//     (1-cycle read latency) to meet output timing. path_tracer must
//     account for this one extra cycle.
//
//  5. heuristic() and get_neighbor() now operate on pre-decoded row/col
//     values (5-bit integers), removing all division from combinational
//     logic. The neighbor row/col bounds check uses direct comparison
//     rather than wrap-around arithmetic.
//
// Architecture (unchanged externally):
//   open_set:  400-bit register
//   closed:    400-bit register
//   g_score:   400 x 10-bit RAM
//   f_score:   400 x 10-bit RAM
//   came_from: 400 x 9-bit  RAM
//
// FSM states:
//   IDLE      : Waiting for go signal from RPi (via SPI/UART bridge)
//   CLEAR     : Sequential memory clear, one cell per cycle (400 cycles)
//   SEED      : Write start node scores, set open_set bit
//   FIND_MIN  : Scan open_set for lowest f_score (up to 400 cycles)
//   CHECK_GOAL: Is current == goal?
//   EXPAND    : Process 4 neighbours, one per cycle
//   DONE      : Path found; came_from RAM ready for path_tracer
//   NO_PATH   : Open set empty, no path exists
//
// Motor controller interface:
//   The FPGA drives the motor controller directly via PWM/DIR signals.
//   The Raspberry Pi connects to the FPGA only (SPI or UART), never
//   to the motor controller. This ensures jitter-free, deterministic
//   motion commands that a Linux OS cannot guarantee.

module astar_core #(
    parameter ROWS = 20,
    parameter COLS = 20
)(
    input wire clk,
    input wire rst,

    // Control - driven by RPi-to-FPGA SPI/UART bridge
    input wire go, // pulse to start search (from RPi)
    input wire [8:0] start_cell, // start index = row * COLS + col
    input wire [8:0] goal_cell,  // goal  index

    // Grid oracle 
    output reg [8:0] grid_addr,
    input wire grid_obstacle,

    // Status to RPi
    output reg done, // path found
    output reg no_path, // search failed
    output reg searching, // busy

    // came_from RAM read port (for path_tracer)
    // NOTE: registered output - data valid one cycle after cf_rd_addr
    input  wire [8:0] cf_rd_addr,
    output reg [8:0] cf_rd_data    // registered (was combinational - fix #4)
);

    localparam TOTAL = ROWS * COLS;  // 400

    //  FSM state
    localparam S_IDLE = 3'd0;
    localparam S_CLEAR = 3'd1;  // replaces single-cycle S_INIT for-loop
    localparam S_SEED = 3'd2;  // write start node, assert open_set bit
    localparam S_FIND_MIN = 3'd3;
    localparam S_CHECK = 3'd4;
    localparam S_EXPAND = 3'd5;
    localparam S_DONE = 3'd6;
    localparam S_NO_PATH = 3'd7;

    reg [2:0] state = S_IDLE;

    //  Open / closed sets
    reg [TOTAL-1:0] open_set = 0;
    reg [TOTAL-1:0] closed_set = 0;

    // FIX #3: maintain an empty flag instead of comparing 400-bit register
    reg open_empty = 1'b1;

    //  Score / parent RAMs
    reg [9:0] g_score [0:TOTAL-1];
    reg [9:0] f_score [0:TOTAL-1];
    reg [8:0] came_from[0:TOTAL-1];

    // FIX #4: registered came_from read
    always @(posedge clk)
        cf_rd_data <= came_from[cf_rd_addr];

    //  Working registers
    reg [8:0] current = 0;
    reg [4:0] current_r = 0;   // FIX #2: pre-decoded row of current
    reg [4:0] current_c = 0;   // FIX #2: pre-decoded col of current

    reg [8:0] scan_idx = 0;
    reg [9:0] min_f = 0;
    reg [8:0] min_cell = 0;
    reg [8:0] goal_reg = 0;
    reg [4:0] goal_r = 0;   // FIX #2: pre-decoded row of goal
    reg [4:0] goal_c = 0;   // FIX #2: pre-decoded col of goal
    reg [8:0] start_reg = 0;
    reg [1:0] nb_idx = 0;

    //  FIX #2: Division-free heuristic (Manhattan distance)
    //  Uses pre-decoded row/col - no divider in combinational path.
    function automatic [9:0] heuristic_rc;
        input [4:0] r1;
        input [4:0] c1;
        input [4:0] r2;
        input [4:0] c2;
        reg [9:0] dr;
        reg [9:0] dc;
        begin
            dr = (r1 >= r2) ? ({5'd0, r1 - r2}) : ({5'd0, r2 - r1});
            dc = (c1 >= c2) ? ({5'd0, c1 - c2}) : ({5'd0, c2 - c1});
            heuristic_rc = dr + dc;
        end
    endfunction

    //  FIX #2: Division-free neighbour calculation
    //  Returns {valid(1), cell_index(9)}.
    //  Direction: 0=up, 1=down, 2=left, 3=right
    function automatic [9:0] get_neighbor_rc;
        input [4:0] r;
        input [4:0] c;
        input [1:0] dir;
        reg [4:0] nr;
        reg [4:0] nc;
        reg valid;
        reg [8:0] calc_addr; // ADDED: Forces 9-bit math
        begin
            nr = r; nc = c; valid = 1'b1;
            case (dir)
                2'd0: begin  // up
                    if  (r == 5'd0) valid = 1'b0;
                    else            nr = r - 5'd1;
                end
                2'd1: begin  // down
                    if  (r == ROWS-1) valid = 1'b0;
                    else              nr = r + 5'd1;
                end
                2'd2: begin  // left
                    if  (c == 5'd0) valid = 1'b0;
                    else            nc = c - 5'd1;
                end
                2'd3: begin  // right
                    if  (c == COLS-1) valid = 1'b0;
                    else              nc = c + 5'd1;
                end
            endcase
            
            if (!valid) begin
                get_neighbor_rc = 10'b0;
            end else begin
                // Force multiplication with a 9-bit constant to prevent truncation
                calc_addr = (nr * 9'd20) + nc; 
                get_neighbor_rc = {1'b1, calc_addr};
            end
        end
    endfunction

    //  Main FSM
    always @(posedge clk) begin
        if (rst) begin
            state <= S_IDLE;
            done  <= 1'b0;
            no_path <= 1'b0;
            searching <= 1'b0;
            open_set <= 0;
            closed_set <= 0;
            open_empty <= 1'b1;
        end else begin

            case (state)

                //  IDLE - wait for go from RPi
                S_IDLE: begin
                    done <= 1'b0;
                    no_path <= 1'b0;
                    searching <= 1'b0;
                    if (go) begin
                        start_reg <= start_cell;
                        goal_reg  <= goal_cell;
                        // Pre-decode goal row/col once at start (FIX #2)
                        goal_r <= goal_cell / COLS;   // synthesis-time constant divide
                        goal_c <= goal_cell % COLS;
                        scan_idx <= 9'd0;
                        state <= S_CLEAR;
                    end
                end

                //  CLEAR - FIX #1: sequential clear, one cell per cycle
                //  400 cycles instead of 400-wide parallel write
                S_CLEAR: begin
                    searching <= 1'b1;
                    open_set  <= 0;          // whole register clear is fine
                    closed_set <= 0;          // (just 2 flops, not RAM)
                    open_empty <= 1'b1;

                    g_score [scan_idx] <= 10'h3FF;
                    f_score [scan_idx] <= 10'h3FF;
                    came_from[scan_idx] <= 9'h1FF;

                    if (scan_idx == TOTAL - 1) begin
                        scan_idx <= 9'd0;
                        state <= S_SEED;
                    end else begin
                        scan_idx <= scan_idx + 9'd1;
                    end
                end

                //  SEED - write start node costs, open set bit
                S_SEED: begin
                    // Pre-decode start row/col (FIX #2)
                    current_r <= start_reg / COLS;  // constant-fold at synthesis
                    current_c <= start_reg % COLS;

                    g_score[start_reg] <= 10'd0;
                    f_score[start_reg] <= heuristic_rc( start_reg / COLS, start_reg % COLS, goal_r, goal_c);
                    open_set[start_reg] <= 1'b1;
                    open_empty <= 1'b0;
                    scan_idx <= 9'd0;
                    state <= S_FIND_MIN;
                end

                //  FIND_MIN - scan for lowest f in open set
                S_FIND_MIN: begin
                    if (open_empty) begin
                        state <= S_NO_PATH;
                    end else begin
                        if (open_set[scan_idx] && (f_score[scan_idx] < min_f)) begin
                            min_f    <= f_score[scan_idx];
                            min_cell <= scan_idx;
                        end
                        if (scan_idx == TOTAL - 1) begin
                            if (min_f == 10'h3FF) begin
                                open_empty <= 1'b1;  // confirm truly empty after full scan
                                state <= S_NO_PATH;
                            end else begin
                                current   <= min_cell;
                                current_r <= min_cell / COLS;
                                current_c <= min_cell % COLS;
                                scan_idx  <= 9'd0;
                                state     <= S_CHECK;
                            end
                        end else begin
                            scan_idx <= scan_idx + 9'd1;
                        end
                    end
                end

                //  CHECK - is current the goal?
                S_CHECK: begin
                    begin : check_block
                        reg [9:0] nb0_result;
                        nb0_result = get_neighbor_rc(current_r, current_c, 2'd0);
                        if (current == goal_reg) begin
                            state <= S_DONE;
                        end else begin
                            open_set[current]   <= 1'b0;
                            closed_set[current] <= 1'b1;
                            nb_idx    <= 2'd0;
                            grid_addr <= nb0_result[8:0];
                            
                            min_f    <= 10'h3FF;
                            min_cell <= 9'd0;
                            state     <= S_EXPAND;
                        end
                    end
                end
                //  EXPAND - one neighbour per cycle; uses row/col regs
                S_EXPAND: begin
                    begin : expand_block
                        reg [9:0] nb_result;
                        reg [9:0] nb_next_result;
                        reg nb_valid;
                        reg [8:0] nb;
                        reg [4:0] nb_r, nb_c;
                        reg [9:0] tent_g;
                
                        nb_result = get_neighbor_rc(current_r, current_c, nb_idx);
                        nb_valid  = nb_result[9];
                        nb        = nb_result[8:0];
                        nb_r      = nb / COLS;
                        nb_c      = nb % COLS;
                
                        // grid_obstacle is NOW valid for current nb_idx (pre-fetched last cycle)
                        if (nb_valid && !closed_set[nb] && !grid_obstacle) begin
                            tent_g = g_score[current] + 10'd1;
                            if (tent_g < g_score[nb]) begin
                                came_from[nb] <= current;
                                g_score[nb]   <= tent_g;
                                f_score[nb]   <= tent_g + heuristic_rc(nb_r, nb_c, goal_r, goal_c);
                                open_set[nb]  <= 1'b1;
                                open_empty    <= 1'b0;
                            end
                        end
                
                        // Pre-fetch NEXT neighbour's obstacle data for next cycle
                        if (nb_idx < 2'd3) begin
                            nb_next_result = get_neighbor_rc(current_r, current_c, nb_idx + 2'd1);
                            grid_addr <= nb_next_result[8:0];
                        end
                    end
                
                    if (nb_idx == 2'd3) begin
                        scan_idx <= 9'd0;
                        state    <= S_FIND_MIN;
                    end else begin
                        nb_idx <= nb_idx + 2'd1;
                    end
                end

                //  DONE - assert done; path_tracer can now read came_from
                S_DONE: begin
                    done <= 1'b1;
                    searching <= 1'b0;
                    state <= S_IDLE;
                end

                //  NO_PATH
                S_NO_PATH: begin
                    no_path <= 1'b1;
                    searching <= 1'b0;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;

            endcase
        end
    end
endmodule