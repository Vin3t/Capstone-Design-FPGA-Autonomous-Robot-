`timescale 1ns / 1ps
// After A* finishes, this module walks the came_from chain from goal -> start
// and writes each cell index into a path_buffer register file.
// The VGA renderer reads path_buffer to colour path cells yellow.

module path_tracer #(
    parameter ROWS = 20,
    parameter COLS = 20,
    parameter MAX_PATH = 400 // worst-case path length
)(
    input wire clk,
    input wire rst,

    // Control
    input wire start_trace, // pulse when astar_core.done goes high
    input wire [8:0] goal_cell,
    input wire [8:0] start_cell,

    // cameFrom RAM interface (read port on astar_core)
    output reg [8:0] cf_addr,
    input wire [8:0] cf_data,     // came_from[cf_addr]

    // Path buffer (read by VGA renderer)
    // path_valid[i] = 1 means cell i is on the path
    output reg [MAX_PATH-1:0] path_valid,
    output reg trace_done
);

    localparam S_IDLE = 2'd0;
    localparam S_TRACE = 2'd1;
    localparam S_DONE = 2'd2;

    reg [1:0] state = S_IDLE;
    reg [8:0] current = 0;
    reg [8:0] start_r = 0;

    always @(posedge clk) begin
        if (rst) begin
            state <= S_IDLE;
            path_valid <= 0;
            trace_done <= 0;
            cf_addr <= 0;
        end else begin
            case (state)
                S_IDLE: begin
                    trace_done <= 0;
                    if (start_trace) begin
                        path_valid <= 0;
                        current <= goal_cell;
                        start_r <= start_cell;
                        cf_addr <= goal_cell;
                        state <= S_TRACE;
                    end
                end
                S_TRACE: begin
                    // cf_data is available one cycle after cf_addr is set
                    path_valid[current] <= 1'b1;

                    if (current == start_r || cf_data == 9'h1FF) begin
                        // Reached start or hit null parent
                        state <= S_DONE;
                    end else begin
                        current <= cf_data;    // step backward
                        cf_addr <= cf_data;    // request next parent
                    end
                end
                S_DONE: begin
                    trace_done <= 1'b1;
                    state <= S_IDLE;
                end
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule