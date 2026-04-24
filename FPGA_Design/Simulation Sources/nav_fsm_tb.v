`timescale 1ns / 1ps
// tb_nav_fsm.v
//
// Tests:
//   1. IDLE ON RESET: motors output STOP, nav_busy=0 after reset
//   2. STRAIGHT EAST: 3-waypoint path going east; no turns needed
//   3. TURN THEN DRIVE: path requires a 90Â° CW turn before driving
//   4. NAV_DONE PULSEL: nav_done asserts for exactly one cycle on arrival
//   5. IDLE AFTER DONE:  FSM returns to IDLE, motors at STOP after arrival
//   6. NEW PATH AFTER:  sending a second path_ready after arrival works
//
// Timing note:
//   DRIVE_TICKS and TURN_TICKS are overridden to small values (100 and 50)
//   so the testbench completes in simulation time rather than 1 real second.

module nav_fsm_tb;
    //  Shortened timing for simulation
    localparam SIM_DRIVE_TICKS = 100;
    localparam SIM_TURN_TICKS  = 50;
    localparam COLS            = 20;

    reg        clk         = 0;
    reg        rst         = 0;
    reg        path_ready  = 0;
    reg [8:0]  wp_data     = 0;
    wire [8:0] wp_addr;
    reg [7:0]  path_length = 0;
    wire signed [7:0] speed_l;
    wire signed [7:0] speed_r;
    wire       nav_done;
    wire       nav_busy;

    //  DUT
    nav_fsm #(.COLS(COLS), .DRIVE_TICKS(SIM_DRIVE_TICKS), .TURN_TICKS(SIM_TURN_TICKS)) 
    dut (
        .clk(clk),
        .rst(rst),
        .path_ready(path_ready),
        .wp_data(wp_data),
        .wp_addr(wp_addr),
        .path_length(path_length),
        .speed_l(speed_l),
        .speed_r(speed_r),
        .nav_done(nav_done),
        .nav_busy(nav_busy)
    );

    // 100 MHz clock (10 ns period)
    always #5 clk = ~clk;

    //  Waypoint buffer loaded by test, muxed to wp_data by address
    reg [8:0] wp_buf [0:31];

    always @(*) begin
        if (wp_addr < 32)
            wp_data = wp_buf[wp_addr];
        else
            wp_data = 9'd0;
    end

    //  Helper tasks
    task do_reset;
        begin
            rst = 1;
            repeat(5) @(posedge clk);
            rst = 0;
            repeat(2) @(posedge clk);
        end
    endtask

    // Load a path and pulse path_ready
    task load_path;
        input integer len;
        // Caller fills wp_buf before calling this
        begin
            path_length = len[7:0];
            @(posedge clk); #1;
            path_ready = 1'b1;
            @(posedge clk); #1;
            path_ready = 1'b0;
        end
    endtask

    // Wait for nav_done pulse, with timeout
    // Returns 1 if nav_done seen, 0 if timeout
    task wait_nav_done;
        output integer found;
        input  integer max_cycles;
        integer c;
        begin
            found = 0;
            for (c = 0; c < max_cycles; c = c + 1) begin
                @(posedge clk);
                if (nav_done) begin
                    found = 1;
                    c = max_cycles;
                end
            end
        end
    endtask

    //  Test counters
    integer pass_count = 0;
    integer fail_count = 0;
    integer found;
    integer nav_done_count;

    //  Test sequence
    initial begin
        $display("  nav_fsm testbench starting");
        $display("  DRIVE_TICKS=%0d  TURN_TICKS=%0d", SIM_DRIVE_TICKS, SIM_TURN_TICKS);

        do_reset;

        //  TEST 1: Idle on reset - speed=0, nav_busy=0
        $display("\n[TEST 1] Idle state after reset");
        repeat(5) @(posedge clk);
        if (speed_l == 0 && speed_r == 0 && nav_busy == 0 && nav_done == 0) begin
            $display("  PASS: speed_l=0 speed_r=0 nav_busy=0 nav_done=0");
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: speed_l=%0d speed_r=%0d nav_busy=%0b nav_done=%0b",
                     speed_l, speed_r, nav_busy, nav_done);
            fail_count = fail_count + 1;
        end

        //  TEST 2: Straight east â€” cells 0â†’1â†’2â†’3
        //  All steps are EAST (delta=+1), so no turns needed.
        //  path_length=3 (we navigate to 3 waypoints after start)
        //  wp_buf: [0]=0(start), [1]=1, [2]=2, [3]=3(goal)
        $display("\n[TEST 2] Straight east path 0->1->2->3");
        do_reset;
        wp_buf[0] = 9'd0;
        wp_buf[1] = 9'd1;
        wp_buf[2] = 9'd2;
        wp_buf[3] = 9'd3;
        load_path(3);

        // nav_busy should assert
        repeat(3) @(posedge clk);
        if (nav_busy) begin
            $display("  nav_busy=1 confirmed");
        end else begin
            $display("  WARN: nav_busy=0 after path_ready (may be 1-cycle delay)");
        end

        wait_nav_done(found, 5000);
        if (found) begin
            $display("  PASS:  nav_done received after straight-east path");
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: timeout waiting for nav_done");
            fail_count = fail_count + 1;
        end

        //  TEST 3: Turn requires path goes SOUTH then EAST
        //  Start heading = NORTH (default after reset)
        //  Waypoints: cell 0 (row0,col0), cell 20 (row1,col0), cell 21 (row1,col1)
        //  Step 1: delta=+20=SOUTH - need to turn from NORTH to SOUTH (2x 90° CW)
        //  Step 2: delta=+1=EAST - need to turn from SOUTH to EAST  (1x 90° CW... 
        //          actually SOUTH - WEST - NORTH - EAST = 3 turns CW, or
        //          since we only turn CW: SOUTH(2) - WEST(3) - NORTH(0) - EAST(1) = 3 turns
        //  The test just checks nav_done arrives without timeout.
        $display("\n[TEST 3] Turn requires south then east");
        do_reset;
        wp_buf[0] = 9'd0;    // start
        wp_buf[1] = 9'd20;   // row=1, col=0  (south)
        wp_buf[2] = 9'd21;   // row=1, col=1  (east from previous)
        load_path(2);
        wait_nav_done(found, 10000);
        if (found) begin
            $display("  PASS: nav_done received after turn+drive sequence");
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: timeout during turn+drive path");
            fail_count = fail_count + 1;
        end

        //  TEST 4: nav_done is exactly a one-cycle pulse
        //  Count how many cycles nav_done stays high must be exactly 1
        $display("\n[TEST 4] nav_done is a single-cycle pulse");
        do_reset;
        wp_buf[0] = 9'd0;
        wp_buf[1] = 9'd1;
        load_path(1);

        // Wait for nav_done, then count consecutive cycles it is high
        wait(nav_done == 1'b1);
        nav_done_count = 0;
        begin : count_done
            integer cd;
            for (cd = 0; cd < 10; cd = cd + 1) begin
                if (nav_done) nav_done_count = nav_done_count + 1;
                @(posedge clk);
            end
        end

        if (nav_done_count == 1) begin
            $display("  PASS: nav_done high for exactly 1 cycle");
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: nav_done high for %0d cycles (expected 1)",
                     nav_done_count);
            fail_count = fail_count + 1;
        end

        //  TEST 5: Motors at STOP and nav_busy=0 after arrival
        $display("\n[TEST 5] Motors STOP and nav_busy=0 after arrival");
        // FSM should already be back in S_IDLE from test 4
        repeat(5) @(posedge clk);
        if (speed_l == 0 && speed_r == 0 && nav_busy == 0) begin
            $display("  PASS: speed_l=0 speed_r=0 nav_busy=0 after arrival");
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: speed_l=%0d speed_r=%0d nav_busy=%0b",
                     speed_l, speed_r, nav_busy);
            fail_count = fail_count + 1;
        end

        //  TEST 6: Second path after first completes (simulates re-send from RPi)
        $display("\n[TEST 6] New path accepted after previous arrival");
        // FSM is in S_IDLE - send another path
        wp_buf[0] = 9'd5;
        wp_buf[1] = 9'd6;
        wp_buf[2] = 9'd7;
        load_path(2);
        wait_nav_done(found, 5000);
        if (found) begin
            $display("  PASS: second path completed successfully");
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: second path timed out");
            fail_count = fail_count + 1;
        end

        //  TEST 7: Reset mid-navigation and motors must stop immediately
        $display("\n[TEST 7] Reset mid-navigation stops motors");
        do_reset;
        wp_buf[0] = 9'd0;
        wp_buf[1] = 9'd1;
        wp_buf[2] = 9'd2;
        wp_buf[3] = 9'd3;
        load_path(3);

        // Let it run into S_DRIVE
        repeat(SIM_DRIVE_TICKS / 2) @(posedge clk);

        // Assert reset
        rst = 1;
        @(posedge clk); #1;
        // Check immediately motors must be STOP on the reset cycle
        if (speed_l == 0 && speed_r == 0) begin
            $display("  PASS: motors stopped immediately on rst");
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: speed_l=%0d speed_r=%0d on rst cycle",
                     speed_l, speed_r);
            fail_count = fail_count + 1;
        end
        rst = 0;

        //  Summary
        $display("  RESULTS: %0d passed, %0d failed", pass_count, fail_count);
        if (fail_count == 0)
            $display("  ALL TESTS PASSED");
        else
            $display("  SOME TESTS FAILED: review above");
        $finish;
    end

    // Watchdog
    initial begin
        #10_000_000;
        $display("WATCHDOG: simulation timeout: force finish");
        $finish;
    end
endmodule