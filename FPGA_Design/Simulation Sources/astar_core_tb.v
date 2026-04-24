`timescale 1ns / 1ps
// tb_astar_core.v
//
// Tests:
//   1. BASIC PATH:     open 4x4 corner of a 20x20 grid, start=(0,0) goal=(2,2) expects done=1 within timeout, came_from chain valid
//   2. OBSTACLE PATH:  wall of obstacles forcing a detour, verifies path exists
//   3. NO PATH:  goal completely surrounded by obstacles, expects no_path=1
//   4. RESET MID-RUN: assert rst while searching, verify clean restart

module astar_core_tb;
    reg clk = 0;
    reg rst = 0;
    reg go = 0;
    reg [8:0] start_cell = 0;
    reg [8:0] goal_cell = 0;
    wire [8:0] grid_addr;
    
    // FIX 1: Removed '= 0' initialization here. It's driven by an always @(*) block, 
    // doing both creates a race condition at time zero.
    reg grid_obstacle; 
    
    wire done;
    wire no_path;
    wire searching;
    reg  [8:0] cf_rd_addr  = 0;
    wire [8:0] cf_rd_data;

    //  DUT instantiation
    astar_core #(.ROWS(20), .COLS(20)) dut(
        .clk(clk),
        .rst(rst),
        .go(go),
        .start_cell(start_cell),
        .goal_cell(goal_cell),
        .grid_addr(grid_addr),
        .grid_obstacle(grid_obstacle),
        .done(done),
        .no_path(no_path),
        .searching(searching),
        .cf_rd_addr(cf_rd_addr),
        .cf_rd_data(cf_rd_data)
    );

    // 100 MHz clock
    always #5 clk = ~clk;

    //  Obstacle memory: 400 cells, written by test tasks
    reg obs_mem [0:399];
    integer i;

    // FIX 2: Initialize the memory at time zero. Without this, obs_mem 
    // is full of 'X's, which gets fed straight into your DUT!
    initial begin
        for (i = 0; i < 400; i = i + 1)
            obs_mem[i] = 1'b0;
    end

    // FIX 3: Safe memory read. If the DUT outputs an unknown address (X or Z) 
    // during reset, this protects the testbench from feeding X back to the DUT.
    always @(*) begin
        if (grid_addr === 9'bxxxxxxxxx || grid_addr === 9'bzzzzzzzzz) begin
            grid_obstacle = 1'b0;
        end else if (grid_addr < 400) begin
            grid_obstacle = obs_mem[grid_addr];
        end else begin
            grid_obstacle = 1'b0;
        end
    end

    //  Helper tasks
    // Clear all obstacles
    task clear_obstacles;
        integer j;
        begin
            for (j = 0; j < 400; j = j + 1)
                obs_mem[j] = 1'b0;
        end
    endtask

    // Set a single obstacle cell
    task set_obstacle;
        input [8:0] obs_cell;
        begin
            obs_mem[obs_cell] = 1'b1;
        end
    endtask

    // Pulse go for one clock
    task pulse_go;
        begin
            @(posedge clk); #1;
            go = 1'b1;
            @(posedge clk); #1;
            go = 1'b0;
        end
    endtask

    // Wait for done or no_path, with timeout
    // Returns 1=done, 2=no_path, 0=timeout
    task wait_result;
        output integer result;
        input integer max_cycles;
        integer cyc;
        begin
            result = 0;
            for (cyc = 0; cyc < max_cycles; cyc = cyc + 1) begin
                @(posedge clk);
                if (done) begin 
                    result = 1; 
                    cyc = max_cycles; 
                end
                if (no_path) begin 
                    result = 2; 
                    cyc = max_cycles; 
                end
            end
        end
    endtask

    // Walk came_from chain from goal back to start, return step count
    // Returns 9'h1FF if chain is broken (null parent reached before start)
    task trace_path;
        input [8:0] goal;
        input [8:0] start;
        output integer steps;
        output integer ok;
        reg [8:0] cur_cell;
        integer s;
        begin
            steps = 32'b0;
            ok = 0;
            cur_cell = goal;
            for (s = 0; s < 400; s = s + 1) begin
                if (cur_cell == start) begin
                    ok    = 1;
                    steps = s; 
                    s     = 400; // break
                end else begin
                    cf_rd_addr = cur_cell;
                    @(posedge clk); // registered read wait one cycle
                    @(posedge clk);
                    cur_cell = cf_rd_data;
                    if (cur_cell == 9'h1FF) begin
                        s = 400; // broken chain
                    end
                end
            end
        end
    endtask

    //  Test counters
    integer pass_count = 0;
    integer fail_count = 0;
    integer result;
    
    // FIX 4: Initialized to 0. If wait_result times out, trace_path is skipped.
    // This default prevents the waveform viewer from showing a solid red X.
    integer steps = 0; 
    
    integer ok = 1'b0;

    //  Test sequence
    initial begin
        $display("  astar_core testbench starting");

        // Global reset
        rst = 1;
        repeat(5) @(posedge clk);
        rst = 0;
        repeat(2) @(posedge clk);

        //  TEST 1: Basic path - start (0,0) to goal (2,2), no obstacles
        //  Manhattan distance = 4, so shortest path = 4 steps
        $display("\n[TEST 1] Basic path (0,0)->(2,2), no obstacles");
        clear_obstacles;
        start_cell = 9'd0;    // row=0, col=0
        goal_cell  = 9'd42;   // row=2, col=2  (2*20+2=42)
        pulse_go;

        wait_result(result, 200_000);

        if (result == 1) begin
            trace_path(goal_cell, start_cell, steps, ok);
            if (ok && steps == 4) begin
                $display("  PASS done=1, path length=%0d (expected 4)", steps);
                pass_count = pass_count + 1;
            end else begin
                $display("  FAIL done=1 but path trace ok=%0d steps=%0d", ok, steps);
                fail_count = fail_count + 1;
            end
        end else if (result == 2) begin
            $display("  FAIL no_path asserted on open grid");
            fail_count = fail_count + 1;
        end else begin
            $display("  FAIL timeout waiting for done");
            fail_count = fail_count + 1;
        end

        repeat(10) @(posedge clk);

        //  TEST 2: Straight-line path, start (0,0) to goal (0,4)
        //  Path goes purely east: cells 0,1,2,3,4  4 steps
        $display("\n[TEST 2] Straight east (0,0)->(0,4), no obstacles");
        clear_obstacles;
        start_cell = 9'd0;
        goal_cell = 9'd4;    // row=0, col=4
        pulse_go;
        wait_result(result, 200_000);

        if (result == 1) begin
            trace_path(goal_cell, start_cell, steps, ok);
            if (ok && steps == 4) begin
                $display("  PASS done=1, path length=%0d (expected 4)", steps);
                pass_count = pass_count + 1;
            end else begin
                $display("  FAIL done=1 but path trace ok=%0d steps=%0d", ok, steps);
                fail_count = fail_count + 1;
            end
        end else begin
            $display("  FAIL result=%0d (1=done,2=nopath,0=timeout)", result);
            fail_count = fail_count + 1;
        end

        repeat(10) @(posedge clk);

        //  TEST 3: Obstacle detour
        //  Block the direct east path with a vertical wall at col=2,
        //  rows 0-3.  Path from (0,0) to (0,5) must go around.
        //  Cells blocked: col=2,row=0-3 = cells 2,22,42,62
        $display("\n[TEST 3] Obstacle detour wall at col=2, rows 0-3");
        clear_obstacles;
        set_obstacle(2);    // row=0, col=2
        set_obstacle(22);   // row=1, col=2
        set_obstacle(42);   // row=2, col=2
        set_obstacle(62);   // row=3, col=2
        start_cell = 9'd0;  // (0,0)
        goal_cell  = 9'd5;  // (0,5)
        pulse_go;
        wait_result(result, 500_000);

        if (result == 1) begin
            trace_path(goal_cell, start_cell, steps, ok);
            if (ok && steps > 4) begin // detour must be longer than direct
                $display("  PASS done=1, detour path length=%0d (>4)", steps);
                pass_count = pass_count + 1;
            end else begin
                $display("  FAIL ok=%0d steps=%0d (expected >4)", ok, steps);
                fail_count = fail_count + 1;
            end
        end else begin
            $display("  FAIL result=%0d", result);
            fail_count = fail_count + 1;
        end

        repeat(10) @(posedge clk);

        //  TEST 4: No-path detection
        //  Surround goal cell (1,1)=cell 21 with obstacles on all 4 sides:
        //  North=(0,1)=1, South=(2,1)=41, West=(1,0)=20, East=(1,2)=22
        $display("\n[TEST 4] No-path goal completely surrounded");
        clear_obstacles;
        set_obstacle(1);    // north of goal
        set_obstacle(41);   // south of goal
        set_obstacle(20);   // west  of goal
        set_obstacle(22);   // east  of goal
        start_cell = 9'd0;  // (0,0)
        goal_cell  = 9'd21; // (1,1)
        pulse_go;
        wait_result(result, 500_000);

        if (result == 2) begin
            $display("  PASS no_path=1 correctly detected");
            pass_count = pass_count + 1;
        end else if (result == 1) begin
            $display("  FAIL done=1 but goal was surrounded");
            fail_count = fail_count + 1;
        end else begin
            $display("  FAIL timeout (expected no_path)");
            fail_count = fail_count + 1;
        end

        repeat(10) @(posedge clk);

        //  TEST 5: Start == Goal
        //  Per astar_core S_CHECK logic, current == goal_reg on first check
        //  so done should assert almost immediately (after CLEAR+SEED+FIND_MIN)
        $display("\n[TEST 5] Start == Goal (cell 10)");
        clear_obstacles;
        start_cell = 9'd10;
        goal_cell  = 9'd10;
        pulse_go;
        wait_result(result, 100_000);

        if (result == 1) begin
            $display("  PASS done=1 when start==goal");
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL result=%0d (expected done)", result);
            fail_count = fail_count + 1;
        end

        repeat(10) @(posedge clk);

        //  TEST 6: Reset mid-search then re-run
        //  Start a search, reset after 1000 cycles, reissue go,
        //  verify that done eventually asserts cleanly
        $display("\n[TEST 6] Reset mid-search, clean restart");
        clear_obstacles;
        start_cell = 9'd0;
        goal_cell  = 9'd399; // far corner, long search
        pulse_go;
        repeat(1000) @(posedge clk);

        // Assert reset mid-search
        rst = 1;
        repeat(5) @(posedge clk);
        rst = 0;
        repeat(2) @(posedge clk);

        // Now run a simple search
        start_cell = 9'd0;
        goal_cell  = 9'd2;
        pulse_go;
        wait_result(result, 300_000);

        if (result == 1) begin
            $display("  PASS done=1 after clean restart from mid-search reset");
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL result=%0d after reset restart", result);
            fail_count = fail_count + 1;
        end

        //  Summary
        $display("  RESULTS: %0d passed, %0d failed", pass_count, fail_count);
        if (fail_count == 0)
            $display("  ALL TESTS PASSED");
        else
            $display("  SOME TESTS FAILED  review above");

        $finish;
    end

    //  Timeout watchdog kills simulation if it hangs unexpectedly
    initial begin
        #100_000_000; // 100 ms sim time  well beyond any expected runtime
        $display("WATCHDOG: simulation exceeded 100 ms force finish");
        $finish;
    end
    
endmodule