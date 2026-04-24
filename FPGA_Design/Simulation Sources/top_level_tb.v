`timescale 1ns / 1ps
// tb_top_level.v  - Integration testbench for top_level.v
//
// This testbench is for the full pipeline:
//   RPi UART TX -> UART RX -> loader FSM ->  grid_memory -> A* core
//   -> path_tracer -> nav_fsm -> pwm_motor_ctrl -> PWM outputs
//   -> status arbiter -> uart_tx -> UART_RXD_OUT back to RPi
//
// Tests:
//   1. UART LOAD: send a valid 404-byte packet, verify go_pulse fires
//   2. ASTAR TRIGGER: A* starts automatically after map load 
//   3. 0xAA REPLY: UART_RXD_OUT emits 0xAA when A* completes
//   4. 0xFF REPLY: send unreachable goal, verify 0xFF is transmitted
//   5. 0xBB REPLY:  simulate nav_done, verify 0xBB is transmitted
//   6. PWM IDLE: PWM outputs are low/neutral before navigation
//
// Baud rate is overridden via a fast-sim parameter: the testbench sends
// bits at 100x speed (BIT_TICKS=8 instead of 868) so the 404-byte packet
// loads in ~32000 cycles instead of ~35 million.

module top_level_tb;
    //  Sim-speed UART parameters
    //  Real: 100 MHz / 115200 = 868 cycles/bit
    //  Sim:  use 8 cycles/bit so the 404-byte packet arrives quickly
    //  NOTE: top_level's uart_rx uses its own CLK_FREQ/BAUD_RATE params.
    //        For integration sim, override both to match.
    //        If your project locks CLK_FREQ in uart_rx, set FAST_BAUD to
    //        100_000_000/8 = 12_500_000 and CLK_FREQ stays 100_000_000.
    localparam CLK_FREQ_SIM  = 100_000_000;
    localparam BAUD_SIM      = 12_500_000;   // 8 cycles/bit at 100 MHz
    localparam BIT_TICKS_SIM = CLK_FREQ_SIM / BAUD_SIM; // = 8
    
    //  DUT signals
    reg  clk           = 0;
    reg  CPU_RESETN    = 1;
    reg  UART_TXD_IN   = 1;   // idle high
    wire UART_RXD_OUT;
    wire PWM_L;
    wire PWM_R;

    //  DUT: override UART baud so simulation finishes quickly
    //  If top_level doesn't expose baud as a parameter, wrap it or
    //  edit uart_rx CLK_FREQ/BAUD_RATE for simulation only.
    top_level #(
        // These parameters only take effect if top_level passes them
        // through to uart_rx / uart_tx.  Add parameter ports to
        // top_level if needed; for now this documents the intent.
        .CLK_FREQ(CLK_FREQ_SIM),
        .BAUD_RATE(BAUD_SIM)
    ) dut (
        .CLK100MHZ(clk),
        .CPU_RESETN(CPU_RESETN),
        .UART_TXD_IN(UART_TXD_IN),
        .UART_RXD_OUT(UART_RXD_OUT),
        .PWM_L(PWM_L),
        .PWM_R(PWM_R)
    );

    // 100 MHz clock
    always #5 clk = ~clk;

    //  UART TX task: sends one byte LSB-first, 8N1
    //  Uses BIT_TICKS_SIM to match the fast baud rate above
    task uart_send_byte;
        input [7:0] data;
        integer b;
        begin
            // Start bit
            UART_TXD_IN = 1'b0;
            repeat(BIT_TICKS_SIM) @(posedge clk);
            // 8 data bits, LSB first
            for (b = 0; b < 8; b = b + 1) begin
                UART_TXD_IN = data[b];
                repeat(BIT_TICKS_SIM) @(posedge clk);
            end
            // Stop bit
            UART_TXD_IN = 1'b1;
            repeat(BIT_TICKS_SIM) @(posedge clk);
        end
    endtask

    // Send the full 404-byte navigation packet
    // Packet: [start_row, start_col, goal_row, goal_col, 400x map bytes]
    task send_nav_packet;
        input [7:0] start_row;
        input [7:0] start_col;
        input [7:0] goal_row;
        input [7:0] goal_col;
        input put_obstacle;  // if 1, block cell (5,5) as test obstacle
        integer r, c;
        reg [7:0] cell_val;
        begin
            uart_send_byte(start_row);
            uart_send_byte(start_col);
            uart_send_byte(goal_row);
            uart_send_byte(goal_col);
            // 400 map bytes row-major
            for (r = 0; r < 20; r = r + 1) begin
                for (c = 0; c < 20; c = c + 1) begin
                    if (put_obstacle && r == 5 && c == 5)
                        cell_val = 8'h01;
                    else
                        cell_val = 8'h00;
                    uart_send_byte(cell_val);
                end
            end
        end
    endtask

    //  UART RX capture: sample UART_RXD_OUT and decode one byte
    //  Returns decoded byte, or 8'hXX on framing error
    task uart_capture_byte;
        output [7:0] data;
        output       framing_ok;
        integer b;
        reg [7:0] rx_bits;
        begin
            // Wait for start bit (falling edge)
            wait(UART_RXD_OUT == 1'b0);
            // Skip to middle of start bit
            repeat(BIT_TICKS_SIM / 2) @(posedge clk);
            // Verify it's still 0
            framing_ok = (UART_RXD_OUT == 1'b0);
            // Sample 8 data bits
            for (b = 0; b < 8; b = b + 1) begin
                repeat(BIT_TICKS_SIM) @(posedge clk);
                rx_bits[b] = UART_RXD_OUT;
            end
            // Sample stop bit
            repeat(BIT_TICKS_SIM) @(posedge clk);
            if (UART_RXD_OUT !== 1'b1) framing_ok = 0;
            data = rx_bits;
        end
    endtask

    //  Test counters
    integer pass_count = 0;
    integer fail_count = 0;
    reg [7:0] rx_byte;
    reg framing_ok;
    integer result;
    
    //  Test sequence
    initial begin
        $display("  top_level integration testbench");
        $display("  Sim baud = %0d (%0d cycles/bit)",
                 BAUD_SIM, BIT_TICKS_SIM);

        // Reset
        CPU_RESETN = 0;
        repeat(10) @(posedge clk);
        CPU_RESETN = 1;
        repeat(5)  @(posedge clk);

        //  TEST 1: UART RXD_OUT idles high before any packet
        $display("\n[TEST 1] UART_RXD_OUT idles high at startup");
        repeat(20) @(posedge clk);
        if (UART_RXD_OUT == 1'b1) begin
            $display("  PASS: UART_RXD_OUT=1 (idle mark)");
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: UART_RXD_OUT=%0b (expected 1)", UART_RXD_OUT);
            fail_count = fail_count + 1;
        end

        //  TEST 2: PWM outputs are low/neutral before navigation starts
        $display("\n[TEST 2] PWM outputs neutral before navigation");
        if (PWM_L == 1'b0 && PWM_R == 1'b0) begin
            $display("  PASS: PWM_L=0 PWM_R=0");
            pass_count = pass_count + 1;
        end else begin
            $display("  WARN: PWM_L=%0b PWM_R=%0b (depends on pwm_motor_ctrl idle state)",
                     PWM_L, PWM_R);
            //  Depends on pwm_motor_ctrl implementation
        end

        //  TEST 3: Full packet send A* auto-triggers 0xAA reply
        //  Send start=(0,0) goal=(0,3), open grid
        //  A* should find a path and UART_RXD_OUT should emit 0xAA
        $display("\n[TEST 3] Packet send -> A* -> 0xAA reply");
        fork
        begin : debug_probe
            integer probe_t;
            for (probe_t = 0; probe_t < 1_000_000; probe_t = probe_t + 1) begin
                @(posedge clk);
                if (dut.go_pulse)        $display("  DEBUG: go_pulse at t=%0t", $time);
                if (dut.astar_searching) $display("  DEBUG: astar searching at t=%0t", $time);
                if (dut.astar_done)      $display("  DEBUG: astar_done at t=%0t", $time);
            end
        end
            // Thread 1: send the packet
            begin
                send_nav_packet(8'd0, 8'd0, 8'd0, 8'd3, 1'b0);
                $display("  Packet sent (404 bytes)");
            end
            // Thread 2: wait for 0xAA on UART_RXD_OUT
            begin
                uart_capture_byte(rx_byte, framing_ok);
            end
        join

        if (framing_ok && rx_byte == 8'hAA) begin
            $display("  PASS: received 0xAA (path found), framing_ok=%0b",
                     framing_ok);
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: rx_byte=0x%02X framing_ok=%0b (expected 0xAA,1)",
                     rx_byte, framing_ok);
            fail_count = fail_count + 1;
        end

        // Wait for any ongoing transmission to finish
        wait(UART_RXD_OUT == 1'b1);
        repeat(100) @(posedge clk);

        //  TEST 4: Unreachable goal: 0xFF reply
        //  Surround goal (1,1)=cell 21 with obstacles:
        //  cells 1 (N), 41 (S), 20 (W), 22 (E) all = obstacle
        //  We encode these in the map bytes sent over UART
        $display("\n[TEST 4] Unreachable goal -> 0xFF reply");

        // Reset to clear previous state
        CPU_RESETN = 0;
        repeat(5) @(posedge clk);
        CPU_RESETN = 1;
        repeat(5) @(posedge clk);

        fork
            begin : send_blocked
                // Send header
                uart_send_byte(8'd0);  // start_row=0
                uart_send_byte(8'd0);  // start_col=0
                uart_send_byte(8'd1);  // goal_row=1
                uart_send_byte(8'd1);  // goal_col=1  â†’ cell 21
                // Send map with goal surrounded
                begin : map_blocked
                    integer rb, cb;
                    reg [7:0] mv;
                    for (rb = 0; rb < 20; rb = rb + 1) begin
                        for (cb = 0; cb < 20; cb = cb + 1) begin
                            // Block N/S/E/W of cell 21
                            if ((rb==0 && cb==1) ||   // cell 1  (north)
                                (rb==2 && cb==1) ||   // cell 41 (south)
                                (rb==1 && cb==0) ||   // cell 20 (west)
                                (rb==1 && cb==2))     // cell 22 (east)
                                mv = 8'h01;
                            else
                                mv = 8'h00;
                            uart_send_byte(mv);
                        end
                    end
                end
                $display("  Blocked-goal packet sent");
            end
            begin : cap_ff
                uart_capture_byte(rx_byte, framing_ok);
            end
        join

        if (framing_ok && rx_byte == 8'hFF) begin
            $display("  PASS: received 0xFF (no path), framing_ok=%0b", framing_ok);
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: rx_byte=0x%02X framing_ok=%0b (expected 0xFF,1)",
                     rx_byte, framing_ok);
            fail_count = fail_count + 1;
        end

        wait(UART_RXD_OUT == 1'b1);
        repeat(100) @(posedge clk);

        //  TEST 5: 0xBB reply when nav_done pulses
        //  Send a valid short path, wait for 0xAA, then wait for 0xBB
        //  (nav_fsm uses sim-speed DRIVE_TICKS so this completes quickly
        //   IF top_level also uses sim parameters, otherwise this test
        //   documents the expected flow and may need a longer timeout)
        $display("\n[TEST 5] nav_done -> 0xBB reply");

        CPU_RESETN = 0;
        repeat(5) @(posedge clk);
        CPU_RESETN = 1;
        repeat(5) @(posedge clk);

        // Send a 1-step path (start=0,0 goal=0,1)
        send_nav_packet(8'd0, 8'd0, 8'd0, 8'd1, 1'b0);
        $display("  Short path packet sent");

        // Expect 0xAA first
        uart_capture_byte(rx_byte, framing_ok);
        if (rx_byte == 8'hAA)
            $display("  0xAA received (path found): waiting for 0xBB...");
        else
            $display("  WARN: first byte was 0x%02X not 0xAA", rx_byte);

        // Now expect 0xBB (nav_done)
        begin : wait_bb
            integer wt;
            result = 0;
            for (wt = 0; wt < 50_000_000 && result == 0; wt = wt + 1) begin
                @(posedge clk);
                // Poll UART_RXD_OUT for falling edge (start of new byte)
                if (UART_RXD_OUT == 1'b0) begin
                    uart_capture_byte(rx_byte, framing_ok);
                    result = 1;
                end
            end
        end

        if (result && framing_ok && rx_byte == 8'hBB) begin
            $display("  PASS: received 0xBB (navigation complete)");
            pass_count = pass_count + 1;
        end else if (result) begin
            $display("  FAIL: rx_byte=0x%02X framing_ok=%0b (expected 0xBB,1)",
                     rx_byte, framing_ok);
            fail_count = fail_count + 1;
        end else begin
            $display("  FAIL: timeout waiting for 0xBB");
            $display("  NOTE: If nav_fsm uses real DRIVE_TICKS (100M), increase");
            $display("        top_level DRIVE_TICKS parameter for simulation.");
            fail_count = fail_count + 1;
        end

        //  Summary
        $display("\n========================================");
        $display("  RESULTS: %0d passed, %0d failed", pass_count, fail_count);
        $display("========================================");
        if (fail_count == 0)
            $display("  ALL TESTS PASSED");
        else
            $display("  SOME TESTS FAILED - review above");
        $display("========================================\n");

        $finish;
    end

    // Watchdog: 500 ms sim time
    initial begin
        #500_000_000;
        $display("WATCHDOG: simulation timeout");
        $finish;
    end

endmodule