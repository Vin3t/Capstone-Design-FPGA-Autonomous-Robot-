"""
robot_gui.py  â€”  Raspberry Pi touchscreen control panel for FPGA Autonomous A* pathfinding service robot (Nexys A7)

Touch/click workflow:
  1. App opens  â†’ robot's current grid position is the start (fixed at boot, update start_point whenever odometry updates it)

  2. User taps  â†’ sets the GOAL cell (single tap, no two-step selection)

  3. "SEND & RUN" button â†’ serialises the 404-byte UART packet and sends it
                  to the FPGA.  The FPGA auto-starts A* the moment the last
                  map byte arrives (load_done rising edge in top_level.v)

  4. GUI polls UART RX for a 1-byte status reply from the FPGA:
       0xAA â†’ path found, navigation running
       0xFF â†’ no path exists
       0xBB â†’ navigation complete (robot arrived)

  5. Once 0xBB received, UNLOCK button activates.

UART packet sent to FPGA (must match top_level.v loader FSM):
  Byte 0        : start_row   (0-19)
  Byte 1        : start_col   (0-19)
  Byte 2        : goal_row    (0-19)
  Byte 3        : goal_col    (0-19)
  Bytes 4-403   : obstacle map, row-major, 400 bytes (0x00=free, 0x01=obstacle)

Dependencies:
  pip install pyserial pillow numpy
"""

import tkinter as tk
from tkinter import messagebox
from PIL import Image, ImageTk
import numpy as np
import serial
import serial.tools.list_ports
import threading
import time

#  Edit SERIAL_PORT to match RPi UART device
#  On Raspberry Pi the hardware UART is /dev/ttyAMA0 or /dev/serial0 I can't remember right now, but if using a USB-UART adapter it will be /dev/ttyUSB0 or similar. Check with `ls /dev/tty*` in terminal.
#  Make sure to enable UART in raspi-config and disable the serial console.
SERIAL_PORT = "/dev/serial0"   # change to "/dev/ttyUSB0" if using USB-UART
BAUD_RATE   = 115200

# FPGA status reply bytes
STATUS_FOUND    = 0xAA   # A* succeeded, motors running
STATUS_NO_PATH  = 0xFF   # no path exists
STATUS_ARRIVED  = 0xBB   # navigation complete

# Grid / display parameters â€” MUST match FPGA (ROWS=20, COLS=20)
GRID_ROWS  = 20
GRID_COLS  = 20
CELL_SIZE  = 22           # pixels per cell  (20*22 = 440px grid)
CANVAS_W   = GRID_COLS * CELL_SIZE   # 440
CANVAS_H   = GRID_ROWS * CELL_SIZE   # 440

# Colors
COL_BG         = "#4f2d10"
COL_FREE       = "#f0f0f0"
COL_OBSTACLE   = "#1a1a1a"
COL_PATH       = "#4A90E2"
COL_START      = "#27ae60"
COL_GOAL       = "#e74c3c"
COL_GRID_LINE  = "#cccccc"

#  State
grid_data  = np.zeros((GRID_ROWS, GRID_COLS), dtype=np.uint8)

# start_point is where the robot current position is.
# In a real system, update this from odometry / encoder feedback.
# Col, Row format to match canvas x/y convention used throughout.
start_point         = (GRID_COLS // 2, GRID_ROWS // 2)    # (col, row) â€” top-left by default
goal_point          = None      # (col, row) â€” chosen by user tap
current_path        = []        # list of (col, row) tuples (preview only)
robot_at_destination = False
box_locked          = True

# Serial port handle (None if not connected)
ser = None

#  Serial helpers
def open_serial():
    "Try to open the UART port; update status label on result."
    global ser
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05)
        set_status(f"Serial open: {SERIAL_PORT}", "lightgreen")
    except serial.SerialException as e:
        ser = None
        set_status(f"Serial error: {e}", "red")


def send_to_fpga():
    """
    Build and transmit the 404-byte packet.
    Byte layout matches top_level.v loader FSM exactly.
    Runs in the calling thread. Call via button handler which already
    runs in main thread (fast enough at 115200 baud, <35 ms total).
    """
    if ser is None or not ser.is_open:
        set_status("Not connected to FPGA!", "red")
        return False

    sr, sc = start_point[1], start_point[0]   # row, col
    gr, gc = goal_point[1],  goal_point[0]

    # Header: 4 bytes
    header = bytes([sr, sc, gr, gc])

    # Map: 400 bytes, row-major  (row 0 first, then row 1, â€¦)
    map_bytes = grid_data.flatten().tobytes()   # shape (400,), dtype uint8

    packet = header + map_bytes                 # 404 bytes total

    try:
        ser.reset_output_buffer()
        ser.write(packet)
        ser.flush()
        return True
    except serial.SerialException as e:
        set_status(f"Send failed: {e}", "red")
        return False


def poll_fpga_status():
    """
    Background thread: reads 1-byte status replies from FPGA.
    FPGA sends a byte when A* finishes or navigation completes.
    """
    global robot_at_destination
    while True:
        if ser and ser.is_open:
            try:
                b = ser.read(1)
                if len(b) == 1:
                    code = b[0]
                    if code == STATUS_FOUND:
                        root.after(0, lambda: set_status(
                            "Path found â€” robot navigatingâ€¦", "lightgreen"))
                    elif code == STATUS_NO_PATH:
                        root.after(0, lambda: set_status(
                            "No path exists! Replan.", "red"))
                    elif code == STATUS_ARRIVED:
                        robot_at_destination = True
                        root.after(0, on_arrived)
            except serial.SerialException:
                pass
        time.sleep(0.05)


#  GUI helpers
def set_status(msg, fg="white"):
    status_label.config(text=msg, fg=fg)

def on_arrived():
    set_status("Robot arrived at destination!", "lightgreen")
    update_lock_button()

def draw_grid():
    """Redraw the entire canvas."""
    canvas.delete("all")

    path_set = set(current_path)

    for row in range(GRID_ROWS):
        for col in range(GRID_COLS):
            x1 = col * CELL_SIZE
            y1 = row * CELL_SIZE
            x2 = x1 + CELL_SIZE
            y2 = y1 + CELL_SIZE

            if grid_data[row][col] == 1:
                fill = COL_OBSTACLE
            elif (col, row) in path_set:
                fill = COL_PATH
            else:
                fill = COL_FREE

            canvas.create_rectangle(x1, y1, x2, y2,
                                     fill=fill, outline=COL_GRID_LINE, width=1)

    # Start marker (green circle)
    sc, sr = start_point
    x1 = sc * CELL_SIZE + 3
    y1 = sr * CELL_SIZE + 3
    canvas.create_oval(x1, y1, x1 + CELL_SIZE - 6, y1 + CELL_SIZE - 6,
                       fill=COL_START, outline="")

    # Goal marker (red circle)
    if goal_point:
        gc_col, gc_row = goal_point
        x1 = gc_col * CELL_SIZE + 3
        y1 = gc_row * CELL_SIZE + 3
        canvas.create_oval(x1, y1, x1 + CELL_SIZE - 6, y1 + CELL_SIZE - 6,
                           fill=COL_GOAL, outline="")

    # Legend dots in top-right corner of canvas (small, unobtrusive)
    _draw_legend()


def _draw_legend():
    lx = CANVAS_W - 5
    for colour, label in [(COL_START, "Start"),
                           (COL_GOAL,  "Goal"),
                           (COL_PATH,  "Path")]:
        canvas.create_oval(lx - 28, 4, lx - 18, 14, fill=colour, outline="")
        canvas.create_text(lx - 2, 9, text=label, anchor="e",
                           font=("Arial", 7), fill="#333")
        # stacked vertically â€” not used; legend is in the sidebar

#  Canvas touch / click handler
def canvas_click(event):
    """
    Single-tap to set GOAL cell.
    The start is always the robot's current position (start_point).
    Tapping an obstacle is ignored.
    Tapping the current goal clears it.
    """
    global goal_point, current_path

    col = event.x // CELL_SIZE
    row = event.y // CELL_SIZE

    if not (0 <= col < GRID_COLS and 0 <= row < GRID_ROWS):
        return
    if grid_data[row][col] == 1:
        set_status("Can't set goal on an obstacle.", "orange")
        return
    if (col, row) == start_point:
        set_status("Goal can't be the same as start.", "orange")
        return

    # Toggle: tap same cell again to clear goal
    if goal_point == (col, row):
        goal_point    = None
        current_path  = []
        set_status("Goal cleared. Tap a cell to set new goal.", "white")
        draw_grid()
        return

    goal_point   = (col, row)
    current_path = []    # clear old preview; FPGA computes the real path
    set_status(f"Goal set â†’ row {row}, col {col}. Tap 'SEND & RUN'.", "white")
    draw_grid()
    btn_send.config(state=tk.NORMAL)


canvas_click_bound = None   # assigned after canvas creation

#  Obstacle toggle (right-click / long-press simulation via Button-3)
def canvas_right_click(event):
    """Toggle obstacle on right-click (or long-press if mapped by OS)."""
    col = event.x // CELL_SIZE
    row = event.y // CELL_SIZE
    if not (0 <= col < GRID_COLS and 0 <= row < GRID_ROWS):
        return
    if (col, row) == start_point or (col, row) == goal_point:
        return
    grid_data[row][col] ^= 1   # toggle 0â†”1
    draw_grid()

#  Button callbacks
def load_lidar_data():
    """
    Placeholder: in production, replace with real LiDAR / depth-camera parse.
    Currently places random obstacles so you can test the UI and UART flow.
    """
    global grid_data
    grid_data = np.zeros((GRID_ROWS, GRID_COLS), dtype=np.uint8)
    rng = np.random.default_rng()
    obs_count = 40
    positions = rng.choice(GRID_ROWS * GRID_COLS, size=obs_count, replace=False)
    for p in positions:
        r, c = divmod(p, GRID_COLS)
        # Don't block start
        if (c, r) != start_point:
            grid_data[r][c] = 1
    draw_grid()
    set_status("LiDAR map loaded. Tap a cell to set goal.", "white")


def clear_grid():
    global grid_data, goal_point, current_path, robot_at_destination
    grid_data            = np.zeros((GRID_ROWS, GRID_COLS), dtype=np.uint8)
    goal_point           = None
    current_path         = []
    robot_at_destination = False
    btn_send.config(state=tk.DISABLED)
    update_lock_button()
    draw_grid()
    set_status("Grid cleared.", "white")


def send_and_run():
    """
    Transmit the 404-byte packet to the FPGA over UART.
    The FPGA will auto-start A* as soon as the last map byte arrives.
    No button press on the FPGA side is needed.
    """
    global robot_at_destination

    if goal_point is None:
        set_status("Set a goal first!", "orange")
        return

    robot_at_destination = False
    update_lock_button()
    btn_send.config(state=tk.DISABLED)

    set_status("Sending to FPGAâ€¦", "yellow")
    root.update_idletasks()   # flush UI before blocking write

    if send_to_fpga():
        set_status("Packet sent â€” FPGA searchingâ€¦", "yellow")
    # Status will be updated by poll_fpga_status() background thread
    # when the FPGA replies.


def toggle_lock():
    global box_locked
    if not robot_at_destination:
        set_status("Robot must reach destination first!", "red")
        return
    if box_locked:
        box_locked = False
        btn_lock.config(text="LOCK", bg="#e67e22")
        set_status("Box unlocked!", "lightgreen")
        # TODO: trigger servo GPIO here, e.g. GPIO.output(SERVO_PIN, HIGH)
    else:
        box_locked = True
        btn_lock.config(text="UNLOCK", bg="#f1c40f")
        set_status("Box locked!", "lightgreen")
        # TODO: reset servo GPIO


def update_lock_button():
    if robot_at_destination:
        btn_lock.config(state=tk.NORMAL)
        btn_lock.config(text="UNLOCK" if box_locked else "LOCK",
                        bg="#f1c40f" if box_locked else "#e67e22")
    else:
        btn_lock.config(state=tk.DISABLED, bg="gray", text="UNLOCK")


def off():
    if ser and ser.is_open:
        ser.close()
    root.destroy()

#  Build UI
root = tk.Tk()
root.title("Robot Control â€” Path Planning")
root.geometry("660x500")
root.configure(bg=COL_BG)
root.resizable(False, False)

# Grid canvas
canvas = tk.Canvas(root, width=CANVAS_W, height=CANVAS_H,
                   bg=COL_FREE, highlightthickness=1,
                   highlightbackground="#888")
canvas.place(x=10, y=10)
canvas.bind("<Button-1>", canvas_click)
canvas.bind("<Button-3>", canvas_right_click)  # right-click = toggle obstacle

# Sidebar 
SB_X = CANVAS_W + 20   # 460

btn_load = tk.Button(root, text="Load LiDAR", command=load_lidar_data,
                     width=13, height=1, bg="#4A90E2", fg="white",
                     font=("Arial", 10))
btn_load.place(x=SB_X, y=20)

btn_clear = tk.Button(root, text="Clear Grid", command=clear_grid,
                      width=13, height=1, bg="#F5A623", fg="white",
                      font=("Arial", 10))
btn_clear.place(x=SB_X, y=58)

btn_send = tk.Button(root, text="SEND & RUN", command=send_and_run,
                     width=13, height=2, bg="#27ae60", fg="white",
                     font=("Arial", 10, "bold"), state=tk.DISABLED)
btn_send.place(x=SB_X, y=96)

btn_lock = tk.Button(root, text="UNLOCK", command=toggle_lock,
                     width=13, height=2, bg="gray", fg="white",
                     font=("Arial", 10, "bold"), state=tk.DISABLED)
btn_lock.place(x=SB_X, y=158)

btn_off = tk.Button(root, text="OFF", command=off,
                    width=13, height=2, bg="#c0392b", fg="white",
                    font=("Arial", 10, "bold"))
btn_off.place(x=SB_X, y=220)

# Legend
legend_text = (
    "â— Green  = Robot start\n"
    "â— Red      = Goal\n"
    "â— Blue      = Path\n"
    "â— Black    = Obstacle\n\n"
    "Left-tap  â†’ set goal\n"
    "Right-tap â†’ toggle obstacle"
)
tk.Label(root, text=legend_text, bg=COL_BG, fg="white",
         font=("Arial", 8), justify=tk.LEFT).place(x=SB_X, y=285)

# Status bar
status_label = tk.Label(root, text="Tap 'Load LiDAR', then tap a goal cell.",
                         bg=COL_BG, fg="white", font=("Arial", 9),
                         anchor="w", width=55)
status_label.place(x=10, y=460)

# Lehigh logo
try:
    img = Image.open("/home/capstone-b3/lehigh_logo.png")
    img = img.resize((300, 60))
    photo = ImageTk.PhotoImage(img)
    img_label = tk.Label(root, image=photo, bg=COL_BG)
    img_label.image = photo
    img_label.place(x=SB_X, y=385)
except Exception:
    print("Image not found")

#  Startup
# Open serial port
open_serial()

# Start background status-polling thread
poll_thread = threading.Thread(target=poll_fpga_status, daemon=True)
poll_thread.start()

# Initial draw
draw_grid()

root.mainloop()