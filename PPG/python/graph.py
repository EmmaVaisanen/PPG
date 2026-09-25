import serial
import time
import matplotlib.pyplot as plt
from collections import deque

# --------------------------------------------------
# Configuration
# --------------------------------------------------

PORT = "/dev/cu.usbserial-1130"
BAUD = 115200

MAX_POINTS = 1000
WINDOW_SECONDS = 10

BASELINE_SAMPLES = 100

Y_MARGIN_FACTOR = 0.30
MIN_Y_MARGIN = 100

# --------------------------------------------------
# State
# --------------------------------------------------

start_time = None

time_data = deque(maxlen=MAX_POINTS)
ir_data = deque(maxlen=MAX_POINTS)

baseline_buffer = deque(maxlen=BASELINE_SAMPLES)
filtered_data = deque(maxlen=MAX_POINTS)

# --------------------------------------------------
# Serial setup
# --------------------------------------------------

def initialize_serial():
    ser = serial.Serial(
        PORT,
        BAUD,
        timeout=0.1
    )

    time.sleep(2)
    ser.reset_input_buffer()

    return ser


# --------------------------------------------------
# Plot setup
# --------------------------------------------------

def initialize_plot():
    plt.ion()

    fig, ax = plt.subplots()
    line, = ax.plot([], [])

    ax.set_title("Live PPG - DC Removed")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("IR AC component [counts]")

    ax.set_xlim(0, WINDOW_SECONDS)
    ax.set_ylim(-1000, 1000)

    plt.show(block=False)

    return fig, ax, line


# --------------------------------------------------
# Parse serial line
# --------------------------------------------------

def parse_data(raw):
    parts = raw.split(",")

    if len(parts) != 10:
        return None

    try:
        time_ms = float(parts[0])
        ir = float(parts[2])

        return time_ms, ir

    except ValueError:
        return None


# --------------------------------------------------
# Remove baseline / DC component
# --------------------------------------------------

def remove_baseline(ir):
    baseline_buffer.append(ir)

    baseline = (
        sum(baseline_buffer)
        / len(baseline_buffer)
    )

    ir_ac = ir - baseline

    return ir_ac


# --------------------------------------------------
# Update graph
# --------------------------------------------------

def update_plot(ax, line, time_s, ir):
    time_data.append(time_s)
    ir_data.append(ir)

    ir_ac = remove_baseline(ir)

    filtered_data.append(ir_ac)

    line.set_data(
        time_data,
        filtered_data
    )

    # -----------------------------
    # Rolling X-axis
    # -----------------------------

    if time_s < WINDOW_SECONDS:
        ax.set_xlim(
            0,
            WINDOW_SECONDS
        )
    else:
        ax.set_xlim(
            time_s - WINDOW_SECONDS,
            time_s
        )

    # -----------------------------
    # Adaptive Y-axis
    # -----------------------------

    if len(filtered_data) > 1:
        ymin = min(filtered_data)
        ymax = max(filtered_data)

        yrange = ymax - ymin

        margin = max(
            yrange * Y_MARGIN_FACTOR,
            MIN_Y_MARGIN
        )

        ax.set_ylim(
            ymin - margin,
            ymax + margin
        )


# --------------------------------------------------
# Main
# --------------------------------------------------

def main():
    global start_time

    ser = initialize_serial()
    fig, ax, line = initialize_plot()

    print("Waiting for ESP32 data...")

    try:
        while plt.fignum_exists(fig.number):

            raw = ser.readline().decode(
                "utf-8",
                errors="ignore"
            ).strip()

            if not raw:
                plt.pause(0.01)
                continue

            data = parse_data(raw)

            if data is None:
                plt.pause(0.01)
                continue

            time_ms, ir = data

            if start_time is None:
                start_time = time_ms

            time_s = (
                time_ms - start_time
            ) / 1000.0

            update_plot(
                ax,
                line,
                time_s,
                ir
            )

            plt.pause(0.01)

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        ser.close()


if __name__ == "__main__":
    main()