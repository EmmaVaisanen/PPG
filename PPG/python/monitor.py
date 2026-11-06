import serial
import time
import matplotlib.pyplot as plt
from collections import deque

# --------------------------------------------------
# Configuration
# --------------------------------------------------

PORT = "/dev/cu.usbserial-120"
BAUD = 115200

MAX_POINTS = 1000
WINDOW_SECONDS = 10

BASELINE_SAMPLES = 100

# --------------------------------------------------
# State
# --------------------------------------------------

start_time = None

time_data = deque(maxlen=MAX_POINTS)

ir_data = deque(maxlen=MAX_POINTS)
ir_filtered = deque(maxlen=MAX_POINTS)
ir_baseline_buffer = deque(maxlen=BASELINE_SAMPLES)

amag_data = deque(maxlen=MAX_POINTS)

# --------------------------------------------------
# Serial
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
# Plot
# --------------------------------------------------

def initialize_plot():
    plt.ion()

    fig, (ax_ppg, ax_imu) = plt.subplots(
        2,
        1,
        sharex=True
    )

    ppg_line, = ax_ppg.plot([], [])
    imu_line, = ax_imu.plot([], [])

    # PPG
    ax_ppg.set_title("Live PPG + IMU")
    ax_ppg.set_ylabel("IR AC [counts]")
    ax_ppg.set_ylim(-1000, 1000)

    # IMU
    ax_imu.set_xlabel("Time [s]")
    ax_imu.set_ylabel("Acceleration [m/s²]")
    ax_imu.set_ylim(5, 15)

    ax_imu.set_xlim(0, WINDOW_SECONDS)

    fig.tight_layout()
    plt.show(block=False)

    return fig, ax_ppg, ax_imu, ppg_line, imu_line


# --------------------------------------------------
# Parse ESP32 data
# --------------------------------------------------

def parse_data(raw):
    parts = raw.split(",")

    if len(parts) != 11:
        return None

    try:
        return {
            "time_ms": float(parts[0]),
            "red": float(parts[1]),
            "ir": float(parts[2]),
            "green": float(parts[3]),

            "ax": float(parts[4]),
            "ay": float(parts[5]),
            "az": float(parts[6]),
            "amag": float(parts[7]),

            "gx": float(parts[8]),
            "gy": float(parts[9]),
            "gz": float(parts[10])
        }

    except ValueError:
        return None


# --------------------------------------------------
# PPG baseline removal
# --------------------------------------------------

def remove_ir_baseline(ir):
    ir_baseline_buffer.append(ir)

    baseline = (
        sum(ir_baseline_buffer)
        / len(ir_baseline_buffer)
    )

    return ir - baseline


# --------------------------------------------------
# Axis helpers
# --------------------------------------------------

def update_time_axis(ax, time_s):
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


def update_y_axis(ax, data, minimum_margin):
    if len(data) < 2:
        return

    ymin = min(data)
    ymax = max(data)

    value_range = ymax - ymin

    margin = max(
        value_range * 0.30,
        minimum_margin
    )

    ax.set_ylim(
        ymin - margin,
        ymax + margin
    )


# --------------------------------------------------
# Update plots
# --------------------------------------------------

def update_plot(
    ax_ppg,
    ax_imu,
    ppg_line,
    imu_line,
    time_s,
    data
):
    # Time
    time_data.append(time_s)

    # -----------------------------
    # PPG
    # -----------------------------

    ir = data["ir"]

    ir_data.append(ir)

    ir_ac = remove_ir_baseline(ir)

    ir_filtered.append(ir_ac)

    ppg_line.set_data(
        time_data,
        ir_filtered
    )

    # -----------------------------
    # IMU
    # -----------------------------

    amag = data["amag"]

    amag_data.append(amag)

    imu_line.set_data(
        time_data,
        amag_data
    )

    # -----------------------------
    # Axes
    # -----------------------------

    update_time_axis(
        ax_imu,
        time_s
    )

    update_y_axis(
        ax_ppg,
        ir_filtered,
        minimum_margin=100
    )

    update_y_axis(
        ax_imu,
        amag_data,
        minimum_margin=0.5
    )


# --------------------------------------------------
# Main
# --------------------------------------------------

def main():
    global start_time

    ser = initialize_serial()

    (
        fig,
        ax_ppg,
        ax_imu,
        ppg_line,
        imu_line
    ) = initialize_plot()

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

            time_ms = data["time_ms"]

            if start_time is None:
                start_time = time_ms

            time_s = (
                time_ms - start_time
            ) / 1000.0

            update_plot(
                ax_ppg,
                ax_imu,
                ppg_line,
                imu_line,
                time_s,
                data
            )

            plt.pause(0.01)

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        ser.close()


if __name__ == "__main__":
    main()