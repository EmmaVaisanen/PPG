import serial
import time
import numpy as np
import matplotlib.pyplot as plt
from collections import deque

# --------------------------------------------------
# Configuration
# --------------------------------------------------

PORT = "/dev/cu.usbserial-1120"
BAUD = 115200

MAX_POINTS = 1000
WINDOW_SECONDS = 10

BASELINE_SAMPLES = 100

NLMS_TAPS = 16
NLMS_MU = 0.1
NLMS_EPSILON = 1e-6

# --------------------------------------------------
# State
# --------------------------------------------------

start_time = None

time_data = deque(maxlen=MAX_POINTS)

ir_data = deque(maxlen=MAX_POINTS)
ir_filtered = deque(maxlen=MAX_POINTS)
ir_baseline_buffer = deque(maxlen=BASELINE_SAMPLES)

amag_data = deque(maxlen=MAX_POINTS)
amag_motion_data = deque(maxlen=MAX_POINTS)
amag_baseline_buffer = deque(maxlen=BASELINE_SAMPLES)

nlms_output_data = deque(maxlen=MAX_POINTS)

# --------------------------------------------------
# NLMS adaptive filter
# --------------------------------------------------

class NLMSFilter:
    def __init__(
        self,
        taps=16,
        mu=0.1,
        epsilon=1e-6
    ):
        self.taps = taps
        self.mu = mu
        self.epsilon = epsilon

        self.weights = np.zeros(taps)
        self.reference_buffer = deque(
            [0.0] * taps,
            maxlen=taps
        )

    def update(self, primary, reference):
        self.reference_buffer.append(reference)

        x = np.array(
            list(self.reference_buffer)[::-1],
            dtype=float
        )

        estimated_motion = np.dot(
            self.weights,
            x
        )

        error = primary - estimated_motion

        norm = np.dot(x, x) + self.epsilon

        self.weights += (
            self.mu
            * error
            * x
            / norm
        )

        return error, estimated_motion


nlms_filter = NLMSFilter(
    taps=NLMS_TAPS,
    mu=NLMS_MU,
    epsilon=NLMS_EPSILON
)

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

    fig, (
        ax_ppg,
        ax_imu,
        ax_nlms
    ) = plt.subplots(
        3,
        1,
        sharex=True
    )

    ppg_line, = ax_ppg.plot([], [])
    imu_line, = ax_imu.plot([], [])
    nlms_line, = ax_nlms.plot([], [])

    ax_ppg.set_title("Live PPG + IMU + NLMS")
    ax_ppg.set_ylabel("IR AC [counts]")
    ax_ppg.set_ylim(-1000, 1000)

    ax_imu.set_ylabel(
        "IMU motion [m/s²]"
    )
    ax_imu.set_ylim(-5, 5)

    ax_nlms.set_xlabel("Time [s]")
    ax_nlms.set_ylabel(
        "NLMS output [counts]"
    )
    ax_nlms.set_ylim(-1000, 1000)

    ax_nlms.set_xlim(
        0,
        WINDOW_SECONDS
    )

    fig.tight_layout()

    plt.show(block=False)

    return (
        fig,
        ax_ppg,
        ax_imu,
        ax_nlms,
        ppg_line,
        imu_line,
        nlms_line
    )

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
# Baseline removal
# --------------------------------------------------

def remove_baseline(value, buffer):
    buffer.append(value)

    baseline = (
        sum(buffer)
        / len(buffer)
    )

    return value - baseline

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


def update_y_axis(
    ax,
    data,
    minimum_margin
):
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
    ax_nlms,
    ppg_line,
    imu_line,
    nlms_line,
    time_s,
    data
):
    time_data.append(time_s)

    # --------------------------------
    # PPG preprocessing
    # --------------------------------

    ir = data["ir"]

    ir_data.append(ir)

    ir_ac = remove_baseline(
        ir,
        ir_baseline_buffer
    )

    ir_filtered.append(ir_ac)

    # --------------------------------
    # IMU preprocessing
    # --------------------------------

    amag = data["amag"]

    amag_data.append(amag)

    amag_motion = remove_baseline(
        amag,
        amag_baseline_buffer
    )

    amag_motion_data.append(
        amag_motion
    )

    # --------------------------------
    # NLMS
    # --------------------------------

    clean_ppg, estimated_motion = (
        nlms_filter.update(
            primary=ir_ac,
            reference=amag_motion
        )
    )

    nlms_output_data.append(
        clean_ppg
    )

    # --------------------------------
    # Update plot data
    # --------------------------------

    ppg_line.set_data(
        time_data,
        ir_filtered
    )

    imu_line.set_data(
        time_data,
        amag_motion_data
    )

    nlms_line.set_data(
        time_data,
        nlms_output_data
    )

    # --------------------------------
    # Axes
    # --------------------------------

    update_time_axis(
        ax_nlms,
        time_s
    )

    update_y_axis(
        ax_ppg,
        ir_filtered,
        minimum_margin=100
    )

    update_y_axis(
        ax_imu,
        amag_motion_data,
        minimum_margin=0.2
    )

    update_y_axis(
        ax_nlms,
        nlms_output_data,
        minimum_margin=100
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
        ax_nlms,
        ppg_line,
        imu_line,
        nlms_line
    ) = initialize_plot()

    print("Waiting for ESP32 data...")

    try:
        while plt.fignum_exists(
            fig.number
        ):

            raw = (
                ser.readline()
                .decode(
                    "utf-8",
                    errors="ignore"
                )
                .strip()
            )

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
                ax_nlms,
                ppg_line,
                imu_line,
                nlms_line,
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