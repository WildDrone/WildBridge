#!/usr/bin/env python3
"""
Telemetry Frequency Analyzer
Connects to the WildBridge TelemetryServer (TCP, port 8081) and measures
the actual rate at which telemetry packets are received.

Also tracks the update rate of the IMU attitude fields (pitch, roll, yaw).
Spin the drone manually during the test to generate observable yaw changes
and see how quickly the telemetry reflects them.

Usage:
    python3 telemetry_frequency_analyzer.py --host <drone_ip> [--port 8081] [--duration 30]
"""

import socket
import time
import json
import argparse
import statistics
from collections import deque

import matplotlib.pyplot as plt


# ── Main analysis ─────────────────────────────────────────────────────────────

def _get_imu_yaw(data: dict) -> float | None:
    """Extract yaw from the telemetry dict, trying common key paths."""
    # Try attitude dict (pitch/roll/yaw sub-dict)
    att = data.get("attitude")
    if isinstance(att, dict):
        yaw = att.get("yaw")
        if yaw is not None:
            return float(yaw)
    # Try flat heading / yaw key
    for key in ("heading", "yaw", "compassHeading"):
        val = data.get(key)
        if val is not None:
            return float(val)
    return None


def analyze_telemetry(host: str, port: int, duration: float, window: int = 50) -> None:
    """
    Connect to the TelemetryServer and measure packet reception frequency,
    plus the update rate of the IMU yaw field.

    Spin the drone during the test to provoke yaw changes — any packet where
    the yaw value differs from the previous is counted as an IMU update.

    Args:
        host:     IP of the Android device running WildBridgeApp.
        port:     TCP port of the TelemetryServer (default 8081).
        duration: Seconds to run (0 = until Ctrl+C).
        window:   Rolling window size for instantaneous frequency.
    """
    print(f"Connecting to {host}:{port} ...")
    print("Spin the drone during the test to observe IMU yaw update rate.\n")

    try:
        with socket.create_connection((host, port), timeout=10) as sock:
            print(f"Connected. Measuring telemetry frequency for "
                  f"{'%.0f s' % duration if duration > 0 else 'unlimited time'} "
                  f"(Ctrl+C to stop).\n")

            sock.settimeout(5.0)
            file = sock.makefile("r")

            # Overall packet tracking
            timestamps: deque = deque(maxlen=window)
            all_timestamps: list[float] = []          # full history for plotting
            packet_count = 0

            # IMU yaw tracking
            imu_timestamps: deque = deque(maxlen=window)
            all_imu_timestamps: list[float] = []      # full history for plotting
            all_yaw_values: list[float] = []           # yaw at each IMU update
            imu_update_count = 0
            last_yaw: float | None = None

            start_time = time.monotonic()
            last_report_time = start_time
            last_data = None

            try:
                while True:
                    now = time.monotonic()
                    if duration > 0 and (now - start_time) >= duration:
                        break

                    line = file.readline()
                    if not line:
                        print("Connection closed by server.")
                        break

                    recv_time = time.monotonic()
                    timestamps.append(recv_time)
                    all_timestamps.append(recv_time)
                    packet_count += 1

                    # Parse JSON
                    try:
                        data = json.loads(line.strip())
                        changed = (data != last_data)
                        last_data = data
                    except json.JSONDecodeError:
                        changed = None
                        data = {}

                    # Track IMU yaw changes
                    yaw = _get_imu_yaw(data) if isinstance(data, dict) else None
                    if yaw is not None and yaw != last_yaw:
                        imu_timestamps.append(recv_time)
                        all_imu_timestamps.append(recv_time)
                        all_yaw_values.append(yaw)
                        imu_update_count += 1
                        last_yaw = yaw

                    # Instantaneous packet frequency
                    if len(timestamps) >= 2:
                        intervals = [timestamps[i] - timestamps[i - 1]
                                     for i in range(1, len(timestamps))]
                        inst_freq: float | None = 1.0 / statistics.mean(intervals)
                    else:
                        inst_freq = None

                    # Instantaneous IMU update frequency
                    if len(imu_timestamps) >= 2:
                        imu_intervals = [imu_timestamps[i] - imu_timestamps[i - 1]
                                         for i in range(1, len(imu_timestamps))]
                        imu_freq: float | None = 1.0 / statistics.mean(imu_intervals)
                    else:
                        imu_freq = None

                    # Print status line every second
                    if recv_time - last_report_time >= 1.0:
                        elapsed = recv_time - start_time
                        overall_freq = packet_count / elapsed if elapsed > 0 else 0.0
                        freq_str = f"{inst_freq:.2f} Hz" if inst_freq is not None else "  --  "
                        change_str = ("CHANGED" if changed else "same   ") if changed is not None else "?"
                        imu_overall = imu_update_count / elapsed if elapsed > 0 else 0.0
                        imu_freq_str = f"{imu_freq:.2f} Hz" if imu_freq is not None else "  --  "
                        yaw_str = f"{last_yaw:+.2f}°" if last_yaw is not None else "  ?.??°"

                        print(
                            f"[{elapsed:6.1f}s] "
                            f"Pkts: {packet_count:5d} | "
                            f"Overall: {overall_freq:5.2f} Hz | "
                            f"Inst ({window}-pkt): {freq_str} | "
                            f"Data: {change_str} | "
                            f"IMU yaw updates: {imu_update_count:5d} "
                            f"({imu_overall:5.2f} Hz, inst {imu_freq_str}) "
                            f"yaw={yaw_str}"
                        )
                        last_report_time = recv_time

            except KeyboardInterrupt:
                pass
            except socket.timeout:
                print("Timed out waiting for data from server.")

            # ── Final summary ─────────────────────────────────────────────────
            elapsed = time.monotonic() - start_time
            print("\n" + "=" * 70)
            print("SUMMARY — Overall telemetry packets")
            print("=" * 70)
            print(f"  Duration          : {elapsed:.2f} s")
            print(f"  Total packets     : {packet_count}")
            if elapsed > 0 and packet_count > 0:
                overall_freq = packet_count / elapsed
                print(f"  Mean frequency    : {overall_freq:.3f} Hz")
                print(f"  Mean period       : {1000.0 / overall_freq:.1f} ms")
                if len(timestamps) >= 2:
                    all_iv = [timestamps[i] - timestamps[i - 1]
                              for i in range(1, len(timestamps))]
                    print(f"  Min interval      : {min(all_iv) * 1000:.1f} ms")
                    print(f"  Max interval      : {max(all_iv) * 1000:.1f} ms")
                    print(f"  Std dev interval  : {statistics.stdev(all_iv) * 1000:.1f} ms")

            print()
            print("SUMMARY — IMU yaw field updates (changed values only)")
            print("=" * 70)
            print(f"  Total IMU updates    : {imu_update_count}")
            if elapsed > 0 and imu_update_count > 0:
                imu_overall = imu_update_count / elapsed
                print(f"  Mean update rate     : {imu_overall:.3f} Hz")
                print(f"  Mean update period   : {1000.0 / imu_overall:.1f} ms")
                if len(imu_timestamps) >= 2:
                    imu_iv = [imu_timestamps[i] - imu_timestamps[i - 1]
                              for i in range(1, len(imu_timestamps))]
                    print(f"  Min interval         : {min(imu_iv) * 1000:.1f} ms")
                    print(f"  Max interval         : {max(imu_iv) * 1000:.1f} ms")
                    print(f"  Std dev interval     : {statistics.stdev(imu_iv) * 1000:.1f} ms")
            else:
                print("  (no yaw changes detected — spin the drone or check telemetry key name)")
            print("=" * 70)

    except Exception as e:
        print(f"Error: {e}")
        return

    _plot_curves(all_timestamps, all_imu_timestamps, all_yaw_values, start_time, window)


def _plot_curves(
    all_timestamps: list[float],
    all_imu_timestamps: list[float],
    all_yaw_values: list[float],
    start_time: float,
    window: int,
) -> None:
    """Compute rolling-window frequency curves and display them with matplotlib."""
    if len(all_timestamps) < 2:
        print("Not enough data to plot.")
        return

    def rolling_freq(ts: list[float], w: int) -> tuple[list[float], list[float]]:
        """Return (elapsed_times, frequencies) using a rolling window of size w."""
        times, freqs = [], []
        for i in range(w, len(ts)):
            segment = ts[i - w: i + 1]
            intervals = [segment[j] - segment[j - 1] for j in range(1, len(segment))]
            freq = 1.0 / statistics.mean(intervals)
            times.append(ts[i] - start_time)
            freqs.append(freq)
        return times, freqs

    pkt_t, pkt_f = rolling_freq(all_timestamps, window)
    imu_t, imu_f = rolling_freq(all_imu_timestamps, window) if len(all_imu_timestamps) > window else ([], [])
    yaw_t = [t - start_time for t in all_imu_timestamps]

    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    fig.suptitle("WildBridge Telemetry Frequency Analysis", fontsize=14)

    axes[0].plot(pkt_t, pkt_f, color="steelblue", linewidth=1.2, label="Packet rate")
    axes[0].set_ylabel("Frequency (Hz)")
    axes[0].set_title(f"All packets  —  rolling window: {window} packets")
    axes[0].legend()
    axes[0].grid(True, alpha=0.4)

    if imu_t:
        axes[1].plot(imu_t, imu_f, color="darkorange", linewidth=1.2, label="IMU yaw update rate")
    else:
        axes[1].text(0.5, 0.5, "No IMU yaw changes detected",
                     ha="center", va="center", transform=axes[1].transAxes)
    axes[1].set_ylabel("Frequency (Hz)")
    axes[1].set_title(f"IMU yaw field updates  —  rolling window: {window} updates")
    axes[1].legend()
    axes[1].grid(True, alpha=0.4)

    if yaw_t:
        axes[2].plot(yaw_t, all_yaw_values, color="mediumseagreen", linewidth=1.2, label="Yaw")
    else:
        axes[2].text(0.5, 0.5, "No yaw data",
                     ha="center", va="center", transform=axes[2].transAxes)
    axes[2].set_ylabel("Yaw (°)")
    axes[2].set_xlabel("Elapsed time (s)")
    axes[2].set_title("IMU yaw value over time")
    axes[2].legend()
    axes[2].grid(True, alpha=0.4)

    plt.tight_layout()
    plt.show()


# ── CLI entry point ───────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Measure WildBridge telemetry frequency and IMU yaw update rate. "
            "Spin the drone during the test to observe how fast yaw changes are received."
        )
    )
    parser.add_argument("--host", default="192.168.50.83",
                        help="IP address of the Android device (default: 192.168.50.83)")
    parser.add_argument("--port", type=int, default=8081,
                        help="TCP port of the TelemetryServer (default: 8081)")
    parser.add_argument("--duration", type=float, default=30.0,
                        help="Measurement duration in seconds (0 = unlimited, default: 30)")
    parser.add_argument("--window", type=int, default=50,
                        help="Rolling window size for instantaneous frequency (default: 50)")
    args = parser.parse_args()

    analyze_telemetry(args.host, args.port, args.duration, args.window)


if __name__ == "__main__":
    main()
